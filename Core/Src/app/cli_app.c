#include "app/cli_app.h"
#include "drivers/uart_cli.h"
#include "stm32f4xx_hal.h"

#include "drivers/i2c_reg.h"
#include "app/imu_app.h"
#include "app/app_config.h"

#if SENSOR_GY91
#include "drivers/mpu9255.h"
#include "drivers/ak8963.h"
#else
#include "drivers/mpu6050.h"
#endif

#include <string.h>
#include <math.h> /* sqrtf for MPU MAG norm display */
#include <ctype.h>
#include <stdlib.h>
#include <stdbool.h>

/* ─────────────────────────────────────────────────────────────
 * Helpers: trim and case conversion
 * ───────────────────────────────────────────────────────────── */

static void trim_inplace(char *s)
{
  /* remove leading spaces by shifting left */
  while (*s && isspace((unsigned char)*s))
  {
    memmove(s, s + 1, strlen(s));
  }

  /* remove trailing spaces */
  size_t n = strlen(s);
  while (n > 0 && isspace((unsigned char)s[n - 1]))
  {
    s[n - 1] = '\0';
    n--;
  }
}

static void to_upper_inplace(char *s)
{
  while (*s)
  {
    *s = (char)toupper((unsigned char)*s);
    s++;
  }
}

/* ─────────────────────────────────────────────────────────────
 * Tokenizer: split "MPU STREAM ON" into argv[]
 * This lets us support multi-word commands without changing CLI core.
 * ───────────────────────────────────────────────────────────── */

static int tokenize(char *s, char *argv[], int max_argv)
{
  int argc = 0;

  while (*s && argc < max_argv)
  {
    while (*s && isspace((unsigned char)*s))
      s++;
    if (!*s)
      break;

    argv[argc++] = s;

    while (*s && !isspace((unsigned char)*s))
      s++;
    if (*s)
    {
      *s = '\0';
      s++;
    }
  }

  return argc;
}

/* Parse "68" or "0x68" or "104" (decimal) */
static uint32_t parse_u32_auto(const char *s)
{
  if (!s)
    return 0;

  if (strlen(s) > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X'))
  {
    return (uint32_t)strtoul(s + 2, NULL, 16);
  }

  /* if it contains hex letters, parse as hex */
  for (const char *p = s; *p; p++)
  {
    if ((*p >= 'A' && *p <= 'F') || (*p >= 'a' && *p <= 'f'))
    {
      return (uint32_t)strtoul(s, NULL, 16);
    }
  }

  return (uint32_t)strtoul(s, NULL, 10);
}

/* ─────────────────────────────────────────────────────────────
 * CLI banner
 * ───────────────────────────────────────────────────────────── */

void app_cli_print_banner(void)
{
  uart_cli_send("\r\n============================\r\n");
  uart_cli_send(" EKF vs Madgwick (STM32F411)\r\n");
  uart_cli_send(" UART CLI enabled\r\n");
  uart_cli_send("============================\r\n");
  uart_cli_send("Type: HELP\r\n");
}

/* ─────────────────────────────────────────────────────────────
 * Command handler
 * ───────────────────────────────────────────────────────────── */

void app_cli_handle_line(const char *line)
{
  /* Make a mutable copy of the line */
  char cmd[128];
  strncpy(cmd, line, sizeof(cmd) - 1);
  cmd[sizeof(cmd) - 1] = '\0';
  trim_inplace(cmd);

  if (cmd[0] == '\0')
    return;

  /* Uppercase copy (we will tokenize this one for comparisons) */
  char up[128];
  strncpy(up, cmd, sizeof(up) - 1);
  up[sizeof(up) - 1] = '\0';
  to_upper_inplace(up);

  /* Tokenize uppercase string for command matching */
  char up_mut[128];
  strncpy(up_mut, up, sizeof(up_mut) - 1);
  up_mut[sizeof(up_mut) - 1] = '\0';

  char *argv[8];
  int argc = tokenize(up_mut, argv, 8);

  if (argc == 0)
    return;

  /* ─────────── Basic commands ─────────── */

  if (strcmp(argv[0], "HELP") == 0)
  {
    uart_cli_send("Commands:\r\n");
    uart_cli_send("  HELP\r\n");
    uart_cli_send("  I2C SCAN\r\n");
    uart_cli_send("  MPU WHOAMI\r\n");
    uart_cli_send("  MPU INIT\r\n");
    uart_cli_send("  MPU STREAM ON|OFF\r\n");
    uart_cli_send("  MAD RESET\r\n");
    uart_cli_send("  EKF RESET\r\n");
    uart_cli_send("  MPU CAL GYRO <ms>\r\n");
    return;
  }

  /* Removed PING and STATUS commands */

  /* ─────────── I2C commands ─────────── */
  if (strcmp(argv[0], "I2C") == 0)
  {
    extern I2C_HandleTypeDef hi2c1;

    if (argc >= 2 && strcmp(argv[1], "SCAN") == 0)
    {
      uint8_t found[16];
      size_t cnt = 0;

      i2c_scan(&hi2c1, found, 16, &cnt);

      uart_cli_sendf("found=%lu\r\n", (unsigned long)cnt);

      size_t n = (cnt > 16) ? 16 : cnt;
      for (size_t i = 0; i < n; i++)
      {
        uart_cli_sendf("0x%02X\r\n", found[i]);
      }

      if (cnt > 16)
        uart_cli_send("(more not shown)\r\n");
      return;
    }

    uart_cli_send("usage: I2C SCAN\r\n");
    return;
  }

  /* ─────────── MPU commands ─────────── */
  if (strcmp(argv[0], "MPU") == 0)
  {
    extern I2C_HandleTypeDef hi2c1;
#if !SENSOR_GY91
    /* cfg is shared across MPU CFG / MPU INIT on the legacy path only.
     * On the GY91 path each handler declares its own local cfg.         */
    static mpu6050_cfg_t cfg;
#endif

    if (argc >= 2 && strcmp(argv[1], "WHOAMI") == 0)
    {
#if SENSOR_GY91
      uint8_t id = 0;
      if (mpu9255_whoami(&hi2c1, MPU9255_ADDR_7BIT, &id) != MPU9255_OK)
      {
        uart_cli_send("ERR: whoami\r\n");
        return;
      }
      uart_cli_sendf("MPU addr=0x%02X WHO_AM_I=0x%02X (%s)\r\n",
                     MPU9255_ADDR_7BIT, id,
                     (id == MPU9255_WHOAMI_VAL) ? "MPU-9255" : (id == MPU9250_WHOAMI_VAL) ? "MPU-9250"
                                                                                          : "unknown");
#else
      uint8_t id = 0;
      if (mpu6050_whoami(&hi2c1, MPU6050_ADDR7_DEFAULT, &id) != MPU6050_OK)
      {
        uart_cli_send("ERR: whoami\r\n");
        return;
      }
      uart_cli_sendf("MPU addr=0x%02X WHO_AM_I=0x%02X\r\n", MPU6050_ADDR7_DEFAULT, id);
#endif
      return;
    }

    if (argc >= 2 && strcmp(argv[1], "INIT") == 0)
    {
#if SENSOR_GY91
      mpu9255_cfg_t cfg;
      mpu9255_status_t st = mpu9255_init_200hz(&hi2c1, MPU9255_ADDR_7BIT, &cfg);

      if (st == MPU9255_ERR_ID)
      {
#if MPU92XX_VARIANT == MPU92XX_VARIANT_9255
        uart_cli_send("ERR: wrong WHO_AM_I (expected MPU-9255)\r\n");
#elif MPU92XX_VARIANT == MPU92XX_VARIANT_9250
        uart_cli_send("ERR: wrong WHO_AM_I (expected MPU-9250)\r\n");
#else
        uart_cli_send("ERR: wrong WHO_AM_I (not MPU-9255/9250?)\r\n");
#endif
        return;
      }
      if (st != MPU9255_OK)
      {
        uart_cli_send("ERR: mpu9255 init failed\r\n");
        return;
      }
      /* Enable bypass to expose AK8963 on the I2C bus */
      if (mpu9255_enable_bypass(&hi2c1, MPU9255_ADDR_7BIT) != MPU9255_OK)
      {
        uart_cli_send("ERR: bypass enable failed\r\n");
        return;
      }
      const char *name = (cfg.whoami == MPU9255_WHOAMI_VAL) ? "MPU-9255" : (cfg.whoami == MPU9250_WHOAMI_VAL) ? "MPU-9250"
                                                                                                              : "unknown";
      uart_cli_sendf("mpu init ok (%s, bypass enabled)\r\n", name);
#else
      mpu6050_status_t st = mpu6050_init_200hz(&hi2c1, MPU6050_ADDR7_DEFAULT, &cfg);
      if (st == MPU6050_ERR_ID)
      {
        uart_cli_send("ERR: wrong WHO_AM_I (not MPU6050?)\r\n");
        return;
      }
      if (st != MPU6050_OK)
      {
        uart_cli_send("ERR: init failed\r\n");
        return;
      }
      uart_cli_send("mpu init ok\r\n");
#endif
      return;
    }

    if (argc >= 3 && strcmp(argv[1], "STREAM") == 0)
    {
      if (strcmp(argv[2], "ON") == 0)
      {
        imu_app_stream_set(true);
        uart_cli_send("stream on\r\n");
        return;
      }
      if (strcmp(argv[2], "OFF") == 0)
      {
        imu_app_stream_set(false);
        uart_cli_send("stream off\r\n");
        return;
      }
      uart_cli_send("usage: MPU STREAM ON|OFF\r\n");
      return;
    }

    if (argc >= 4 && strcmp(argv[1], "CAL") == 0 && strcmp(argv[2], "GYRO") == 0)
    {
      uint32_t ms = parse_u32_auto(argv[3]);
      if (ms == 0 || ms > 60000)
      {
        uart_cli_send("ERR: ms out of range\r\n");
        return;
      }
      uart_cli_send("Calibrating gyro... keep still\r\n");
      if (imu_app_cal_gyro(ms))
      {
        int16_t x, y, z;
        imu_app_cal_get(&x, &y, &z);
        uart_cli_sendf("done. offsets=(%d %d %d)\r\n", x, y, z);
      }
      else
      {
        uart_cli_send("ERR: calibration failed\r\n");
      }
      return;
    }

    if (argc >= 3 && strcmp(argv[1], "PRINT") == 0)
    {
      uint32_t n = parse_u32_auto(argv[2]);
      imu_app_set_print_div(n);
      uart_cli_send("ok\r\n");
      return;
    }

    uart_cli_send("usage: MPU WHOAMI | INIT | STREAM ON|OFF | PRINT <N> | CAL GYRO <ms>\r\n");
    return;
  }

  /* ─────────── Madgwick commands ─────────── */
  if (strcmp(argv[0], "MAD") == 0)
  {
    if (argc >= 2 && strcmp(argv[1], "RESET") == 0)
    {
      imu_app_madgwick_reset();
      uart_cli_send("ok\r\n");
      return;
    }

    uart_cli_send("usage: MAD RESET\r\n");
    return;
  }

  /* ─────────── EKF commands ─────────── */
  if (strcmp(argv[0], "EKF") == 0)
  {
    if (argc >= 2 && strcmp(argv[1], "RESET") == 0)
    {
      imu_app_ekf_reset();
      uart_cli_send("ok\r\n");
      return;
    }

    uart_cli_send("usage: EKF RESET\r\n");
    return;
  }

  uart_cli_send("ERR: unknown command. Type HELP\r\n");
}
