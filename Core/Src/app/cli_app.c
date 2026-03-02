#include "app/cli_app.h"
#include "drivers/uart_cli.h"
#include "stm32f4xx_hal.h"

#include "drivers/i2c_reg.h"
#include "drivers/mpu6050.h"
#include "app/imu_app.h"

#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdbool.h>

/* ─────────────────────────────────────────────────────────────
 * Helpers: trim and case conversion
 * ───────────────────────────────────────────────────────────── */

static void trim_inplace(char *s)
{
  /* remove leading spaces by shifting left */
  while (*s && isspace((unsigned char)*s)) {
    memmove(s, s + 1, strlen(s));
  }

  /* remove trailing spaces */
  size_t n = strlen(s);
  while (n > 0 && isspace((unsigned char)s[n - 1])) {
    s[n - 1] = '\0';
    n--;
  }
}

static void to_upper_inplace(char *s)
{
  while (*s) {
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

  while (*s && argc < max_argv) {
    while (*s && isspace((unsigned char)*s)) s++;
    if (!*s) break;

    argv[argc++] = s;

    while (*s && !isspace((unsigned char)*s)) s++;
    if (*s) {
      *s = '\0';
      s++;
    }
  }

  return argc;
}

/* Parse "68" or "0x68" or "104" (decimal) */
static uint32_t parse_u32_auto(const char *s)
{
  if (!s) return 0;

  if (strlen(s) > 2 && s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) {
    return (uint32_t)strtoul(s + 2, NULL, 16);
  }

  /* if it contains hex letters, parse as hex */
  for (const char *p = s; *p; p++) {
    if ((*p >= 'A' && *p <= 'F') || (*p >= 'a' && *p <= 'f')) {
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

  if (cmd[0] == '\0') return;

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

  if (argc == 0) return;

  /* ─────────── Basic commands ─────────── */

  if (strcmp(argv[0], "HELP") == 0) {
    uart_cli_send("Commands:\r\n");
    uart_cli_send("  HELP\r\n");
    uart_cli_send("  PING\r\n");
    uart_cli_send("  STATUS\r\n");
    uart_cli_send("  I2C SCAN\r\n");
    uart_cli_send("  I2C R <addr> <reg> [len]\r\n");
    uart_cli_send("  I2C W <addr> <reg> <val>\r\n");
    uart_cli_send("  MPU WHOAMI\r\n");
    uart_cli_send("  MPU INIT\r\n");
    uart_cli_send("  MPU CFG\r\n");
    uart_cli_send("  MPU READ\r\n");
    uart_cli_send("  MPU STREAM ON|OFF\r\n");
    uart_cli_send("  MPU PRINT <N>\r\n");
    uart_cli_send("  MPU RATE\r\n");
    uart_cli_send("  MPU STATS [RESET]\r\n");
    return;
  }

  if (strcmp(argv[0], "PING") == 0) {
    uart_cli_send("PONG\r\n");
    return;
  }

  if (strcmp(argv[0], "STATUS") == 0) {
    uart_cli_sendf("uptime_ms=%lu\r\n", (unsigned long)HAL_GetTick());
    uart_cli_send("uart=USART2 115200\r\n");
    uart_cli_send("i2c=I2C1 PB8/PB9\r\n");
    uart_cli_sendf("imu_stream=%d print_div=%lu\r\n",
                   (int)imu_app_stream_get(),
                   (unsigned long)imu_app_get_print_div());
    return;
  }

  /* ─────────── I2C commands ─────────── */
  if (strcmp(argv[0], "I2C") == 0) {
    extern I2C_HandleTypeDef hi2c1;

    if (argc >= 2 && strcmp(argv[1], "SCAN") == 0) {
      uint8_t found[16];
      size_t cnt = 0;

      i2c_scan(&hi2c1, found, 16, &cnt);

      uart_cli_sendf("found=%lu\r\n", (unsigned long)cnt);

      size_t n = (cnt > 16) ? 16 : cnt;
      for (size_t i = 0; i < n; i++) {
        uart_cli_sendf("0x%02X\r\n", found[i]);
      }

      if (cnt > 16) uart_cli_send("(more not shown)\r\n");
      return;
    }

    if (argc >= 4 && strcmp(argv[1], "R") == 0) {
      uint8_t addr = (uint8_t)parse_u32_auto(argv[2]);
      uint8_t reg  = (uint8_t)parse_u32_auto(argv[3]);
      uint32_t len = (argc >= 5) ? parse_u32_auto(argv[4]) : 1u;

      if (len == 0 || len > 32) {
        uart_cli_send("ERR: len must be 1..32\r\n");
        return;
      }

      uint8_t data[32];
      if (i2c_read_reg(&hi2c1, addr, reg, data, len, 100) != I2C_REG_OK) {
        uart_cli_send("ERR: i2c read failed\r\n");
        return;
      }

      uart_cli_sendf("0x%02X: ", reg);
      for (uint32_t i = 0; i < len; i++) uart_cli_sendf("%02X ", data[i]);
      uart_cli_send("\r\n");
      return;
    }

    if (argc >= 5 && strcmp(argv[1], "W") == 0) {
      uint8_t addr = (uint8_t)parse_u32_auto(argv[2]);
      uint8_t reg  = (uint8_t)parse_u32_auto(argv[3]);
      uint8_t val  = (uint8_t)parse_u32_auto(argv[4]);

      if (i2c_write_reg(&hi2c1, addr, reg, val, 100) != I2C_REG_OK) {
        uart_cli_send("ERR: i2c write failed\r\n");
        return;
      }

      uart_cli_send("ok\r\n");
      return;
    }

    uart_cli_send("usage: I2C SCAN | I2C R <addr> <reg> [len] | I2C W <addr> <reg> <val>\r\n");
    return;
  }

  /* ─────────── MPU commands ─────────── */
  if (strcmp(argv[0], "MPU") == 0) {
    extern I2C_HandleTypeDef hi2c1;
    static mpu6050_cfg_t cfg;

    if (argc >= 2 && strcmp(argv[1], "WHOAMI") == 0) {
      uint8_t id = 0;
      if (mpu6050_whoami(&hi2c1, MPU6050_ADDR7_DEFAULT, &id) != MPU6050_OK) {
        uart_cli_send("ERR: whoami\r\n");
        return;
      }
      uart_cli_sendf("MPU addr=0x%02X WHO_AM_I=0x%02X\r\n", MPU6050_ADDR7_DEFAULT, id);
      return;
    }

    if (argc >= 2 && strcmp(argv[1], "INIT") == 0) {
      mpu6050_status_t st = mpu6050_init_100hz(&hi2c1, MPU6050_ADDR7_DEFAULT, &cfg);

      if (st == MPU6050_ERR_ID) {
        uart_cli_send("ERR: wrong WHO_AM_I (not MPU6050?)\r\n");
        return;
      }
      if (st != MPU6050_OK) {
        uart_cli_send("ERR: init failed\r\n");
        return;
      }

      uart_cli_send("mpu init ok\r\n");
      return;
    }

    if (argc >= 2 && strcmp(argv[1], "CFG") == 0) {
      if (mpu6050_read_cfg(&hi2c1, MPU6050_ADDR7_DEFAULT, &cfg) != MPU6050_OK) {
        uart_cli_send("ERR: cfg\r\n");
        return;
      }

      uart_cli_sendf("WHOAMI=0x%02X PWR=0x%02X DIV=%u CFG=0x%02X GYRO=0x%02X ACC=0x%02X\r\n",
                     cfg.whoami, cfg.pwr_mgmt_1, cfg.smplrt_div, cfg.config,
                     cfg.gyro_config, cfg.accel_config);
      return;
    }

    if (argc >= 2 && strcmp(argv[1], "READ") == 0) {
      mpu6050_raw_t r;
      if (mpu6050_read_raw(&hi2c1, MPU6050_ADDR7_DEFAULT, &r) != MPU6050_OK) {
        uart_cli_send("ERR: read\r\n");
        return;
      }

      uart_cli_sendf("ax=%d ay=%d az=%d temp=%d gx=%d gy=%d gz=%d\r\n",
                     (int)r.ax, (int)r.ay, (int)r.az, (int)r.temp,
                     (int)r.gx, (int)r.gy, (int)r.gz);
      return;
    }

    if (argc >= 3 && strcmp(argv[1], "STREAM") == 0) {
      if (strcmp(argv[2], "ON") == 0)  { imu_app_stream_set(true);  uart_cli_send("stream on\r\n");  return; }
      if (strcmp(argv[2], "OFF") == 0) { imu_app_stream_set(false); uart_cli_send("stream off\r\n"); return; }
      uart_cli_send("usage: MPU STREAM ON|OFF\r\n");
      return;
    }

    if (argc >= 3 && strcmp(argv[1], "PRINT") == 0) {
      uint32_t n = parse_u32_auto(argv[2]);
      imu_app_set_print_div(n);
      uart_cli_send("ok\r\n");
      return;
    }

    if (argc >= 2 && strcmp(argv[1], "RATE") == 0) {
    	uint32_t mhz = imu_app_get_rate_mhz();
    	uart_cli_sendf("rate_hz=%lu.%03lu\r\n",
    	               (unsigned long)(mhz/1000u),
    	               (unsigned long)(mhz%1000u));
      return;
    }

    if (argc >= 2 && strcmp(argv[1], "STATS") == 0) {
      if (argc >= 3 && strcmp(argv[2], "RESET") == 0) {
        imu_app_stats_reset();
        uart_cli_send("ok\r\n");
        return;
      }

      imu_stats_t st;
      imu_app_get_stats(&st);

      uart_cli_sendf("elapsed_ms=%lu stream=%u tick_due=%u print_div=%lu\r\n",
                     (unsigned long)st.elapsed_ms,
                     (unsigned)st.stream_en,
                     (unsigned)st.tick_due,
                     (unsigned long)imu_app_get_print_div());

      uart_cli_sendf("ticks=%lu samples=%lu missed=%lu\r\n",
                     (unsigned long)st.ticks,
                     (unsigned long)st.samples,
                     (unsigned long)st.missed);

      uart_cli_sendf("rate_hz=%.2f dt_us(min/avg/max)=%lu/%lu/%lu\r\n",
                     (double)st.rate_hz,
                     (unsigned long)st.dt_min_us,
                     (unsigned long)st.dt_avg_us,
                     (unsigned long)st.dt_max_us);

      uart_cli_sendf("svc_us(last/max)=%lu/%lu last_miss_tick=%lu\r\n",
                     (unsigned long)st.svc_last_us,
                     (unsigned long)st.svc_max_us,
                     (unsigned long)st.last_miss_tick);

      if (st.ticks > 0) {
        uint32_t miss_ppm = (uint32_t)(((uint64_t)st.missed * 1000000ull) / (uint64_t)st.ticks);
        uart_cli_sendf("miss_ppm=%lu\r\n", (unsigned long)miss_ppm);
      }

      return;
    }

    uart_cli_send("usage: MPU WHOAMI|INIT|CFG|READ|STREAM ON|OFF|PRINT <N>|RATE|STATS [RESET]\r\n");
    return;
  }

  uart_cli_send("ERR: unknown command. Type HELP\r\n");
}
