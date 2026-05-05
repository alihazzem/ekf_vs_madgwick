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
#if SENSOR_GY91
    uart_cli_send("  MPU MAG\r\n");
    uart_cli_send("  MPU MAG STREAM ON|OFF\r\n");
#endif
    uart_cli_send("  MAD SHOW\r\n");
    uart_cli_send("  MAD BETA <value>\r\n");
    uart_cli_send("  MAD RESET\r\n");
    uart_cli_send("  EKF SHOW\r\n");
    uart_cli_send("  EKF RESET\r\n");
    uart_cli_send("  EKF BIAS\r\n");
    uart_cli_send("  EKF DIAG\r\n");
    uart_cli_send("  EKF TUNE <sigma_g> <sigma_b> <sigma_a> <sigma_m> <r_k>\r\n");
    uart_cli_send("  MPU CAL GYRO <ms>\r\n");
    uart_cli_send("  MPU CAL SHOW\r\n");
    uart_cli_send("  MPU CAL CLEAR\r\n");
    return;
  }

  if (strcmp(argv[0], "PING") == 0)
  {
    uart_cli_send("PONG\r\n");
    return;
  }

  if (strcmp(argv[0], "STATUS") == 0)
  {
    uart_cli_sendf("uptime_ms=%lu\r\n", (unsigned long)HAL_GetTick());
    uart_cli_send("uart=USART2 115200\r\n");
    uart_cli_send("i2c=I2C1 PB8/PB9\r\n");
    uart_cli_sendf("imu_stream=%d print_div=%lu\r\n",
                   (int)imu_app_stream_get(),
                   (unsigned long)imu_app_get_print_div());
    return;
  }

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

    if (argc >= 4 && strcmp(argv[1], "R") == 0)
    {
      uint8_t addr = (uint8_t)parse_u32_auto(argv[2]);
      uint8_t reg = (uint8_t)parse_u32_auto(argv[3]);
      uint32_t len = (argc >= 5) ? parse_u32_auto(argv[4]) : 1u;

      if (len == 0 || len > 32)
      {
        uart_cli_send("ERR: len must be 1..32\r\n");
        return;
      }

      uint8_t data[32];
      if (i2c_read_reg(&hi2c1, addr, reg, data, len, 100) != I2C_REG_OK)
      {
        uart_cli_send("ERR: i2c read failed\r\n");
        return;
      }

      uart_cli_sendf("0x%02X: ", reg);
      for (uint32_t i = 0; i < len; i++)
        uart_cli_sendf("%02X ", data[i]);
      uart_cli_send("\r\n");
      return;
    }

    if (argc >= 5 && strcmp(argv[1], "W") == 0)
    {
      uint8_t addr = (uint8_t)parse_u32_auto(argv[2]);
      uint8_t reg = (uint8_t)parse_u32_auto(argv[3]);
      uint8_t val = (uint8_t)parse_u32_auto(argv[4]);

      if (i2c_write_reg(&hi2c1, addr, reg, val, 100) != I2C_REG_OK)
      {
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

    if (argc >= 2 && strcmp(argv[1], "CFG") == 0)
    {
#if SENSOR_GY91
      mpu9255_cfg_t cfg;
      if (mpu9255_read_cfg(&hi2c1, MPU9255_ADDR_7BIT, &cfg) != MPU9255_OK)
      {
        uart_cli_send("ERR: cfg\r\n");
        return;
      }
      uart_cli_sendf("WHOAMI=0x%02X PWR=0x%02X DIV=%u CFG=0x%02X GYRO=0x%02X ACC=0x%02X\r\n",
                     cfg.whoami, cfg.pwr_mgmt_1, cfg.smplrt_div, cfg.config,
                     cfg.gyro_config, cfg.accel_config);
#else
      if (mpu6050_read_cfg(&hi2c1, MPU6050_ADDR7_DEFAULT, &cfg) != MPU6050_OK)
      {
        uart_cli_send("ERR: cfg\r\n");
        return;
      }
      uart_cli_sendf("WHOAMI=0x%02X PWR=0x%02X DIV=%u CFG=0x%02X GYRO=0x%02X ACC=0x%02X\r\n",
                     cfg.whoami, cfg.pwr_mgmt_1, cfg.smplrt_div, cfg.config,
                     cfg.gyro_config, cfg.accel_config);
#endif
      return;
    }

    if (argc >= 2 && strcmp(argv[1], "READ") == 0)
    {
#if SENSOR_GY91
      mpu9255_raw_t r;
      if (mpu9255_read_raw(&hi2c1, MPU9255_ADDR_7BIT, &r) != MPU9255_OK)
      {
        uart_cli_send("ERR: read\r\n");
        return;
      }
      uart_cli_sendf("ax=%d ay=%d az=%d temp=%d gx=%d gy=%d gz=%d\r\n",
                     (int)r.ax, (int)r.ay, (int)r.az, (int)r.temp,
                     (int)r.gx, (int)r.gy, (int)r.gz);
#else
      mpu6050_raw_t r;
      if (mpu6050_read_raw(&hi2c1, MPU6050_ADDR7_DEFAULT, &r) != MPU6050_OK)
      {
        uart_cli_send("ERR: read\r\n");
        return;
      }
      uart_cli_sendf("ax=%d ay=%d az=%d temp=%d gx=%d gy=%d gz=%d\r\n",
                     (int)r.ax, (int)r.ay, (int)r.az, (int)r.temp,
                     (int)r.gx, (int)r.gy, (int)r.gz);
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

    if (argc >= 3 && strcmp(argv[1], "PRINT") == 0)
    {
      uint32_t n = parse_u32_auto(argv[2]);
      imu_app_set_print_div(n);
      uart_cli_send("ok\r\n");
      return;
    }

    if (argc >= 2 && strcmp(argv[1], "RATE") == 0)
    {
      uint32_t mhz = imu_app_get_rate_mhz();
      uart_cli_sendf("rate_hz=%lu.%03lu\r\n",
                     (unsigned long)(mhz / 1000u),
                     (unsigned long)(mhz % 1000u));
      return;
    }

    if (argc >= 2 && strcmp(argv[1], "STATS") == 0)
    {
      if (argc >= 3 && strcmp(argv[2], "RESET") == 0)
      {
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

      if (st.ticks > 0)
      {
        uint32_t miss_ppm = (uint32_t)(((uint64_t)st.missed * 1000000ull) / (uint64_t)st.ticks);
        uart_cli_sendf("miss_ppm=%lu\r\n", (unsigned long)miss_ppm);
      }

      return;
    }

    if (argc >= 2 && strcmp(argv[1], "CAL") == 0)
    {

      if (argc >= 3 && strcmp(argv[2], "CLEAR") == 0)
      {
        imu_app_cal_clear();
        uart_cli_send("gyro offsets cleared\r\n");
        return;
      }

      if (argc >= 3 && strcmp(argv[2], "SHOW") == 0)
      {
        int16_t x, y, z;
        if (imu_app_cal_get(&x, &y, &z))
        {
          uart_cli_sendf("gyro_off_raw=(%d %d %d)\r\n", x, y, z);
        }
        else
        {
          uart_cli_send("ERR\r\n");
        }
        return;
      }

      if (argc >= 4 && strcmp(argv[2], "GYRO") == 0)
      {
        uint32_t ms = parse_u32_auto(argv[3]);

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

      uart_cli_send("usage: MPU CAL GYRO <ms> | MPU CAL SHOW | MPU CAL CLEAR\r\n");
      return;
    }

#if SENSOR_GY91
    if (argc >= 4 && strcmp(argv[1], "MAG") == 0 && strcmp(argv[2], "STREAM") == 0)
    {
      if (strcmp(argv[3], "ON") == 0)
      {
        imu_app_mag_stream_set(true);
        uart_cli_send("mag cal stream (M,...) on\r\n");
      }
      else if (strcmp(argv[3], "OFF") == 0)
      {
        imu_app_mag_stream_set(false);
        uart_cli_send("mag cal stream off\r\n");
      }
      else
      {
        uart_cli_send("usage: MPU MAG STREAM ON|OFF\r\n");
      }
      return;
    }

    if (argc == 2 && strcmp(argv[1], "MAG") == 0)
    {
      /* Direct live read — bypass the poll-loop buffer.
       * AK8963 at 100 Hz: new sample every ~10 ms.
       * Retry up to 5 times (×3 ms = 15 ms max) to catch DRDY. */
      ak8963_raw_t mag;
      mag.valid = 0;
      for (int attempt = 0; attempt < 5; attempt++)
      {
        if (ak8963_read_raw(&hi2c1, &mag) == AK8963_OK && mag.valid)
          break;
        HAL_Delay(3);
      }

      /* ASA correction factors (read once at boot, stored in imu_app) */
      ak8963_cfg_t ak;
      imu_app_get_ak_cfg(&ak);

      /* Body-frame µT: apply remap then ASA × 0.15 µT/LSB
       * REMAP swaps X↔Y and negates Z (AK8963 die vs accel/gyro die).
       * Print × 100 for 2 decimal places without float printf.            */
      int16_t mx_b = (int16_t)REMAP_MX(mag.mx, mag.my, mag.mz);
      int16_t my_b = (int16_t)REMAP_MY(mag.mx, mag.my, mag.mz);
      int16_t mz_b = (int16_t)REMAP_MZ(mag.mx, mag.my, mag.mz);

      int32_t mx_100 = (int32_t)((float)mx_b * ak.asa_x * 15.0f); /* × 0.15 × 100 = × 15 */
      int32_t my_100 = (int32_t)((float)my_b * ak.asa_y * 15.0f);
      int32_t mz_100 = (int32_t)((float)mz_b * ak.asa_z * 15.0f);

      /* Norm × 100 (integer sqrt via Newton step — good enough for display) */
      float mx_f = (float)mx_100 * 0.01f;
      float my_f = (float)my_100 * 0.01f;
      float mz_f = (float)mz_100 * 0.01f;
      int32_t norm_100 = (int32_t)(sqrtf(mx_f * mx_f + my_f * my_f + mz_f * mz_f) * 100.0f);

      /* ASA as X.XXX */
      int32_t asa_xi = (int32_t)(ak.asa_x * 1000.0f);
      int32_t asa_yi = (int32_t)(ak.asa_y * 1000.0f);
      int32_t asa_zi = (int32_t)(ak.asa_z * 1000.0f);

#define _ABS(x) ((x) < 0 ? -(x) : (x))
      uart_cli_sendf("--- AK8963 sensor frame (raw) ---\r\n");
      uart_cli_sendf("mx_s=%d  my_s=%d  mz_s=%d  valid=%u\r\n",
                     (int)mag.mx, (int)mag.my, (int)mag.mz, (unsigned)mag.valid);

      uart_cli_sendf("--- Body frame (remapped + ASA + 0.15 uT/LSB) ---\r\n");
      uart_cli_sendf("mx=%ld.%02lu  my=%ld.%02lu  mz=%ld.%02lu  uT\r\n",
                     (long)(mx_100 / 100), (unsigned long)_ABS(mx_100 % 100),
                     (long)(my_100 / 100), (unsigned long)_ABS(my_100 % 100),
                     (long)(mz_100 / 100), (unsigned long)_ABS(mz_100 % 100));
      uart_cli_sendf("norm=%ld.%02lu uT\r\n",
                     (long)(norm_100 / 100), (unsigned long)(norm_100 % 100));

      uart_cli_sendf("--- Body frame (calibrated) ---\r\n");
      float mx_cal = MAG_CAL_S11 * (mx_f - MAG_CAL_BX) + MAG_CAL_S12 * (my_f - MAG_CAL_BY) + MAG_CAL_S13 * (mz_f - MAG_CAL_BZ);
      float my_cal = MAG_CAL_S21 * (mx_f - MAG_CAL_BX) + MAG_CAL_S22 * (my_f - MAG_CAL_BY) + MAG_CAL_S23 * (mz_f - MAG_CAL_BZ);
      float mz_cal = MAG_CAL_S31 * (mx_f - MAG_CAL_BX) + MAG_CAL_S32 * (my_f - MAG_CAL_BY) + MAG_CAL_S33 * (mz_f - MAG_CAL_BZ);

      int32_t mx_cal_100 = (int32_t)(mx_cal * 100.0f);
      int32_t my_cal_100 = (int32_t)(my_cal * 100.0f);
      int32_t mz_cal_100 = (int32_t)(mz_cal * 100.0f);
      int32_t norm_cal_100 = (int32_t)(sqrtf(mx_cal * mx_cal + my_cal * my_cal + mz_cal * mz_cal) * 100.0f);

      uart_cli_sendf("mx=%ld.%02lu  my=%ld.%02lu  mz=%ld.%02lu  uT\r\n",
                     (long)(mx_cal_100 / 100), (unsigned long)_ABS(mx_cal_100 % 100),
                     (long)(my_cal_100 / 100), (unsigned long)_ABS(my_cal_100 % 100),
                     (long)(mz_cal_100 / 100), (unsigned long)_ABS(mz_cal_100 % 100));
      uart_cli_sendf("norm=%ld.%02lu uT\r\n",
                     (long)(norm_cal_100 / 100), (unsigned long)(norm_cal_100 % 100));

      uart_cli_sendf("asa=%ld.%03lu / %ld.%03lu / %ld.%03lu  (x/y/z)\r\n",
                     (long)(asa_xi / 1000), (unsigned long)(asa_xi % 1000),
                     (long)(asa_yi / 1000), (unsigned long)(asa_yi % 1000),
                     (long)(asa_zi / 1000), (unsigned long)(asa_zi % 1000));
#undef _ABS
      return;
    }
#endif /* SENSOR_GY91 */

    uart_cli_send("usage: MPU WHOAMI|INIT|CFG|READ|STREAM ON|OFF|PRINT <N>|RATE|STATS [RESET]|CAL ..."
#if SENSOR_GY91
                  "|MAG|MAG STREAM"
#endif
                  "\r\n");
    return;
  }

  /* ─────────── Madgwick commands ─────────── */
  if (strcmp(argv[0], "MAD") == 0)
  {

    if (argc >= 2 && strcmp(argv[1], "SHOW") == 0)
    {
      Attitude_t a;
      if (!imu_app_get_madgwick(&a))
      {
        uart_cli_send("ERR: madgwick not ready (enable MPU STREAM ON)\r\n");
        return;
      }

      int32_t r = (int32_t)(a.roll_deg * 1000.0f);
      int32_t p = (int32_t)(-a.pitch_deg * 1000.0f);
      int32_t y = (int32_t)(a.yaw_deg * 1000.0f);

      uart_cli_sendf("rpy_mdeg=(%ld %ld %ld)\r\n",
                     (long)r, (long)p, (long)y);

      uart_cli_sendf("step_us=%lu\r\n", (unsigned long)imu_app_mad_last_us());
      return;
    }

    if (argc >= 3 && strcmp(argv[1], "BETA") == 0)
    {
      // allow float like 0.08
      float beta = (float)strtod(argv[2], NULL);
      if (beta <= 0.0f || beta > 5.0f)
      {
        uart_cli_send("ERR: beta range (0..5]\r\n");
        return;
      }
      imu_app_madgwick_set_beta(beta);
      uart_cli_sendf("ok beta=%.4f\r\n", (double)imu_app_madgwick_get_beta());
      return;
    }

    if (argc >= 2 && strcmp(argv[1], "RESET") == 0)
    {
      imu_app_madgwick_reset();
      uart_cli_send("ok\r\n");
      return;
    }

    uart_cli_send("usage: MAD SHOW | MAD BETA <value> | MAD RESET\r\n");
    return;
  }

  /* ─────────── EKF commands ─────────── */
  if (strcmp(argv[0], "EKF") == 0)
  {

    if (argc >= 2 && strcmp(argv[1], "SHOW") == 0)
    {
      Attitude_t a;
      if (!imu_app_get_ekf(&a))
      {
        uart_cli_send("ERR: EKF not ready (enable MPU STREAM ON)\r\n");
        return;
      }

      int32_t r = (int32_t)(a.roll_deg * 1000.0f);
      int32_t p = (int32_t)(-a.pitch_deg * 1000.0f);
      int32_t y = (int32_t)(a.yaw_deg * 1000.0f);
      uart_cli_sendf("rpy_mdeg=(%ld %ld %ld)\r\n",
                     (long)r, (long)p, (long)y);
      uart_cli_sendf("step_us=%lu\r\n", (unsigned long)imu_app_ekf_last_us());
      return;
    }

    if (argc >= 2 && strcmp(argv[1], "RESET") == 0)
    {
      imu_app_ekf_reset();
      uart_cli_send("ok\r\n");
      return;
    }

    if (argc >= 2 && strcmp(argv[1], "BIAS") == 0)
    {
      float bx, by, bz;
      imu_app_ekf_get_bias(&bx, &by, &bz);
      /* print in µrad/s as integers */
      int32_t ibx = (int32_t)(bx * 1e6f);
      int32_t iby = (int32_t)(by * 1e6f);
      int32_t ibz = (int32_t)(bz * 1e6f);
      uart_cli_sendf("bias_uradps=(%ld %ld %ld)\r\n",
                     (long)ibx, (long)iby, (long)ibz);
      return;
    }

    if (argc >= 2 && strcmp(argv[1], "DIAG") == 0)
    {
      /* trace(P) — decreases as the filter converges */
      float tr = imu_app_ekf_trace_p();
      int32_t tr_1e6 = (int32_t)(tr * 1e6f);

      float bx, by, bz;
      imu_app_ekf_get_bias(&bx, &by, &bz);
      int32_t ibx = (int32_t)(bx * 1e6f);
      int32_t iby = (int32_t)(by * 1e6f);
      int32_t ibz = (int32_t)(bz * 1e6f);

      uart_cli_sendf("traceP_1e6=%ld\r\n", (long)tr_1e6);
      uart_cli_sendf("bias_uradps=(%ld %ld %ld)\r\n", (long)ibx, (long)iby, (long)ibz);
      uart_cli_sendf("step_us=%lu\r\n", (unsigned long)imu_app_ekf_last_us());
      return;
    }

    if (argc >= 7 && strcmp(argv[1], "TUNE") == 0)
    {
      float sg = (float)strtod(argv[2], NULL);
      float sb = (float)strtod(argv[3], NULL);
      float sa = (float)strtod(argv[4], NULL);
      float sm = (float)strtod(argv[5], NULL);
      float rk = (float)strtod(argv[6], NULL);
      if (sg <= 0.0f || sb <= 0.0f || sa <= 0.0f || sm <= 0.0f || rk < 0.0f)
      {
        uart_cli_send("ERR: all params must be > 0 (r_k >= 0)\r\n");
        return;
      }
      imu_app_ekf_set_noise(sg, sb, sa, sm, rk);
      uart_cli_send("ok\r\n");
      return;
    }

    uart_cli_send("usage: EKF SHOW | EKF RESET | EKF BIAS | EKF DIAG | EKF TUNE <sg> <sb> <sa> <sm> <rk>\r\n");
    return;
  }

  uart_cli_send("ERR: unknown command. Type HELP\r\n");
}
