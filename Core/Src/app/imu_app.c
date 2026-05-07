#include "app/imu_app.h"
#include "drivers/uart_cli.h"
#include "utils/timebase.h"
#include "filters/madgwick.h"
#include "filters/ekf.h"
#include "utils/math3d.h"
#include "app/imu_types.h"
#include "app/app_config.h"
#include "FreeRTOS.h"
#include "task.h"

#if SENSOR_GY91
#include "drivers/mpu9255.h"
#include "drivers/ak8963.h"
#else
#include "drivers/mpu6050.h"
#endif

#include <string.h>
#include <math.h> /* sqrtf for mag norm */

static I2C_HandleTypeDef *s_hi2c = NULL;

static volatile bool s_stream_en = false;

static uint32_t s_print_div = 0;

static uint32_t s_tick_count = 0;
static uint32_t s_sample_count = 0;
static uint32_t s_missed = 0;

#if SENSOR_GY91
static mpu9255_raw_t s_last;
static ak8963_raw_t s_last_mag;
static ak8963_cfg_t s_ak_cfg;
/* Body-frame magnetometer (µT): remap + ASA applied, not yet calibrated */
static float s_mx_ut = 0.0f;
static float s_my_ut = 0.0f;
static float s_mz_ut = 0.0f;
static float s_m_norm = 0.0f;
static uint8_t s_mag_body_valid = 0;
/* Calibrated body-frame mag (µT): hard + soft iron correction applied */
static float s_mx_cal_ut = 0.0f;
static float s_my_cal_ut = 0.0f;
static float s_mz_cal_ut = 0.0f;
static float s_m_cal_norm = 0.0f;
static uint8_t s_mag_cal_valid = 0;
/* Flag: emit M, lines for offline ellipsoid calibration capture */
static volatile bool s_mag_stream_en = false;
#else
static mpu6050_raw_t s_last;
#endif

// time + dt stats
static uint32_t s_reset_ms = 0;
static uint32_t s_last_sample_cyc = 0; // store cycles (wrap-safe)
static uint32_t s_dt_min_us = 0xFFFFFFFFu;
static uint32_t s_dt_max_us = 0;
static uint64_t s_dt_sum_us = 0;
static uint32_t s_svc_last_us = 0;
static uint32_t s_svc_max_us = 0;
static uint32_t s_last_miss_tick = 0;

// rate window (~1s) using cycles
static uint32_t s_win_start_cyc = 0;
static uint32_t s_win_samples = 0;

// avoid float printf issues: store milli-Hz (100000 = 100.000 Hz)
static uint32_t s_last_rate_mhz = 0;

// Madgwick state/output
static madgwick_t s_mad;
static Attitude_t s_mad_att;
static uint8_t s_mad_valid = 0;
static uint8_t s_mad_aligned = 0;  // set to 1 after first-sample gravity alignment
static uint32_t s_mad_last_us = 0; // CPU time of last Madgwick step

// EKF state/output
static ekf7_t s_ekf;
static Attitude_t s_ekf_att;
static uint8_t s_ekf_valid = 0;
static uint8_t s_ekf_aligned = 0;  // set to 1 after first-sample gravity alignment
static uint32_t s_ekf_last_us = 0; // CPU time of last EKF step

// --- Gyro raw offsets ---
static int16_t s_gx_off = 0;
static int16_t s_gy_off = 0;
static int16_t s_gz_off = 0;

static const float ACC_LSB_PER_G = 16384.0f;
static const float GYRO_LSB_PER_DPS = 131.0f;
static const float DEG2RAD = 0.017453292519943295f;

void imu_app_init(I2C_HandleTypeDef *hi2c)
{
  s_hi2c = hi2c;
  imu_app_stats_reset();

#if SENSOR_GY91
  /* ---- GY-91: MPU-9255 + AK8963 bring-up ---- */
  {
    mpu9255_cfg_t mpu_cfg;
    mpu9255_status_t st = mpu9255_init_200hz(hi2c, MPU9255_ADDR_7BIT, &mpu_cfg);
    if (st == MPU9255_OK)
    {
      /* Enable bypass so STM32 can reach AK8963 on the same bus */
      mpu9255_enable_bypass(hi2c, MPU9255_ADDR_7BIT);

      /* Read sensitivity adjustment from AK8963 fuse ROM */
      if (ak8963_read_asa(hi2c, &s_ak_cfg) == AK8963_OK)
      {
        /* Start continuous 100 Hz 16-bit measurement */
        ak8963_init_continuous_100hz(hi2c);
      }
    }
    /* Zero-initialise last mag sample — valid=0 until first read */
    s_last_mag.mx = 0;
    s_last_mag.my = 0;
    s_last_mag.mz = 0;
    s_last_mag.valid = 0;
  }
#endif /* SENSOR_GY91 */

  // Madgwick init
  madgwick_init(&s_mad, MADGWICK_BETA);
  madgwick_set_accel_reject(&s_mad,
                            MADGWICK_ACCEL_REJECT_EN,
                            MADGWICK_ACCEL_MIN_G,
                            MADGWICK_ACCEL_MAX_G);
  madgwick_set_bias_gain(&s_mad, MADGWICK_ZETA);
  madgwick_set_adaptive_beta(&s_mad, MADGWICK_BETA_START, MADGWICK_BETA_DECAY_S);
  madgwick_set_motion_gain(&s_mad, MADGWICK_BETA_MOTION_K, MADGWICK_BETA_MIN);

  s_mad_valid = 0;
  s_mad_aligned = 0;

#if RUN_EKF
  ekf7_init(&s_ekf, EKF_SIGMA_GYRO, EKF_SIGMA_BIAS, EKF_SIGMA_ACCEL, EKF_SIGMA_MAG,
            EKF_R_ADAPT_K, EKF_P0);
  /* Hard-reject window configured via EKF_ACCEL_* (adaptive R still applies) */
  ekf7_set_accel_reject(&s_ekf, EKF_ACCEL_REJECT_EN, EKF_ACCEL_MIN_G, EKF_ACCEL_MAX_G, EKF_ACCEL_TIMEOUT_S);
  s_ekf_valid = 0;
  s_ekf_aligned = 0;
#endif
}

void imu_app_add_missed(uint32_t count)
{
  s_missed += count;
  s_last_miss_tick = s_tick_count; // Approximation
}

void imu_app_step(void)
{
  s_tick_count++;

  if (!s_stream_en)
    return;
  if (!s_hi2c)
    return;

  // capture timestamp for this sample (used for dt + rate window)
  uint32_t now_cyc = timebase_cycles();
  uint32_t prev_cyc = s_last_sample_cyc;

  // measure service time around the I2C read
  uint32_t t0 = now_cyc;

  // read IMU (accel + gyro)
#if SENSOR_GY91
  if (mpu9255_read_raw(s_hi2c, MPU9255_ADDR_7BIT, &s_last) == MPU9255_OK)
  {
    /* Best-effort AK8963 read — if DRDY=0, s_last_mag.valid stays 0
     * and the previous sample is preserved unchanged.              */
    ak8963_read_raw(s_hi2c, &s_last_mag);
#else
  if (mpu6050_read_raw(s_hi2c, MPU6050_ADDR_7BIT, &s_last) == MPU6050_OK)
  {
#endif /* SENSOR_GY91 */

    // ---- compute dt (seconds) from last successful sample ----
    float dt_s = 0.0f;
    if (prev_cyc != 0u)
    {
      uint32_t dt_cyc = now_cyc - prev_cyc;

      /* Use full 84MHz resolution (11.9ns) for the filter integration time.
       * This avoids the ~1us rounding error inherent in timebase_cycles_to_us(). */
      dt_s = (float)dt_cyc / 84000000.0f;

      // Keep integer dt_us for stats and display
      uint32_t dt_us = timebase_cycles_to_us(dt_cyc);

      // dt stats
      if (dt_us < s_dt_min_us)
        s_dt_min_us = dt_us;
      if (dt_us > s_dt_max_us)
        s_dt_max_us = dt_us;
      s_dt_sum_us += dt_us;
    }

    // update last sample timestamp (cycles)
    s_last_sample_cyc = now_cyc;

    // sample counter
    s_sample_count++;

    // ---- rate window (~1 second) ----
    if (s_win_start_cyc == 0)
      s_win_start_cyc = now_cyc;
    s_win_samples++;

    uint32_t win_cyc = now_cyc - s_win_start_cyc;
    uint32_t win_us = timebase_cycles_to_us(win_cyc);

    if (win_us >= 1000000u)
    {
      s_last_rate_mhz = (uint32_t)(((uint64_t)s_win_samples * 1000000000ull) / (uint64_t)win_us);
      s_win_start_cyc = now_cyc;
      s_win_samples = 0;
    }

    // Sensor-frame accel (g) — bias-corrected then scaled
    float ax_s = (float)(s_last.ax - ACCEL_BIAS_X) / ACC_LSB_PER_G;
    float ay_s = (float)(s_last.ay - ACCEL_BIAS_Y) / ACC_LSB_PER_G;
    float az_s = (float)(s_last.az - ACCEL_BIAS_Z) / ACC_LSB_PER_G;

    // Sensor-frame gyro (rad/s)
    int16_t gx_corr = s_last.gx - s_gx_off;
    int16_t gy_corr = s_last.gy - s_gy_off;
    int16_t gz_corr = s_last.gz - s_gz_off;

    float wx_s = ((float)gx_corr / GYRO_LSB_PER_DPS) * DEG2RAD;
    float wy_s = ((float)gy_corr / GYRO_LSB_PER_DPS) * DEG2RAD;
    float wz_s = ((float)gz_corr / GYRO_LSB_PER_DPS) * DEG2RAD;

    // ---- REMAP sensor -> body (defined in app_config.h) ----
    float ax_g = REMAP_AX_G(ax_s, ay_s, az_s);
    float ay_g = REMAP_AY_G(ax_s, ay_s, az_s);
    float az_g = REMAP_AZ_G(ax_s, ay_s, az_s);

    float wx = REMAP_WX(wx_s, wy_s, wz_s);
    float wy = REMAP_WY(wx_s, wy_s, wz_s);
    float wz = REMAP_WZ(wx_s, wy_s, wz_s);

#if SENSOR_GY91
    // ---- AK8963 → body-frame µT (remap + ASA + scale) ----
    if (s_last_mag.valid)
    {
      float mx_s = (float)REMAP_MX(s_last_mag.mx, s_last_mag.my, s_last_mag.mz);
      float my_s = (float)REMAP_MY(s_last_mag.mx, s_last_mag.my, s_last_mag.mz);
      float mz_s = (float)REMAP_MZ(s_last_mag.mx, s_last_mag.my, s_last_mag.mz);

      s_mx_ut = mx_s * s_ak_cfg.asa_x * AK8963_UT_PER_LSB;
      s_my_ut = my_s * s_ak_cfg.asa_y * AK8963_UT_PER_LSB;
      s_mz_ut = mz_s * s_ak_cfg.asa_z * AK8963_UT_PER_LSB;

      float n2 = s_mx_ut * s_mx_ut + s_my_ut * s_my_ut + s_mz_ut * s_mz_ut;
      s_m_norm = (n2 > 0.0f) ? sqrtf(n2) : 0.0f;
      s_mag_body_valid = (s_m_norm >= 5.0f && s_m_norm <= 100.0f) ? 1u : 0u;

      // ---- Apply hard/soft-iron calibration (from app_config.h) ----
      float bx = s_mx_ut - MAG_CAL_BX;
      float by = s_my_ut - MAG_CAL_BY;
      float bz = s_mz_ut - MAG_CAL_BZ;
      s_mx_cal_ut = MAG_CAL_S11 * bx + MAG_CAL_S12 * by + MAG_CAL_S13 * bz;
      s_my_cal_ut = MAG_CAL_S21 * bx + MAG_CAL_S22 * by + MAG_CAL_S23 * bz;
      s_mz_cal_ut = MAG_CAL_S31 * bx + MAG_CAL_S32 * by + MAG_CAL_S33 * bz;
      float cn2 = s_mx_cal_ut * s_mx_cal_ut + s_my_cal_ut * s_my_cal_ut + s_mz_cal_ut * s_mz_cal_ut;
      s_m_cal_norm = (cn2 > 0.0f) ? sqrtf(cn2) : 0.0f;
      s_mag_cal_valid = (s_m_cal_norm >= 5.0f && s_m_cal_norm <= 100.0f) ? 1u : 0u;

      // ---- Emit M, line for offline calibration capture ----
      if (s_mag_stream_en)
      {
        /* Emit body-frame (uncalibrated) µT × 100 as integers:
         * the Python script divides by 100 to recover float µT. */
        int32_t mx_100 = (int32_t)(s_mx_ut * 100.0f);
        int32_t my_100 = (int32_t)(s_my_ut * 100.0f);
        int32_t mz_100 = (int32_t)(s_mz_ut * 100.0f);
        uart_cli_sendf("M,%ld,%ld,%ld\r\n",
                       (long)mx_100, (long)my_100, (long)mz_100);
      }
    }
#endif /* SENSOR_GY91 */

    // On the very first valid sample initialise each filter's quaternion from
    // the accelerometer so roll/pitch are correct immediately.
    if (!s_mad_aligned)
    {
#if SENSOR_GY91
      if (s_mag_cal_valid)
      {
        madgwick_init_from_marg(&s_mad, ax_g, ay_g, az_g, s_mx_cal_ut, s_my_cal_ut, s_mz_cal_ut);
      }
      else
      {
        madgwick_init_from_accel(&s_mad, ax_g, ay_g, az_g);
      }
#else
      madgwick_init_from_accel(&s_mad, ax_g, ay_g, az_g);
#endif
      s_mad_aligned = 1;
    }
#if RUN_EKF
    if (!s_ekf_aligned)
    {
#if SENSOR_GY91
      if (s_mag_cal_valid)
      {
        ekf7_init_from_marg(&s_ekf, ax_g, ay_g, az_g, s_mx_cal_ut, s_my_cal_ut, s_mz_cal_ut);
      }
      else
      {
        ekf7_init_from_accel(&s_ekf, ax_g, ay_g, az_g);
      }
#else
      ekf7_init_from_accel(&s_ekf, ax_g, ay_g, az_g);
#endif
      s_ekf_aligned = 1;
    }
#endif

    // ---- Madgwick update (body-frame inputs, measured dt) ----
#if RUN_MADGWICK
    if (dt_s > 0.0f)
    {
      uint32_t t_mad0 = timebase_cycles();
#if SENSOR_GY91
      // Safety threshold: Only use mag if the calibrated norm is roughly Earth-like (5 to 100 uT).
      // This protects the robot from spinning out if it drives over a strong magnet.
      if (s_m_cal_norm > 5.0f && s_m_cal_norm < 100.0f)
      {
        madgwick_update_marg(&s_mad, wx, wy, wz, ax_g, ay_g, az_g, s_mx_cal_ut, s_my_cal_ut, s_mz_cal_ut, dt_s);
      }
      else
      {
        madgwick_update_imu(&s_mad, wx, wy, wz, ax_g, ay_g, az_g, dt_s);
      }
#else
      madgwick_update_imu(&s_mad, wx, wy, wz, ax_g, ay_g, az_g, dt_s);
#endif
      s_mad_last_us = timebase_cycles_to_us(timebase_cycles() - t_mad0);

      s_mad_att.q0 = s_mad.q0;
      s_mad_att.q1 = s_mad.q1;
      s_mad_att.q2 = s_mad.q2;
      s_mad_att.q3 = s_mad.q3;

      math3d_quat_to_euler_deg(s_mad.q0, s_mad.q1, s_mad.q2, s_mad.q3,
                               &s_mad_att.roll_deg, &s_mad_att.pitch_deg, &s_mad_att.yaw_deg);
      s_mad_valid = 1;
    }
#else
    (void)dt_s; // avoid unused warning when compiling without Madgwick
#endif

    // ---- EKF update (body-frame inputs, measured dt) ----
#if RUN_EKF
    if (dt_s > 0.0f)
    {
      uint32_t t_ekf0 = timebase_cycles();
#if SENSOR_GY91
      // Safety threshold: Only use mag if the calibrated norm is roughly Earth-like (5 to 100 uT).
      // This protects the robot from spinning out if it drives over a strong magnet.
      if (s_last_mag.valid && s_mag_cal_valid)
      {
        ekf7_step_marg(&s_ekf, wx, wy, wz, ax_g, ay_g, az_g, s_mx_cal_ut, s_my_cal_ut, s_mz_cal_ut, dt_s);
      }
      else
      {
        ekf7_step(&s_ekf, wx, wy, wz, ax_g, ay_g, az_g, dt_s);
      }
#else
      ekf7_step(&s_ekf, wx, wy, wz, ax_g, ay_g, az_g, dt_s);
#endif
      s_ekf_last_us = timebase_cycles_to_us(timebase_cycles() - t_ekf0);

      float q_tmp[4];
      ekf7_get_quat(&s_ekf, q_tmp);
      s_ekf_att.q0 = q_tmp[0];
      s_ekf_att.q1 = q_tmp[1];
      s_ekf_att.q2 = q_tmp[2];
      s_ekf_att.q3 = q_tmp[3];
      math3d_quat_to_euler_deg(s_ekf_att.q0, s_ekf_att.q1, s_ekf_att.q2, s_ekf_att.q3,
                               &s_ekf_att.roll_deg, &s_ekf_att.pitch_deg, &s_ekf_att.yaw_deg);
      s_ekf_valid = 1;
    }
#endif

    // ---- optional streaming print (decimated) ----
    // CSV format (15 fields):
    //  D, t_ms,
    //  ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw,  (int16 sensor counts)
    //  mad_roll_mdeg, mad_pitch_mdeg, mad_yaw_mdeg,      (Madgwick output, mdeg)
    //  mad_us,                                           (Madgwick step CPU time µs)
    //  ekf_roll_mdeg, ekf_pitch_mdeg, ekf_yaw_mdeg,      (EKF output, mdeg)
    //  ekf_us                                            (EKF step CPU time µs)
    //
    // Columns are 0 when the respective filter is disabled or not yet valid.
    // The "D," prefix lets capture scripts ignore CLI chatter lines.
    if (s_print_div != 0 && (s_sample_count % s_print_div) == 0)
    {
      // Madgwick fields
      int32_t mad_r = s_mad_valid ? (int32_t)(s_mad_att.roll_deg * 1000.0f) : 0;
      int32_t mad_p = s_mad_valid ? (int32_t)(-s_mad_att.pitch_deg * 1000.0f) : 0;
      int32_t mad_y = s_mad_valid ? (int32_t)(s_mad_att.yaw_deg * 1000.0f) : 0;

      // EKF fields
      int32_t ekf_r = s_ekf_valid ? (int32_t)(s_ekf_att.roll_deg * 1000.0f) : 0;
      int32_t ekf_p = s_ekf_valid ? (int32_t)(-s_ekf_att.pitch_deg * 1000.0f) : 0;
      int32_t ekf_y = s_ekf_valid ? (int32_t)(s_ekf_att.yaw_deg * 1000.0f) : 0;
      uart_cli_sendf("D,%lu,%d,%d,%d,%d,%d,%d,"
                     "%ld,%ld,%ld,%lu,"
                     "%ld,%ld,%ld,%lu\r\n",
                     /* time */
                     (unsigned long)HAL_GetTick(),
                     /* raw sensor */
                     (int)s_last.ax, (int)s_last.ay, (int)s_last.az,
                     (int)s_last.gx, (int)s_last.gy, (int)s_last.gz,
                     /* Madgwick Euler */
                     (long)mad_r, (long)mad_p, (long)mad_y,
                     (unsigned long)s_mad_last_us,
                     /* EKF Euler */
                     (long)ekf_r, (long)ekf_p, (long)ekf_y,
                     (unsigned long)s_ekf_last_us);
    }
  }
  else
  {
#if SENSOR_GY91
    uart_cli_send("mpu9255 read error\r\n");
#else
    uart_cli_send("mpu read error\r\n");
#endif
  }

  uint32_t t1 = timebase_cycles();
  uint32_t svc_us = timebase_cycles_to_us(t1 - t0);
  s_svc_last_us = svc_us;
  if (svc_us > s_svc_max_us)
    s_svc_max_us = svc_us;
}

void imu_app_stream_set(bool en)
{
  s_stream_en = en;
}

bool imu_app_stream_get(void)
{
  return s_stream_en;
}

void imu_app_set_print_div(uint32_t n)
{
  s_print_div = n;
}

uint32_t imu_app_get_print_div(void)
{
  return s_print_div;
}

void imu_app_stats_reset(void)
{
  s_tick_count = 0;
  s_sample_count = 0;
  s_missed = 0;

  memset(&s_last, 0, sizeof(s_last));
#if SENSOR_GY91
  memset(&s_last_mag, 0, sizeof(s_last_mag));
  s_mx_ut = 0.0f;
  s_my_ut = 0.0f;
  s_mz_ut = 0.0f;
  s_m_norm = 0.0f;
  s_mx_cal_ut = 0.0f;
  s_my_cal_ut = 0.0f;
  s_mz_cal_ut = 0.0f;
  s_m_cal_norm = 0.0f;
  s_mag_body_valid = 0;
  s_mag_cal_valid = 0;
#endif

  s_reset_ms = HAL_GetTick();
  s_last_sample_cyc = 0;
  s_dt_min_us = 0xFFFFFFFFu;
  s_dt_max_us = 0;
  s_dt_sum_us = 0;
  s_svc_last_us = 0;
  s_svc_max_us = 0;
  s_last_miss_tick = 0;

  s_win_start_cyc = timebase_cycles();
  s_win_samples = 0;
  s_last_rate_mhz = 0;

  // reset Madgwick output validity and alignment flag
  s_mad_valid = 0;
  s_mad_aligned = 0;

  // reset EKF output validity and alignment flag
  s_ekf_valid = 0;
  s_ekf_aligned = 0;
}

// integer rate getter (recommended for printing)
uint32_t imu_app_get_rate_mhz(void)
{
  return s_last_rate_mhz;
}

void imu_app_get_stats(imu_stats_t *out)
{
  if (!out)
    return;

  // Snapshot volatile-ish state
  taskENTER_CRITICAL();
  uint32_t ticks = s_tick_count;
  uint32_t samples = s_sample_count;
  uint32_t missed = s_missed;
  bool stream_en = s_stream_en;
  taskEXIT_CRITICAL();

  uint32_t elapsed_ms = HAL_GetTick() - s_reset_ms;

  uint32_t dt_avg = 0;
  if (samples > 1)
  {
    dt_avg = (uint32_t)(s_dt_sum_us / (uint64_t)(samples - 1));
  }

  out->ticks = ticks;
  out->samples = samples;
  out->missed = missed;

  out->stream_en = (uint8_t)(stream_en ? 1 : 0);
  out->tick_due = 0; // Not used in RTOS mode

  out->dt_min_us = (s_dt_min_us == 0xFFFFFFFFu) ? 0 : s_dt_min_us;
  out->dt_avg_us = dt_avg;
  out->dt_max_us = s_dt_max_us;

  out->svc_last_us = s_svc_last_us;
  out->svc_max_us = s_svc_max_us;
  out->last_miss_tick = s_last_miss_tick;

  out->rate_hz = (float)s_last_rate_mhz / 1000.0f;

  out->elapsed_ms = elapsed_ms;
}

float imu_app_get_rate_hz(void)
{
  return (float)imu_app_get_rate_mhz() / 1000.0f;
}

bool imu_app_get_madgwick(Attitude_t *out)
{
  if (!out)
    return false;

  // Snapshot to avoid tearing if called during updates
  taskENTER_CRITICAL();
  uint8_t valid = s_mad_valid;
  Attitude_t tmp = s_mad_att;
  taskEXIT_CRITICAL();

  if (!valid)
    return false;
  *out = tmp;
  return true;
}

void imu_app_madgwick_reset(void)
{
  taskENTER_CRITICAL();
  madgwick_reset(&s_mad);
  s_mad_valid = 0;
  s_mad_aligned = 0; // will re-align on next sample
  taskEXIT_CRITICAL();
}

void imu_app_madgwick_set_beta(float beta)
{
  taskENTER_CRITICAL();
  madgwick_set_beta(&s_mad, beta);
  taskEXIT_CRITICAL();
}

float imu_app_madgwick_get_beta(void)
{
  float b;
  taskENTER_CRITICAL();
  b = madgwick_get_beta(&s_mad);
  taskEXIT_CRITICAL();
  return b;
}

// ------------------------------------------------------------
// Gyro calibration (blocking, uses raw LSB averaging)
// ------------------------------------------------------------

void imu_app_cal_clear(void)
{
  s_gx_off = 0;
  s_gy_off = 0;
  s_gz_off = 0;
}

bool imu_app_cal_get(int16_t *gx_off, int16_t *gy_off, int16_t *gz_off)
{
  if (!gx_off || !gy_off || !gz_off)
    return false;
  *gx_off = s_gx_off;
  *gy_off = s_gy_off;
  *gz_off = s_gz_off;
  return true;
}

bool imu_app_cal_gyro(uint32_t duration_ms)
{
  if (!s_hi2c)
    return false;
  if (duration_ms < 200)
    return false;

  uint32_t start = HAL_GetTick();
  uint32_t count = 0;

  int64_t sum_x = 0;
  int64_t sum_y = 0;
  int64_t sum_z = 0;

#if SENSOR_GY91
  mpu9255_raw_t r;
  while ((HAL_GetTick() - start) < duration_ms)
  {
    if (mpu9255_read_raw(s_hi2c, MPU9255_ADDR_7BIT, &r) == MPU9255_OK)
    {
      sum_x += r.gx;
      sum_y += r.gy;
      sum_z += r.gz;
      count++;
    }
    HAL_Delay(2);
  }
#else
  mpu6050_raw_t r;
  while ((HAL_GetTick() - start) < duration_ms)
  {
    if (mpu6050_read_raw(s_hi2c, MPU6050_ADDR_7BIT, &r) == MPU6050_OK)
    {
      sum_x += r.gx;
      sum_y += r.gy;
      sum_z += r.gz;
      count++;
    }
    HAL_Delay(2); // small delay (~500 Hz max read)
  }
#endif /* SENSOR_GY91 */

  if (count == 0)
    return false;

  s_gx_off = (int16_t)(sum_x / (int64_t)count);
  s_gy_off = (int16_t)(sum_y / (int64_t)count);
  s_gz_off = (int16_t)(sum_z / (int64_t)count);

  return true;
}

// ------------------------------------------------------------
// Madgwick timing
// ------------------------------------------------------------

uint32_t imu_app_mad_last_us(void)
{
  return s_mad_last_us;
}

// ------------------------------------------------------------
// EKF getters / setters
// ------------------------------------------------------------

bool imu_app_get_ekf(Attitude_t *out)
{
  if (!out)
    return false;

  taskENTER_CRITICAL();
  uint8_t valid = s_ekf_valid;
  Attitude_t tmp = s_ekf_att;
  taskEXIT_CRITICAL();

  if (!valid)
    return false;
  *out = tmp;
  return true;
}

void imu_app_ekf_reset(void)
{
  taskENTER_CRITICAL();
  ekf7_reset(&s_ekf);
  s_ekf_valid = 0;
  s_ekf_aligned = 0; // will re-align from accel on next sample
  taskEXIT_CRITICAL();
}

void imu_app_ekf_set_noise(float sigma_gyro, float sigma_bias,
                           float sigma_accel, float sigma_mag, float r_adapt_k)
{
  taskENTER_CRITICAL();
  ekf7_set_noise(&s_ekf, sigma_gyro, sigma_bias, sigma_accel, sigma_mag, r_adapt_k);
  taskEXIT_CRITICAL();
}

float imu_app_ekf_trace_p(void)
{
  float tr;
  taskENTER_CRITICAL();
  tr = ekf7_trace_P(&s_ekf);
  taskEXIT_CRITICAL();
  return tr;
}

void imu_app_ekf_get_bias(float *bx, float *by, float *bz)
{
  if (!bx || !by || !bz)
    return;
  float b[3];
  taskENTER_CRITICAL();
  ekf7_get_bias(&s_ekf, b);
  taskEXIT_CRITICAL();
  *bx = b[0];
  *by = b[1];
  *bz = b[2];
}

uint32_t imu_app_ekf_last_us(void)
{
  return s_ekf_last_us;
}

#if SENSOR_GY91
/* ----------------------------------------------------------------
 * Magnetometer raw getters (CLI diagnostic use only in Phase 1).
 * ---------------------------------------------------------------- */
void imu_app_get_mag_raw(ak8963_raw_t *out)
{
  if (!out)
    return;
  taskENTER_CRITICAL();
  *out = s_last_mag;
  taskEXIT_CRITICAL();
}

void imu_app_get_ak_cfg(ak8963_cfg_t *out)
{
  if (!out)
    return;
  *out = s_ak_cfg; /* written once at init, never changes */
}

/* Body-frame magnetometer (uncalibrated) getter. */
void imu_app_get_mag_body(float *mx_ut, float *my_ut, float *mz_ut,
                          float *norm, uint8_t *valid)
{
  taskENTER_CRITICAL();
  if (mx_ut)
    *mx_ut = s_mx_ut;
  if (my_ut)
    *my_ut = s_my_ut;
  if (mz_ut)
    *mz_ut = s_mz_ut;
  if (norm)
    *norm = s_m_norm;
  if (valid)
    *valid = s_mag_body_valid;
  taskEXIT_CRITICAL();
}

/* Calibrated body-frame magnetometer getter (hard + soft iron applied). */
void imu_app_get_mag_cal(float *mx_ut, float *my_ut, float *mz_ut,
                         float *norm, uint8_t *valid)
{
  taskENTER_CRITICAL();
  if (mx_ut)
    *mx_ut = s_mx_cal_ut;
  if (my_ut)
    *my_ut = s_my_cal_ut;
  if (mz_ut)
    *mz_ut = s_mz_cal_ut;
  if (norm)
    *norm = s_m_cal_norm;
  if (valid)
    *valid = s_mag_cal_valid;
  taskEXIT_CRITICAL();
}

/* Enable/disable M, line emission for offline calibration capture.
 * Automatically starts the main poll loop if not already running.  */
void imu_app_mag_stream_set(bool en)
{
  s_mag_stream_en = en;
  if (en)
    imu_app_stream_set(true); /* poll loop must be running */
}

bool imu_app_mag_stream_get(void)
{
  return s_mag_stream_en;
}
#endif /* SENSOR_GY91 */
