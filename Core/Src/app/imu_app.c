#include "app/imu_app.h"
#include "drivers/uart_cli.h"
#include "drivers/mpu6050.h"
#include "utils/timebase.h"
#include "filters/madgwick.h"
#include "filters/ekf.h"
#include "utils/math3d.h"
#include "app/imu_types.h"
#include "app/app_config.h"

#include <string.h>

static I2C_HandleTypeDef *s_hi2c = NULL;

static volatile bool s_tick_due = false;
static volatile bool s_stream_en = false;

static uint32_t s_print_div = 0;

static uint32_t s_tick_count = 0;
static uint32_t s_sample_count = 0;
static uint32_t s_missed = 0;

static mpu6050_raw_t s_last;

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
  ekf7_init(&s_ekf, EKF_SIGMA_GYRO, EKF_SIGMA_BIAS, EKF_SIGMA_ACCEL,
            EKF_R_ADAPT_K, EKF_P0);
  /* Hard-reject window disabled — adaptive R handles dynamics gracefully */
  ekf7_set_accel_reject(&s_ekf, true, 0.85f, 1.15f);
  s_ekf_valid = 0;
  s_ekf_aligned = 0;
#endif
}

void imu_app_on_100hz_tick(void)
{
  s_tick_count++;

  if (!s_stream_en)
    return;

  // If previous tick wasn't handled yet => overrun/missed tick
  if (s_tick_due)
  {
    s_missed++;
    s_last_miss_tick = s_tick_count;
    return;
  }

  s_tick_due = true;
}

void imu_app_poll(void)
{
  if (!s_stream_en)
    return;
  if (!s_tick_due)
    return;
  if (!s_hi2c)
    return;

  // consume the tick
  s_tick_due = false;

  // capture timestamp for this sample (used for dt + rate window)
  uint32_t now_cyc = timebase_cycles();
  uint32_t prev_cyc = s_last_sample_cyc;

  // measure service time around the I2C read
  uint32_t t0 = now_cyc;

  // read MPU
  if (mpu6050_read_raw(s_hi2c, MPU6050_ADDR_7BIT, &s_last) == MPU6050_OK)
  {

    // ---- compute dt (seconds) from last successful sample ----
    float dt_s = 0.0f;
    if (prev_cyc != 0u)
    {
      uint32_t dt_cyc = now_cyc - prev_cyc;
      uint32_t dt_us = timebase_cycles_to_us(dt_cyc);

      dt_s = (float)dt_us * 1e-6f;

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

    // Sensor-frame accel (g)
    float ax_s = (float)s_last.ax / ACC_LSB_PER_G;
    float ay_s = (float)s_last.ay / ACC_LSB_PER_G;
    float az_s = (float)s_last.az / ACC_LSB_PER_G;

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
    // ---- First-sample gravity alignment (both filters) ----
    // On the very first valid sample initialise each filter's quaternion from
    // the accelerometer so roll/pitch are correct immediately.
    if (!s_mad_aligned)
    {
      madgwick_init_from_accel(&s_mad, ax_g, ay_g, az_g);
      s_mad_aligned = 1;
    }
#if RUN_EKF
    if (!s_ekf_aligned)
    {
      ekf7_init_from_accel(&s_ekf, ax_g, ay_g, az_g);
      s_ekf_aligned = 1;
    }
#endif

    // ---- Madgwick update (body-frame inputs, measured dt) ----
#if RUN_MADGWICK
    if (dt_s > 0.0f)
    {
      uint32_t t_mad0 = timebase_cycles();
      madgwick_update_imu(&s_mad, wx, wy, wz, ax_g, ay_g, az_g, dt_s);
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
      ekf7_step(&s_ekf, wx, wy, wz, ax_g, ay_g, az_g, dt_s);
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
    // CSV format (20 fields):
    //  D, t_ms,
    //  ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw,  (int16 sensor counts)
    //  mad_roll_mdeg, mad_pitch_mdeg, mad_yaw_mdeg,      (Madgwick output, mdeg)
    //  mad_us,                                           (Madgwick step CPU time µs)
    //  ekf_roll_mdeg, ekf_pitch_mdeg, ekf_yaw_mdeg,      (EKF output, mdeg)
    //  traceP_1e6,                                       (EKF trace(P) * 1e6)
    //  ekf_us,                                           (EKF step CPU time µs)
    //  bx_uradps, by_uradps, bz_uradps                  (EKF bias µrad/s)
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
      int32_t traceP = s_ekf_valid ? (int32_t)(ekf7_trace_P(&s_ekf) * 1e6f) : 0;

      float bx_f = 0.0f, by_f = 0.0f, bz_f = 0.0f;
#if RUN_EKF
      {
        float b_tmp[3] = {0.0f, 0.0f, 0.0f};
        ekf7_get_bias(&s_ekf, b_tmp);
        bx_f = b_tmp[0]; by_f = b_tmp[1]; bz_f = b_tmp[2];
      }
#endif
      int32_t bx_ur = (int32_t)(bx_f * 1e6f); // µrad/s
      int32_t by_ur = (int32_t)(by_f * 1e6f);
      int32_t bz_ur = (int32_t)(bz_f * 1e6f);

      uart_cli_sendf("D,%lu,%d,%d,%d,%d,%d,%d,"
                     "%ld,%ld,%ld,%lu,"
                     "%ld,%ld,%ld,%ld,%lu,"
                     "%ld,%ld,%ld\r\n",
                     /* time */
                     (unsigned long)HAL_GetTick(),
                     /* raw sensor */
                     (int)s_last.ax, (int)s_last.ay, (int)s_last.az,
                     (int)s_last.gx, (int)s_last.gy, (int)s_last.gz,
                     /* Madgwick */
                     (long)mad_r, (long)mad_p, (long)mad_y,
                     (unsigned long)s_mad_last_us,
                     /* EKF */
                     (long)ekf_r, (long)ekf_p, (long)ekf_y,
                     (long)traceP, (unsigned long)s_ekf_last_us,
                     /* EKF bias */
                     (long)bx_ur, (long)by_ur, (long)bz_ur);
    }
  }
  else
  {
    uart_cli_send("mpu read error\r\n");
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

  // If you turn streaming off, clear pending tick to avoid counting stale work
  if (!en)
    s_tick_due = false;
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
  s_tick_due = false;

  s_tick_count = 0;
  s_sample_count = 0;
  s_missed = 0;

  memset(&s_last, 0, sizeof(s_last));

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
  __disable_irq();
  uint32_t ticks = s_tick_count;
  uint32_t samples = s_sample_count;
  uint32_t missed = s_missed;
  bool tick_due = s_tick_due;
  bool stream_en = s_stream_en;
  __enable_irq();

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
  out->tick_due = (uint8_t)(tick_due ? 1 : 0);

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
  __disable_irq();
  uint8_t valid = s_mad_valid;
  Attitude_t tmp = s_mad_att;
  __enable_irq();

  if (!valid)
    return false;
  *out = tmp;
  return true;
}

void imu_app_madgwick_reset(void)
{
  __disable_irq();
  madgwick_reset(&s_mad);
  s_mad_valid = 0;
  s_mad_aligned = 0; // will re-align on next sample
  __enable_irq();
}

void imu_app_madgwick_set_beta(float beta)
{
  __disable_irq();
  madgwick_set_beta(&s_mad, beta);
  __enable_irq();
}

float imu_app_madgwick_get_beta(void)
{
  float b;
  __disable_irq();
  b = madgwick_get_beta(&s_mad);
  __enable_irq();
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

  __disable_irq();
  uint8_t valid = s_ekf_valid;
  Attitude_t tmp = s_ekf_att;
  __enable_irq();

  if (!valid)
    return false;
  *out = tmp;
  return true;
}

void imu_app_ekf_reset(void)
{
  __disable_irq();
  ekf7_reset(&s_ekf);
  s_ekf_valid = 0;
  s_ekf_aligned = 0; // will re-align from accel on next sample
  __enable_irq();
}

void imu_app_ekf_set_noise(float sigma_gyro, float sigma_bias,
                           float sigma_accel, float r_adapt_k)
{
  __disable_irq();
  ekf7_set_noise(&s_ekf, sigma_gyro, sigma_bias, sigma_accel, r_adapt_k);
  __enable_irq();
}

float imu_app_ekf_trace_p(void)
{
  float tr;
  __disable_irq();
  tr = ekf7_trace_P(&s_ekf);
  __enable_irq();
  return tr;
}

void imu_app_ekf_get_bias(float *bx, float *by, float *bz)
{
  if (!bx || !by || !bz)
    return;
  float b[3];
  __disable_irq();
  ekf7_get_bias(&s_ekf, b);
  __enable_irq();
  *bx = b[0];
  *by = b[1];
  *bz = b[2];
}

uint32_t imu_app_ekf_last_us(void)
{
  return s_ekf_last_us;
}
