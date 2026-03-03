#include "app/imu_app.h"
#include "drivers/uart_cli.h"
#include "drivers/mpu6050.h"
#include "utils/timebase.h"
#include "filters/madgwick.h"
#include "utils/math3d.h"
#include "app/imu_types.h"
#include "app/app_config.h"

#include <string.h>

static I2C_HandleTypeDef *s_hi2c = NULL;

static volatile bool s_tick_due   = false;
static volatile bool s_stream_en  = false;

static uint32_t s_print_div = 0;

static uint32_t s_tick_count   = 0;
static uint32_t s_sample_count = 0;
static uint32_t s_missed       = 0;

static mpu6050_raw_t s_last;

// time + dt stats
static uint32_t s_reset_ms         = 0;
static uint32_t s_last_sample_cyc  = 0;          // store cycles (wrap-safe)
static uint32_t s_dt_min_us        = 0xFFFFFFFFu;
static uint32_t s_dt_max_us        = 0;
static uint64_t s_dt_sum_us        = 0;
static uint32_t s_svc_last_us      = 0;
static uint32_t s_svc_max_us       = 0;
static uint32_t s_last_miss_tick   = 0;

// rate window (~1s) using cycles
static uint32_t s_win_start_cyc    = 0;
static uint32_t s_win_samples      = 0;

// avoid float printf issues: store milli-Hz (100000 = 100.000 Hz)
static uint32_t s_last_rate_mhz    = 0;

// Madgwick state/output
static madgwick_t s_mad;
static Attitude_t s_mad_att;
static uint8_t s_mad_valid = 0;

// --- Gyro raw offsets ---
static int16_t s_gx_off = 0;
static int16_t s_gy_off = 0;
static int16_t s_gz_off = 0;

static const float ACC_LSB_PER_G    = 16384.0f;
static const float GYRO_LSB_PER_DPS = 131.0f;
static const float DEG2RAD          = 0.017453292519943295f;

void imu_app_init(I2C_HandleTypeDef *hi2c)
{
  s_hi2c = hi2c;
  imu_app_stats_reset();

  // Madgwick init (beta is a tuning parameter)
  madgwick_init(&s_mad, MADGWICK_BETA);
  // optional accel rejection (helps during strong linear accel)
  madgwick_set_accel_reject(&s_mad,
                            MADGWICK_ACCEL_REJECT_EN,
                            MADGWICK_ACCEL_MIN_G,
                            MADGWICK_ACCEL_MAX_G);

  s_mad_valid = 0;
}

void imu_app_on_100hz_tick(void)
{
  s_tick_count++;

  if (!s_stream_en) return;

  // If previous tick wasn't handled yet => overrun/missed tick
  if (s_tick_due) {
    s_missed++;
    s_last_miss_tick = s_tick_count;
    return;
  }

  s_tick_due = true;
}

void imu_app_poll(void)
{
  if (!s_stream_en) return;
  if (!s_tick_due)  return;
  if (!s_hi2c)      return;

  // consume the tick
  s_tick_due = false;

  // capture timestamp for this sample (used for dt + rate window)
  uint32_t now_cyc  = timebase_cycles();
  uint32_t prev_cyc = s_last_sample_cyc;

  // measure service time around the I2C read
  uint32_t t0 = now_cyc;

  // read MPU
  if (mpu6050_read_raw(s_hi2c, MPU6050_ADDR_7BIT, &s_last) == MPU6050_OK) {

    // ---- compute dt (seconds) from last successful sample ----
    float dt_s = 0.0f;
    if (prev_cyc != 0u) {
      uint32_t dt_cyc = now_cyc - prev_cyc;
      uint32_t dt_us  = timebase_cycles_to_us(dt_cyc);

      dt_s = (float)dt_us * 1e-6f;

      // dt stats
      if (dt_us < s_dt_min_us) s_dt_min_us = dt_us;
      if (dt_us > s_dt_max_us) s_dt_max_us = dt_us;
      s_dt_sum_us += dt_us;
    }

    // update last sample timestamp (cycles)
    s_last_sample_cyc = now_cyc;

    // sample counter
    s_sample_count++;

    // ---- rate window (~1 second) ----
    if (s_win_start_cyc == 0) s_win_start_cyc = now_cyc;
    s_win_samples++;

    uint32_t win_cyc = now_cyc - s_win_start_cyc;
    uint32_t win_us  = timebase_cycles_to_us(win_cyc);

    if (win_us >= 1000000u) {
      s_last_rate_mhz = (uint32_t)(((uint64_t)s_win_samples * 1000000000ull) / (uint64_t)win_us);
      s_win_start_cyc = now_cyc;
      s_win_samples   = 0;
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

    // ---- REMAP sensor -> body (final: permutation + sign + X/Y swap) ----
    // Base permutation from 3-pose:
    //   candidate X = Z_sensor
    //   candidate Y = Y_sensor
    //   Z_body      = X_sensor
    float bx = az_s;   // candidate X
    float by = ay_s;   // candidate Y
    float bz = ax_s;   // Z

    float gx = wz_s;   // candidate wx
    float gy = wy_s;   // candidate wy
    float gz = wx_s;   // wz

    // Sign fixes found from tests:
    bx = -bx;   gx = -gx;   // flip candidate X
    by = -by;   gy = -gy;   // flip candidate Y
    // bz, gz unchanged

    // Swap X and Y to match your physical tilt directions:
    float ax_g = by;
    float ay_g = bx;
    float az_g = bz;

    float wx   = gy;
    float wy   = gx;
    float wz   = gz;

    // ---- Madgwick update (body-frame inputs, measured dt) ----
#if RUN_MADGWICK
    if (dt_s > 0.0f) {

      madgwick_update_imu(&s_mad, wx, wy, wz, ax_g, ay_g, az_g, dt_s);

      s_mad_att.q0 = s_mad.q0; s_mad_att.q1 = s_mad.q1;
      s_mad_att.q2 = s_mad.q2; s_mad_att.q3 = s_mad.q3;

      math3d_quat_to_euler_deg(s_mad.q0, s_mad.q1, s_mad.q2, s_mad.q3,
                               &s_mad_att.roll_deg, &s_mad_att.pitch_deg, &s_mad_att.yaw_deg);
      s_mad_valid = 1;
    }
#else
    (void)dt_s; // avoid unused warning if you ever compile w/o madgwick
#endif

    // optional streaming print (decimated)
    if (s_print_div != 0 && (s_sample_count % s_print_div) == 0) {
      uart_cli_sendf("raw ax=%d ay=%d az=%d gx=%d gy=%d gz=%d\r\n",
                     (int)s_last.ax, (int)s_last.ay, (int)s_last.az,
                     (int)s_last.gx, (int)s_last.gy, (int)s_last.gz);
    }

  } else {
    uart_cli_send("mpu read error\r\n");
  }

  uint32_t t1 = timebase_cycles();
  uint32_t svc_us = timebase_cycles_to_us(t1 - t0);
  s_svc_last_us = svc_us;
  if (svc_us > s_svc_max_us) s_svc_max_us = svc_us;
}

void imu_app_stream_set(bool en)
{
  s_stream_en = en;

  // If you turn streaming off, clear pending tick to avoid counting stale work
  if (!en) s_tick_due = false;
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
  s_tick_due     = false;

  s_tick_count   = 0;
  s_sample_count = 0;
  s_missed       = 0;

  memset(&s_last, 0, sizeof(s_last));

  s_reset_ms        = HAL_GetTick();
  s_last_sample_cyc = 0;
  s_dt_min_us       = 0xFFFFFFFFu;
  s_dt_max_us       = 0;
  s_dt_sum_us       = 0;
  s_svc_last_us     = 0;
  s_svc_max_us      = 0;
  s_last_miss_tick  = 0;

  s_win_start_cyc   = timebase_cycles();
  s_win_samples     = 0;
  s_last_rate_mhz   = 0;

  // reset Madgwick output validity (optional)
  s_mad_valid = 0;
}

// integer rate getter (recommended for printing)
uint32_t imu_app_get_rate_mhz(void)
{
  return s_last_rate_mhz;
}

void imu_app_get_stats(imu_stats_t *out)
{
  if (!out) return;

  // Snapshot volatile-ish state
  __disable_irq();
  uint32_t ticks   = s_tick_count;
  uint32_t samples = s_sample_count;
  uint32_t missed  = s_missed;
  bool tick_due    = s_tick_due;
  bool stream_en   = s_stream_en;
  __enable_irq();

  uint32_t elapsed_ms = HAL_GetTick() - s_reset_ms;

  uint32_t dt_avg = 0;
  if (samples > 1) {
    dt_avg = (uint32_t)(s_dt_sum_us / (uint64_t)(samples - 1));
  }

  out->ticks      = ticks;
  out->samples    = samples;
  out->missed     = missed;

  out->stream_en  = (uint8_t)(stream_en ? 1 : 0);
  out->tick_due   = (uint8_t)(tick_due ? 1 : 0);

  out->dt_min_us  = (s_dt_min_us == 0xFFFFFFFFu) ? 0 : s_dt_min_us;
  out->dt_avg_us  = dt_avg;
  out->dt_max_us  = s_dt_max_us;

  out->svc_last_us = s_svc_last_us;
  out->svc_max_us  = s_svc_max_us;
  out->last_miss_tick = s_last_miss_tick;

  out->rate_hz    = (float)s_last_rate_mhz / 1000.0f;

  out->elapsed_ms = elapsed_ms;
}

float imu_app_get_rate_hz(void)
{
  return (float)imu_app_get_rate_mhz() / 1000.0f;
}

bool imu_app_get_madgwick(Attitude_t *out)
{
  if (!out) return false;

  // Snapshot to avoid tearing if called during updates
  __disable_irq();
  uint8_t valid = s_mad_valid;
  Attitude_t tmp = s_mad_att;
  __enable_irq();

  if (!valid) return false;
  *out = tmp;
  return true;
}

void imu_app_madgwick_reset(void)
{
  __disable_irq();
  madgwick_reset(&s_mad);
  s_mad_valid = 0;
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
  if (!gx_off || !gy_off || !gz_off) return false;
  *gx_off = s_gx_off;
  *gy_off = s_gy_off;
  *gz_off = s_gz_off;
  return true;
}

bool imu_app_cal_gyro(uint32_t duration_ms)
{
  if (!s_hi2c) return false;
  if (duration_ms < 200) return false;

  uint32_t start = HAL_GetTick();
  uint32_t count = 0;

  int64_t sum_x = 0;
  int64_t sum_y = 0;
  int64_t sum_z = 0;

  mpu6050_raw_t r;

  while ((HAL_GetTick() - start) < duration_ms) {

    if (mpu6050_read_raw(s_hi2c, MPU6050_ADDR_7BIT, &r) == MPU6050_OK) {
      sum_x += r.gx;
      sum_y += r.gy;
      sum_z += r.gz;
      count++;
    }

    HAL_Delay(2);  // small delay (~500 Hz max read)
  }

  if (count == 0) return false;

  s_gx_off = (int16_t)(sum_x / (int64_t)count);
  s_gy_off = (int16_t)(sum_y / (int64_t)count);
  s_gz_off = (int16_t)(sum_z / (int64_t)count);

  return true;
}
