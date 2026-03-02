#include "app/imu_app.h"
#include "drivers/uart_cli.h"
#include "drivers/mpu6050.h"
#include "utils/timebase.h"

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
static uint32_t s_svc_last_us = 0;
static uint32_t s_svc_max_us  = 0;
static uint32_t s_last_miss_tick = 0;

// rate window (~1s) using cycles
static uint32_t s_win_start_cyc    = 0;
static uint32_t s_win_samples      = 0;

// avoid float printf issues: store milli-Hz (100000 = 100.000 Hz)
static uint32_t s_last_rate_mhz    = 0;

void imu_app_init(I2C_HandleTypeDef *hi2c)
{
  s_hi2c = hi2c;
  imu_app_stats_reset();
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
  if (!s_tick_due) return;
  if (!s_hi2c) return;

  // consume the tick
  s_tick_due = false;

  // capture timestamp for this sample (used for dt + rate window)
  uint32_t now_cyc = timebase_cycles();

  // measure service time around the I2C read
  uint32_t t0 = now_cyc;

  // read MPU
  if (mpu6050_read_raw(s_hi2c, MPU6050_ADDR7_DEFAULT, &s_last) == MPU6050_OK) {

    // dt stats (only for successful samples)
    if (s_last_sample_cyc != 0) {
      uint32_t dt_cyc = now_cyc - s_last_sample_cyc;
      uint32_t dt_us  = timebase_cycles_to_us(dt_cyc);

      if (dt_us < s_dt_min_us) s_dt_min_us = dt_us;
      if (dt_us > s_dt_max_us) s_dt_max_us = dt_us;
      s_dt_sum_us += dt_us;
    }
    s_last_sample_cyc = now_cyc;

    // sample counter
    s_sample_count++;

    // rate window (~1 second)
    if (s_win_start_cyc == 0) s_win_start_cyc = now_cyc;
    s_win_samples++;

    uint32_t win_cyc = now_cyc - s_win_start_cyc;
    uint32_t win_us  = timebase_cycles_to_us(win_cyc);

    if (win_us >= 1000000u) {
      s_last_rate_mhz = (uint32_t)(((uint64_t)s_win_samples * 1000000000ull) / (uint64_t)win_us);
      s_win_start_cyc = now_cyc;
      s_win_samples   = 0;
    }

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
  s_svc_last_us = 0;
  s_svc_max_us  = 0;
  s_last_miss_tick = 0;

  s_win_start_cyc   = timebase_cycles();
  s_win_samples     = 0;
  s_last_rate_mhz   = 0;
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

  // keep the float field if it exists in your struct, but set it from integer:
  out->rate_hz    = (float)s_last_rate_mhz / 1000.0f;

  out->elapsed_ms = elapsed_ms;
}
