#include "app/imu_app.h"
#include "drivers/uart_cli.h"
#include "drivers/mpu6050.h"
#include <string.h>

static I2C_HandleTypeDef *s_hi2c = NULL;

static volatile bool s_tick_due = false;
static volatile bool s_stream_en = false;

static uint32_t s_print_div = 0;
static uint32_t s_tick_count = 0;
static uint32_t s_sample_count = 0;
static uint32_t s_missed = 0;

static mpu6050_raw_t s_last;

void imu_app_init(I2C_HandleTypeDef *hi2c)
{
  s_hi2c = hi2c;
  imu_app_stats_reset();
}

void imu_app_on_100hz_tick(void)
{
  s_tick_count++;

  if (!s_stream_en) return;

  // if previous tick not handled yet => CPU busy / blocked
  if (s_tick_due) {
    s_missed++;
    return;
  }

  s_tick_due = true;
}

void imu_app_poll(void)
{
  if (!s_stream_en) return;
  if (!s_tick_due) return;
  if (!s_hi2c) return;

  s_tick_due = false;

  if (mpu6050_read_raw(s_hi2c, MPU6050_ADDR7_DEFAULT, &s_last) == MPU6050_OK) {
    s_sample_count++;

    if (s_print_div != 0 && (s_sample_count % s_print_div) == 0) {
      uart_cli_sendf("raw ax=%d ay=%d az=%d gx=%d gy=%d gz=%d\r\n",
                     (int)s_last.ax, (int)s_last.ay, (int)s_last.az,
                     (int)s_last.gx, (int)s_last.gy, (int)s_last.gz);
    }
  } else {
    uart_cli_send("mpu read error\r\n");
  }
}

void imu_app_stream_set(bool en) { s_stream_en = en; }
bool imu_app_stream_get(void) { return s_stream_en; }

void imu_app_set_print_div(uint32_t n) { s_print_div = n; }
uint32_t imu_app_get_print_div(void) { return s_print_div; }

void imu_app_stats_reset(void)
{
  s_tick_due = false;
  s_tick_count = 0;
  s_sample_count = 0;
  s_missed = 0;
  memset(&s_last, 0, sizeof(s_last));
}
