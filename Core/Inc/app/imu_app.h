#pragma once
#include "app/imu_types.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct
{
  uint32_t ticks;
  uint32_t samples;
  uint32_t missed;

  uint8_t stream_en;
  uint8_t tick_due;

  uint32_t svc_last_us;
  uint32_t svc_max_us;
  uint32_t last_miss_tick;

  // dt between successful samples (microseconds)
  uint32_t dt_min_us;
  uint32_t dt_avg_us;
  uint32_t dt_max_us;

  // last computed rate (Hz)
  float rate_hz;

  // time since last reset
  uint32_t elapsed_ms;
} imu_stats_t;

// Core app hooks
void imu_app_init(I2C_HandleTypeDef *hi2c);
void imu_app_on_100hz_tick(void); // call from TIM2 callback
void imu_app_poll(void);          // call from main loop

// control
void imu_app_stream_set(bool en);
bool imu_app_stream_get(void);

void imu_app_set_print_div(uint32_t n); // print every n samples (0=off)
uint32_t imu_app_get_print_div(void);

// stats
void imu_app_stats_reset(void);
void imu_app_get_stats(imu_stats_t *out);
float imu_app_get_rate_hz(void);
uint32_t imu_app_get_rate_mhz(void);

bool imu_app_get_madgwick(Attitude_t *out);
void imu_app_madgwick_reset(void);
void imu_app_madgwick_set_beta(float beta);
float imu_app_madgwick_get_beta(void);
uint32_t imu_app_mad_last_us(void); /* CPU time of last Madgwick step (µs) */

// --- EKF ---
bool imu_app_get_ekf(Attitude_t *out);
void imu_app_ekf_reset(void);
void imu_app_ekf_set_noise(float sigma_gyro, float sigma_bias,
                           float sigma_accel, float r_adapt_k);
float imu_app_ekf_trace_p(void);
void imu_app_ekf_get_bias(float *bx, float *by, float *bz);
uint32_t imu_app_ekf_last_us(void); /* CPU time of last EKF step (µs)     */

// --- Gyro calibration (raw LSB offsets) ---
void imu_app_cal_clear(void);
bool imu_app_cal_get(int16_t *gx_off, int16_t *gy_off, int16_t *gz_off);
bool imu_app_cal_gyro(uint32_t duration_ms); // blocking calibration
