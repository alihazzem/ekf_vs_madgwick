#pragma once
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

void imu_app_init(I2C_HandleTypeDef *hi2c);
void imu_app_on_100hz_tick(void);   // call from TIM2 callback
void imu_app_poll(void);            // call from main loop

// control / stats
void imu_app_stream_set(bool en);
bool imu_app_stream_get(void);
void imu_app_set_print_div(uint32_t n); // print every n samples (0=off)
uint32_t imu_app_get_print_div(void);
void imu_app_stats_reset(void);
