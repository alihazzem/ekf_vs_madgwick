#pragma once
#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stddef.h>

/**
 * Minimal I2C register utilities:
 * - scan for devices
 * - read register(s)
 * - write register byte
 */

typedef enum {
  I2C_REG_OK = 0,
  I2C_REG_TIMEOUT,
  I2C_REG_ERROR
} i2c_reg_status_t;

i2c_reg_status_t i2c_scan(I2C_HandleTypeDef *hi2c, uint8_t *found, size_t max_found, size_t *out_count);

i2c_reg_status_t i2c_read_reg(I2C_HandleTypeDef *hi2c, uint8_t addr7, uint8_t reg,
                              uint8_t *buf, size_t len, uint32_t timeout_ms);

i2c_reg_status_t i2c_write_reg(I2C_HandleTypeDef *hi2c, uint8_t addr7, uint8_t reg,
                               uint8_t val, uint32_t timeout_ms);
