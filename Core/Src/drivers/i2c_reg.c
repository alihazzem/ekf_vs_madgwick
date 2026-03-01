#include "drivers/i2c_reg.h"

static i2c_reg_status_t map_status(HAL_StatusTypeDef st) {
  if (st == HAL_TIMEOUT) return I2C_REG_TIMEOUT;
  if (st == HAL_OK)      return I2C_REG_OK;
  return I2C_REG_ERROR;
}

i2c_reg_status_t i2c_scan(I2C_HandleTypeDef *hi2c, uint8_t *found, size_t max_found, size_t *out_count)
{
  if (!hi2c || !found || !out_count) return I2C_REG_ERROR;

  size_t cnt = 0;
  for (uint8_t a = 0x03; a <= 0x77; a++) {
    if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(a << 1), 2, 10) == HAL_OK) {
      if (cnt < max_found) found[cnt] = a;
      cnt++;
    }
  }
  *out_count = cnt;
  return I2C_REG_OK;
}

i2c_reg_status_t i2c_read_reg(I2C_HandleTypeDef *hi2c, uint8_t addr7, uint8_t reg,
                              uint8_t *buf, size_t len, uint32_t timeout_ms)
{
  if (!hi2c || !buf || len == 0) return I2C_REG_ERROR;

  uint16_t addr8 = (uint16_t)(addr7 << 1);

  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Read(hi2c,
                       addr8,
                       reg,
                       I2C_MEMADD_SIZE_8BIT,
                       buf,
                       (uint16_t)len,
                       timeout_ms);

  return map_status(st);
}

i2c_reg_status_t i2c_write_reg(I2C_HandleTypeDef *hi2c, uint8_t addr7, uint8_t reg,
                               uint8_t val, uint32_t timeout_ms)
{
  if (!hi2c) return I2C_REG_ERROR;

  uint16_t addr8 = (uint16_t)(addr7 << 1);

  HAL_StatusTypeDef st =
      HAL_I2C_Mem_Write(hi2c,
                        addr8,
                        reg,
                        I2C_MEMADD_SIZE_8BIT,
                        &val,
                        1,
                        timeout_ms);

  return map_status(st);
}
