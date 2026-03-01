#include "drivers/mpu6050.h"
#include "drivers/i2c_reg.h"

static int16_t be16(const uint8_t *p) {
  return (int16_t)(((int16_t)p[0] << 8) | p[1]);
}

mpu6050_status_t mpu6050_whoami(I2C_HandleTypeDef *hi2c, uint8_t addr7, uint8_t *out_id)
{
  if (!hi2c || !out_id) return MPU6050_ERR_PARAM;

  uint8_t id = 0;
  if (i2c_read_reg(hi2c, addr7, MPU6050_REG_WHO_AM_I, &id, 1, 50) != I2C_REG_OK)
    return MPU6050_ERR_I2C;

  *out_id = id;
  return MPU6050_OK;
}

mpu6050_status_t mpu6050_init_100hz(I2C_HandleTypeDef *hi2c, uint8_t addr7, mpu6050_cfg_t *out_cfg)
{
  if (!hi2c || !out_cfg) return MPU6050_ERR_PARAM;

  // 1) Validate identity
  uint8_t id = 0;
  if (mpu6050_whoami(hi2c, addr7, &id) != MPU6050_OK) return MPU6050_ERR_I2C;

  // MPU6050 WHO_AM_I is typically 0x68
  if (id != 0x68u) return MPU6050_ERR_ID;

  // 2) Wake up + set clock source (PWR_MGMT_1)
  //    0x01: SLEEP=0, CLKSEL=1 (PLL with gyro X)
  if (i2c_write_reg(hi2c, addr7, MPU6050_REG_PWR_MGMT_1, 0x01u, 50) != I2C_REG_OK)
    return MPU6050_ERR_I2C;

  HAL_Delay(10);

  // 3) Sample rate: with DLPF enabled, internal rate = 1kHz
  //    Output rate = 1000 / (1 + SMPLRT_DIV)
  //    For 100 Hz => SMPLRT_DIV = 9
  if (i2c_write_reg(hi2c, addr7, MPU6050_REG_SMPLRT_DIV, 9u, 50) != I2C_REG_OK)
    return MPU6050_ERR_I2C;

  // 4) DLPF (CONFIG): choose a safe moderate filter (DLPF_CFG=3)
  if (i2c_write_reg(hi2c, addr7, MPU6050_REG_CONFIG, 0x03u, 50) != I2C_REG_OK)
    return MPU6050_ERR_I2C;

  // 5) Gyro full-scale: ±250 dps (FS_SEL=0)
  if (i2c_write_reg(hi2c, addr7, MPU6050_REG_GYRO_CONFIG, 0x00u, 50) != I2C_REG_OK)
    return MPU6050_ERR_I2C;

  // 6) Accel full-scale: ±2g (AFS_SEL=0)
  if (i2c_write_reg(hi2c, addr7, MPU6050_REG_ACCEL_CONFIG, 0x00u, 50) != I2C_REG_OK)
    return MPU6050_ERR_I2C;

  // 7) Read back config for CLI "mpu cfg"
  out_cfg->addr7 = addr7;
  out_cfg->whoami = id;
  return mpu6050_read_cfg(hi2c, addr7, out_cfg);
}

mpu6050_status_t mpu6050_read_cfg(I2C_HandleTypeDef *hi2c, uint8_t addr7, mpu6050_cfg_t *cfg)
{
  if (!hi2c || !cfg) return MPU6050_ERR_PARAM;

  uint8_t v = 0;

  if (mpu6050_whoami(hi2c, addr7, &v) != MPU6050_OK) return MPU6050_ERR_I2C;
  cfg->whoami = v;

  if (i2c_read_reg(hi2c, addr7, MPU6050_REG_PWR_MGMT_1, &v, 1, 50) != I2C_REG_OK) return MPU6050_ERR_I2C;
  cfg->pwr_mgmt_1 = v;

  if (i2c_read_reg(hi2c, addr7, MPU6050_REG_SMPLRT_DIV, &v, 1, 50) != I2C_REG_OK) return MPU6050_ERR_I2C;
  cfg->smplrt_div = v;

  if (i2c_read_reg(hi2c, addr7, MPU6050_REG_CONFIG, &v, 1, 50) != I2C_REG_OK) return MPU6050_ERR_I2C;
  cfg->config = v;

  if (i2c_read_reg(hi2c, addr7, MPU6050_REG_GYRO_CONFIG, &v, 1, 50) != I2C_REG_OK) return MPU6050_ERR_I2C;
  cfg->gyro_config = v;

  if (i2c_read_reg(hi2c, addr7, MPU6050_REG_ACCEL_CONFIG, &v, 1, 50) != I2C_REG_OK) return MPU6050_ERR_I2C;
  cfg->accel_config = v;

  cfg->addr7 = addr7;
  return MPU6050_OK;
}

mpu6050_status_t mpu6050_read_raw(I2C_HandleTypeDef *hi2c, uint8_t addr7, mpu6050_raw_t *out_raw)
{
  if (!hi2c || !out_raw) return MPU6050_ERR_PARAM;

  uint8_t buf[14];
  if (i2c_read_reg(hi2c, addr7, MPU6050_REG_ACCEL_XOUT_H, buf, sizeof(buf), 50) != I2C_REG_OK)
    return MPU6050_ERR_I2C;

  out_raw->ax   = be16(&buf[0]);
  out_raw->ay   = be16(&buf[2]);
  out_raw->az   = be16(&buf[4]);
  out_raw->temp = be16(&buf[6]);
  out_raw->gx   = be16(&buf[8]);
  out_raw->gy   = be16(&buf[10]);
  out_raw->gz   = be16(&buf[12]);

  return MPU6050_OK;
}
