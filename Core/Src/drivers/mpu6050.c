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

mpu6050_status_t mpu6050_init_200hz(I2C_HandleTypeDef *hi2c, uint8_t addr7, mpu6050_cfg_t *out_cfg)
{
  if (!hi2c || !out_cfg) return MPU6050_ERR_PARAM;

  // 1) Validate identity
  uint8_t id = 0;
  if (mpu6050_whoami(hi2c, addr7, &id) != MPU6050_OK) return MPU6050_ERR_I2C;

  // MPU6050 WHO_AM_I is 0x68. MPU6500 is 0x70 or 0x71.
  if (id != 0x68u && id != 0x70u && id != 0x71u && id != 0x75u) return MPU6050_ERR_ID;

  // 2) Wake up + set clock source (PWR_MGMT_1)
  //    0x01: SLEEP=0, CLKSEL=1 (PLL with gyro X)
  if (i2c_write_reg(hi2c, addr7, MPU6050_REG_PWR_MGMT_1, 0x01u, 50) != I2C_REG_OK)
    return MPU6050_ERR_I2C;

  HAL_Delay(10);

  // 3) Sample rate: with DLPF enabled, internal gyro rate = 1 kHz
  //    Output rate = 1000 / (1 + SMPLRT_DIV)
  //    For 200 Hz => SMPLRT_DIV = 4
  if (i2c_write_reg(hi2c, addr7, MPU6050_REG_SMPLRT_DIV, 0x04u, 50) != I2C_REG_OK)
    return MPU6050_ERR_I2C;

  // 4) DLPF (CONFIG): 98 Hz gyro bandwidth (DLPF_CFG=2).
  //    CFG=3 (42 Hz) was chosen for 200 Hz; at 500 Hz a wider bandwidth
  //    preserves fast transients the EKF needs to track quick wrist motion.
  if (i2c_write_reg(hi2c, addr7, MPU6050_REG_CONFIG, 0x02u, 50) != I2C_REG_OK)
    return MPU6050_ERR_I2C;

  // 5) Gyro full-scale: ±1000 dps (FS_SEL=2, 32.8 LSB/dps)
  //    ±250 dps saturates at ~250 deg/s; a fast wrist roll can easily exceed
  //    this, clipping the gyro and permanently corrupting the quaternion.
  //    ±1000 dps is a practical upper bound for wrist/arm motion.
  if (i2c_write_reg(hi2c, addr7, MPU6050_REG_GYRO_CONFIG, 0x10u, 50) != I2C_REG_OK)
    return MPU6050_ERR_I2C;

  /* 3) Accel config: AFS_SEL=2 (±8g) -> 0x10 */
  if (i2c_write_reg(hi2c, addr7, MPU6050_REG_ACCEL_CONFIG, 0x10u, 50) != I2C_REG_OK)
    return MPU6050_ERR_I2C;

  // 7) Read back config for CLI "mpu cfg"
  out_cfg->addr7 = addr7;
  out_cfg->whoami = id;
  return mpu6050_read_cfg(hi2c, addr7, out_cfg);
}

/* Per-IMU custom initialisation — same sequence as init_200hz but driven by
 * the caller-supplied mpu6050_hw_cfg_t so each IMU can have its own range. */
mpu6050_status_t mpu6050_init_custom(I2C_HandleTypeDef *hi2c, uint8_t addr7,
                                     const mpu6050_hw_cfg_t *hw, mpu6050_cfg_t *out_cfg)
{
  if (!hi2c || !hw || !out_cfg) return MPU6050_ERR_PARAM;

  uint8_t id = 0;
  if (mpu6050_whoami(hi2c, addr7, &id) != MPU6050_OK) return MPU6050_ERR_I2C;
  if (id != 0x68u && id != 0x70u && id != 0x71u && id != 0x75u) return MPU6050_ERR_ID;

  if (i2c_write_reg(hi2c, addr7, MPU6050_REG_PWR_MGMT_1,  0x01u,       50) != I2C_REG_OK) return MPU6050_ERR_I2C;
  HAL_Delay(10);
  if (i2c_write_reg(hi2c, addr7, MPU6050_REG_SMPLRT_DIV,  hw->smplrt_div, 50) != I2C_REG_OK) return MPU6050_ERR_I2C;
  if (i2c_write_reg(hi2c, addr7, MPU6050_REG_CONFIG,       hw->dlpf_cfg,  50) != I2C_REG_OK) return MPU6050_ERR_I2C;
  if (i2c_write_reg(hi2c, addr7, MPU6050_REG_GYRO_CONFIG,  hw->gyro_fs,   50) != I2C_REG_OK) return MPU6050_ERR_I2C;
  if (i2c_write_reg(hi2c, addr7, MPU6050_REG_ACCEL_CONFIG, hw->accel_fs,  50) != I2C_REG_OK) return MPU6050_ERR_I2C;

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
