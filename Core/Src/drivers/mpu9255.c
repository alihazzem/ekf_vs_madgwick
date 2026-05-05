#include "drivers/mpu9255.h"
#include "drivers/i2c_reg.h"
#include "app/app_config.h"

static int16_t be16(const uint8_t *p)
{
    return (int16_t)(((int16_t)p[0] << 8) | p[1]);
}

static int mpu92xx_id_matches(uint8_t id)
{
#if MPU92XX_VARIANT == MPU92XX_VARIANT_9255
    return (id == MPU9255_WHOAMI_VAL);
#elif MPU92XX_VARIANT == MPU92XX_VARIANT_9250
    return (id == MPU9250_WHOAMI_VAL);
#else
    return (id == MPU9255_WHOAMI_VAL || id == MPU9250_WHOAMI_VAL);
#endif
}

/* ----------------------------------------------------------------
 * mpu9255_whoami
 * Read WHO_AM_I register.  Does NOT validate the value — caller
 * decides what to accept (0x73 for MPU-9255, 0x71 for MPU-9250).
 * ---------------------------------------------------------------- */
mpu9255_status_t mpu9255_whoami(I2C_HandleTypeDef *hi2c, uint8_t addr7, uint8_t *out_id)
{
    if (!hi2c || !out_id)
        return MPU9255_ERR_PARAM;

    uint8_t id = 0;
    if (i2c_read_reg(hi2c, addr7, MPU9255_REG_WHO_AM_I, &id, 1, 50) != I2C_REG_OK)
        return MPU9255_ERR_I2C;

    *out_id = id;
    return MPU9255_OK;
}

/* ----------------------------------------------------------------
 * mpu9255_init_200hz
 * Configure accel ±2 g, gyro ±250 dps, DLPF=3, output @ 200 Hz.
 * Accepts WHO_AM_I based on MPU92XX_VARIANT in app_config.h.
 * ---------------------------------------------------------------- */
mpu9255_status_t mpu9255_init_200hz(I2C_HandleTypeDef *hi2c, uint8_t addr7, mpu9255_cfg_t *out_cfg)
{
    if (!hi2c || !out_cfg)
        return MPU9255_ERR_PARAM;

    /* 1) Validate identity */
    uint8_t id = 0;
    if (mpu9255_whoami(hi2c, addr7, &id) != MPU9255_OK)
        return MPU9255_ERR_I2C;

    if (!mpu92xx_id_matches(id))
        return MPU9255_ERR_ID;

    /* 2) Wake up + select PLL clock (PWR_MGMT_1 = 0x01) */
    if (i2c_write_reg(hi2c, addr7, MPU9255_REG_PWR_MGMT_1, 0x01u, 50) != I2C_REG_OK)
        return MPU9255_ERR_I2C;
    HAL_Delay(10);

    /* 3) Sample rate: 1 kHz / (1 + SMPLRT_DIV) = 200 Hz → DIV = 4 */
    if (i2c_write_reg(hi2c, addr7, MPU9255_REG_SMPLRT_DIV, 4u, 50) != I2C_REG_OK)
        return MPU9255_ERR_I2C;

    /* 4) DLPF_CFG = 3 (accel BW ~44 Hz, gyro BW ~42 Hz) */
    if (i2c_write_reg(hi2c, addr7, MPU9255_REG_CONFIG, 0x03u, 50) != I2C_REG_OK)
        return MPU9255_ERR_I2C;

    /* 5) Gyro full-scale ±250 dps (FS_SEL = 0) */
    if (i2c_write_reg(hi2c, addr7, MPU9255_REG_GYRO_CONFIG, 0x00u, 50) != I2C_REG_OK)
        return MPU9255_ERR_I2C;

    /* 6) Accel full-scale ±2 g (AFS_SEL = 0) */
    if (i2c_write_reg(hi2c, addr7, MPU9255_REG_ACCEL_CONFIG, 0x00u, 50) != I2C_REG_OK)
        return MPU9255_ERR_I2C;

    /* 7) Read back config for CLI display */
    out_cfg->addr7 = addr7;
    out_cfg->whoami = id;
    return mpu9255_read_cfg(hi2c, addr7, out_cfg);
}

/* ----------------------------------------------------------------
 * mpu9255_read_cfg
 * Read back current register values.
 * ---------------------------------------------------------------- */
mpu9255_status_t mpu9255_read_cfg(I2C_HandleTypeDef *hi2c, uint8_t addr7, mpu9255_cfg_t *cfg)
{
    if (!hi2c || !cfg)
        return MPU9255_ERR_PARAM;

    uint8_t v = 0;

    if (mpu9255_whoami(hi2c, addr7, &v) != MPU9255_OK)
        return MPU9255_ERR_I2C;
    cfg->whoami = v;

    if (i2c_read_reg(hi2c, addr7, MPU9255_REG_PWR_MGMT_1, &v, 1, 50) != I2C_REG_OK)
        return MPU9255_ERR_I2C;
    cfg->pwr_mgmt_1 = v;

    if (i2c_read_reg(hi2c, addr7, MPU9255_REG_SMPLRT_DIV, &v, 1, 50) != I2C_REG_OK)
        return MPU9255_ERR_I2C;
    cfg->smplrt_div = v;

    if (i2c_read_reg(hi2c, addr7, MPU9255_REG_CONFIG, &v, 1, 50) != I2C_REG_OK)
        return MPU9255_ERR_I2C;
    cfg->config = v;

    if (i2c_read_reg(hi2c, addr7, MPU9255_REG_GYRO_CONFIG, &v, 1, 50) != I2C_REG_OK)
        return MPU9255_ERR_I2C;
    cfg->gyro_config = v;

    if (i2c_read_reg(hi2c, addr7, MPU9255_REG_ACCEL_CONFIG, &v, 1, 50) != I2C_REG_OK)
        return MPU9255_ERR_I2C;
    cfg->accel_config = v;

    cfg->addr7 = addr7;
    return MPU9255_OK;
}

/* ----------------------------------------------------------------
 * mpu9255_read_raw
 * Burst-read 14 bytes: ACCEL_XOUT_H → GYRO_ZOUT_L.
 * Same register layout as MPU-6050.
 * ---------------------------------------------------------------- */
mpu9255_status_t mpu9255_read_raw(I2C_HandleTypeDef *hi2c, uint8_t addr7, mpu9255_raw_t *out_raw)
{
    if (!hi2c || !out_raw)
        return MPU9255_ERR_PARAM;

    uint8_t buf[14];
    if (i2c_read_reg(hi2c, addr7, MPU9255_REG_ACCEL_XOUT_H, buf, sizeof(buf), 50) != I2C_REG_OK)
        return MPU9255_ERR_I2C;

    out_raw->ax = be16(&buf[0]);
    out_raw->ay = be16(&buf[2]);
    out_raw->az = be16(&buf[4]);
    out_raw->temp = be16(&buf[6]);
    out_raw->gx = be16(&buf[8]);
    out_raw->gy = be16(&buf[10]);
    out_raw->gz = be16(&buf[12]);

    return MPU9255_OK;
}

/* ----------------------------------------------------------------
 * mpu9255_enable_bypass
 * Disable internal I2C master and enable bypass mode so the STM32
 * can talk directly to the AK8963 on the same I2C bus.
 *
 * USER_CTRL  [5] I2C_MST_EN = 0
 * INT_PIN_CFG[1] BYPASS_EN  = 1
 * ---------------------------------------------------------------- */
mpu9255_status_t mpu9255_enable_bypass(I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
    if (!hi2c)
        return MPU9255_ERR_PARAM;

    uint8_t v = 0;

    /* Clear I2C_MST_EN in USER_CTRL */
    if (i2c_read_reg(hi2c, addr7, MPU9255_REG_USER_CTRL, &v, 1, 50) != I2C_REG_OK)
        return MPU9255_ERR_I2C;
    v &= (uint8_t)(~MPU9255_I2C_MST_EN);
    if (i2c_write_reg(hi2c, addr7, MPU9255_REG_USER_CTRL, v, 50) != I2C_REG_OK)
        return MPU9255_ERR_I2C;

    /* Set BYPASS_EN in INT_PIN_CFG */
    if (i2c_read_reg(hi2c, addr7, MPU9255_REG_INT_PIN_CFG, &v, 1, 50) != I2C_REG_OK)
        return MPU9255_ERR_I2C;
    v |= MPU9255_BYPASS_EN;
    if (i2c_write_reg(hi2c, addr7, MPU9255_REG_INT_PIN_CFG, v, 50) != I2C_REG_OK)
        return MPU9255_ERR_I2C;

    HAL_Delay(5); /* let the bus stabilise */
    return MPU9255_OK;
}
