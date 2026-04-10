#include "drivers/ak8963.h"
#include "drivers/i2c_reg.h"

/* ----------------------------------------------------------------
 * ak8963_whoami
 * Read WIA register and return the raw ID value.
 * Caller checks against AK8963_WHOAMI_VAL (0x48).
 * ---------------------------------------------------------------- */
ak8963_status_t ak8963_whoami(I2C_HandleTypeDef *hi2c, uint8_t *out_id)
{
    if (!hi2c || !out_id) return AK8963_ERR_PARAM;

    uint8_t id = 0;
    if (i2c_read_reg(hi2c, AK8963_ADDR7, AK8963_REG_WIA, &id, 1, 50) != I2C_REG_OK)
        return AK8963_ERR_I2C;

    *out_id = id;
    return AK8963_OK;
}

/* ----------------------------------------------------------------
 * ak8963_read_asa
 * Read the three factory sensitivity adjustment (ASA) bytes from
 * fuse ROM and compute float correction coefficients:
 *   asa_x = (ASA_raw - 128) / 256.0 + 1.0
 *
 * Sequence: power-down → fuse-ROM mode → read ASA → power-down.
 * ---------------------------------------------------------------- */
ak8963_status_t ak8963_read_asa(I2C_HandleTypeDef *hi2c, ak8963_cfg_t *out_cfg)
{
    if (!hi2c || !out_cfg) return AK8963_ERR_PARAM;

    /* Validate identity first */
    uint8_t id = 0;
    if (ak8963_whoami(hi2c, &id) != AK8963_OK) return AK8963_ERR_I2C;
    if (id != AK8963_WHOAMI_VAL)                return AK8963_ERR_ID;

    /* Power-down to reset mode bits cleanly */
    if (i2c_write_reg(hi2c, AK8963_ADDR7, AK8963_REG_CNTL1, AK8963_MODE_POWERDOWN, 50) != I2C_REG_OK)
        return AK8963_ERR_I2C;
    HAL_Delay(10);

    /* Enter fuse ROM access mode */
    if (i2c_write_reg(hi2c, AK8963_ADDR7, AK8963_REG_CNTL1, AK8963_MODE_FUSE_ROM, 50) != I2C_REG_OK)
        return AK8963_ERR_I2C;
    HAL_Delay(10);

    /* Burst-read ASAX, ASAY, ASAZ */
    uint8_t asa[3] = {0, 0, 0};
    if (i2c_read_reg(hi2c, AK8963_ADDR7, AK8963_REG_ASAX, asa, 3, 50) != I2C_REG_OK)
        return AK8963_ERR_I2C;

    /* Compute correction factors */
    out_cfg->asa_x = ((float)(int8_t)(asa[0] - 128u)) / 256.0f + 1.0f;
    out_cfg->asa_y = ((float)(int8_t)(asa[1] - 128u)) / 256.0f + 1.0f;
    out_cfg->asa_z = ((float)(int8_t)(asa[2] - 128u)) / 256.0f + 1.0f;

    /* Return to power-down before switching to continuous mode */
    if (i2c_write_reg(hi2c, AK8963_ADDR7, AK8963_REG_CNTL1, AK8963_MODE_POWERDOWN, 50) != I2C_REG_OK)
        return AK8963_ERR_I2C;
    HAL_Delay(10);

    return AK8963_OK;
}

/* ----------------------------------------------------------------
 * ak8963_init_continuous_100hz
 * Set CNTL1 = 0x16: continuous measurement mode 2 (100 Hz), 16-bit.
 * Must be called AFTER ak8963_read_asa() (which leaves the device
 * in power-down mode ready for mode change).
 * ---------------------------------------------------------------- */
ak8963_status_t ak8963_init_continuous_100hz(I2C_HandleTypeDef *hi2c)
{
    if (!hi2c) return AK8963_ERR_PARAM;

    if (i2c_write_reg(hi2c, AK8963_ADDR7, AK8963_REG_CNTL1, AK8963_MODE_CONT2_16B, 50) != I2C_REG_OK)
        return AK8963_ERR_I2C;

    HAL_Delay(10); /* first measurement takes up to 10 ms */
    return AK8963_OK;
}

/* ----------------------------------------------------------------
 * ak8963_read_raw
 * Status-safe read: ST1 → HXL..HZH (6 bytes) → ST2.
 * Rules:
 *  - If DRDY=0 in ST1  → valid=0, return OK (sample not ready yet)
 *  - If HOFL=1 in ST2  → valid=0 (magnetic overflow)
 *  - Otherwise          → valid=1, fill mx/my/mz
 *
 * ST2 MUST be read after data registers to signal end-of-read to
 * the AK8963 so it starts the next measurement.
 * ---------------------------------------------------------------- */
ak8963_status_t ak8963_read_raw(I2C_HandleTypeDef *hi2c, ak8963_raw_t *out_raw)
{
    if (!hi2c || !out_raw) return AK8963_ERR_PARAM;

    out_raw->valid = 0;

    /* Check DRDY bit in ST1 */
    uint8_t st1 = 0;
    if (i2c_read_reg(hi2c, AK8963_ADDR7, AK8963_REG_ST1, &st1, 1, 50) != I2C_REG_OK)
        return AK8963_ERR_I2C;

    if (!(st1 & AK8963_ST1_DRDY))
    {
        /* Data not ready — not an error, caller should keep previous sample */
        return AK8963_OK;
    }

    /* Burst-read HXL..HZH (6 bytes, little-endian) */
    uint8_t buf[6];
    if (i2c_read_reg(hi2c, AK8963_ADDR7, AK8963_REG_HXL, buf, 6, 50) != I2C_REG_OK)
        return AK8963_ERR_I2C;

    /* MUST read ST2 to unlock next measurement */
    uint8_t st2 = 0;
    if (i2c_read_reg(hi2c, AK8963_ADDR7, AK8963_REG_ST2, &st2, 1, 50) != I2C_REG_OK)
        return AK8963_ERR_I2C;

    if (st2 & AK8963_ST2_HOFL)
    {
        /* Magnetic overflow — discard this sample */
        return AK8963_OK;
    }

    /* AK8963 is little-endian (opposite to accel/gyro) */
    out_raw->mx = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
    out_raw->my = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
    out_raw->mz = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);
    out_raw->valid = 1;

    return AK8963_OK;
}
