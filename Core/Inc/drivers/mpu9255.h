#pragma once
#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ----------------------------------------------------------------
 * MPU-9255 / MPU-9250 driver
 * Register map is fully compatible with MPU-6050.
 * Only WHO_AM_I differs: MPU-9255 = 0x73, MPU-9250 = 0x71.
 * Both values are accepted so the same firmware runs on either.
 * BMP280 (also present on the GY-91 board) is intentionally unused.
 * ---------------------------------------------------------------- */

#define MPU9255_ADDR7_DEFAULT     0x68u  /* SAO/SDO low  */
#define MPU9255_ADDR7_HIGH        0x69u  /* SAO/SDO high */

#define MPU9255_WHOAMI_VAL        0x73u  /* MPU-9255 */
#define MPU9250_WHOAMI_VAL        0x71u  /* MPU-9250 (also accepted) */

/* Registers (identical to MPU-6050) */
#define MPU9255_REG_SMPLRT_DIV    0x19u
#define MPU9255_REG_CONFIG        0x1Au
#define MPU9255_REG_GYRO_CONFIG   0x1Bu
#define MPU9255_REG_ACCEL_CONFIG  0x1Cu
#define MPU9255_REG_INT_PIN_CFG   0x37u  /* BYPASS_EN lives here */
#define MPU9255_REG_USER_CTRL     0x6Au  /* I2C_MST_EN lives here */
#define MPU9255_REG_PWR_MGMT_1   0x6Bu
#define MPU9255_REG_WHO_AM_I      0x75u
#define MPU9255_REG_ACCEL_XOUT_H  0x3Bu

/* INT_PIN_CFG bits */
#define MPU9255_BYPASS_EN         (1u << 1)

/* USER_CTRL bits */
#define MPU9255_I2C_MST_EN        (1u << 5)

/* ── Raw burst-read output (14 bytes starting at ACCEL_XOUT_H) ── */
typedef struct {
    int16_t ax, ay, az;   /* accel  raw counts */
    int16_t temp;          /* temperature raw  */
    int16_t gx, gy, gz;   /* gyro   raw counts */
} mpu9255_raw_t;

/* ── Config snapshot (for MPU CFG CLI command) ── */
typedef struct {
    uint8_t addr7;
    uint8_t whoami;
    uint8_t pwr_mgmt_1;
    uint8_t smplrt_div;
    uint8_t config;
    uint8_t gyro_config;
    uint8_t accel_config;
} mpu9255_cfg_t;

/* ── Return codes ── */
typedef enum {
    MPU9255_OK = 0,
    MPU9255_ERR_I2C,
    MPU9255_ERR_ID,
    MPU9255_ERR_PARAM
} mpu9255_status_t;

/* ── API ── */
mpu9255_status_t mpu9255_whoami(I2C_HandleTypeDef *hi2c, uint8_t addr7, uint8_t *out_id);
mpu9255_status_t mpu9255_init_200hz(I2C_HandleTypeDef *hi2c, uint8_t addr7, mpu9255_cfg_t *out_cfg);
mpu9255_status_t mpu9255_read_cfg(I2C_HandleTypeDef *hi2c, uint8_t addr7, mpu9255_cfg_t *cfg);
mpu9255_status_t mpu9255_read_raw(I2C_HandleTypeDef *hi2c, uint8_t addr7, mpu9255_raw_t *out_raw);
mpu9255_status_t mpu9255_enable_bypass(I2C_HandleTypeDef *hi2c, uint8_t addr7);
