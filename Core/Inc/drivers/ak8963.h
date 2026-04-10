#pragma once
#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ----------------------------------------------------------------
 * AK8963 magnetometer driver
 * The AK8963 die is embedded inside the MPU-9255 package.
 * It is accessed via the auxiliary I2C bus exposed through the
 * MPU-9255 bypass mode (INT_PIN_CFG.BYPASS_EN = 1).
 *
 * I2C address: 0x0C (fixed, not configurable).
 * ---------------------------------------------------------------- */

#define AK8963_ADDR7          0x0Cu

/* AK8963 registers */
#define AK8963_REG_WIA        0x00u  /* Device ID — expect 0x48 */
#define AK8963_REG_ST1        0x02u  /* Status 1  (DRDY, DOR)   */
#define AK8963_REG_HXL        0x03u  /* Mag X LSB (start of 6-byte burst) */
#define AK8963_REG_ST2        0x09u  /* Status 2  (HOFL)        */
#define AK8963_REG_CNTL1      0x0Au  /* Control 1 (mode + bits) */
#define AK8963_REG_CNTL2      0x0Bu  /* Control 2 (SRST soft reset) */
#define AK8963_REG_ASAX       0x10u  /* Sensitivity adj X (fuse ROM) */
#define AK8963_REG_ASAY       0x11u
#define AK8963_REG_ASAZ       0x12u

#define AK8963_WHOAMI_VAL     0x48u

/* CNTL1 values */
#define AK8963_MODE_POWERDOWN 0x00u
#define AK8963_MODE_FUSE_ROM  0x0Fu
#define AK8963_MODE_CONT2_16B 0x16u  /* Continuous mode 2, 16-bit: 100 Hz */

/* ST1 bits */
#define AK8963_ST1_DRDY       (1u << 0)
#define AK8963_ST1_DOR        (1u << 1)

/* ST2 bits */
#define AK8963_ST2_HOFL       (1u << 3)

/* ── Sensitivity adjustment (ASA) floats, computed once at init ── */
typedef struct {
    float asa_x;   /* ((ASA_raw - 128) / 256) + 1   */
    float asa_y;
    float asa_z;
} ak8963_cfg_t;

/* ── Raw magnetometer sample ── */
typedef struct {
    int16_t mx, my, mz;   /* raw 16-bit counts (before ASA correction) */
    uint8_t valid;         /* 1 = DRDY and no overflow, 0 = discard     */
} ak8963_raw_t;

/* ── Return codes ── */
typedef enum {
    AK8963_OK = 0,
    AK8963_ERR_I2C,
    AK8963_ERR_ID,
    AK8963_ERR_PARAM
} ak8963_status_t;

/* ── API ── */
ak8963_status_t ak8963_whoami(I2C_HandleTypeDef *hi2c, uint8_t *out_id);
ak8963_status_t ak8963_read_asa(I2C_HandleTypeDef *hi2c, ak8963_cfg_t *out_cfg);
ak8963_status_t ak8963_init_continuous_100hz(I2C_HandleTypeDef *hi2c);
ak8963_status_t ak8963_read_raw(I2C_HandleTypeDef *hi2c, ak8963_raw_t *out_raw);
