#pragma once
#include "stm32f4xx_hal.h"
#include <stdint.h>

#define MPU6050_ADDR7_DEFAULT      0x68u

// Registers
#define MPU6050_REG_SMPLRT_DIV     0x19u
#define MPU6050_REG_CONFIG         0x1Au
#define MPU6050_REG_GYRO_CONFIG    0x1Bu
#define MPU6050_REG_ACCEL_CONFIG   0x1Cu
#define MPU6050_REG_WHO_AM_I       0x75u
#define MPU6050_REG_PWR_MGMT_1     0x6Bu
#define MPU6050_REG_ACCEL_XOUT_H   0x3Bu

typedef struct {
  int16_t ax, ay, az;
  int16_t temp;
  int16_t gx, gy, gz;
} mpu6050_raw_t;

typedef struct {
  uint8_t addr7;
  uint8_t whoami;
  uint8_t pwr_mgmt_1;
  uint8_t smplrt_div;
  uint8_t config;
  uint8_t gyro_config;
  uint8_t accel_config;
} mpu6050_cfg_t;

typedef enum {
  MPU6050_OK = 0,
  MPU6050_ERR_I2C,
  MPU6050_ERR_ID,
  MPU6050_ERR_PARAM
} mpu6050_status_t;

mpu6050_status_t mpu6050_whoami(I2C_HandleTypeDef *hi2c, uint8_t addr7, uint8_t *out_id);
mpu6050_status_t mpu6050_init_200hz(I2C_HandleTypeDef *hi2c, uint8_t addr7, mpu6050_cfg_t *out_cfg);
mpu6050_status_t mpu6050_read_cfg(I2C_HandleTypeDef *hi2c, uint8_t addr7, mpu6050_cfg_t *cfg);
mpu6050_status_t mpu6050_read_raw(I2C_HandleTypeDef *hi2c, uint8_t addr7, mpu6050_raw_t *out_raw);

/* Per-IMU hardware configuration for mpu6050_init_custom() */
typedef struct {
    uint8_t smplrt_div;   /* 0x04 = 200Hz when DLPF enabled */
    uint8_t dlpf_cfg;     /* CONFIG register: 0x02=98Hz BW, 0x03=42Hz BW */
    uint8_t gyro_fs;      /* GYRO_CONFIG register: 0x00=250dps, 0x08=500dps, 0x10=1000dps */
    uint8_t accel_fs;     /* ACCEL_CONFIG register: 0x00=2g, 0x08=4g, 0x10=8g */
    float   gyro_lsb_per_dps; /* Scale: 131.0=250dps, 65.5=500dps, 32.8=1000dps */
    float   acc_lsb_per_g;   /* Scale: 16384=2g, 8192=4g, 4096=8g */
} mpu6050_hw_cfg_t;

mpu6050_status_t mpu6050_init_custom(I2C_HandleTypeDef *hi2c, uint8_t addr7,
                                     const mpu6050_hw_cfg_t *hw, mpu6050_cfg_t *out_cfg);
