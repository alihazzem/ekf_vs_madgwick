#ifndef APP_FLASH_STORAGE_H
#define APP_FLASH_STORAGE_H

#include <stdint.h>
#include <stdbool.h>
#include "app/app_config.h"

#define FLASH_CALIB_MAGIC 0xDEADBEEF
#define NUM_IMUS_MAX 4

typedef struct {
    uint32_t magic;
    int16_t gx_off[NUM_IMUS_MAX];
    int16_t gy_off[NUM_IMUS_MAX];
    int16_t gz_off[NUM_IMUS_MAX];
    int16_t ax_off[NUM_IMUS_MAX];
    int16_t ay_off[NUM_IMUS_MAX];
    int16_t az_off[NUM_IMUS_MAX];
} FlashCalibData_t;

#ifdef __cplusplus
extern "C" {
#endif

bool flash_storage_read(FlashCalibData_t *data);
bool flash_storage_write(const FlashCalibData_t *data);

#ifdef __cplusplus
}
#endif

#endif // APP_FLASH_STORAGE_H
