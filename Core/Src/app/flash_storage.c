#include "app/flash_storage.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* Sector 7 starts at 0x08060000 on STM32F411 (128 KB) */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000)

bool flash_storage_read(FlashCalibData_t *data) {
    if (!data) return false;
    
    FlashCalibData_t *flash_data = (FlashCalibData_t *)ADDR_FLASH_SECTOR_7;
    
    if (flash_data->magic == FLASH_CALIB_MAGIC) {
        memcpy(data, flash_data, sizeof(FlashCalibData_t));
        return true;
    }
    
    return false;
}

bool flash_storage_write(const FlashCalibData_t *data) {
    if (!data) return false;

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SectorError = 0;

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    EraseInitStruct.Sector = FLASH_SECTOR_7;
    EraseInitStruct.NbSectors = 1;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    uint32_t *pSource = (uint32_t *)data;
    uint32_t address = ADDR_FLASH_SECTOR_7;
    uint32_t words = (sizeof(FlashCalibData_t) + 3) / 4;

    for (uint32_t i = 0; i < words; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, pSource[i]) == HAL_OK) {
            address += 4;
        } else {
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();
    return true;
}
