#ifndef DRIVERS_SSD1306_H
#define DRIVERS_SSD1306_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ── Hardware constants ───────────────────────────────────────────────────── */
#define SSD1306_I2C_ADDR   0x78U   /* 7-bit addr 0x3C → 8-bit write = 0x78 */
#define SSD1306_WIDTH      128U
#define SSD1306_PAGES      8U      /* 64 pixels / 8 bits per page */
#define SSD1306_BUF_SIZE   (SSD1306_WIDTH * SSD1306_PAGES)

/* ── Public API ───────────────────────────────────────────────────────────── */

/**
 * @brief Initialize the SSD1306 with a standard startup command sequence.
 * @param hi2c  Pointer to the I2C handle (must be hi2c2 for this project).
 * @return HAL_OK on success.
 */
HAL_StatusTypeDef ssd1306_init(I2C_HandleTypeDef *hi2c);

/** @brief Clear the RAM framebuffer (does NOT push to display). */
void ssd1306_clear(void);

/**
 * @brief Push the entire framebuffer to the display via I2C.
 * @return HAL_OK on success.
 */
HAL_StatusTypeDef ssd1306_flush(I2C_HandleTypeDef *hi2c);

/**
 * @brief Draw an ASCII string at a character grid position.
 * @param col   Character column (0 = left edge, max 20 for 6-wide glyphs).
 * @param page  Page row (0 = top, max 7).
 * @param str   Null-terminated ASCII string.
 */
void ssd1306_puts(uint8_t col, uint8_t page, const char *str);

#endif /* DRIVERS_SSD1306_H */
