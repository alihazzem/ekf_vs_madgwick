#pragma once
#include "stm32f4xx_hal.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint8_t  g_emg_speed;
extern volatile bool     g_emg_speed_valid;
extern volatile uint32_t g_emg_last_rx_ms;

void emg_uart_init(UART_HandleTypeDef *huart);
void emg_uart_on_rx_event(UART_HandleTypeDef *huart, uint16_t size);
void emg_uart_recover(void);

#ifdef __cplusplus
}
#endif
