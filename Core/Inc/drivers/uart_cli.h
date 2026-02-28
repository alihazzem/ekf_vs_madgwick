#pragma once
#include "stm32f4xx_hal.h"

void uart_cli_init(UART_HandleTypeDef *huart);
void uart_cli_poll(void);

void uart_cli_send(const char *s);
void uart_cli_sendf(const char *fmt, ...);

// call from HAL_UART_RxCpltCallback()
void uart_cli_on_rx_byte(UART_HandleTypeDef *huart);
