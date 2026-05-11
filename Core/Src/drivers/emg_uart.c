#include "drivers/emg_uart.h"
#include <string.h>

volatile uint8_t  g_emg_speed       = 0;
volatile bool     g_emg_speed_valid  = false;
volatile uint32_t g_emg_last_rx_ms  = 0;

#define EMG_RX_BUF_SIZE 128
static uint8_t s_rx_buf[EMG_RX_BUF_SIZE];

void emg_uart_init(UART_HandleTypeDef *huart)
{
    g_emg_speed       = 0;
    g_emg_speed_valid  = false;
    g_emg_last_rx_ms  = 0;

    memset(s_rx_buf, 0, sizeof(s_rx_buf));

    HAL_UARTEx_ReceiveToIdle_DMA(huart, s_rx_buf, EMG_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

void emg_uart_on_rx_event(UART_HandleTypeDef *huart, uint16_t size)
{
    for (uint16_t i = 0; i + 2 < size; i++)
    {
        if (s_rx_buf[i] == 0xAA)
        {
            uint8_t speed    = s_rx_buf[i + 1];
            uint8_t checksum = s_rx_buf[i + 2];

            if (checksum == (uint8_t)(0xAA ^ speed))
            {
                g_emg_speed       = speed;
                g_emg_speed_valid  = true;
                g_emg_last_rx_ms  = HAL_GetTick();
                i += 2;
            }
        }
    }

    HAL_UARTEx_ReceiveToIdle_DMA(huart, s_rx_buf, EMG_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}
