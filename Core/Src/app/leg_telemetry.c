#include "app/leg_telemetry.h"

#if ROBOT_MODE == ROBOT_MODE_LEG || ROBOT_MODE == ROBOT_MODE_FULL

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ── UART handle (internal to this module) ──────────────────────────────── */
static UART_HandleTypeDef s_huart6;

/* ── Packed packet struct ────────────────────────────────────────────────── */
#pragma pack(push, 1)
typedef struct {
    uint8_t  header1;            /* 0xAA */
    uint8_t  header2;            /* 0x55 */
    int16_t  thigh_L_deg100;
    int16_t  shin_L_deg100;
    int16_t  knee_L_deg100;
    int16_t  thigh_R_deg100;
    int16_t  shin_R_deg100;
    int16_t  knee_R_deg100;
    uint8_t  status;             /* 0=OK, 1=Lost Connection */
    uint8_t  checksum;
} EspTelemetry_t;
#pragma pack(pop)

/* ── Init ────────────────────────────────────────────────────────────────── */
static EspTelemetry_t s_pkt;
DMA_HandleTypeDef hdma_usart6_tx;

void leg_telemetry_init(void)
{
    __HAL_RCC_USART6_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    /* PA11 → USART6_TX (AF8), push-pull, no pull, very-high speed */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_11;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOA, &gpio);

    s_huart6.Instance          = USART6;
    s_huart6.Init.BaudRate     = 115200;
    s_huart6.Init.WordLength   = UART_WORDLENGTH_8B;
    s_huart6.Init.StopBits     = UART_STOPBITS_1;
    s_huart6.Init.Parity       = UART_PARITY_NONE;
    s_huart6.Init.Mode         = UART_MODE_TX;
    s_huart6.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    s_huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&s_huart6);

    /* Configure DMA for USART6 TX (DMA2, Stream 6, Channel 5) */
    hdma_usart6_tx.Instance = DMA2_Stream6;
    hdma_usart6_tx.Init.Channel = DMA_CHANNEL_5;
    hdma_usart6_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart6_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart6_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart6_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart6_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart6_tx.Init.Mode = DMA_NORMAL;
    hdma_usart6_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart6_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_usart6_tx);

    __HAL_LINKDMA(&s_huart6, hdmatx, hdma_usart6_tx);

    HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0); 
    HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
}

/* ── Send ────────────────────────────────────────────────────────────────── */
void leg_telemetry_send(float thigh_L, float shin_L, float knee_L,
                        float thigh_R, float shin_R, float knee_R)
{
    if (s_huart6.gState != HAL_UART_STATE_READY) return;

    s_pkt.header1        = 0xAA;
    s_pkt.header2        = 0x55;
    s_pkt.thigh_L_deg100 = (int16_t)(thigh_L * 100.0f);
    s_pkt.shin_L_deg100  = (int16_t)(shin_L  * 100.0f);
    s_pkt.knee_L_deg100  = (int16_t)(knee_L  * 100.0f);
    s_pkt.thigh_R_deg100 = (int16_t)(thigh_R * 100.0f);
    s_pkt.shin_R_deg100  = (int16_t)(shin_R  * 100.0f);
    s_pkt.knee_R_deg100  = (int16_t)(knee_R  * 100.0f);
    s_pkt.status         = 0; /* OK */

    uint8_t chk = s_pkt.header1 + s_pkt.header2;
    chk += (uint8_t)(s_pkt.thigh_L_deg100 & 0xFF) + (uint8_t)(s_pkt.thigh_L_deg100 >> 8);
    chk += (uint8_t)(s_pkt.shin_L_deg100  & 0xFF) + (uint8_t)(s_pkt.shin_L_deg100  >> 8);
    chk += (uint8_t)(s_pkt.knee_L_deg100  & 0xFF) + (uint8_t)(s_pkt.knee_L_deg100  >> 8);
    chk += (uint8_t)(s_pkt.thigh_R_deg100 & 0xFF) + (uint8_t)(s_pkt.thigh_R_deg100 >> 8);
    chk += (uint8_t)(s_pkt.shin_R_deg100  & 0xFF) + (uint8_t)(s_pkt.shin_R_deg100  >> 8);
    chk += (uint8_t)(s_pkt.knee_R_deg100  & 0xFF) + (uint8_t)(s_pkt.knee_R_deg100  >> 8);
    chk += s_pkt.status;
    s_pkt.checksum = chk;

    HAL_UART_Transmit_DMA(&s_huart6, (uint8_t *)&s_pkt, sizeof(s_pkt));
}

#endif /* ROBOT_MODE_LEG || ROBOT_MODE_FULL */
