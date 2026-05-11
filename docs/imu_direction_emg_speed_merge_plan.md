# IMU Direction + EMG Speed Motor Control — Merge Plan

## 1. Motivation

Currently the STM32 controls both motor speed AND direction from the IMU (MPU-9255 via EKF/Madgwick attitude). The ESP32 already acquires 8-channel EMG from the Myo armband, processes it (HP filter + RMS envelope), and sends quantised speed commands over UART — but the STM32 ignores them.

This plan merges the two control channels:
- **IMU Roll → Steering direction** (differential drive — unchanged)
- **EMG RMS → Motor speed** (replaces IMU pitch → speed mapping)
- **IMU Pitch → Forward/Backward direction** only (no more speed from pitch)
- **UART reception uses DMA** so it doesn't load the CPU

---

## 2. Current Architecture

```
MPU-9255 →(I2C)→ STM32 → EKF q_delta → Pitch→Speed + Roll→Steering → MotorCmd

ESP32 →(BLE)→ Myo → EMG HP+RMS → UART2 → 3-byte packet → STM32 (ignored)
```

**Problems:**
1. Both speed and direction come from the IMU — no EMG influence
2. EMG packets from the ESP32 arrive on UART but are discarded (non-printable bytes, filtered by ASCII CLI parser)
3. UART RX uses per-byte interrupts — these pile up at 40 Hz × 3 bytes = 120 ISRs/s from the EMG stream alone

---

## 3. Target Architecture

```
MPU-9255 →(I2C)→ STM32 → EKF q_delta → Roll→Steering
                                                          ↘
ESP32 →(BLE)→ Myo → EMG HP+RMS → UART2 → STM32 USART1 DMA → EMG Speed → MotorCmd → L298N
```

- USART2 (PA2/PA3) stays dedicated to PC CLI + CSV streaming
- USART1 (PA10 RX-only) is the dedicated EMG link from the ESP32
- DMA (no CPU involvement during byte reception) with idle-line detection

---

## 4. ESP32 → STM32 EMG Protocol (unchanged)

Already implemented in `esp32_myo/esp32_myo.ino`:

```
Byte 0:  0xAA               (sync header)
Byte 1:  speed              (0=stop, 33=slow, 66=medium, 100=fast)
Byte 2:  XOR checksum       (0xAA ^ speed)
```

- 921600 baud, 8N1
- Sent every ~25 ms (40 Hz) from ESP32 UART2 (GPIO17 TX)
- Speed values use 10% hysteresis in the ESP32 to prevent rapid toggling

**No changes needed on the ESP32 side.**

---

## 5. STM32 DMA Architecture

```
ESP32 → USART1_RX → DMA2 Stream2 Ch4 → s_rx_buf[128]
                                              ↓ (IDLE line detected)
                                         HAL_UARTEx_RxEventCallback
                                              ↓
                                         emg_uart_on_rx_event()
                                         ├─ Scan for 0xAA sync markers
                                         ├─ Validate XOR checksum
                                         ├─ Update g_emg_speed
                                         ├─ Update g_emg_last_rx_ms
                                         └─ Re-arm HAL_UARTEx_ReceiveToIdle_DMA
```

- **DMA mode**: Normal (not circular) — re-armed after each idle-line callback
- **Idle-line detection**: fires when no byte received for > 1 byte time (~87 µs at 921600 baud)
- **Buffer size**: 128 bytes (handles multiple 3-byte packets if they cluster)
- **CPU load**: 1 interrupt ~40 Hz instead of 120 ISRs/s with per-byte mode

---

## 6. Files to Create

| File | Purpose |
|------|---------|
| `Core/Inc/drivers/emg_uart.h` | EMG UART DMA driver header |
| `Core/Src/drivers/emg_uart.c` | EMG UART DMA driver implementation |

---

## 7. Files to Modify

| File | Changes |
|------|---------|
| `Core/Src/main.c` | USART1+DMA init, RX event callback, modify `imu_task_fn` motor logic |
| `Core/Inc/main.h` | Add `extern huart1`, `extern hdma_usart1_rx` |
| `Core/Inc/motor_test.h` | Add `emg_speed` field to `MotorCmd_t` |
| `Core/Src/stm32f4xx_hal_msp.c` | Add `HAL_UART_MspInit` case for USART1 with DMA |
| `Core/Src/stm32f4xx_it.c` | Add `USART1_IRQHandler` and `DMA2_Stream2_IRQHandler` |
| `Core/Inc/stm32f4xx_it.h` | Add prototypes for new IRQ handlers |
| `Core/Inc/app/app_config.h` | Add `EMG_SPEED_TIMEOUT_MS` config |

---

## 8. Detailed Changes

### 8.1 NEW: `Core/Inc/drivers/emg_uart.h`

```c
#pragma once
#include "stm32f4xx_hal.h"
#include <stdbool.h>

// Globals read by the motor control loop (set from ISR context)
extern volatile uint8_t  g_emg_speed;        // 0, 33, 66, 100 from ESP32
extern volatile bool     g_emg_speed_valid;   // true if at least one valid packet received
extern volatile uint32_t g_emg_last_rx_ms;    // HAL_GetTick() of last valid packet

void emg_uart_init(UART_HandleTypeDef *huart);
void emg_uart_on_rx_event(UART_HandleTypeDef *huart, uint16_t size);
```

**Key design decisions:**
- Speed values are the raw ESP32 quantized values (0/33/66/100) — mapped to PWM duty in the motor control layer, not here
- `g_emg_last_rx_ms` enables the watchdog timeout (stop motors if EMG data stops flowing)
- `volatile` globals since written from ISR context and read from task context
- Single-byte reads/writes are atomic on Cortex-M4 (no need for critical sections)

### 8.2 NEW: `Core/Src/drivers/emg_uart.c`

```c
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

    // Start DMA reception with idle-line detection
    HAL_UARTEx_ReceiveToIdle_DMA(huart, s_rx_buf, EMG_RX_BUF_SIZE);

    // Disable half-transfer interrupt — only complete + idle are needed
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}

void emg_uart_on_rx_event(UART_HandleTypeDef *huart, uint16_t size)
{
    // Scan received bytes for EMG packets: 0xAA, speed, XOR_checksum
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
                i += 2;  // skip past validated packet
            }
            // If checksum fails, fall through — the next byte
            // might be a real 0xAA. No increment so we re-test.
        }
    }

    // Re-arm DMA for next reception
    HAL_UARTEx_ReceiveToIdle_DMA(huart, s_rx_buf, EMG_RX_BUF_SIZE);
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
}
```

### 8.3 MODIFY: `Core/Inc/motor_test.h`

Add `emg_speed` field to `MotorCmd_t`:

```c
typedef struct {
    MotorDir_t dirA;
    MotorDir_t dirB;
    uint16_t   speedA;
    uint16_t   speedB;
    uint8_t    emg_speed;   // raw EMG speed for telemetry/logging (0,33,66,100)
} MotorCmd_t;
```

### 8.4 MODIFY: `Core/Inc/main.h`

Add after existing externs:

```c
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
```

### 8.5 MODIFY: `Core/Src/main.c`

**8.5a — Add includes:**
```c
#include "drivers/emg_uart.h"
```

**8.5b — Add global handles:**
```c
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
```

**8.5c — Add `MX_USART1_UART_Init()`:**
```c
static void MX_USART1_UART_Init(void)
{
    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 921600;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_RX;   // RX-only, no TX to ESP32
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
        Error_Handler();
}
```

Call this in `main()` alongside other `MX_*_Init()` calls, **before** `osKernelInitialize()`.

**8.5d — Add `HAL_UARTEx_RxEventCallback()`:**

This callback is invoked by HAL when a DMA transfer completes (either buffer full or idle line detected):

```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        emg_uart_on_rx_event(huart, Size);
    }
}
```

**8.5e — Modify `imu_task_fn()` startup:**

After existing `imu_app_cal_gyro(5000)` call, add:

```c
emg_uart_init(&huart1);
```

**8.5f — Modify motor control logic in `imu_task_fn()` (replaces lines 512–543):**

Replace the Pitch→Speed block with EMG-based speed:

```c
/* ── EMG → Speed ── */
uint16_t   speed   = 0;
uint8_t    emg_raw = 0;
uint32_t   now_ms  = HAL_GetTick();

if (g_emg_speed_valid && (now_ms - g_emg_last_rx_ms) < EMG_SPEED_TIMEOUT_MS)
{
    emg_raw = g_emg_speed;
    // Map EMG speed (0,33,66,100) → PWM duty (0–1000)
    speed = (uint16_t)(((uint32_t)emg_raw * 1000u) / 100u);
}
else
{
    speed   = 0;
    emg_raw = 0;
}

/* ── Pitch → Direction (direction only, no more speed from pitch) ── */
MotorDir_t dir   = DIR_STOP;
float      abs_p = (pitch_val < 0.0f) ? -pitch_val : pitch_val;

if (abs_p >= 0.0436f)  // 5° deadzone (sin(5°/2))
{
    dir = (pitch_val > 0.0f) ? DIR_FORWARD : DIR_BACKWARD;
}

// If EMG speed is zero, force direction to STOP
if (speed == 0)
{
    dir = DIR_STOP;
}
```

**8.5g — Update `MotorCmd_t` assembly:**

```c
cmd.speedA   = speed;
cmd.speedB   = speed;
cmd.dirA     = dir;
cmd.dirB     = dir;
cmd.emg_speed = emg_raw;   // log raw EMG value

/* ── Roll → Steering (unchanged) ── */
if (roll_val > 0.0872f)       // Sin(10°/2)
{
    cmd.dirB   = DIR_STOP;
    cmd.speedB = 0;
}
else if (roll_val < -0.0872f)
{
    cmd.dirA   = DIR_STOP;
    cmd.speedA = 0;
}
```

### 8.6 MODIFY: `Core/Src/stm32f4xx_hal_msp.c`

In `HAL_UART_MspInit()`, add a `USART1` branch after the existing `USART2` block:

```c
if (huart->Instance == USART1)
{
    /* Peripheral clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* USART1_RX GPIO: PA10 → AF7 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin       = GPIO_PIN_10;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* DMA init for USART1_RX: DMA2 Stream 2 Channel 4 */
    __HAL_RCC_DMA2_CLK_ENABLE();

    hdma_usart1_rx.Instance                 = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel             = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode                = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority            = DMA_PRIORITY_LOW;
    hdma_usart1_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
    HAL_DMA_Init(&hdma_usart1_rx);

    __HAL_LINKDMA(huart, hdmarx, hdma_usart1_rx);

    /* NVIC for DMA */
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

    /* NVIC for USART1 */
    HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}
```

Also update `HAL_UART_MspDeInit()` with a corresponding `USART1` branch.

### 8.7 MODIFY: `Core/Src/stm32f4xx_it.c`

Add at the top of the file (in the `extern` block):

```c
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
```

Add two new interrupt handlers before the `USER CODE BEGIN 1` section:

```c
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

void DMA2_Stream2_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_usart1_rx);
}
```

### 8.8 MODIFY: `Core/Inc/stm32f4xx_it.h`

Add prototypes after the existing ones:

```c
void USART1_IRQHandler(void);
void DMA2_Stream2_IRQHandler(void);
```

### 8.9 MODIFY: `Core/Inc/app/app_config.h`

Add at the end:

```c
#define EMG_SPEED_TIMEOUT_MS 500  // stop motors if no valid EMG packet for 500 ms
```

---

## 9. Wiring

| ESP32 | STM32F411 |
|-------|-----------|
| GPIO17 (TX) | PA10 (USART1_RX) |
| GND | GND |

- USART2 (PA2/PA3) stays connected to the PC USB-UART adapter for CLI + CSV streaming
- ESP32 also connects to PC via USB for debug output (unchanged)

---

## 10. Control Logic Summary (Runtime)

| Input | Source | Maps to | Details |
|-------|--------|---------|---------|
| EMG Speed | ESP32 via USART1 DMA | PWM Duty (both motors) | 0→0, 33→330, 66→660, 100→1000 |
| IMU Pitch | EKF/Madgwick `q_delta[2]` | Forward/Backward | Sign determines direction, 5° deadzone |
| IMU Roll | EKF/Madgwick `q_delta[1]` | Differential Steering | 10° deadzone, disables one motor |

---

## 11. Safety Features

| Mechanism | Behaviour |
|-----------|-----------|
| EMG timeout | If `HAL_GetTick() - g_emg_last_rx_ms > 500 ms`, force speed = 0 |
| Startup guard | Motors stopped until EKF converged (50 samples) + gyro calibration done |
| Pitch deadzone | Pitch below 5° → `DIR_STOP` regardless of EMG |
| Zero-speed override | If EMG speed = 0, direction forced to `DIR_STOP` |
| Lost BLE on ESP32 | ESP32 auto-reconnects, STM32 times out to speed=0 via watchdog |
| Checksum validation | Corrupted packets are silently dropped |

---

## 12. Interrupt Priority Map

| IRQ | Priority | Purpose |
|-----|----------|---------|
| TIM2 | 5 | IMU sample trigger (200 Hz) |
| USART2 | 5 → overwritten to 1 | CLI RX (single-byte ring buffer, needs fast servicing) |
| USART1 | 6 | EMG DMA idle-line detection |
| DMA2_Stream2 | 6 | EMG DMA transfer complete |
| SysTick | 15 | FreeRTOS tick |

All priorities are above FreeRTOS `configMAX_SYSCALL_INTERRUPT_PRIORITY` (5 for preemption priority bits = 4 → 5 << 4 = 80 → NVIC priority 5 and above). Wait — FreeRTOS on Cortex-M4 uses `configMAX_SYSCALL_INTERRUPT_PRIORITY` which is typically 5 << 4 = 80 (if using 4 preemption bits). Priority 1 (highest) down to 5 can call FreeRTOS ISR-safe functions. Priority 6 cannot — but `emg_uart_on_rx_event` makes NO FreeRTOS calls, so priority 6 is correct.

---

## 13. Verification Steps

1. Wire: ESP32 GPIO17 → STM32 PA10, common GND
2. Verify ESP32 has `esp32_myo.ino` flashed and Myo armband connected (green LED = active)
3. Flash STM32 with modified firmware
4. Open STM32 CLI on USART2 at 921600 baud — type `HELP` to verify CLI still works
5. Wear Myo armband on forearm, flex — motors should spin proportionally to flex strength
6. Tilt hand forward/back — motors should reverse direction
7. Tilt hand left/right — robot should steer (one motor slows/stops)
8. Stop flexing — motors should stop within 500 ms (EMG timeout)
9. Power-cycle ESP32 — motors should stop (timeout), resume when ESP32 reconnects

---

## 14. Potential Risks & Mitigations

| Risk | Mitigation |
|------|------------|
| DMA2_Stream2 conflicts with other peripherals (SPI, SDIO, I2S) | Verify no other peripherals use DMA2 Stream 2 Channel 4. The current project only uses I2C1 (no DMA), so no conflict. |
| `HAL_UARTEx_ReceiveToIdle_DMA()` not in older HAL | Use manual IDLE detection as fallback: enable `UART_IT_IDLE`, check flag in `USART1_IRQHandler`, read SR+DR, compute received bytes from `__HAL_DMA_GET_COUNTER`. |
| BLE latency causes irregular EMG packet spacing | The 500 ms timeout is generous — even worst-case BLE jitter is < 100 ms. The checksum catches corruption. |
| EMG electrical noise causes false 0xAA bytes | The XOR checksum and the 3-byte framing make false positives extremely unlikely. Even if one slips through, the speed value is bounded (0–100). |
| USART1 baud rate mismatch | Both ESP32 and STM32 set to 921600. Verify with oscilloscope is possible. |

---

## 15. ESP32 Code: No Changes

`esp32_myo/esp32_myo.ino` already:

- Connects to Myo armband via BLE
- Processes 8-channel EMG: 20 Hz Butterworth highpass, 15-sample sliding RMS, per-channel average
- Quantizes combined RMS to 0/33/66/100 (10% hysteresis)
- Sends 3-byte packet `{0xAA, speed, XOR_checksum}` on UART2 at 921600 baud
- Auto-reconnects on BLE drop
- Button (GPIO34) re-triggers calibration

**No modifications needed.**

---

## 16. Summary

This plan introduces a clean separation of concerns:
- **ESP32** handles BLE + EMG signal processing + calibration
- **STM32** handles IMU (EKF/Madgwick) + motor control + CLI
- **USART1 DMA** is the dedicated, low-overhead bridge between them
- **USART2** remains unchanged for PC communication

The control strategy becomes: **flex to go, tilt to steer** — the intuitive mapping for a forearm-worn controller driving a differential-drive robot.
