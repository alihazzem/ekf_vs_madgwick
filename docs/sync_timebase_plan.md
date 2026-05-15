# STM32-ESP32 Shared Timebase Sync + Bug Fixes Plan

## Architecture Summary

```
STM32 TIM2 (200 Hz) ── PA0 (200 Hz square wave) ──────► ESP32 GPIO33 (EXTI input)
        │                                                         │
        │ period ISR: g_shared_tick++                             │ EXTI ISR: shared_tick++
        │                                                         │
        │ IMU sample + EKF/Madgwick                               │ EMG processing
        │ reads g_emg_tick from packet                            │ sends 5-byte packet with tick
        │ latency = g_shared_tick - g_emg_tick                    │
        ▼                                                         ▼
   Motor Command ←──────────────── UART (5-byte pkt) ←────────── ESP32
                                              [0xAA, speed, tick_lo, tick_hi, checksum]
```

Both boards increment a counter from the **same physical 200 Hz signal**. One clock. Zero drift. Any latency between EMG measurement and motor usage is known and quantifiable.

---

## Physical Wiring

| From | Pin | To | Pin |
|---|---|---|---|
| STM32 | PA0 (TIM2_CH1) | ESP32 | GPIO33 |
| STM32 | PA10 (USART1 RX) | ESP32 | GPIO17 (TX) |
| Both | GND | Both | GND |

---

## STM32 Changes

### 1. `Core/Src/stm32f4xx_hal_msp.c` — Add PA0 as TIM2_CH1 output

In `HAL_TIM_Base_MspInit`, after the existing TIM2 config, add:

```c
// PA0 = TIM2_CH1 → outputs 200 Hz square wave to ESP32
__HAL_RCC_GPIOA_CLK_ENABLE();
GPIO_InitStruct.Pin       = GPIO_PIN_0;
GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
GPIO_InitStruct.Pull      = GPIO_NOPULL;
GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
```

### 2. `Core/Src/main.c` — Configure TIM2 CH1 output compare + shared counter

In `MX_TIM2_Init`, after the existing master config, add OC channel:

```c
TIM_OC_InitTypeDef sConfigOC = {0};
sConfigOC.OCMode   = TIM_OCMODE_TOGGLE;
sConfigOC.Pulse    = 49;          // toggle at ARR match → sync with ISR
sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
```

In `HAL_TIM_PeriodElapsedCallback` (TIM2 case), add:

```c
g_shared_tick++;  // increment shared counter at each period
```

In `imu_task_fn`, after `HAL_TIM_Base_Start_IT(&htim2)`, add:

```c
HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);  // start 200 Hz output on PA0
```

### 3. `Core/Inc/drivers/emg_uart.h` — New protocol fields

```c
extern volatile uint16_t g_emg_tick;       // tick from ESP32 packet
extern volatile uint16_t g_shared_tick;     // local shared counter
```

### 4. `Core/Src/drivers/emg_uart.c` — Parse 5-byte packet

Packet format: `[0xAA] [speed] [tick_lo] [tick_hi] [checksum]`  
Checksum: `0xAA ^ speed ^ tick_lo ^ tick_hi`

In `emg_uart_on_rx_event`, update the parsing loop:

- Change `i + 2 < size` → `i + 4 < size`
- Read `speed = buf[i+1]`, `tick_lo = buf[i+2]`, `tick_hi = buf[i+3]`, `cksum = buf[i+4]`
- Validate `cksum == (0xAA ^ speed ^ tick_lo ^ tick_hi)`
- Store `g_emg_tick = tick_lo | (tick_hi << 8)`
- Advance `i += 4`

### 5. `Core/Src/main.c` — Use tick for latency in motor control

```c
uint16_t latency = g_shared_tick - g_emg_tick;
if (g_emg_speed_valid && (now_ms - g_emg_last_rx_ms) < EMG_SPEED_TIMEOUT_MS && latency < 10)
{
    emg_raw = g_emg_speed;
    speed = (uint16_t)(((uint32_t)emg_raw * 999u) / 100u);  // FIXED scaling
}
```

---

## ESP32 Changes

### 6. `esp32_myo/esp32_myo.ino` — Clock input + new protocol

Add definitions:
```cpp
#define SYNC_PIN 33        // 200 Hz clock from STM32 PA0
volatile uint16_t shared_tick = 0;
```

In `setup()`, add:
```cpp
pinMode(SYNC_PIN, INPUT_PULLDOWN);
attachInterrupt(digitalPinToInterrupt(SYNC_PIN), []() { shared_tick++; }, RISING);
```

In `emgCallback`, replace the packet send:
```cpp
uint16_t tick = shared_tick;
uint8_t pkt[5] = {
    0xAA, speed,
    (uint8_t)(tick & 0xFF), (uint8_t)(tick >> 8),
    (uint8_t)(0xAA ^ speed ^ (tick & 0xFF) ^ (tick >> 8))
};
STM32Serial.write(pkt, 5);
```

Update reconnect stop packet to 5 bytes:
```cpp
uint8_t stopPkt[5] = { 0xAA, 0, 0, 0, 0xAA };
```

---

## Bug Fixes

### 7. `Core/Src/main.c:563` — PWM scaling fix **(CRITICAL)**

```diff
- speed = (uint16_t)(((uint32_t)emg_raw * 1000u) / 100u);
+ speed = (uint16_t)(((uint32_t)emg_raw * 999u)  / 100u);
```

This is the root cause of motor stopping at high EMG values. TIM3 ARR = 999, so CCR must be ≤ 999. At emg_raw = 100, the old code produced CCR = 1000 > ARR, resulting in 0% duty cycle instead of ~100%.

### 8. `esp32_myo/esp32_myo.ino` — `sampleCount` overflow fix

`sampleCount` is a `uint16_t` that wraps after ~164 seconds, causing a ~75 ms data dropout. After sending packet at line 293, reset:

```cpp
sampleCount = 0;
for (int ch = 0; ch < N_CH; ch++) rmsAccum[ch] = 0;
```

### 9. `Core/Src/drivers/emg_uart.c` — `g_emg_speed_valid` cleanup

Reset `g_emg_speed_valid = false` in `emg_uart_init()` (already done). The timestamp timeout guard prevents stale data, but resetting the flag on timeout in the IMU task improves readability.

---

## Files Changed Summary

| # | File | Change | Priority |
|---|---|---|---|
| 1 | `Core/Src/main.c:563` | PWM scaling fix | **Critical** |
| 2 | `esp32_myo/esp32_myo.ino` | sampleCount overflow fix | **High** |
| 3 | `Core/Src/stm32f4xx_hal_msp.c` | PA0 AF config for TIM2_CH1 | Sync |
| 4 | `Core/Src/main.c` | TIM2 CH1 OC config, OC start, shared_tick++ | Sync |
| 5 | `Core/Inc/drivers/emg_uart.h` | New protocol fields | Sync |
| 6 | `Core/Src/drivers/emg_uart.c` | Parse 5-byte packet | Sync |
| 7 | `esp32_myo/esp32_myo.ino` | GPIO33 interrupt, 5-byte packet | Sync |

---

## Protocol Specification

### Current (3 bytes)
```
[0] 0xAA           | Header
[1] speed           | 0, 33, 66, or 100
[2] checksum        | 0xAA XOR speed
```

### New (5 bytes)
```
[0] 0xAA           | Header
[1] speed           | 0, 33, 66, or 100
[2] tick_lo         | shared_tick & 0xFF
[3] tick_hi         | shared_tick >> 8
[4] checksum        | 0xAA XOR speed XOR tick_lo XOR tick_hi
```

### Latency Guard

STM32 compares its own `g_shared_tick` with the tick from the ESP32 packet:

```
latency = g_shared_tick - packet.tick   (in 5 ms units at 200 Hz)
```

If latency > 10 (50 ms), the data is considered stale and discarded. This is a second safety net alongside the 500 ms millisecond timeout.
