# FreeRTOS Integration Plan for EKF + Madgwick (Corrected)

## Goal
Migrate the existing bare-metal, timer-driven EKF and Madgwick pipeline to FreeRTOS using
CMSIS-RTOS v2, without changing any filter math or data flow. The IMU sample timing must
remain precisely 200 Hz, and the filters must run in the same order with the same inputs.

## Non-Goals
- No new filter implementation, no algorithm changes, and no refactor of EKF or Madgwick math.
- No change to sensor conversion, remap, or calibration logic.
- No change to CLI commands or output formats.

## Baseline (Current Behavior)
The current execution flow is a TIM2 interrupt-driven superloop. The control flow, rate, and
data pipeline inside `imu_app_poll()` must be preserved exactly.

---

## FreeRTOS Architecture (Target)

### Core Design Decisions
- Use CubeMX to generate FreeRTOS with CMSIS-RTOS v2.
- Single IMU task running the exact same read + update order.
- Keep TIM2 as the 200 Hz source and notify the IMU task from the ISR using **Native FreeRTOS
  Task Notifications** (bypassing CMSIS-RTOS wrappers for maximum speed and lowest jitter in
  the ISR).
- Move UART CLI to its own low-priority task.
- **Hardware FPU must be enabled and verified to save context during task switches.**

### Task Model
| Task / ISR | Priority | Stack | Implementation |
| :--- | :--- | :--- | :--- |
| **TIM2 ISR** | HW Interrupt (Level 5) | — | `vTaskNotifyGiveFromISR` |
| **IMU Task** | `osPriorityRealtime` (6) | **2048 words (8 KB)** | Blocks on `ulTaskNotifyTake` |
| **CLI Task** | `osPriorityLow` (1) | 256 words | Polling ring buffer with `osDelay` |

> **Why 2048 words for IMU Task?** A single 9×9 float matrix is 324 bytes. During the EKF
> update you have multiple temporaries live simultaneously (innovation, Kalman gain, intermediate
> products). Add the FreeRTOS FPU context frame (~200 bytes for S16–S31) and 4 KB (1024 words)
> is dangerously close. Start at 8 KB and tune down using the watermark (see Phase 7).

### ISR-to-Task Synchronization
- TIM2 ISR uses direct-to-task notification: `vTaskNotifyGiveFromISR()`.
- IMU Task blocks on `ulTaskNotifyTake(pdTRUE, portMAX_DELAY)`.

---

## FreeRTOS Configuration (`FreeRTOSConfig.h`)

```c
#define configUSE_PREEMPTION                    1
#define configMAX_PRIORITIES                    7
#define configTICK_RATE_HZ                      1000   // MUST be set explicitly
#define configTOTAL_HEAP_SIZE                   16384  // 16 KB minimum; increase if needed
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY  5

// FPU: verify port.c is the CM4F variant after CubeMX generation.
// The correct port saves S16-S31 during context switches automatically.
// configENABLE_FPU may not exist in your FreeRTOS version — rely on port selection instead.
// Confirm by checking that vPortSVCHandler in port.c saves floating-point registers.

// Development phase — enable both of these before tuning:
#define configUSE_TRACE_FACILITY                1
#define configCHECK_FOR_STACK_OVERFLOW          2      // Method 2: checks pattern at stack end
```

> **`configTICK_RATE_HZ` must be 1000.** At 200 Hz IMU rate, `osDelay(5)` in the CLI task
> gives a clean 5 ms yield. If the tick is set to 100 Hz, `osDelay(5)` becomes 50 ms and the
> CLI becomes unresponsive.

> **`configCHECK_FOR_STACK_OVERFLOW` must be 2 during development.** Without it, a stack
> overflow will silently corrupt adjacent memory and manifest as bizarre filter divergence or a
> hard fault with no obvious cause.

**Implement the overflow hook in `main.c`:**
```c
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    __BKPT(0);  // Break in debugger immediately; replace with error handler in production
}
```

---

## Detailed Implementation Plan

### Phase 1: CubeMX Configuration

1. Enable FreeRTOS (CMSIS-RTOS v2) in CubeMX.

2. **FPU Context Saving — Critical:**
   After code generation, verify that the selected `port.c` is the **Cortex-M4F** variant,
   not the plain CM4. The CM4F port saves FP registers (S16–S31) during context switches.
   Confirm by checking that `xPortPendSVHandler` in `port.c` contains VPUSH/VPOP instructions.
   If CubeMX generated the wrong port, replace it manually with the CM4F version.

3. Configure RTOS heap and tick:
   - Heap scheme: `Heap_4`.
   - `configTOTAL_HEAP_SIZE`: 16384 bytes minimum.
   - `configTICK_RATE_HZ`: **1000**.

4. Confirm interrupt priorities:
   - `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`: **5**.
   - TIM2 and USART2 IRQ priorities must be numerically **≥ 5** (e.g., 5 or 6).
   - **Never call `FromISR` APIs from an interrupt with priority 0–4.** Doing so bypasses
     the FreeRTOS critical section mechanism and causes undefined behavior.

5. Generate code.

6. Add to `FreeRTOSConfig.h`:
   - `configUSE_TRACE_FACILITY 1`
   - `configCHECK_FOR_STACK_OVERFLOW 2`
   - `configTICK_RATE_HZ 1000`

---

### Phase 2: Task Creation and Startup Sequence

1. Replace the superloop in `main.c` with RTOS startup.

2. Initialize all peripherals and modules **before** starting the scheduler:
   ```c
   HAL_Init();
   SystemClock_Config();
   MX_GPIO_Init();
   MX_I2C1_Init();
   MX_TIM2_Init();
   MX_USART2_UART_Init();
   timebase_init();        // DWT cycle counter
   uart_cli_init();
   app_cli_print_banner();
   imu_app_init();
   ```

3. Create tasks:
   ```c
   // IMU Task: highest priority, large stack for EKF matrix math
   osThreadNew(imu_task_fn, NULL, &(osThreadAttr_t){
       .name       = "IMU",
       .priority   = osPriorityRealtime,
       .stack_size = 2048 * 4   // 2048 words → 8 KB
   });

   // CLI Task: low priority, small stack
   osThreadNew(cli_task_fn, NULL, &(osThreadAttr_t){
       .name       = "CLI",
       .priority   = osPriorityLow,
       .stack_size = 256 * 4    // 256 words → 1 KB
   });
   ```

4. **Do not start TIM2 here.** Start it inside the IMU task (see Phase 3). This guarantees
   the task is ready to receive notifications before the first interrupt fires.

5. Start the scheduler:
   ```c
   osKernelStart();
   ```

---

### Phase 3: IMU Task

#### 3a. Split `imu_app_poll()` into a single-step worker
Rename the body of `imu_app_poll()` to `imu_app_step()`. The function signature and internal
logic do not change — only the outer timing mechanism is replaced by the task notification.

#### 3b. IMU Task Loop
```c
static void imu_task_fn(void *arg)
{
    (void)arg;

    // Start TIM2 inside the task — guarantees task is ready before first ISR fires
    HAL_TIM_Base_Start_IT(&htim2);

    while (1)
    {
        // Block until TIM2 ISR wakes us; returns accumulated notification count
        uint32_t count = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Missed tick accounting — correct location (NOT in the ISR)
        // If count > 1, the task was not ready in time and missed (count - 1) ticks
        if (count > 1)
        {
            s_missed += (count - 1);
        }

        // Run one sample of the existing pipeline (unchanged logic)
        imu_app_step();

        // Stack watermark check — during development only; remove before release
        #ifdef DEBUG
        UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
        if (hwm < 100)
        {
            // WARNING: Stack nearly full — increase stack size in CubeMX
            __BKPT(0);
        }
        #endif
    }
}
```

> **Why missed-tick detection belongs in the task, not the ISR:**
> `vTaskNotifyGiveFromISR` only increments the notification counter — it has no visibility into
> whether the previous count was consumed. The task receives the accumulated count from
> `ulTaskNotifyTake`, so if it returns 2, exactly 1 tick was missed. This is ISR-safe and
> requires no additional state.

#### 3c. TIM2 ISR
```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(imuTaskHandle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
```

---

### Phase 4: Validate `osPriorityRealtime` Mapping

With `configMAX_PRIORITIES = 7`, FreeRTOS priorities run 0 (lowest) to 6 (highest).
`osPriorityRealtime` in CMSIS-RTOS v2 should map to priority 6. After generation, verify in
`cmsis_os2.c` that the mapping does not cap at a lower value due to CubeMX version differences.

```c
// Confirm in cmsis_os2.c or the generated wrapper:
// osPriorityRealtime → FreeRTOS priority 6 (= configMAX_PRIORITIES - 1)
```

If the mapping is wrong, set the FreeRTOS priority directly using `xTaskCreate` instead of
`osThreadNew`.

---

### Phase 5: CLI Task

```c
static void cli_task_fn(void *arg)
{
    (void)arg;
    while (1)
    {
        uart_cli_poll();
        osDelay(5);   // Yield ~5 ms; CLI stays responsive without blocking IMU task
    }
}
```

- UART RX ISR remains unchanged.
- `osDelay(5)` works correctly only when `configTICK_RATE_HZ = 1000` (5 ticks = 5 ms).

---

### Phase 6: Data Sharing and Synchronization

CLI reads of filter outputs (floats written by the IMU task) must be protected against torn
reads. Use critical sections inside the existing `imu_app_get_*` accessor functions:

```c
void imu_app_get_euler(float *roll, float *pitch, float *yaw)
{
    taskENTER_CRITICAL();
    *roll  = s_euler[0];
    *pitch = s_euler[1];
    *yaw   = s_euler[2];
    taskEXIT_CRITICAL();
}
```

> On Cortex-M4, individual 32-bit aligned float reads are atomic, but a struct of multiple
> floats is not — the task can be preempted between reads. Critical sections here are
> mandatory for correctness.

---

### Phase 7: Build, Verification, and Stack Tuning

#### Step 1 — Build and flash
Confirm zero build errors and zero warnings related to stack or heap size.

#### Step 2 — Stack watermark check
With `configUSE_TRACE_FACILITY = 1`, call inside the IMU task after the EKF runs:
```c
UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
// Transmit hwm over UART or check in debugger
// If hwm < 100 words → increase stack size; if hwm > 600 words → you can safely reduce it
```
Start at 2048 words and tune down. Never go below a watermark of ~200 words headroom.

#### Step 3 — I2C timing check
Monitor `s_svc_max_us`. At 200 Hz you have a strict **5.0 ms window** per sample.
- If `s_svc_max_us > 3500 µs` consistently, the blocking `HAL_I2C_Mem_Read()` is cutting too
  close and you must transition to `HAL_I2C_Mem_Read_DMA()`.

#### Step 4 — Missed tick verification
Verify `s_missed` remains exactly **0** during normal operation. Any non-zero value means the
IMU task is not finishing within the 5 ms window before the next TIM2 interrupt.

#### Step 5 — Filter output baseline check
Log Euler angles and compare against pre-RTOS baseline recordings. Any divergence indicates
a data sharing issue (see Phase 6) or an FPU context save problem (see Phase 1 step 2).

#### Step 6 — Disable development guards before release
```c
// Remove or gate behind a build flag before production flash:
#undef configCHECK_FOR_STACK_OVERFLOW   // or set to 0
#undef configUSE_TRACE_FACILITY         // or set to 0
// Remove DEBUG watermark check from imu_task_fn
```

---

## Exact Execution Order (Must Not Change)

This order must be preserved identically inside `imu_app_step()`:

1. TIM2 tick → IMU task wake (via task notification).
2. Read accel/gyro first, then mag (best effort).
3. Remap accel/gyro to body frame.
4. Apply mag ASA and remap.
5. Apply hard/soft iron calibration to mag.
6. First-sample alignment from accel or marg.
7. Madgwick update (marg or imu mode).
8. EKF update (marg or imu mode).
9. Euler conversion and stats update.

---

## Corrections Summary vs. Original Plan

| Item | Original | Corrected |
| :--- | :--- | :--- |
| Missed tick detection | ISR (incorrect) | Task — use `ulTaskNotifyTake` return value |
| IMU task stack size | 1024 words (4 KB) | **2048 words (8 KB)**; tune down with watermark |
| FPU enablement | `configENABLE_FPU 1` | Verify CM4F `port.c` variant; macro may not exist |
| Stack overflow detection | Not mentioned | `configCHECK_FOR_STACK_OVERFLOW 2` + hook |
| `configTICK_RATE_HZ` | Not mentioned | **Must be set to 1000 explicitly** |
| Priority mapping check | Not mentioned | Verify `osPriorityRealtime` → FreeRTOS priority 6 |