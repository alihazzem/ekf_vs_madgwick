# Modifications & Issues Found

Review of the full `ekf_madgwick_comparsion` codebase.

---

## 1. BUG — Duplicate Dead `MAD RESET` Handler (cli_app.c)

**File:** `Core/Src/app/cli_app.c` lines ~422–433

The `MAD RESET` command is handled **twice**. The second block is unreachable dead code that still contains a stale `"ERR: RESET not wired yet"` message from an earlier stage of development:

```c
// FIRST handler (correct — this one runs)
if (argc >= 2 && strcmp(argv[1], "RESET") == 0) {
    imu_app_madgwick_reset();
    uart_cli_send("ok\r\n");
    return;
}

// SECOND handler (DEAD CODE — never reached)
if (argc >= 2 && strcmp(argv[1], "RESET") == 0) {
    uart_cli_send("ERR: RESET not wired yet\r\n");
    return;
}
```

**Fix:** Delete the second (dead) `MAD RESET` block entirely.

---

## 2. BUG — Pitch Sign Negated in `MAD SHOW` Output (cli_app.c)

**File:** `Core/Src/app/cli_app.c` ~line 414

```c
int32_t p = (int32_t)(-a.pitch_deg * 1000.0f);   // <-- unexplained negation
```

Roll and yaw are printed as-is, but pitch is silently negated. If this is intentional (NED vs. ENU convention), it should be documented with a comment. If not, it's a sign error that will confuse logging/comparison.

**Fix:** Either remove the negation or add a clear comment explaining the convention choice.

---

## 3. ISSUE — Unused Global Variable `imu_tick` (main.c)

**File:** `Core/Src/main.c` line 57

```c
volatile uint8_t imu_tick = 0;
```

This variable is declared but **never read or written** anywhere in the codebase. It was likely an early placeholder before `imu_app_on_100hz_tick()` was implemented.

**Fix:** Delete the declaration.

---

## 4. ISSUE — EKF Filter Is Completely Unimplemented

**Files:** `Core/Src/filters/ekf.c`, `Core/Inc/filters/ekf.h`

Both files are empty stubs. The project is named "EKF vs Madgwick comparison" and `app_config.h` defines `RUN_EKF 0`, but there is **zero** EKF code. The `FusionOut_t` type in `imu_types.h` has an `ekf` field and `ekf_acc_used` that are never populated.

**Fix:** Implement the EKF or, at minimum, add a `TODO` / `#warning` in `ekf.c` so this isn't silently forgotten.

---

## 5. ISSUE — Empty Stub Files (uart_logger, app_imu_task)

The following files are completely empty (no code, just auto-generated header comments):

- `Core/Src/drivers/uart_logger.c` + `Core/Inc/drivers/uart_logger.h`
- `Core/Src/app/app_imu_task.c` + `Core/Inc/app/app_imu_task.h`

They add noise to the project, inflate build time, and may confuse contributors.

**Fix:** Either implement them or remove them from the source tree and the build system.

---

## 6. PERFORMANCE — Blocking UART TX in `uart_cli_send` (uart_cli.c)

**File:** `Core/Src/drivers/uart_cli.c` line 34

```c
HAL_UART_Transmit(s_huart, (uint8_t*)s, (uint16_t)strlen(s), 1000);
```

Every `uart_cli_send` / `uart_cli_sendf` call blocks the CPU for up to **1 second** per transmission. When streaming IMU data with a print divisor, this blocking transmit adds directly to the service time of each sample, increasing `svc_max_us` and risking missed ticks.

**Fix:** Switch to DMA transmit (`HAL_UART_Transmit_DMA`) with a TX ring buffer, or at least reduce the timeout to a reasonable value (e.g., 50 ms).

---

## 7. PERFORMANCE — Inefficient `trim_inplace` (cli_app.c)

**File:** `Core/Src/app/cli_app.c` lines 21–30

```c
while (*s && isspace((unsigned char)*s)) {
    memmove(s, s + 1, strlen(s));   // O(n) per leading space → O(n²) total
}
```

For each leading space, the entire remaining string is shifted by one byte. This is O(n²) in the number of leading spaces.

**Fix:** Find the first non-space character, then do a single `memmove`:

```c
char *start = s;
while (*start && isspace((unsigned char)*start)) start++;
if (start != s) memmove(s, start, strlen(start) + 1);
```

---

## 8. ROBUSTNESS — No Auto-Init or Guard for MPU Before Streaming

**Flow:** User must manually type `MPU INIT` before `MPU STREAM ON`. If they forget, `mpu6050_read_raw` will fail every tick (returns `MPU6050_ERR_I2C`), printing `"mpu read error"` 100 times/second.

**Fix options:**
- Call `mpu6050_init_100hz` inside `imu_app_init()` automatically, **or**
- In the `MPU STREAM ON` handler, check whether the MPU was initialized and warn the user.

---

## 9. ROBUSTNESS — `__disable_irq()` / `__enable_irq()` Is Too Broad

**File:** `Core/Src/app/imu_app.c` (multiple locations)

Globally disabling all interrupts to protect a few shared variables blocks **every** ISR (SysTick, UART RX, DMA, etc.), not just TIM2. On a Cortex-M4 with nested interrupts, this can cause UART byte drops.

**Fix:** Use `BASEPRI`-based critical sections that only mask the timer priority level, or use a dedicated critical-section macro for clarity:

```c
#define IMU_ENTER_CRITICAL()  uint32_t _primask = __get_PRIMASK(); __disable_irq()
#define IMU_EXIT_CRITICAL()   __set_PRIMASK(_primask)
```

This at least makes intent explicit and safely handles nested critical sections (the current code would unconditionally re-enable IRQs even if they were already disabled).

---

## 10. ROBUSTNESS — No Watchdog Configured

No IWDG or WWDG is enabled. If the firmware hangs (e.g., I2C bus lockup, hard-fault in user code), the device stays bricked until manually power-cycled.

**Fix:** Enable IWDG with a ~2-second timeout. Refresh it in the main loop after `imu_app_poll()`.

---

## 11. MINOR — Ring Buffer `rb_push` Silently Drops Data on Overflow

**File:** `Core/Src/utils/ringbuf.c`

When the ring buffer is full, `rb_push` returns `-1` but the caller in `uart_cli_on_rx_byte` ignores the return value:

```c
(void)rb_push(&s_rb, s_rx_byte);   // drop on floor if full
```

If UART data arrives faster than the main loop processes it (e.g., during heavy streaming output), bytes are silently lost.

**Fix:** Optionally track an overflow counter so the user can see it in `STATUS` output.

---

## 12. MINOR — `uart_cli_sendf` Output Silently Truncated at 256 Bytes

**File:** `Core/Src/drivers/uart_cli.c`

```c
char buf[256];
```

If a formatted string exceeds 255 characters (possible with long `MPU STATS` output), it is silently truncated by `vsnprintf`.

**Fix:** Increase to 512, or use a two-pass approach (`vsnprintf(NULL, 0, ...)` to measure, then allocate/send).

---

## 13. COSMETIC — Project Name Typo

The project folder and several files use `comparsion` instead of `comparison`.

**Fix:** Rename if practical; otherwise, note as known typo.

---

## Summary Table

| # | Severity | File | Issue |
|---|----------|------|-------|
| 1 | **Bug** | cli_app.c | Duplicate dead `MAD RESET` handler |
| 2 | **Bug** | cli_app.c | Pitch sign silently negated in `MAD SHOW` |
| 3 | Minor | main.c | Unused `imu_tick` variable |
| 4 | Major | ekf.c/h | EKF completely unimplemented |
| 5 | Minor | uart_logger, app_imu_task | Empty stub files |
| 6 | Perf | uart_cli.c | Blocking UART TX stalls main loop |
| 7 | Perf | cli_app.c | O(n²) `trim_inplace` |
| 8 | Robust | imu_app / cli | No MPU init guard before streaming |
| 9 | Robust | imu_app.c | `__disable_irq` too broad; unsafe nesting |
| 10 | Robust | system | No watchdog |
| 11 | Minor | ringbuf / uart_cli | Silent RX overflow drops |
| 12 | Minor | uart_cli.c | TX format buffer truncation at 256 |
| 13 | Cosmetic | project | "comparsion" typo |
