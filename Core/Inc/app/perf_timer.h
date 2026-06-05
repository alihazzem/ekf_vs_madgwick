/**
 * @file perf_timer.h
 * @brief Lightweight DWT-based execution-time profiler.
 *
 * Usage
 * -----
 *  1. Call perf_timer_reset() once at startup (or via CLI).
 *  2. Wrap any code block with:
 *       uint32_t _t0 = perf_timer_start();
 *       ... code ...
 *       perf_timer_stop(PERF_SLOT_FOO, _t0);
 *  3. Enable reporting:  g_perf_enable = 1;
 *     The IMU task calls perf_timer_report_if_due() every loop iteration;
 *     it prints a one-liner over UART every PERF_REPORT_INTERVAL_LOOPS loops.
 *
 * All measurements are in microseconds (µs).
 * The module is zero-overhead when g_perf_enable == 0 — the slot counters
 * are simply not updated and the report is never sent.
 */
#pragma once
#include <stdint.h>

/* ── Slot IDs ─────────────────────────────────────────────────────── */
typedef enum {
    PERF_SLOT_EMG_PARSE   = 0,  /**< EMG UART packet parsing (ISR)     */
    PERF_SLOT_SWING_TWIST = 1,  /**< Swing-twist + EMA + servo write   */
    PERF_SLOT_SERVO_PWM   = 2,  /**< __HAL_TIM_SET_COMPARE calls only  */
    PERF_SLOT_GRIPPER     = 3,  /**< gripper_update()                  */
    PERF_SLOT_DISP_QUEUE  = 4,  /**< osMessageQueuePut                 */
    PERF_NUM_SLOTS        = 5
} PerfSlot_t;

/* ── Report cadence ───────────────────────────────────────────────── */
/** Print a report every this many IMU loop iterations (~1 s at 200 Hz). */
#define PERF_REPORT_INTERVAL_LOOPS  200u

/* ── Runtime enable flag ──────────────────────────────────────────── */
/**
 * Set to 1 (e.g. via CLI: PERF ON) to enable accumulation + reporting.
 * Set to 0 (PERF OFF) to disable — the DWT reads still occur but results
 * are discarded, so overhead is two register reads per instrumented block.
 */
extern volatile uint8_t g_perf_enable;

/* ── API ──────────────────────────────────────────────────────────── */

/** Reset all slot statistics and the loop counter. */
void perf_timer_reset(void);

/**
 * Start a measurement.  Returns the raw CYCCNT value captured immediately.
 * Call this right before the code under test.
 */
static inline uint32_t perf_timer_start(void)
{
    extern uint32_t timebase_cycles(void);
    return timebase_cycles();
}

/**
 * Stop a measurement and, if g_perf_enable is set, accumulate into slot.
 * @param slot  Which PERF_SLOT_* to update.
 * @param t0    Value returned by perf_timer_start().
 */
void perf_timer_stop(PerfSlot_t slot, uint32_t t0);

/**
 * Call once per IMU loop iteration.
 * Increments the internal loop counter and, when the counter reaches
 * PERF_REPORT_INTERVAL_LOOPS and g_perf_enable is set, prints one
 * report line to the CLI UART then resets the accumulators.
 */
void perf_timer_report_if_due(void);
