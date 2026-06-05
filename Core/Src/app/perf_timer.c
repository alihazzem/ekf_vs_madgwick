#include "app/perf_timer.h"
#include "drivers/uart_cli.h"
#include "utils/timebase.h"
#include <string.h>

/* ── Runtime enable flag ──────────────────────────────────────────── */
volatile uint8_t g_perf_enable = 0;

/* ── Per-slot statistics ──────────────────────────────────────────── */
typedef struct {
    uint32_t count;     /* number of samples accumulated             */
    uint32_t min_us;    /* minimum observed µs                       */
    uint32_t max_us;    /* maximum observed µs                       */
    uint64_t sum_us;    /* running sum for average                   */
} PerfStat_t;

static PerfStat_t s_stats[PERF_NUM_SLOTS];
static uint32_t   s_loop_count = 0;

/* Human-readable slot labels — must match PerfSlot_t order */
static const char * const SLOT_LABEL[PERF_NUM_SLOTS] = {
    "EMG_PARSE  ",
    "SWING_TWIST",
    "SERVO_PWM  ",
    "GRIPPER    ",
    "DISP_QUEUE ",
};

/* ── Implementation ───────────────────────────────────────────────── */

void perf_timer_reset(void)
{
    for (int i = 0; i < PERF_NUM_SLOTS; i++) {
        s_stats[i].count  = 0;
        s_stats[i].min_us = 0xFFFFFFFFu;
        s_stats[i].max_us = 0;
        s_stats[i].sum_us = 0;
    }
    s_loop_count = 0;
}

void perf_timer_stop(PerfSlot_t slot, uint32_t t0)
{
    if (!g_perf_enable)
        return;
    if ((unsigned)slot >= PERF_NUM_SLOTS)
        return;

    uint32_t dt_us = timebase_cycles_to_us(timebase_cycles() - t0);

    PerfStat_t *s = &s_stats[slot];
    s->count++;
    if (dt_us < s->min_us) s->min_us = dt_us;
    if (dt_us > s->max_us) s->max_us = dt_us;
    s->sum_us += dt_us;
}

void perf_timer_report_if_due(void)
{
    s_loop_count++;

    if (!g_perf_enable)
        return;
    if (s_loop_count < PERF_REPORT_INTERVAL_LOOPS)
        return;

    /* ── Print header ── */
    uart_cli_send("PERF[us] slot         |  cnt |  min |  avg |  max\r\n");
    uart_cli_send("---------|------------|------|------|------|------\r\n");

    for (int i = 0; i < PERF_NUM_SLOTS; i++) {
        PerfStat_t *s = &s_stats[i];
        uint32_t avg_us = (s->count > 0)
                          ? (uint32_t)(s->sum_us / (uint64_t)s->count)
                          : 0;
        uint32_t min_us = (s->min_us == 0xFFFFFFFFu) ? 0 : s->min_us;

        uart_cli_sendf("PERF[us] %s | %4lu | %4lu | %4lu | %4lu\r\n",
                       SLOT_LABEL[i],
                       (unsigned long)s->count,
                       (unsigned long)min_us,
                       (unsigned long)avg_us,
                       (unsigned long)s->max_us);
    }
    uart_cli_send("\r\n");

    /* Reset accumulators for next window, keep loop counter ticking */
    for (int i = 0; i < PERF_NUM_SLOTS; i++) {
        s_stats[i].count  = 0;
        s_stats[i].min_us = 0xFFFFFFFFu;
        s_stats[i].max_us = 0;
        s_stats[i].sum_us = 0;
    }
    s_loop_count = 0;
}
