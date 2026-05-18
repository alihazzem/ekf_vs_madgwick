/**
 * @file display_task.c
 * @brief RTOS display task — consumer of the SystemState_t message queue.
 *
 * ARCHITECTURE (producer / consumer):
 *
 *  ┌─────────────────────────────────────────────────────────────────────┐
 *  │  IMU Task (osPriorityRealtime, 200 Hz)                              │
 *  │                                                                     │
 *  │  1. Compute pitch, roll, ccr_pitch, ccr_roll                        │
 *  │  2. Populate SystemState_t snapshot                                 │
 *  │  3. osMessageQueuePut(displayQueueHandle, &state, 0, 0)             │
 *  │                                                                     │
 *  │  timeout=0 ← THE KEY PROTECTION:                                   │
 *  │  If the display task hasn't consumed the previous message yet,      │
 *  │  osMessageQueuePut returns osErrorResource immediately.             │
 *  │  The IMU task drops the frame and continues its loop                │
 *  │  with ZERO blocking. The real-time deadline is never missed.        │
 *  └─────────────────────────────────────────────────────────────────────┘
 *                              │ Queue depth = 1
 *                              ▼
 *  ┌─────────────────────────────────────────────────────────────────────┐
 *  │  Display Task (osPriorityLow, ~10 Hz)                               │
 *  │                                                                     │
 *  │  1. osMessageQueueGet(displayQueueHandle, &state, 0, portMAX_DELAY) │
 *  │     → Blocks until the IMU task puts a new message                  │
 *  │  2. Render layout into ssd1306 framebuffer                          │
 *  │  3. ssd1306_flush(&hi2c2)                                           │
 *  │  4. osDelay(100ms) → caps refresh to max 10 Hz                     │
 *  └─────────────────────────────────────────────────────────────────────┘
 */

#include "app/display_task.h"
#include "drivers/ssd1306.h"

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>

/* ── External I2C handle for the OLED bus ────────────────────────────────── */
/* hi2c2 must be initialized in main.c / MX_I2C2_Init() before the           */
/* scheduler starts. The display task only calls HAL functions from one       */
/* task, so no mutex is needed.                                               */
extern I2C_HandleTypeDef hi2c2;

/* ── Message queue (depth = 1, holds one SystemState_t snapshot) ─────────── */
osMessageQueueId_t displayQueueHandle = NULL;

/* ── Display layout constants ────────────────────────────────────────────── */
/*
 * 128×64 display, 6-pixel-wide characters (5px glyph + 1px gap), 8 pages.
 *
 * Layout (page = 8-pixel tall row):
 *
 *   Page 0:  ── Status line ─────────────────────────────────────────────
 *            "CALIBRATING" or "RUNNING"
 *
 *   Page 2:  ── Left column (angles) ─────────────────────────────────────
 *            "P: +12.3 d"
 *
 *   Page 4:  "R: -45.6 d"
 *
 *   Page 2:  ── Right column (servo µs) (starting at col 70) ─────────────
 *            "PS:1567us"
 *
 *   Page 4:  "RS:2100us"
 */
#define COL_LEFT   0
#define COL_RIGHT  70
#define PAGE_STATUS 0
#define PAGE_PITCH  2
#define PAGE_ROLL   4

/* ── Task implementation ─────────────────────────────────────────────────── */
void display_task_fn(void *arg)
{
    (void)arg;

    /* Initialize the OLED over the dedicated hi2c2 bus */
    ssd1306_init(&hi2c2);
    ssd1306_clear();
    ssd1306_puts(0, 0, "IMU GIMBAL");
    ssd1306_puts(0, 2, "Calibrating...");
    ssd1306_flush(&hi2c2);

    SystemState_t state;
    char line[22];   /* Max 21 chars across a 128-px display at 6px/char */

    while (1) {
        /* Block here until the IMU task publishes a new snapshot.
         * portMAX_DELAY = wait forever. This task consumes zero CPU
         * while blocked — FreeRTOS places it in the "Blocked" state. */
        osStatus_t rc = osMessageQueueGet(displayQueueHandle,
                                          &state, NULL, portMAX_DELAY);
        if (rc != osOK) continue;

        /* ── Render framebuffer ──────────────────────────────────────── */
        ssd1306_clear();

        /* Row 0: System status */
        if (state.status == SYS_STATUS_CALIBRATING) {
            ssd1306_puts(COL_LEFT, PAGE_STATUS, "** CALIBRATING **");
        } else if (state.status == SYS_STATUS_REZEROED) {
            ssd1306_puts(COL_LEFT, PAGE_STATUS, "** RE-ZEROED  **");
        } else {
            ssd1306_puts(COL_LEFT, PAGE_STATUS, "   IMU GIMBAL   ");
        }

        /* Row 2 (left): Pitch angle
         * newlib-nano strips float printf, so we format manually:
         * e.g. -12.3 -> sign='-', whole=12, frac=3  */
        {
            float v = state.pitch;
            char sign = (v < 0.0f) ? '-' : '+';
            int whole = (int)(v < 0.0f ? -v : v);
            int frac  = (int)(((v < 0.0f ? -v : v) - (float)whole) * 10.0f);
            snprintf(line, sizeof(line), "P:%c%d.%d", sign, whole, frac);
        }
        ssd1306_puts(COL_LEFT, PAGE_PITCH, line);

        /* Row 4 (left): Roll angle */
        {
            float v = state.roll;
            char sign = (v < 0.0f) ? '-' : '+';
            int whole = (int)(v < 0.0f ? -v : v);
            int frac  = (int)(((v < 0.0f ? -v : v) - (float)whole) * 10.0f);
            snprintf(line, sizeof(line), "R:%c%d.%d", sign, whole, frac);
        }
        ssd1306_puts(COL_LEFT, PAGE_ROLL, line);

        /* Row 2 (right): Pitch servo in degrees (display only, not used for control)
         * Formula: deg = (us - 1500) * 180 / 2000
         * Range: 1167us=-30d, 1500us=0d, 1833us=+30d */
        {
            int deg = ((int)state.servo_pitch_us - 1500) * 180 / 2000;
            char sign = (deg < 0) ? '-' : '+';
            if (deg < 0) deg = -deg;
            snprintf(line, sizeof(line), "PS:%c%dd", sign, deg);
        }
        ssd1306_puts(COL_RIGHT, PAGE_PITCH, line);

        /* Row 4 (right): Roll servo in degrees (display only, not used for control)
         * Formula: deg = (us - 1500) * 180 / 2000
         * Range: 500us=-90d, 1500us=0d, 2500us=+90d */
        {
            int deg = ((int)state.servo_roll_us - 1500) * 180 / 2000;
            char sign = (deg < 0) ? '-' : '+';
            if (deg < 0) deg = -deg;
            snprintf(line, sizeof(line), "RS:%c%dd", sign, deg);
        }
        ssd1306_puts(COL_RIGHT, PAGE_ROLL, line);

        /* Push framebuffer to OLED */
        ssd1306_flush(&hi2c2);

        /* Rate-limit to max 10 Hz.
         * Even if the IMU task spams 200 messages/s, we only render every
         * 100 ms. The queue depth=1 means we always display the freshest
         * data; stale messages are automatically overwritten by the producer. */
        osDelay(100U);
    }
}
