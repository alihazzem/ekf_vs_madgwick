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

#define COL_LEFT    0
#define COL_RIGHT   64
#define PAGE_STATUS 0
#define PAGE_PITCH  2
#define PAGE_ROLL   3
#define PAGE_SERVO_P 4
#define PAGE_SERVO_R 5

/* ── Task implementation ─────────────────────────────────────────────────── */
void display_task_fn(void *arg)
{
    (void)arg;

    /* SSD1306 needs ≥100 ms after VDD stable before accepting commands.
     * Without this delay init is unreliable, especially at 500 Hz where
     * the FreeRTOS scheduler starts faster than at lower sample rates. */
    osDelay(120U);

    /* Initialize the OLED over the dedicated hi2c2 bus */
    ssd1306_init(&hi2c2);
    ssd1306_clear();
    ssd1306_puts(0, 0, "DUAL IMU GIMBAL");
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

        /* Row 1: Column headers */
        ssd1306_puts(COL_LEFT,  1, "IMU0");
        ssd1306_puts(COL_RIGHT, 1, "IMU1");

        /* Row 2 (page 2): Pitch — IMU0 left, IMU1 right */
        {
            float v = state.pitch_0;
            char sign = (v < 0.0f) ? '-' : '+';
            int whole = (int)(v < 0.0f ? -v : v);
            int frac  = (int)(((v < 0.0f ? -v : v) - (float)whole) * 10.0f);
            snprintf(line, sizeof(line), "0P:%c%d.%d", sign, whole, frac);
        }
        ssd1306_puts(COL_LEFT, PAGE_PITCH, line);

        {
            float v = state.pitch_1;
            char sign = (v < 0.0f) ? '-' : '+';
            int whole = (int)(v < 0.0f ? -v : v);
            int frac  = (int)(((v < 0.0f ? -v : v) - (float)whole) * 10.0f);
            snprintf(line, sizeof(line), "1P:%c%d.%d", sign, whole, frac);
        }
        ssd1306_puts(COL_RIGHT, PAGE_PITCH, line);

        /* Row 3 (page 3): Roll — IMU0 left, IMU1 right */
        {
            float v = state.roll_0;
            char sign = (v < 0.0f) ? '-' : '+';
            int whole = (int)(v < 0.0f ? -v : v);
            int frac  = (int)(((v < 0.0f ? -v : v) - (float)whole) * 10.0f);
            snprintf(line, sizeof(line), "0R:%c%d.%d", sign, whole, frac);
        }
        ssd1306_puts(COL_LEFT, PAGE_ROLL, line);

        {
            float v = state.roll_1;
            char sign = (v < 0.0f) ? '-' : '+';
            int whole = (int)(v < 0.0f ? -v : v);
            int frac  = (int)(((v < 0.0f ? -v : v) - (float)whole) * 10.0f);
            snprintf(line, sizeof(line), "1R:%c%d.%d", sign, whole, frac);
        }
        ssd1306_puts(COL_RIGHT, PAGE_ROLL, line);

        /* Row 4 (page 4): Pitch servo degrees — IMU0 left, IMU1 right */
        {
            int deg = ((int)state.servo_pitch_us_0 - 1500) * 180 / 2000;
            char sign = (deg < 0) ? '-' : '+';
            if (deg < 0) deg = -deg;
            snprintf(line, sizeof(line), "0PS:%c%dd", sign, deg);
        }
        ssd1306_puts(COL_LEFT, PAGE_SERVO_P, line);

        {
            int deg = ((int)state.servo_pitch_us_1 - 1500) * 180 / 2000;
            char sign = (deg < 0) ? '-' : '+';
            if (deg < 0) deg = -deg;
            snprintf(line, sizeof(line), "1PS:%c%dd", sign, deg);
        }
        ssd1306_puts(COL_RIGHT, PAGE_SERVO_P, line);

        /* Row 5 (page 5): Roll servo degrees — IMU0 left, IMU1 right */
        {
            int deg = ((int)state.servo_roll_us_0 - 1500) * 180 / 2000;
            char sign = (deg < 0) ? '-' : '+';
            if (deg < 0) deg = -deg;
            snprintf(line, sizeof(line), "0RS:%c%dd", sign, deg);
        }
        ssd1306_puts(COL_LEFT, PAGE_SERVO_R, line);

        {
            int deg = ((int)state.servo_roll_us_1 - 1500) * 180 / 2000;
            char sign = (deg < 0) ? '-' : '+';
            if (deg < 0) deg = -deg;
            snprintf(line, sizeof(line), "1RS:%c%dd", sign, deg);
        }
        ssd1306_puts(COL_RIGHT, PAGE_SERVO_R, line);

        /* Push framebuffer to OLED */
        ssd1306_flush(&hi2c2);

        /* Rate-limit to max 10 Hz.
         * Even if the IMU task spams 200 messages/s, we only render every
         * 100 ms. The queue depth=1 means we always display the freshest
         * data; stale messages are automatically overwritten by the producer. */
        osDelay(100U);
    }
}
