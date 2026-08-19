#include "app/imu_task.h"

#include "app/app_config.h"
#include "app/imu_app.h"
#include "app/gait_app.h"
#include "app/leg_telemetry.h"
#include "app/display_task.h" /* SystemState_t, displayQueueHandle */
#include "app/perf_timer.h"
#include "utils/math3d.h"
#include "drivers/emg_uart.h"
#include "drivers/gripper.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"

#include <math.h>

/* --- External Hardware Handles from main.c --- */
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart1;
extern osMessageQueueId_t displayQueueHandle;

/* --- Local Hardware Handles --- */
I2C_HandleTypeDef hi2c3;

/* Global re-zero request flag */
volatile int g_rezero_requested = 0;

#if STACK_TUNING_MODE
volatile UBaseType_t s_imu_hwm = 0;
#endif

#if ROBOT_MODE == ROBOT_MODE_LEG || ROBOT_MODE == ROBOT_MODE_FULL
GaitState_t g_leg1_state;
#if NUM_LEGS > 1
GaitState_t g_leg2_state;
#endif
#endif

static void MX_I2C3_Init(void)
{
  __HAL_RCC_I2C3_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  
  /* I2C3 GPIO Configuration
     PA8     ------> I2C3_SCL
     PB4     ------> I2C3_SDA 
  */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Alternate = GPIO_AF9_I2C3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c3);
}

void imu_task_fn(void *arg)
{
  (void)arg;
  uint32_t tick_count = 0;
  (void)tick_count;

  /* --- Startup: LED on PC13 signals calibration --- */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef led = {0};
  led.Pin = GPIO_PIN_13;
  led.Mode = GPIO_MODE_OUTPUT_PP;
  led.Pull = GPIO_NOPULL;
  led.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &led);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); /* active-low: ON */

  /* --- Safety Delay ---
   * Wait 2 seconds before sending power to the servos,
   * giving you time to move your hands clear after plugging in the battery.
   */
  osDelay(pdMS_TO_TICKS(2000));

#if NUM_IMUS > 2
  /* Initialize I2C3 for the second set of IMUs */
  MX_I2C3_Init();
#endif

#if ROBOT_MODE == ROBOT_MODE_LEG || ROBOT_MODE == ROBOT_MODE_FULL
  gait_app_init_state(&g_leg1_state);
#if NUM_LEGS > 1
  gait_app_init_state(&g_leg2_state);
#endif
#endif

#if ROBOT_MODE == ROBOT_MODE_ARM || ROBOT_MODE == ROBOT_MODE_FULL
  /* --- Init Servos to Safe Lock Position during calibration --- */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500); // Pitch 1
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 500);  // Roll 1
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1500); // Pitch 2
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 500);  // Roll 2
#endif

  /* --- Now begin the lengthy IMU initialization and calibration --- */
  imu_app_stream_set(false);
  imu_app_init(&hi2c1, &hi2c3);
  imu_app_ekf_reset();
  imu_app_madgwick_reset();
  
  imu_app_cal_gyro(4000);
  // imu_app_stream_set(true); /* Removed so CLI starts quiet by default */

#if ROBOT_MODE == ROBOT_MODE_ARM || ROBOT_MODE == ROBOT_MODE_FULL
  emg_uart_init(&huart1);
#endif

  /* Let EKF converge (~50 samples ≈ 250 ms at 200 Hz) */
  HAL_TIM_Base_Start_IT(&htim2);
#if ROBOT_MODE == ROBOT_MODE_ARM || ROBOT_MODE == ROBOT_MODE_FULL
  Attitude_t ekf_att[NUM_IMUS];
#endif
  for (int i = 0; i < 50; i++)
  {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
    imu_app_step();
  }

#if ROBOT_MODE == ROBOT_MODE_ARM || ROBOT_MODE == ROBOT_MODE_FULL
  /* Capture q_ref = neutral pose (filtered IMUs only) */
  float q_ref[NUM_IMUS][4];
  for (int i = 0; i < NUM_IMUS; i++)
  {
    q_ref[i][0] = 1.0f;
    q_ref[i][1] = 0.0f;
    q_ref[i][2] = 0.0f;
    q_ref[i][3] = 0.0f;
#if IMU1_RAW_ACCEL
    if (i == 1) continue; /* raw-accel IMU needs no quaternion reference */
#endif
    if (imu_app_get_ekf(i, &ekf_att[i]))
    {
      q_ref[i][0] = ekf_att[i].q0;
      q_ref[i][1] = ekf_att[i].q1;
      q_ref[i][2] = ekf_att[i].q2;
      q_ref[i][3] = ekf_att[i].q3;
    }
  }
#endif

#if ROBOT_MODE == ROBOT_MODE_ARM || ROBOT_MODE == ROBOT_MODE_FULL
  /* ── Servo Smooth Startup ────────────────────────────────────────────── */
  float smoothed_pitch[NUM_IMUS];
  float smoothed_roll[NUM_IMUS];

  for (int i = 0; i < NUM_IMUS; i++)
  {
    smoothed_pitch[i] = 1500.0f;
    smoothed_roll[i] = 500.0f;

#if IMU1_RAW_ACCEL
    if (i == 1)
    {
      /* Seed IMU 1 from raw accel tilt — no quaternion needed */
      float ax1s, ay1s, az1s;
      imu_app_get_accel_g(1, &ax1s, &ay1s, &az1s);
      float _p1 = atan2f(-ax1s, sqrtf(ay1s * ay1s + az1s * az1s)) * (180.0f / 3.14159265f);
      float _r1 = atan2f(ay1s, az1s) * (180.0f / 3.14159265f);
      smoothed_pitch[1] = fmaxf(1055.6f, fminf(1944.4f, 1500.0f + ((_p1 * PITCH_SERVO_DIR) * (2000.0f / 180.0f))));
      smoothed_roll[1]  = fmaxf(500.0f,  fminf(2500.0f, 1500.0f + (_r1 * (2000.0f / 180.0f))));
      continue;
    }
#endif
    if (imu_app_get_ekf(i, &ekf_att[i]))
    {
#if BYPASS_IMU_FILTER
      float ax, ay, az;
      imu_app_get_accel_g(i, &ax, &ay, &az);
      float _p = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / 3.14159265f);
      float _r = atan2f(ay, az) * (180.0f / 3.14159265f);
#else
      float q_conj[4] = {q_ref[i][0], -q_ref[i][1], -q_ref[i][2], -q_ref[i][3]};
      float q_now_seed[4] = {ekf_att[i].q0, ekf_att[i].q1, ekf_att[i].q2, ekf_att[i].q3};
      float q_delta_seed[4];
      math3d_quat_multiply(q_conj, q_now_seed, q_delta_seed);

      float _p, _r;
      math3d_quat_delta_to_pitch_roll_deg(q_delta_seed, &_p, &_r);
#endif

      float pitch_us = 1500.0f + ((_p * PITCH_SERVO_DIR) * (2000.0f / 180.0f));
      float roll_us = 1500.0f + (_r * (2000.0f / 180.0f));
      smoothed_pitch[i] = fmaxf(1055.6f, fminf(1944.4f, pitch_us));
      smoothed_roll[i] = fmaxf(500.0f, fminf(2500.0f, roll_us));
    }
  }

  /* Pre-drive servos to seeded position and let them physically settle */
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)smoothed_pitch[0]);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)smoothed_roll[0]);
  if (NUM_IMUS > 1)
  {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)smoothed_pitch[1]);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)smoothed_roll[1]);
  }
  osDelay(pdMS_TO_TICKS(300));

  gripper_home();
#endif

  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); /* active-low: OFF */

#if IMU1_RAW_ACCEL
  /* Re-zero reference angles for raw-accel IMU 1 (updated on button press) */
  float raw_ref_pitch_1 = 0.0f;
  float raw_ref_roll_1  = 0.0f;
#endif
  uint32_t rezero_flash_count = 0;

  /* --- Main loop: IMU-driven motor control --- */
  while (1)
  {
    uint32_t count = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (count > 1)
    {
      imu_app_add_missed(count - 1);
    }
    imu_app_step();

#if ROBOT_MODE == ROBOT_MODE_LEG || ROBOT_MODE == ROBOT_MODE_FULL
    {
      float t_sec = (float)HAL_GetTick() / 1000.0f;
      int16_t gy_shin1 = 0;
#if NUM_LEGS > 1
      int16_t gy_shin2 = 0;
#endif
      imu_app_get_gyro_raw(1, NULL, &gy_shin1, NULL);
      gait_app_auto_tare_check(&g_leg1_state, 0, 1);
      gait_app_update(&g_leg1_state, t_sec, gy_shin1, 0, 1);
#if NUM_LEGS > 1
      imu_app_get_gyro_raw(3, NULL, &gy_shin2, NULL);
      gait_app_auto_tare_check(&g_leg2_state, 2, 3);
      gait_app_update(&g_leg2_state, t_sec, gy_shin2, 2, 3);
#endif
      leg_telemetry_send(
        g_leg1_state.thigh_pitch, g_leg1_state.shin_pitch, g_leg1_state.knee_angle,
#if NUM_LEGS > 1
        g_leg2_state.thigh_pitch, g_leg2_state.shin_pitch, g_leg2_state.knee_angle
#else
        0.0f, 0.0f, 0.0f
#endif
      );
    }
#endif
    if (g_rezero_requested)
    {
      g_rezero_requested = 0; /* Clear FIRST to avoid double-trigger */

#if ROBOT_MODE == ROBOT_MODE_ARM || ROBOT_MODE == ROBOT_MODE_FULL
      {
#if IMU1_RAW_ACCEL
        /* Capture current raw tilt as the new zero reference for IMU 1 */
        float ax1r, ay1r, az1r;
        imu_app_get_accel_g(1, &ax1r, &ay1r, &az1r);
        raw_ref_pitch_1 = atan2f(-ax1r, sqrtf(ay1r * ay1r + az1r * az1r)) * (180.0f / 3.14159265f);
        raw_ref_roll_1  = atan2f(ay1r, az1r) * (180.0f / 3.14159265f);
#endif
        for (int i = 0; i < NUM_IMUS; i++)
        {
#if ROBOT_MODE == ROBOT_MODE_FULL
          /* In FULL mode, IMUs 0 and 1 belong to Leg 1. Only re-zero Arm IMUs (>= 2). */
          if (i < (NUM_LEGS * 2)) continue;
#endif
#if IMU1_RAW_ACCEL
          if (i == 1) continue; /* raw-accel IMU 1 handled above */
#endif
          if (imu_app_get_ekf(i, &ekf_att[i]))
          {
            q_ref[i][0] = ekf_att[i].q0;
            q_ref[i][1] = ekf_att[i].q1;
            q_ref[i][2] = ekf_att[i].q2;
            q_ref[i][3] = ekf_att[i].q3;
          }
        }
      }
#endif

#if ROBOT_MODE == ROBOT_MODE_LEG || ROBOT_MODE == ROBOT_MODE_FULL
      /* Tare Leg 1 */
      gait_app_tare(&g_leg1_state, 0, 1);
#if NUM_LEGS > 1
      /* Tare Leg 2 */
      gait_app_tare(&g_leg2_state, 2, 3);
#endif
#endif

      rezero_flash_count = (uint32_t)IMU_FS_HZ; /* Flash display for 1 second */
    }
    if (rezero_flash_count > 0U)
      rezero_flash_count--;

#if ROBOT_MODE == ROBOT_MODE_ARM || ROBOT_MODE == ROBOT_MODE_FULL
    float final_pitch_deg[NUM_IMUS] = {0};
    float final_roll_deg[NUM_IMUS] = {0};

    /* ── Thesis measurement 4: Tilt-only + deadzone + EMA block ── */
    uint32_t _t_st = perf_timer_start();

    for (int i = 0; i < NUM_IMUS; i++)
    {
#if ROBOT_MODE == ROBOT_MODE_FULL
      /* Skip Leg IMUs in Full Mode for servo control */
      if (i < (NUM_LEGS * 2)) continue;
#endif

      float pitch_deg = 0.0f;
      float roll_deg  = 0.0f;
      int   _valid    = 0;

#if IMU1_RAW_ACCEL
      if (i == 1)
      {
        /* IMU 1: raw accelerometer tilt only — no filter, no gyro fusion */
        float ax1, ay1, az1;
        imu_app_get_accel_g(1, &ax1, &ay1, &az1);
        pitch_deg = atan2f(-ax1, sqrtf(ay1 * ay1 + az1 * az1)) * (180.0f / 3.14159265f) - raw_ref_pitch_1;
        roll_deg  = atan2f(ay1, az1) * (180.0f / 3.14159265f) - raw_ref_roll_1;
        _valid = 1;
      }
      else
#endif
      if (imu_app_get_ekf(i, &ekf_att[i]))
      {
#if BYPASS_IMU_FILTER
        float ax, ay, az;
        imu_app_get_accel_g(i, &ax, &ay, &az);
        pitch_deg = atan2f(-ax, sqrtf(ay * ay + az * az)) * (180.0f / 3.14159265f);
        roll_deg  = atan2f(ay, az) * (180.0f / 3.14159265f);
#else
        float q_now[4] = {ekf_att[i].q0, ekf_att[i].q1, ekf_att[i].q2, ekf_att[i].q3};

        float q_ref_conj[4] = {q_ref[i][0], -q_ref[i][1], -q_ref[i][2], -q_ref[i][3]};
        float q_delta[4];
        math3d_quat_multiply(q_ref_conj, q_now, q_delta);
        /* Fix: use Swing-Twist decomposition — no gimbal lock at pitch ±90° */
        math3d_quat_delta_to_pitch_roll_deg(q_delta, &pitch_deg, &roll_deg);
#endif
        _valid = 1;
      }

      if (_valid)
      {
#if IMU1_RAW_ACCEL
        if (i == 1)
        {
          /* IMU 1: purely raw — only angle-to-µs conversion + output clamping.
           * No deadzone, no EMA, no deadband. The servo limits are the only
           * constraint applied (same physical limits as IMU 0). */
          final_pitch_deg[1] = pitch_deg;
          final_roll_deg[1]  = roll_deg;
          float p_us1 = 1500.0f + ((pitch_deg * PITCH_SERVO_DIR) * (2000.0f / 180.0f));
          float r_us1 = 1500.0f + (roll_deg * (2000.0f / 180.0f));
          smoothed_pitch[1] = fmaxf(1055.6f, fminf(1944.4f, p_us1));
          smoothed_roll[1]  = fmaxf(500.0f,  fminf(2500.0f, r_us1));
        }
        else
#endif
        {
          /* IMU 0 (or IMU 1 when IMU1_RAW_ACCEL=0): full deadzone + EMA pipeline */
#define PITCH_DEADZONE_DEG 2.0f
#define ROLL_DEADZONE_DEG 2.0f

          if (fabsf(pitch_deg) < PITCH_DEADZONE_DEG)
            pitch_deg = 0.0f;
          else
            pitch_deg = (pitch_deg > 0) ? (pitch_deg - PITCH_DEADZONE_DEG)
                                        : (pitch_deg + PITCH_DEADZONE_DEG);

          if (fabsf(roll_deg) < ROLL_DEADZONE_DEG)
            roll_deg = 0.0f;
          else
            roll_deg = (roll_deg > 0) ? (roll_deg - ROLL_DEADZONE_DEG)
                                      : (roll_deg + ROLL_DEADZONE_DEG);

          final_pitch_deg[i] = pitch_deg;
          final_roll_deg[i] = roll_deg;


          float pitch_servo_us = 1500.0f + ((pitch_deg * PITCH_SERVO_DIR) * (2000.0f / 180.0f));
          uint32_t ccr_pitch = (uint32_t)fmaxf(1055.6f, fminf(1944.4f, pitch_servo_us));

          float roll_servo_us = 1500.0f + (roll_deg * (2000.0f / 180.0f));
          uint32_t ccr_roll = (uint32_t)fmaxf(500.0f, fminf(2500.0f, roll_servo_us));

#define SERVO_EMA_ALPHA_PITCH 0.1f
#define SERVO_EMA_ALPHA_ROLL 0.1f
#define SERVO_DEADBAND_US_PITCH 2.0f
#define SERVO_DEADBAND_US_ROLL 2.0f

          float err_pitch = (float)ccr_pitch - smoothed_pitch[i];
          if (fabsf(err_pitch) > SERVO_DEADBAND_US_PITCH)
          {
            smoothed_pitch[i] += err_pitch * SERVO_EMA_ALPHA_PITCH;
          }

          float err_roll = (float)ccr_roll - smoothed_roll[i];
          if (fabsf(err_roll) > SERVO_DEADBAND_US_ROLL)
          {
            smoothed_roll[i] += err_roll * SERVO_EMA_ALPHA_ROLL;
          }
        }
      }
    }

    perf_timer_stop(PERF_SLOT_SWING_TWIST, _t_st); /* end of swing-twist block */

    /* ── Thesis measurement 2: Servo PWM register write time ── */
    uint32_t _t_pwm = perf_timer_start();
#if ROBOT_MODE == ROBOT_MODE_FULL
    int arm_idx = (NUM_LEGS * 2);
    if (NUM_ARMS >= 1 && arm_idx < NUM_IMUS) {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)smoothed_pitch[arm_idx]);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)smoothed_roll[arm_idx]);
    }
    if (NUM_ARMS >= 2 && (arm_idx + 1) < NUM_IMUS) {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)smoothed_pitch[arm_idx + 1]);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)smoothed_roll[arm_idx + 1]);
    }
#else
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)smoothed_pitch[0]);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)smoothed_roll[0]);
    if (NUM_IMUS > 1)
    {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)smoothed_pitch[1]);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)smoothed_roll[1]);
    }
#endif
    perf_timer_stop(PERF_SLOT_SERVO_PWM, _t_pwm);

    /* ── Thesis measurement 3: Gripper update time ── */
    uint32_t _t_grip = perf_timer_start();
    gripper_update();
    perf_timer_stop(PERF_SLOT_GRIPPER, _t_grip);
#endif

    {
      SystemState_t disp_state;
#if ROBOT_MODE == ROBOT_MODE_ARM
      disp_state.pitch_0 = final_pitch_deg[0];
      disp_state.roll_0 = final_roll_deg[0];
      disp_state.servo_pitch_us_0 = (uint32_t)smoothed_pitch[0];
      disp_state.servo_roll_us_0 = (uint32_t)smoothed_roll[0];
      if (NUM_IMUS > 1)
      {
        disp_state.pitch_1 = final_pitch_deg[1];
        disp_state.roll_1 = final_roll_deg[1];
        disp_state.servo_pitch_us_1 = (uint32_t)smoothed_pitch[1];
        disp_state.servo_roll_us_1 = (uint32_t)smoothed_roll[1];
      }
      else
      {
        disp_state.pitch_1 = 0.0f;
        disp_state.roll_1 = 0.0f;
        disp_state.servo_pitch_us_1 = 1500U;
        disp_state.servo_roll_us_1 = 1500U;
      }
#else
      /* In Leg Mode or Full Mode, pull the tared bone-frame angles from leg 1 state */
      disp_state.pitch_0 = g_leg1_state.thigh_pitch;
      disp_state.roll_0 = g_leg1_state.knee_angle; /* using roll_0 field to show knee angle on the screen */
      disp_state.servo_pitch_us_0 = 1500U;
      disp_state.servo_roll_us_0 = 1500U;
#if ROBOT_MODE == ROBOT_MODE_FULL
      int arm_idx = (NUM_LEGS * 2);
      if (arm_idx < NUM_IMUS) {
        disp_state.pitch_1 = final_pitch_deg[arm_idx];
        disp_state.roll_1 = final_roll_deg[arm_idx];
        disp_state.servo_pitch_us_1 = (uint32_t)smoothed_pitch[arm_idx];
        disp_state.servo_roll_us_1 = (uint32_t)smoothed_roll[arm_idx];
      } else {
        disp_state.pitch_1 = 0.0f;
        disp_state.roll_1 = 0.0f;
        disp_state.servo_pitch_us_1 = 1500U;
        disp_state.servo_roll_us_1 = 1500U;
      }
#else
      if (NUM_IMUS > 1) {
        disp_state.pitch_1 = g_leg1_state.shin_pitch;
        disp_state.roll_1 = 0.0f;
      } else {
        disp_state.pitch_1 = 0.0f;
        disp_state.roll_1 = 0.0f;
      }
      disp_state.servo_pitch_us_1 = 1500U;
      disp_state.servo_roll_us_1 = 1500U;
#endif
#endif
      /* Flash RE-ZEROED on display for 1 second after button press */
      disp_state.status = (rezero_flash_count > 0U) ? SYS_STATUS_REZEROED
                                                    : SYS_STATUS_RUNNING;
      /* ── Thesis measurement 5: Display queue post time ── */
      uint32_t _t_disp = perf_timer_start();
      osMessageQueuePut(displayQueueHandle, &disp_state, 0U, 0U);
      perf_timer_stop(PERF_SLOT_DISP_QUEUE, _t_disp);
    }

#if STACK_TUNING_MODE
    if (++tick_count >= 1000)
    {
      tick_count = 0;
      s_imu_hwm = uxTaskGetStackHighWaterMark(NULL);
    }
#endif

#ifdef DEBUG
    UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
    if (hwm < 100)
    {
      __BKPT(0);
    }
#endif

    /* ── Thesis perf report: prints table every ~1 s when PERF ON ── */
    perf_timer_report_if_due();
  }
}
