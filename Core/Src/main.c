/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "app/app_config.h"
#include "app/cli_app.h"
#include "app/imu_app.h"
#include "drivers/uart_cli.h"
#include "task.h"
#include "utils/timebase.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

// #include "motor_test.h"
#include "app/display_task.h" /* SystemState_t, displayQueueHandle */
#include "drivers/emg_uart.h"
#include "drivers/gripper.h"
#include "utils/math3d.h"
#include "app/perf_timer.h" /* DWT execution-time profiler */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BYPASS_IMU_FILTER 0
#define PITCH_SERVO_DIR -1.0f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2; /* OLED SSD1306 dedicated bus */

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* USER CODE BEGIN PV */
osThreadId_t imuTaskHandle;
// osMessageQueueId_t motorCmdQueueHandle;

/* Re-zero button: set to 1 by cli_task_fn (button ISR poll), cleared by
 * imu_task_fn. volatile ensures both tasks see updates without a mutex
 * (single-byte atomic write). */
volatile uint8_t g_rezero_requested = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void); /* OLED SSD1306 I2C bus */
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
static void imu_task_fn(void *arg);
static void cli_task_fn(void *arg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static inline float clampf(float v, float lo, float hi)
{
  return fminf(hi, fmaxf(lo, v));
}

static inline void quat_to_tilt_deg(const float q[4], float *pitch_deg, float *roll_deg)
{
  float gx = 2.0f * (q[1] * q[3] - q[0] * q[2]);
  float gy = 2.0f * (q[0] * q[1] + q[2] * q[3]);
  float gz = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

  gx = clampf(gx, -1.0f, 1.0f);

  if (pitch_deg)
  {
    *pitch_deg = atan2f(-gx, sqrtf(gy * gy + gz * gz)) * (180.0f / 3.14159265f);
  }
  if (roll_deg)
  {
    *roll_deg = atan2f(gy, gz) * (180.0f / 3.14159265f);
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init(); /* OLED SSD1306 must be ready before scheduler starts */
  MX_TIM2_Init();
  MX_USART2_UART_Init();

  /* Deselect onboard SPI Flash (PA4=CS) so it releases PA6 (MISO) / PA7 (MOSI).
   * On the Black Pill the flash shares these pins with TIM3_CH1/CH2.         */
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef flash_cs = {0};
    flash_cs.Pin = GPIO_PIN_4;
    flash_cs.Mode = GPIO_MODE_OUTPUT_PP;
    flash_cs.Pull = GPIO_NOPULL;
    flash_cs.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &flash_cs);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  }

  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  gripper_init();
  app_cli_print_banner();
  uart_cli_init(&huart2);
  timebase_init();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  // motorCmdQueueHandle = osMessageQueueNew(10, sizeof(MotorCmd_t), NULL);

  /* Display queue: depth=1 so the IMU task always drops stale frames.
   * osMessageQueuePut with timeout=0 returns immediately if full —
   * the real-time loop is never blocked by the low-priority display task. */
  displayQueueHandle = osMessageQueueNew(1, sizeof(SystemState_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle =
      osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  imuTaskHandle = osThreadNew(imu_task_fn, NULL,
                              &(osThreadAttr_t){.name = "IMU",
                                                .priority = osPriorityRealtime,
                                                .stack_size = 512 * 4});

  osThreadNew(cli_task_fn, NULL,
              &(osThreadAttr_t){.name = "CLI",
                                .priority = osPriorityLow,
                                .stack_size = 512 * 4});

  /* Display task: low priority, 512-word stack, renders OLED at ~10 Hz */
  osThreadNew(display_task_fn, NULL,
              &(osThreadAttr_t){.name = "DISP",
                                .priority = osPriorityLow,
                                .stack_size = 512 * 4});

  // osThreadNew(motor_test_task_fn, NULL, &(osThreadAttr_t){
  //     .name       = "MotorTest",
  //     .priority   = osPriorityNormal,
  //     .stack_size = 256 * 4
  // });
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8399;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  /* Timer clock = 84MHz / (8399+1) = 10,000 Hz.
   * Period = (10000 / IMU_FS_HZ) - 1 */
  htim2.Init.Period = (uint32_t)(10000.0f / IMU_FS_HZ) - 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* Re-zero button on PA0: INPUT, internal pull-up, active-low.
   * Wire the button between PA0 and GND. No external resistor needed. */
  GPIO_InitTypeDef btn = {0};
  btn.Pin = GPIO_PIN_0;
  btn.Mode = GPIO_MODE_INPUT;
  btn.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &btn);
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART1)
  {
    /* ── Thesis measurement 1: EMG UART parse time ── */
    uint32_t _t_emg = perf_timer_start();
    emg_uart_on_rx_event(huart, Size);
    perf_timer_stop(PERF_SLOT_EMG_PARSE, _t_emg);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    emg_uart_recover();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uart_cli_on_rx_byte(huart);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;
  __BKPT(0);
}

/* USER CODE END 4 */

#if STACK_TUNING_MODE
static volatile UBaseType_t s_imu_hwm = 0;
#endif

static void imu_task_fn(void *arg)
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

  /* --- Init Servos to Safe Lock Position during calibration --- */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500); // Pitch 1
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 500);  // Roll 1
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1500); // Pitch 2
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 500);  // Roll 2

  /* --- Now begin the lengthy IMU initialization and calibration --- */
  imu_app_stream_set(false);
  imu_app_init(&hi2c1);
  imu_app_ekf_reset();
  imu_app_madgwick_reset();
  imu_app_cal_gyro(4000);
  imu_app_stream_set(true);

  emg_uart_init(&huart1);

  /* Let EKF converge (~50 samples ≈ 250 ms at 200 Hz) */
  HAL_TIM_Base_Start_IT(&htim2);
  Attitude_t ekf_att[NUM_IMUS];
  for (int i = 0; i < 50; i++)
  {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
    imu_app_step();
  }

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

      /* Swing-Twist decomposition (twist axis = X / roll).
       * q_delta = q_ref_conj * q_now  →  left-compose convention
       * Therefore: q_delta = q_twist * q_swing
       *            q_swing = q_twist_conj * q_delta  (left-multiply) */
      float twist_seed_w = q_delta_seed[0];
      float twist_seed_x = q_delta_seed[1];
      float twist_seed_norm = sqrtf(twist_seed_w * twist_seed_w + twist_seed_x * twist_seed_x);
      float q_twist_seed[4];
      if (twist_seed_norm < 1e-6f)
      {
        q_twist_seed[0] = 1.0f;
        q_twist_seed[1] = 0.0f;
        q_twist_seed[2] = 0.0f;
        q_twist_seed[3] = 0.0f;
      }
      else
      {
        q_twist_seed[0] = twist_seed_w / twist_seed_norm;
        q_twist_seed[1] = twist_seed_x / twist_seed_norm;
        q_twist_seed[2] = 0.0f;
        q_twist_seed[3] = 0.0f;
      }
      float q_twist_seed_conj[4] = {q_twist_seed[0], -q_twist_seed[1], 0.0f, 0.0f};
      float q_swing_seed[4];
      math3d_quat_multiply(q_twist_seed_conj, q_delta_seed, q_swing_seed); /* q_swing = q_twist_conj * q_delta */

      float w = q_swing_seed[0], x = q_swing_seed[1], y = q_swing_seed[2], z = q_swing_seed[3];
      float gx = 2.0f * (x * z - w * y);
      gx = clampf(gx, -1.0f, 1.0f);
      float _p = asinf(-gx) * (180.0f / 3.14159265f);
      float _r = 2.0f * atan2f(q_twist_seed[1], q_twist_seed[0]) * (180.0f / 3.14159265f);
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

    if (g_rezero_requested)
    {
      g_rezero_requested = 0; /* Clear FIRST to avoid double-trigger */
      for (int i = 0; i < NUM_IMUS; i++)
      {
#if IMU1_RAW_ACCEL
        if (i == 1)
        {
          /* Capture current raw tilt as the new zero reference for IMU 1 */
          float ax1r, ay1r, az1r;
          imu_app_get_accel_g(1, &ax1r, &ay1r, &az1r);
          raw_ref_pitch_1 = atan2f(-ax1r, sqrtf(ay1r * ay1r + az1r * az1r)) * (180.0f / 3.14159265f);
          raw_ref_roll_1  = atan2f(ay1r, az1r) * (180.0f / 3.14159265f);
          continue;
        }
#endif
        if (imu_app_get_ekf(i, &ekf_att[i]))
        {
          q_ref[i][0] = ekf_att[i].q0;
          q_ref[i][1] = ekf_att[i].q1;
          q_ref[i][2] = ekf_att[i].q2;
          q_ref[i][3] = ekf_att[i].q3;
        }
      }
      rezero_flash_count = 200U; /* Flash display for 1 second */
    }
    if (rezero_flash_count > 0U)
      rezero_flash_count--;

    float final_pitch_deg[NUM_IMUS] = {0};
    float final_roll_deg[NUM_IMUS] = {0};

    /* ── Thesis measurement 4: Tilt-only + deadzone + EMA block ── */
    uint32_t _t_st = perf_timer_start();

    for (int i = 0; i < NUM_IMUS; i++)
    {
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
        quat_to_tilt_deg(q_delta, &pitch_deg, &roll_deg);
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
#define PITCH_DEADZONE_DEG 1.0f
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
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, (uint32_t)smoothed_pitch[0]);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, (uint32_t)smoothed_roll[0]);
    if (NUM_IMUS > 1)
    {
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint32_t)smoothed_pitch[1]);
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, (uint32_t)smoothed_roll[1]);
    }
    perf_timer_stop(PERF_SLOT_SERVO_PWM, _t_pwm);

    /* ── Thesis measurement 3: Gripper update time ── */
    uint32_t _t_grip = perf_timer_start();
    gripper_update();
    perf_timer_stop(PERF_SLOT_GRIPPER, _t_grip);

    {
      SystemState_t disp_state;
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
      /* Flash RE-ZEROED on display for 1 second after button press */
      disp_state.status = (rezero_flash_count > 0U) ? SYS_STATUS_REZEROED
                                                    : SYS_STATUS_RUNNING;
      /* ── Thesis measurement 5: Display queue post time ── */
      uint32_t _t_disp = perf_timer_start();
      osMessageQueuePut(displayQueueHandle, &disp_state, 0U, 0U);
      perf_timer_stop(PERF_SLOT_DISP_QUEUE, _t_disp);
    }

#if 0
      /* ── EMG → Speed ── */
      uint16_t   speed   = 0;
      uint8_t    emg_raw = 0;
      uint32_t   now_ms  = HAL_GetTick();

      if (g_emg_speed_valid && (now_ms - g_emg_last_rx_ms) < EMG_SPEED_TIMEOUT_MS)
      {
        emg_raw = g_emg_speed;
        speed = (uint16_t)(((uint32_t)emg_raw * 999u) / 100u);
      }

      if (!g_emg_speed_valid && (now_ms - g_emg_last_rx_ms) >= 3000)
      {
        emg_uart_recover();
      }

      /* ── Pitch → Direction ── */
      MotorDir_t dir   = DIR_STOP;
      float abs_p = (pitch_val < 0.0f) ? -pitch_val : pitch_val;

      if (abs_p >= 0.0436f) {  /* 5° deadzone */
        dir = (pitch_val > 0.0f) ? DIR_FORWARD : DIR_BACKWARD;
      }

      if (speed == 0) {
        dir = DIR_STOP;
      }

      /* ── Roll → Steering ── */
      cmd.speedA = speed;
      cmd.speedB = speed;
      cmd.dirA   = dir;
      cmd.dirB   = dir;
      cmd.emg_speed = emg_raw;

      if (roll_val > 0.0872f) {
        /* Tilted right → disable right motor (B) */
        cmd.dirB   = DIR_STOP;
        cmd.speedB = 0;
      } else if (roll_val < -0.0872f) {
        /* Tilted left  → disable left motor (A) */
        cmd.dirA   = DIR_STOP;
        cmd.speedA = 0;
      }

      osMessageQueuePut(motorCmdQueueHandle, &cmd, 0, 0);
#endif
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

static void cli_task_fn(void *arg)
{
  (void)arg;
  uint32_t tick_count = 0;
  (void)tick_count;

  /* Button debounce state */
  uint8_t btn_prev = 1;      /* 1 = not pressed (pull-up idle) */
  uint32_t btn_press_ms = 0; /* timestamp of when press started */

  while (1)
  {
    uart_cli_poll();
    osDelay(5); /* 5ms poll interval */

    /* ── Re-zero button poll (PA0, active-low, 50ms debounce) ── */
    uint8_t btn_now = (uint8_t)HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

    if (btn_prev == 1 && btn_now == 0)
    {
      /* Falling edge: button just pressed */
      btn_press_ms = HAL_GetTick();
    }
    else if (btn_prev == 0 && btn_now == 1)
    {
      /* Rising edge: button released */
      uint32_t held_ms = HAL_GetTick() - btn_press_ms;
      if (held_ms >= 50U && held_ms < 2000U)
      {
        /* Valid short press (50ms–2000ms) → request re-zero */
        g_rezero_requested = 1;
      }
    }
    btn_prev = btn_now;

#if STACK_TUNING_MODE
    if (++tick_count >= 200)
    {
      tick_count = 0;
      UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
      char buf[128];
      snprintf(buf, sizeof(buf),
               "[CLI] HWM: %lu words  |  [IMU] HWM: %lu words\r\n",
               (unsigned long)hwm, (unsigned long)s_imu_hwm);
      HAL_UART_Transmit(&huart2, (uint8_t *)buf, strlen(buf), 100);
    }
#endif
  }
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  IMU Task implementation.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM1 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if (htim->Instance == TIM2)
  {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(imuTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @brief I2C2 Initialization Function (OLED SSD1306 dedicated bus)
 *
 * Pins (STM32F411CEU6 48-pin Black Pill):
 *   PB10 → I2C2_SCL  (AF4, Open-Drain, pull-up)
 *   PB3  → I2C2_SDA  (AF9, Open-Drain, pull-up)
 *
 * Speed: 400 kHz (Fast-Mode). The SSD1306 supports up to 400 kHz.
 */
static void MX_I2C2_Init(void)
{
  /* Enable clocks */
  __HAL_RCC_I2C2_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure PB10 (SCL) — AF4 */
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_10;
  gpio.Mode = GPIO_MODE_AF_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOB, &gpio);

  /* Configure PB3 (SDA) — AF9 (different AF from SCL on this package!) */
  gpio.Pin = GPIO_PIN_3;
  gpio.Alternate = GPIO_AF9_I2C2;
  HAL_GPIO_Init(GPIOB, &gpio);

  /* Configure I2C2 peripheral */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000U; /* 400 kHz Fast-Mode */
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
}
