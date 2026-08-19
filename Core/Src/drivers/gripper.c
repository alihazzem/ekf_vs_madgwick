#include "drivers/gripper.h"
#include "app/app_config.h"  /* NUM_DC_MOTORS */
#include "cmsis_os.h"  /* For osDelay */
#include "drivers/emg_uart.h"  /* For g_emg_speed and g_emg_speed_valid */

/* State variables */
static GripperState_t s_gripper_state = GRIPPER_IDLE_OPEN;
static uint32_t s_state_start_ms = 0;
static uint8_t s_prev_emg_state = 0; /* 0 = RELAXED, 1 = ACTIVE */

/* 10ms stop delay state machine */
static uint8_t s_reversing = 0;
static uint32_t s_reverse_stop_ms = 0;
static GripperState_t s_pending_state = GRIPPER_IDLE_OPEN;

/* Hardware abstraction for motor states */
static void motor_stop(void) {
  /* Motor 1 (CH1/CH2) and Motor 2 (CH3/CH4) — both stopped */
  TIM4->CCR1 = 0;
  TIM4->CCR2 = 0;
#if NUM_DC_MOTORS > 1
  TIM4->CCR3 = 0;
  TIM4->CCR4 = 0;
#endif
}

static void motor_forward(uint16_t pwm) {
  /* Both motors forward: A-side=LOW, B-side=PWM */
  TIM4->CCR1 = 0;
  TIM4->CCR2 = pwm;
#if NUM_DC_MOTORS > 1
  TIM4->CCR3 = 0;
  TIM4->CCR4 = pwm;
#endif
}

static void motor_backward(uint16_t pwm) {
  /* Both motors backward: A-side=PWM, B-side=LOW */
  TIM4->CCR1 = pwm;
  TIM4->CCR2 = 0;
#if NUM_DC_MOTORS > 1
  TIM4->CCR3 = pwm;
  TIM4->CCR4 = 0;
#endif
}

void gripper_init(void) {
  /* Enable GPIOB and TIM4 clocks */
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

  /* Configure PB6 as Alternate Function 2 (TIM4_CH1) */
  GPIOB->MODER &= ~(3U << (6 * 2));
  GPIOB->MODER |=  (2U << (6 * 2)); /* Alternate function mode */
  GPIOB->OSPEEDR |= (3U << (6 * 2)); /* High speed */
  GPIOB->PUPDR &= ~(3U << (6 * 2)); /* No pull-up, no pull-down */
  GPIOB->AFR[0] &= ~(0xFU << (6 * 4)); /* AF02 for TIM4_CH1 */
  GPIOB->AFR[0] |=  (2U << (6 * 4));

  /* Configure PB7 as Alternate Function 2 (TIM4_CH2) */
  GPIOB->MODER &= ~(3U << (7 * 2));
  GPIOB->MODER |=  (2U << (7 * 2)); /* Alternate function mode */
  GPIOB->OSPEEDR |= (3U << (7 * 2)); /* High speed */
  GPIOB->PUPDR &= ~(3U << (7 * 2)); /* No pull-up, no pull-down */
  GPIOB->AFR[0] &= ~(0xFU << (7 * 4)); /* AF02 for TIM4_CH2 */
  GPIOB->AFR[0] |=  (2U << (7 * 4));

  /* Initialize TIM4 for PWM */
  TIM4->CR1 = 0; /* Disable counter, upcounting mode */
  TIM4->PSC = 84 - 1; /* 1MHz clock (assuming 84MHz APB1 timer clock) */
  TIM4->ARR = 20000 - 1; /* 50Hz PWM frequency (20ms period) */
  
  /* Configure Channel 1 for PWM mode 1 */
  TIM4->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
  TIM4->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos); /* PWM mode 1 */
  TIM4->CCMR1 |= TIM_CCMR1_OC1PE; /* Preload enable */
  
  /* Configure Channel 2 for PWM mode 1 */
  TIM4->CCMR1 &= ~TIM_CCMR1_OC2M_Msk;
  TIM4->CCMR1 |= (6U << TIM_CCMR1_OC2M_Pos); /* PWM mode 1 */
  TIM4->CCMR1 |= TIM_CCMR1_OC2PE; /* Preload enable */
  
  /* Initial Duty Cycle 0% */
  TIM4->CCR1 = 0;
  TIM4->CCR2 = 0;
  
  /* Enable Capture/Compare for Channel 1 and 2 */
  TIM4->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;
  /* Note: TIM4 is a general purpose timer, so no BDTR register / MOE bit is needed */
  
#if NUM_DC_MOTORS > 1
  /* Configure PB8 as Alternate Function 2 (TIM4_CH3) */
  GPIOB->MODER &= ~(3U << (8 * 2));
  GPIOB->MODER |=  (2U << (8 * 2)); /* Alternate function mode */
  GPIOB->OSPEEDR |= (3U << (8 * 2)); /* High speed */
  GPIOB->PUPDR &= ~(3U << (8 * 2)); /* No pull */
  GPIOB->AFR[1] &= ~(0xFU << ((8 - 8) * 4)); /* PB8 is in AFR[1] (pins 8-15) */
  GPIOB->AFR[1] |=  (2U  << ((8 - 8) * 4)); /* AF02 for TIM4_CH3 */

  /* Configure PB9 as Alternate Function 2 (TIM4_CH4) */
  GPIOB->MODER &= ~(3U << (9 * 2));
  GPIOB->MODER |=  (2U << (9 * 2)); /* Alternate function mode */
  GPIOB->OSPEEDR |= (3U << (9 * 2)); /* High speed */
  GPIOB->PUPDR &= ~(3U << (9 * 2)); /* No pull */
  GPIOB->AFR[1] &= ~(0xFU << ((9 - 8) * 4));
  GPIOB->AFR[1] |=  (2U  << ((9 - 8) * 4)); /* AF02 for TIM4_CH4 */

  /* Configure Channel 3 for PWM mode 1 (CCMR2 bits [6:4]) */
  TIM4->CCMR2 &= ~TIM_CCMR2_OC3M_Msk;
  TIM4->CCMR2 |= (6U << TIM_CCMR2_OC3M_Pos); /* PWM mode 1 */
  TIM4->CCMR2 |= TIM_CCMR2_OC3PE; /* Preload enable */

  /* Configure Channel 4 for PWM mode 1 (CCMR2 bits [14:12]) */
  TIM4->CCMR2 &= ~TIM_CCMR2_OC4M_Msk;
  TIM4->CCMR2 |= (6U << TIM_CCMR2_OC4M_Pos); /* PWM mode 1 */
  TIM4->CCMR2 |= TIM_CCMR2_OC4PE; /* Preload enable */

  /* Initial Duty Cycle 0% for CH3 and CH4 */
  TIM4->CCR3 = 0;
  TIM4->CCR4 = 0;

  /* Enable Capture/Compare outputs for CH3 and CH4 */
  TIM4->CCER |= TIM_CCER_CC3E | TIM_CCER_CC4E;
#endif /* NUM_DC_MOTORS > 1 */

  /* Start TIM4 */
  TIM4->CR1 |= TIM_CR1_CEN;

  s_gripper_state = GRIPPER_IDLE_OPEN;
  s_prev_emg_state = 0;
  s_reversing = 0;
}

void gripper_home(void) {
  /* Run motor in OPEN direction (backward) */
  motor_backward(10000);
  
  /* Wait for full travel time */
  osDelay(pdMS_TO_TICKS(GRIPPER_HOMING_MS));
  
  /* Stop motor */
  motor_stop();
  
  /* Settle delay */
  osDelay(pdMS_TO_TICKS(200));
  
  s_gripper_state = GRIPPER_IDLE_OPEN;
}

void gripper_update(void) {
  uint32_t now = HAL_GetTick();
  uint8_t current_emg_state = 0;

  /* Check for physical disconnect or frozen sender timeout */
  if (g_emg_speed_valid && (now - g_emg_last_rx_ms) >= 500) {
      g_emg_speed_valid = false;
  }

  /* STEP 1: Determine EMG binary state */
  if (g_emg_speed_valid && g_emg_speed > 0) {
    current_emg_state = 1; /* ACTIVE */
  } else {
    current_emg_state = 0; /* RELAXED */
  }

  uint8_t rising_edge = (s_prev_emg_state == 0 && current_emg_state == 1);
  uint8_t falling_edge = (s_prev_emg_state == 1 && current_emg_state == 0);

  /* Delay state machine for safe reversing (10ms stop) */
  if (s_reversing) {
    if (now - s_reverse_stop_ms >= 10) {
      s_reversing = 0;
      s_gripper_state = s_pending_state;
      s_state_start_ms = now;
      if (s_gripper_state == GRIPPER_CLOSING) {
        uint16_t pwm = (g_emg_speed * 20000) / 100;
        motor_forward(pwm);
      } else if (s_gripper_state == GRIPPER_OPENING) {
        motor_backward(10000);
      }
    } else {
      /* Waiting for 10ms stop to finish. Update state and return. */
      s_prev_emg_state = current_emg_state;
      return; 
    }
  }

  /* STEP 2: Run state machine */
  switch (s_gripper_state) {
    case GRIPPER_IDLE_OPEN:
      if (rising_edge) {
        /* User flexed: start closing */
        s_reversing = 1;
        s_reverse_stop_ms = now;
        s_pending_state = GRIPPER_CLOSING;
        motor_stop();
      }
      break;

    case GRIPPER_CLOSING:
      if (now - s_state_start_ms >= GRIPPER_CLOSE_MS) {
        /* Finished closing */
        motor_stop();
        s_gripper_state = GRIPPER_IDLE_CLOSED;
      } else if (falling_edge) {
        /* User relaxed before fully closed. Reverse direction. */
        s_reversing = 1;
        s_reverse_stop_ms = now;
        s_pending_state = GRIPPER_OPENING;
        motor_stop();
      } else {
        /* Continuously update PWM based on EMG speed */
        uint16_t pwm = (g_emg_speed * 20000) / 100;
        motor_forward(pwm);
      }
      break;

    case GRIPPER_IDLE_CLOSED:
      if (falling_edge) {
        /* User relaxed: start opening */
        s_reversing = 1;
        s_reverse_stop_ms = now;
        s_pending_state = GRIPPER_OPENING;
        motor_stop();
      }
      break;

    case GRIPPER_OPENING:
      if (now - s_state_start_ms >= GRIPPER_OPEN_MS) {
        /* Finished opening */
        motor_stop();
        s_gripper_state = GRIPPER_IDLE_OPEN;
      } else if (rising_edge) {
        /* User flexed before fully open. Reverse direction. */
        s_reversing = 1;
        s_reverse_stop_ms = now;
        s_pending_state = GRIPPER_CLOSING;
        motor_stop();
      }
      break;
  }

  /* STEP 3: Update previous state for edge detection */
  s_prev_emg_state = current_emg_state;
}
