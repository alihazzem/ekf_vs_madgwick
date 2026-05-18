#ifndef DRIVERS_GRIPPER_H_
#define DRIVERS_GRIPPER_H_

#include "stm32f4xx_hal.h"

// ── Hardware pins ──
// D0 -> TIM4_CH1 (PWM)
#define GRIPPER_D0_PIN        GPIO_PIN_6
#define GRIPPER_D0_PORT       GPIOB
// D1 -> TIM4_CH2 (PWM)
#define GRIPPER_D1_PIN        GPIO_PIN_7
#define GRIPPER_D1_PORT       GPIOB

// ── Timing ──
#define GRIPPER_HOMING_MS     1000   // open direction run time at startup
#define GRIPPER_CLOSE_MS      1000   // full close travel time at 50% PWM
#define GRIPPER_OPEN_MS       1000   // full open travel time at 50% PWM

// ── Fixed speed ──
#define GRIPPER_PWM_DUTY      50    // never change this

// ── Gripper states ──
typedef enum {
  GRIPPER_IDLE_OPEN,
  GRIPPER_CLOSING,
  GRIPPER_IDLE_CLOSED,
  GRIPPER_OPENING
} GripperState_t;

// ── Public API ──
void gripper_init(void);    // init GPIO and PWM timer
void gripper_home(void);    // blocking homing sequence, call once at startup
void gripper_update(void);  // call every main loop iteration

#endif // DRIVERS_GRIPPER_H_
