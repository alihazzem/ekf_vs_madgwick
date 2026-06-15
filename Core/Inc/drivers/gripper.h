#ifndef DRIVERS_GRIPPER_H_
#define DRIVERS_GRIPPER_H_

#include "stm32f4xx_hal.h"

// ── Hardware pins ──
// D0 -> TIM4_CH1 (PWM, PB6)  – gripper / motor 1
#define GRIPPER_D0_PIN        GPIO_PIN_6
#define GRIPPER_D0_PORT       GPIOB
// D1 -> TIM4_CH2 (PWM, PB7)  – gripper / motor 1
#define GRIPPER_D1_PIN        GPIO_PIN_7
#define GRIPPER_D1_PORT       GPIOB
// TIM4_CH3 (PB8) and TIM4_CH4 (PB9) – motor 2, mirrors motor 1 automatically

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
void gripper_init(void);    // init GPIO + PWM for both motors
void gripper_home(void);    // blocking homing sequence, call once at startup
void gripper_update(void);  // call every main loop iteration

#endif // DRIVERS_GRIPPER_H_
