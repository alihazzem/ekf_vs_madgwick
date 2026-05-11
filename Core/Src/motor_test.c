#include "motor_test.h"
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

extern TIM_HandleTypeDef htim3;

#ifndef IN1_Pin
#define IN1_Pin GPIO_PIN_12
#define IN1_GPIO_Port GPIOB
#define IN2_Pin GPIO_PIN_13
#define IN2_GPIO_Port GPIOB
#define IN3_Pin GPIO_PIN_14
#define IN3_GPIO_Port GPIOB
#define IN4_Pin GPIO_PIN_15
#define IN4_GPIO_Port GPIOB
#endif

static void motor_gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable clock for GPIOB where the pins are located
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Set initial state to RESET (low)
    HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);

    // Configure the pins as Push-Pull outputs
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = IN1_Pin;
    HAL_GPIO_Init(IN1_GPIO_Port, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = IN2_Pin;
    HAL_GPIO_Init(IN2_GPIO_Port, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = IN3_Pin;
    HAL_GPIO_Init(IN3_GPIO_Port, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = IN4_Pin;
    HAL_GPIO_Init(IN4_GPIO_Port, &GPIO_InitStruct);
}

static void motor_set_dir_a(MotorDir_t dir)
{
    switch (dir) {
    case DIR_FORWARD:
        HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
        break;
    case DIR_BACKWARD:
        HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
        break;
    default:
        HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
        break;
    }
}

static void motor_set_dir_b(MotorDir_t dir)
{
    switch (dir) {
    case DIR_FORWARD:
        HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
        break;
    case DIR_BACKWARD:
        HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
        break;
    default:
        HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
        break;
    }
}

void motor_test_task_fn(void *arg)
{
    (void)arg;

    motor_gpio_init();

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    MotorCmd_t cmd;
    while (1)
    {
        if (osMessageQueueGet(motorCmdQueueHandle, &cmd, NULL, osWaitForever) == osOK)
        {
            motor_set_dir_a(cmd.dirA);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, cmd.speedA);

            motor_set_dir_b(cmd.dirB);
            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, cmd.speedB);
        }
    }
}
