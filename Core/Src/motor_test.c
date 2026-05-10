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

void motor_test_task_fn(void *arg)
{
    (void)arg;

    // Manually initialize directional GPIOs
    motor_gpio_init();

    // Start PWM on TIM3 Channels 1 and 2 (for ENA and ENB)
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    while (1)
    {
        // 1. Motors Forward at 100% speed
        // Set ENA and ENB to 100% (PWM = 1000)
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);

        // Motor A Forward (IN1=1, IN2=0)
        HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
        
        // Motor B Forward (IN3=1, IN4=0)
        HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
        
        osDelay(2000);

        // 2. Stop
        // Set ENA and ENB to 0%
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
        
        // Disable directional pins
        HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
        
        osDelay(1000);

        // 3. Motors Backward at 100% speed
        // Set ENA and ENB to 100%
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);

        // Motor A Backward (IN1=0, IN2=1)
        HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
        
        // Motor B Backward (IN3=0, IN4=1)
        HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
        
        osDelay(2000);

        // 4. Stop
        // Set ENA and ENB to 0%
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);

        // Disable directional pins
        HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);

        osDelay(1000);
    }
}
