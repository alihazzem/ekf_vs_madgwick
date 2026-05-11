#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

#include "cmsis_os.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DIR_STOP = 0,
    DIR_FORWARD = 1,
    DIR_BACKWARD = 2
} MotorDir_t;

typedef struct {
    MotorDir_t dirA;
    MotorDir_t dirB;
    uint16_t   speedA;
    uint16_t   speedB;
    uint8_t    emg_speed;
} MotorCmd_t;

extern osMessageQueueId_t motorCmdQueueHandle;

void motor_test_task_fn(void *arg);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_TEST_H */
