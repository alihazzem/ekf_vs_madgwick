/**
 * @file leg_telemetry.h
 * @brief UART6 telemetry transmitter: sends joint angles to the ESP32.
 *
 * Active only when ROBOT_MODE == ROBOT_MODE_LEG or ROBOT_MODE_FULL.
 * TX pin: PA11 (USART6_TX, AF8), 115200 8N1, TX-only.
 *
 * Packet format (15 bytes, packed):
 *   [0xAA] [0x55] [Thigh_L*100] [Shin_L*100] [Knee_L*100] [Thigh_R*100] [Shin_R*100] [Knee_R*100] [checksum]
 */
#ifndef APP_LEG_TELEMETRY_H
#define APP_LEG_TELEMETRY_H

#include "app/app_config.h"

#if ROBOT_MODE == ROBOT_MODE_LEG || ROBOT_MODE == ROBOT_MODE_FULL

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise USART6 peripheral and GPIO for ESP32 TX.
 *        Call once during application init (from imu_app_init).
 */
void leg_telemetry_init(void);

/**
 * @brief Build and transmit one telemetry packet to the ESP32 for both legs.
 */
void leg_telemetry_send(float thigh_L, float shin_L, float knee_L,
                        float thigh_R, float shin_R, float knee_R);

#ifdef __cplusplus
}
#endif

#endif /* ROBOT_MODE_LEG || ROBOT_MODE_FULL */
#endif /* APP_LEG_TELEMETRY_H */
