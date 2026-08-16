#ifndef APP_CONFIG_LEG_H
#define APP_CONFIG_LEG_H

/**
 * @file config_leg.h
 * @brief Configuration parameters specific to the Leg Robot Mode.
 *
 * All definitions in this file are only relevant when ROBOT_MODE == ROBOT_MODE_LEG.
 */

#if ROBOT_MODE == ROBOT_MODE_LEG

/* ====== HARDWARE CONFIGURATION ====== */
/**
 * Number of active legs.
 * 1 = Single leg on I2C1 (2 IMUs: Thigh, Shin)
 * 2 = Dual leg on I2C1 and I2C3 (4 IMUs total)
 */
#define NUM_LEGS 2

#endif /* ROBOT_MODE_LEG */
#endif /* APP_CONFIG_LEG_H */
