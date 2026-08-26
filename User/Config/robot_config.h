#ifndef __ROBOT_CONFIG_H
#define __ROBOT_CONFIG_H

/**
 * @file robot_config.h
 * @brief Robot hardware and mechanical configuration parameters.
 *        Modify these parameters to port the software to different robot chassis.
 */

/* ── Mechanical parameters — single source of truth for whole project ─ */

/**
 * @brief Wheel radius in meters (bán kính bánh xe)
 */
#define WHEEL_RADIUS      0.05f

/**
 * @brief Distance between left and right wheels in meters (khoảng cách 2 bánh)
 */
#define WHEEL_BASE        0.30f

/* ── Hardware configuration ─────────────────────────────────────────── */

/**
 * @brief Motor mounting directions.
 * Sign: +1.0f if positive motor velocity → wheel moves FORWARD
 *       -1.0f if positive motor velocity → wheel moves BACKWARD
 * (mirrored-mount motors face each other, one side is inverted)
 */
#define MOTOR_LEFT_SIGN   ( 1.0f)  /* motor ID 1 — FDCAN1 */
#define MOTOR_RIGHT_SIGN  (-1.0f)  /* motor ID 2 — FDCAN3 */

/* ── Control algorithm tuning ───────────────────────────────────────── */

/**
 * @brief Control loop P-gain for MIT velocity-mode damping.
 * Torque = TORQUE_P_GAIN * (v_target - v_actual)
 * Note: Used by RobStride operation control mode.
 */
#define TORQUE_P_GAIN     0.1f

#endif /* __ROBOT_CONFIG_H */
