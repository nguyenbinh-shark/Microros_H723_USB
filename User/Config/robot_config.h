#ifndef __ROBOT_CONFIG_H
#define __ROBOT_CONFIG_H

#include "main.h"   /* GPIO_PIN_SET / GPIO_PIN_RESET */

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
#define MOTOR_RIGHT_SIGN  (-1.0f)  /* motor ID 1 — FDCAN3 (separate bus) */

/* ── CAN transceiver enable pins (PC13 / PC14 / PC15) ───────────────── */

/**
 * @brief Level that enables the CAN transceivers on CtrBoard-H7.
 *
 * This board uses an active-HIGH enable, so these pins must be driven HIGH —
 * confirmed against the hardware, do not "fix" this to LOW. (Bare TJA1042-class
 * parts have an active-HIGH *standby* pin where the polarity is the opposite;
 * that is a different arrangement from this board's.)
 *
 * The [CAN_PHY] log on UART7 reports what the driver is actually doing:
 *   LEC=BIT0        -> transmitted dominant, read back recessive: nothing is
 *                      driving the bus (transceiver unpowered, or wrong pins)
 *   LEC=ACK         -> the driver works, but no node answers: wiring, 120R
 *                      termination, motor power, or motor CAN ID
 *   LEC=STUFF/FORM  -> baudrate mismatch
 */
#define CAN_XCVR_NORMAL_MODE_LEVEL   GPIO_PIN_SET

/* ── Safety ──────────────────────────────────────────────────────────── */

/**
 * @brief Stop the robot if the active command topic (/cmd_vel, /cmd_mit or
 *        /cmd_torque) is silent for this many milliseconds.
 * Without it the last commanded velocity is held forever when the ROS 2 agent,
 * the USB link, or the publishing node dies.
 */
#define CMD_VEL_TIMEOUT_MS   500U

/**
 * @brief Per-wheel torque limit for /cmd_torque, Nm.
 * That topic runs with no speed or damping loop, so this clamp is the only
 * thing bounding acceleration (10 Nm on a 0.05 m wheel is 200 N of push).
 * Kept well under the 60 Nm protocol range on purpose; raise it only after
 * testing on the actual RobStride model fitted.
 */
#define CMD_TORQUE_MAX_NM    10.0f

/* ── /imu/data covariance (diagonal variances, published in imu_link) ── */

/**
 * @brief Gyro variance, (rad/s)^2.
 * BMI088 noise density 0.014 dps/sqrt(Hz) at the configured 230 Hz bandwidth
 * gives sigma ~0.004 rad/s per sample; 2.5e-5 (sigma 0.005) adds margin for
 * residual bias. Re-measure on the robot and replace if it differs.
 */
#define IMU_GYRO_VARIANCE          2.5e-5

/**
 * @brief Accelerometer variance, (m/s^2)^2.
 * Datasheet noise alone is ~0.03 m/s^2 per sample; motor and chassis
 * vibration dominates on a moving robot, hence sigma 0.1 m/s^2.
 */
#define IMU_ACCEL_VARIANCE         1.0e-2

/**
 * @brief Roll / pitch variance, rad^2 (sigma ~1.8 deg, gravity-referenced).
 */
#define IMU_ORIENT_RP_VARIANCE     1.0e-3

/**
 * @brief Yaw variance, rad^2. Yaw is relative to startup and drifts without
 * bound (no magnetometer), so it is published as effectively unknown. An EKF
 * that fuses it anyway will give it almost no weight.
 */
#define IMU_ORIENT_YAW_VARIANCE    1.0e6

/* ── Control algorithm tuning ───────────────────────────────────────── */

/**
 * @brief Control loop P-gain for MIT velocity-mode damping.
 * Torque = TORQUE_P_GAIN * (v_target - v_actual)
 * Note: Used by RobStride operation control mode.
 */
#define TORQUE_P_GAIN     0.1f

#endif /* __ROBOT_CONFIG_H */
