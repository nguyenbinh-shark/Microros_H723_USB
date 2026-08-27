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
#define MOTOR_RIGHT_SIGN  (-1.0f)  /* motor ID 2 — FDCAN3 */

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
 * @brief Stop the robot if no /cmd_vel arrives within this many milliseconds.
 * Without it the last commanded velocity is held forever when the ROS 2 agent,
 * the USB link, or the publishing node dies.
 */
#define CMD_VEL_TIMEOUT_MS   500U

/* ── Control algorithm tuning ───────────────────────────────────────── */

/**
 * @brief Control loop P-gain for MIT velocity-mode damping.
 * Torque = TORQUE_P_GAIN * (v_target - v_actual)
 * Note: Used by RobStride operation control mode.
 */
#define TORQUE_P_GAIN     0.1f

#endif /* __ROBOT_CONFIG_H */
