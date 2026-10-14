#ifndef __MOTOR_TASK_H
#define __MOTOR_TASK_H

#include <stdint.h>
#include "cmsis_os.h"
#include "robot_config.h"
#include "kinematics.h"

/* ── Control mode — chosen by whichever command topic spoke last ──────
 * Each command topic maps to exactly one actuator run_mode:
 *   /cmd_vel  (geometry_msgs/Twist)        -> CTRL_MODE_VELOCITY, run_mode 2
 *   /cmd_mit  (std_msgs/Float64MultiArray) -> CTRL_MODE_MIT,      run_mode 0
 *   /cmd_torque (std_msgs/Float64MultiArray) -> CTRL_MODE_MIT, torque only
 * Motor_task performs the switch (stop -> write run_mode -> enable). */
typedef enum
{
    CTRL_MODE_VELOCITY = 0,
    CTRL_MODE_MIT      = 1
} CtrlMode_t;

/* One wheel's MIT setpoint, motor-shaft frame. The /cmd_mit callback has
   already converted pos/vel/tau from the wheel frame with MOTOR_*_SIGN.
   tau = tau_ff + kp * (pos - p_act) + kd * (vel - v_act) */
typedef struct
{
    float pos;   /* rad          [-12.57, 12.57] */
    float vel;   /* rad/s        [-20, 20]       */
    float kp;    /* Nm/rad       [0, 5000]       */
    float kd;    /* Nm/(rad/s)   [0, 100]        */
    float tau;   /* Nm, feedforward [-60, 60]    */
} MotorMitCmd_t;

/* Number of float64 values /cmd_mit carries: [pos, vel, kp, kd, tau] per wheel,
   left wheel first. */
#define CMD_MIT_LEN   10U

/* Number of float64 values /cmd_torque carries: [tau_left, tau_right]. */
#define CMD_TORQUE_LEN 2U

extern volatile float g_cmd_velocity;
extern volatile float g_cmd_yaw_rate;
extern volatile uint8_t g_system_enabled;

extern osMutexId_t motor_state_mutex;

/* Called from micro-ROS /cmd_vel callback (freertos.c). Requests Velocity mode. */
void robot_cmd_set(float vel, float yaw, uint8_t en);

/* Called from micro-ROS /cmd_mit callback (freertos.c). Requests MIT mode. */
void motor_mit_cmd_set(const MotorMitCmd_t *left, const MotorMitCmd_t *right);

/* Enable / Disable motor from ROS topic (/motor_enable).
   Records the request only; Motor_task performs the CAN transaction. */
void motor_enable_set(uint8_t enable);

/* 1 while the active command topic has been silent for CMD_VEL_TIMEOUT_MS */
uint8_t motor_cmd_timed_out(void);

/* Read current motor enabled state safely */
uint8_t motor_get_enabled_state(void);

/* Run mode the motors are actually in right now (not merely requested) */
CtrlMode_t motor_get_control_mode(void);

/* Periodic CAN bus / motor feedback log. Call from a low-priority task —
   never from the control loop. Rate-limits itself internally. */
void motor_diag_print(void);

/* Task body — called from Motor_Task_Entry in freertos.c */
void Motor_task(void);

#endif
