#ifndef __MOTOR_TASK_H
#define __MOTOR_TASK_H

#include <stdint.h>
#include "cmsis_os.h"
#include "robot_config.h"
#include "kinematics.h"

extern volatile float g_cmd_velocity;
extern volatile float g_cmd_yaw_rate;
extern volatile uint8_t g_system_enabled;

extern osMutexId_t motor_state_mutex;

/* Called from micro-ROS cmd_vel callback (freertos.c) */
void robot_cmd_set(float vel, float yaw, uint8_t en);

/* Enable / Disable motor from ROS topic (/motor_enable).
   Records the request only; Motor_task performs the CAN transaction. */
void motor_enable_set(uint8_t enable);

/* 1 while no /cmd_vel has arrived within CMD_VEL_TIMEOUT_MS */
uint8_t motor_cmd_timed_out(void);

/* Read current motor enabled state safely */
uint8_t motor_get_enabled_state(void);

/* Periodic CAN bus / motor feedback log. Call from a low-priority task —
   never from the control loop. Rate-limits itself internally. */
void motor_diag_print(void);

/* Task body — called from Motor_Task_Entry in freertos.c */
void Motor_task(void);

#endif
