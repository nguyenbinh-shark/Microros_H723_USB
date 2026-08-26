#ifndef __MOTOR_TASK_H
#define __MOTOR_TASK_H

#include <stdint.h>
#include "cmsis_os.h"
#include "robot_config.h"
#include "kinematics.h"

extern volatile float g_cmd_velocity;
extern volatile float g_cmd_yaw_rate;
extern volatile uint8_t g_system_enabled;

/* Called from micro-ROS cmd_vel callback (freertos.c) */
void robot_cmd_set(float vel, float yaw, uint8_t en);

/* Enable / Disable motor from ROS topic (/motor_enable) */
void motor_enable_set(uint8_t enable);

/* Task body — called from Motor_Task_Entry in freertos.c */
void Motor_task(void);

#endif
