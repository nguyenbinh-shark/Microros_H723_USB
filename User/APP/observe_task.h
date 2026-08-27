#ifndef __OBSERVE_TASK_H
#define __OBSERVE_TASK_H

#include "stdint.h"
#include "motor_task.h"   /* WHEEL_RADIUS, WHEEL_BASE, MOTOR_LEFT/RIGHT_SIGN */

/* ── Output struct ─────────────────────────────────────────────────── */
typedef struct {
    float v_filter;     /* Kalman-filtered fused linear velocity (m/s)  */
    float omega_filter; /* Fused yaw rate, gyro.z (rad/s)               */
    float x_filter;     /* Integrated position (m)                      */
    float v_odom;       /* Raw wheel odometry linear velocity (m/s)     */
    float omega_odom;   /* Raw wheel differential yaw rate (rad/s)      */
    float slip_ratio;   /* Slip factor [0.0 = full grip, 1.0 = heavy slip] */
    uint8_t is_slipping;/* 1 if slip detected, 0 if normal traction     */
} Observe_t;

extern Observe_t Observe;

extern void Observe_task(void);

#endif /* __OBSERVE_TASK_H */
