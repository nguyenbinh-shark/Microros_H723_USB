#ifndef __OBSERVE_TASK_H
#define __OBSERVE_TASK_H

#include "stdint.h"
#include "motor_task.h"   /* WHEEL_RADIUS, WHEEL_BASE, MOTOR_LEFT/RIGHT_SIGN */

/* ── Output struct ─────────────────────────────────────────────────── */
typedef struct {
    float v_filter;     /* Kalman-filtered linear velocity  (m/s)   */
    float omega_filter; /* IMU yaw rate, gyro.z             (rad/s) */
    float x_filter;     /* integrated position              (m)     */
} Observe_t;

extern Observe_t Observe;

extern void Observe_task(void);

#endif
