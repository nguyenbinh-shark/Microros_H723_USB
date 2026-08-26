/**
 * observe_task.c — Differential-drive velocity observer
 *
 * Kalman state: [v_linear (m/s),  a_linear (m/s²)]
 * Predictions : constant-acceleration model, dt = OBSERVE_DT_MS / 1000
 * Measurements: z[0] = wheel odometry linear velocity
 *               z[1] = IMU forward acceleration (MotionAccel_b[0])
 *
 * Outputs (Observe struct):
 *   v_filter     — fused linear velocity
 *   omega_filter — yaw rate from IMU gyro.z (already reliable)
 *   x_filter     — integrated position
 */

#include "observe_task.h"
#include "kalman_filter.h"
#include "INS_task.h"
#include "can_bsp.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>

#define OBSERVE_DT_MS  3U                          /* task period (ms)  */
#define OBSERVE_DT_S   (OBSERVE_DT_MS * 0.001f)   /* period in seconds */

Observe_t Observe = {0};

/* ── Kalman matrices (2×2 flat arrays, row-major) ─────────────────── */

/* State transition: constant-acceleration model */
static float KF_F[4] = {
    1.0f, OBSERVE_DT_S,
    0.0f, 1.0f
};

/* Initial estimation error covariance */
static float KF_P[4] = {
    1.0f, 0.0f,
    0.0f, 1.0f
};

/* Process noise — increase to trust measurements more, trust model less */
static float KF_Q[4] = {
    0.5f, 0.0f,
    0.0f, 0.5f
};

/* Measurement noise — increase R to trust measurements less (more smoothing) */
static float KF_R[4] = {
    100.0f, 0.0f,
    0.0f,  100.0f
};

/* Measurement matrix — z directly observes [v, a] */
static const float KF_H[4] = {
    1.0f, 0.0f,
    0.0f, 1.0f
};

static KalmanFilter_t velKF;
static float vel_acc[2];   /* filtered [velocity, acceleration] */

/* ── Init ─────────────────────────────────────────────────────────── */
static void velKF_Init(void)
{
    Kalman_Filter_Init(&velKF, 2, 0, 2);
    memcpy(velKF.F_data, KF_F, sizeof(KF_F));
    memcpy(velKF.P_data, KF_P, sizeof(KF_P));
    memcpy(velKF.Q_data, KF_Q, sizeof(KF_Q));
    memcpy(velKF.R_data, KF_R, sizeof(KF_R));
    memcpy(velKF.H_data, KF_H, sizeof(KF_H));
}

/* ── Update ───────────────────────────────────────────────────────── */
static void velKF_Update(float v_odom, float a_imu)
{
    memcpy(velKF.Q_data, KF_Q, sizeof(KF_Q));
    memcpy(velKF.R_data, KF_R, sizeof(KF_R));

    velKF.MeasuredVector[0] = v_odom;
    velKF.MeasuredVector[1] = a_imu;

    Kalman_Filter_Update(&velKF);

    vel_acc[0] = velKF.FilteredValue[0];
    vel_acc[1] = velKF.FilteredValue[1];
}

/* ── Task ─────────────────────────────────────────────────────────── */
void Observe_task(void)
{
    printf("[OBSERVE] Task started. Waiting for INS convergence...\r\n");
    /* Wait until Mahony filter has converged (ins_flag set after 3s) */
    while (INS.ins_flag == 0)
    {
        osDelay(1);
    }

    printf("[OBSERVE] INS Converged! Velocity Observer Kalman Filter Active.\r\n");
    velKF_Init();

    for (;;)
    {
        /* ① Wheel odometry — linear velocity */
        float v_left  = CAN_GetMotorVel(1) * MOTOR_LEFT_SIGN  * WHEEL_RADIUS;
        float v_right = CAN_GetMotorVel(2) * MOTOR_RIGHT_SIGN * WHEEL_RADIUS;
        float v_odom  = (v_left + v_right) * 0.5f;

        /* ② IMU forward acceleration (body X-axis) */
        float a_imu = INS.MotionAccel_b[0];

        /* ③ Kalman update */
        velKF_Update(v_odom, a_imu);

        /* ④ Write outputs */
        Observe.v_filter     = vel_acc[0];
        Observe.omega_filter = INS.Gyro[2];   /* gyro yaw rate, already accurate */
        Observe.x_filter    += Observe.v_filter * OBSERVE_DT_S;

        osDelay(OBSERVE_DT_MS);
    }
}
