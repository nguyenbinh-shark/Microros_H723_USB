/**
 * observe_task.c — Differential-drive Adaptive Slip-Aware Velocity Observer
 *
 * Fuses Wheel Odometry + BMI088 IMU Accelerometer + Gyroscope using an
 * Adaptive Extended Kalman Filter with Wheel Slip & Skid Detection.
 *
 * Kalman state : x = [v_linear (m/s),  a_linear (m/s²)]^T
 * Predictions  : Constant-acceleration model, dt = 3 ms
 * Measurements : z = [v_wheel_odom,  a_imu_body_x]^T
 *
 * Adaptive Slip Detection & Covariance Scaling:
 * 1. Rotational slip: |w_odom - w_imu_gyro|
 * 2. Longitudinal slip: |a_wheel_diff - a_imu_accel|
 * When slip is detected, measurement noise R_v is dynamically scaled up (up to 400x),
 * forcing the filter to reject slipping wheel readings and rely on inertial integration.
 */

#include "observe_task.h"
#include "kalman_filter.h"
#include "INS_task.h"
#include "can_bsp.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

#define OBSERVE_DT_MS     3U                          /* Task period: 3 ms (333.3 Hz) */
#define OBSERVE_DT_S      (OBSERVE_DT_MS * 0.001f)   /* Period in seconds: 0.003 s    */

/* Baseline Noise Covariances */
#define KF_BASE_R_V       0.5f                        /* Nominal wheel velocity noise */
#define KF_BASE_R_A       2.0f                        /* Nominal IMU acceleration noise */
#define KF_Q_V            0.05f                       /* Velocity process noise       */
#define KF_Q_A            0.30f                       /* Acceleration process noise   */

/* Slip Detection Thresholds */
#define SLIP_ROT_THRESH   0.20f                       /* rad/s difference threshold   */
#define SLIP_ACC_THRESH   1.20f                       /* m/s² difference threshold    */

Observe_t Observe = {0};

/* ── Kalman matrices (2×2 flat arrays, row-major) ─────────────────── */
static float KF_F[4] = {
    1.0f, OBSERVE_DT_S,
    0.0f, 1.0f
};

static float KF_P[4] = {
    1.0f, 0.0f,
    0.0f, 1.0f
};

static float KF_Q[4] = {
    KF_Q_V, 0.0f,
    0.0f,   KF_Q_A
};

static float KF_R[4] = {
    KF_BASE_R_V, 0.0f,
    0.0f,        KF_BASE_R_A
};

static const float KF_H[4] = {
    1.0f, 0.0f,
    0.0f, 1.0f
};

static KalmanFilter_t velKF;
static float vel_acc[2];
static float s_v_odom_prev = 0.0f;
static float s_a_odom_filt = 0.0f;

/* ── Init ─────────────────────────────────────────────────────────── */
static void velKF_Init(void)
{
    Kalman_Filter_Init(&velKF, 2, 0, 2);
    memcpy(velKF.F_data, KF_F, sizeof(KF_F));
    memcpy(velKF.P_data, KF_P, sizeof(KF_P));
    memcpy(velKF.Q_data, KF_Q, sizeof(KF_Q));
    memcpy(velKF.R_data, KF_R, sizeof(KF_R));
    memcpy(velKF.H_data, KF_H, sizeof(KF_H));
    s_v_odom_prev = 0.0f;
    s_a_odom_filt = 0.0f;
    memset(&Observe, 0, sizeof(Observe));
}

/* ── Adaptive Kalman Update with Slip Scaling ─────────────────────── */
static void velKF_AdaptiveUpdate(float v_odom, float a_imu, float slip_factor)
{
    /* Dynamically scale R_v up when slipping (distrust wheel encoder) */
    float r_v_adaptive = KF_BASE_R_V * (1.0f + slip_factor * slip_factor * 350.0f);

    KF_R[0] = r_v_adaptive;
    KF_R[3] = KF_BASE_R_A;

    memcpy(velKF.Q_data, KF_Q, sizeof(KF_Q));
    memcpy(velKF.R_data, KF_R, sizeof(KF_R));

    velKF.MeasuredVector[0] = v_odom;
    velKF.MeasuredVector[1] = a_imu;

    Kalman_Filter_Update(&velKF);

    vel_acc[0] = velKF.FilteredValue[0];
    vel_acc[1] = velKF.FilteredValue[1];
}

/* ── Task Body ────────────────────────────────────────────────────── */
void Observe_task(void)
{
    printf("[OBSERVE] Task started. Waiting for INS convergence...\r\n");
    /* Wait until Mahony filter has converged (ins_flag set after 3s) */
    while (INS.ins_flag == 0)
    {
        osDelay(2);
    }

    printf("[OBSERVE] INS Converged! Adaptive Slip-Aware Velocity Observer Active.\r\n");
    velKF_Init();

    /* The filter's F matrix hard-codes dt = OBSERVE_DT_S, so the loop must
       actually run at that period. osDelay() drifts by the execution time of
       each iteration; osDelayUntil() holds the rate. */
    uint32_t next_wake = osKernelGetTickCount();

    for (;;)
    {
        next_wake += OBSERVE_DT_MS;
        {   /* Re-anchor after an overrun instead of spinning to catch up. */
            uint32_t now_tick = osKernelGetTickCount();
            if ((int32_t)(next_wake - now_tick) <= 0) next_wake = now_tick + OBSERVE_DT_MS;
        }
        osDelayUntil(next_wake);

        /* 1. Wheel Odometry Kinematics */
        float v_left  = CAN_GetMotorVel(1) * MOTOR_LEFT_SIGN  * WHEEL_RADIUS;
        float v_right = CAN_GetMotorVel(2) * MOTOR_RIGHT_SIGN * WHEEL_RADIUS;

        float v_odom     = (v_left + v_right) * 0.5f;
        float omega_odom = (v_right - v_left) / WHEEL_BASE;

        /* Wheel acceleration derivative with LPF */
        float a_odom_raw = (v_odom - s_v_odom_prev) / OBSERVE_DT_S;
        s_v_odom_prev = v_odom;
        s_a_odom_filt = 0.85f * s_a_odom_filt + 0.15f * a_odom_raw;

        /* 2. IMU Measurements (Gravity-compensated linear acceleration & Gyro yaw rate) */
        float a_imu     = INS.MotionAccel_b[0];
        float omega_imu = INS.Gyro[2];

        /* 3. Slip & Skid Detection Metrics */
        float err_omega = fabsf(omega_odom - omega_imu);
        float err_accel = fabsf(s_a_odom_filt - a_imu);

        float raw_slip = 0.0f;
        /* Ignore trivial zero-speed jitter */
        if (fabsf(v_odom) > 0.04f || fabsf(omega_odom) > 0.08f || fabsf(omega_imu) > 0.08f)
        {
            float s_rot = (err_omega - SLIP_ROT_THRESH) / 0.60f;
            float s_acc = (err_accel - SLIP_ACC_THRESH) / 3.00f;
            if (s_rot < 0.0f) s_rot = 0.0f;
            if (s_acc < 0.0f) s_acc = 0.0f;
            raw_slip = (s_rot > s_acc) ? s_rot : s_acc;
            if (raw_slip > 1.0f) raw_slip = 1.0f;
        }

        /* Exponential moving average for smooth slip transition */
        Observe.slip_ratio  = 0.90f * Observe.slip_ratio + 0.10f * raw_slip;
        Observe.is_slipping = (Observe.slip_ratio > 0.25f) ? 1U : 0U;

        /* 4. Adaptive Kalman Fusion Update */
        velKF_AdaptiveUpdate(v_odom, a_imu, Observe.slip_ratio);

        /* 5. Write Observer Outputs */
        Observe.v_odom       = v_odom;
        Observe.omega_odom   = omega_odom;
        Observe.v_filter     = vel_acc[0];
        Observe.omega_filter = omega_imu;   /* IMU Gyro yaw rate is drift-free and immune to slip */
        Observe.x_filter    += Observe.v_filter * OBSERVE_DT_S;
    }
}
