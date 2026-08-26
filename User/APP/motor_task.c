/**
 * motor_task.c — Closed-loop Velocity Control with Differential Kinematics
 *
 * Receives true velocity command from /cmd_vel:
 *   linear.x  → robot forward linear speed (m/s)
 *   angular.z → robot turn rate (rad/s)
 *
 * Converts (v, w) -> wheel angular velocity (rad/s) via Kinematics:
 *   w_left  = (v - w * L/2) / R
 *   w_right = (v + w * L/2) / R
 *
 * Controls RobStride motors via Operation Control Mode (Type 1):
 *   Tau = Kd * (v_target - v_actual)
 *
 * CAN Bus: FDCAN1 = left motor (ID 1), FDCAN3 = right motor (ID 1)
 */

#include "motor_task.h"
#include "can_bsp.h"
#include "robstride_drv.h"
#include "fdcan.h"
#include "cmsis_os.h"
#include "robot_config.h"
#include "kinematics.h"
#include "bsp_sbus.h"
#include <stdio.h>

/* Motor CAN IDs and master ID */
#define MOTOR_ID_LEFT    1U
#define MOTOR_ID_RIGHT   1U
#define MASTER_ID        RS_DEFAULT_MASTER_ID   /* 0xFD */

/* Velocity Tracking Gain (Kd) — FOC velocity closed-loop stiffness */
#define VEL_TRACKING_KD  1.5f   /* Nm / (rad/s) — hệ số bám vận tốc khi chạy */
#define VEL_IDLE_KD      0.8f   /* Nm / (rad/s) — hệ số hãm giữ vị trí khi đứng yên */

/* ── Shared state ─────────────────────────────────────────────────── */
volatile float g_cmd_velocity = 0.0f;  /* m/s */
volatile float g_cmd_yaw_rate = 0.0f;  /* rad/s */
volatile uint8_t g_system_enabled = 1U;

/* ── API for /cmd_vel callback ────────────────────────────────────── */
void robot_cmd_set(float vel, float yaw, uint8_t en)
{
    (void)en;
    g_cmd_velocity = vel;
    g_cmd_yaw_rate = yaw;
}

/* ── API for /motor_enable topic callback ─────────────────────────── */
void motor_enable_set(uint8_t enable)
{
    g_system_enabled = enable;
    if (enable)
    {
        printf("[MOTOR] Enabling FDCAN1 & FDCAN3 motors...\r\n");
        rs_ext_enable(&hfdcan1, 1U, MASTER_ID);   /* Left motor  */
        rs_ext_enable(&hfdcan3, 1U, MASTER_ID);   /* Right motor */
    }
    else
    {
        printf("[MOTOR] Disabling FDCAN1 & FDCAN3 motors...\r\n");
        rs_ext_disable(&hfdcan1, 1U, MASTER_ID, 0);     /* Left motor  */
        rs_ext_disable(&hfdcan3, 1U, MASTER_ID, 0);     /* Right motor */
    }
}

/* ── Helper: clamp value ──────────────────────────────────────────── */
static inline float clampf(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/* ── Task body — Closed-loop Velocity Control ─────────────────────── */
void Motor_task(void)
{
    printf("[MOTOR] Motor_task started.\r\n");
    /* Enable Transceivers CAN1, CAN3 (PC13, PC15 HIGH) */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_SET);
    osDelay(50);

    /* Khởi tạo cấu hình bộ lọc và ngắt FDCAN1 + FDCAN3 */
    FDCAN1_Config();
    FDCAN3_Config();
    printf("[MOTOR] FDCAN1 & FDCAN3 Filters Configured.\r\n");
    osDelay(50);

    /* Enable tất cả motor lúc khởi động */
    motor_enable_set(1U);
    osDelay(100);  /* Chờ motor vào trạng thái Run */

    for (;;)
    {
        float effective_vx = g_cmd_velocity;
        float effective_wz = g_cmd_yaw_rate;
        uint8_t effective_enable = g_system_enabled;

        /* ── SBUS Remote Controller Arbitration ─────────────────── */
        if (sbus_is_online())
        {
            RC_ControlMode_t rc_mode = sbus_get_control_mode();
            if (rc_mode == RC_MODE_MANUAL_RC)
            {
                /* Manual Teleop: Max 1.0 m/s linear, 2.5 rad/s turn */
                sbus_get_motion_cmd(1.0f, 2.5f, &effective_vx, &effective_wz);
                effective_enable = 1U;
            }
            else if (rc_mode == RC_MODE_EMERGENCY_STOP)
            {
                effective_vx = 0.0f;
                effective_wz = 0.0f;
                effective_enable = 0U;
            }
            /* If RC_MODE_AUTO_ROS2: keeps ROS2 /cmd_vel commands */
        }

        /* Nếu hệ thống bị Disable: dừng phanh motor */
        if (!effective_enable)
        {
            rs_ext_control_cmd(&hfdcan1, 1U, 0.0f, 0.0f, 0.0f, 0.0f, VEL_IDLE_KD);
            rs_ext_control_cmd(&hfdcan3, 1U, 0.0f, 0.0f, 0.0f, 0.0f, VEL_IDLE_KD);
            osDelay(10);
            continue;
        }

        RobotCmd_t cmd;
        cmd.velocity = effective_vx;   /* m/s */
        cmd.yaw_rate = effective_wz;   /* rad/s */
        cmd.enable   = effective_enable;

        float target_left_vel = 0.0f;    /* rad/s */
        float target_right_vel = 0.0f;   /* rad/s */

        /* 1. Tính toán động học vi sai (Kinematics) đổi (v, w) -> (w_left, w_right) */
        robot_control_algo(&cmd, &target_left_vel, &target_right_vel);

        /* 2. Điều khiển vận tốc vòng kín:
         *    t = Kp*(p_des - p_act) + Kd*(v_des - v_act) + t_ff
         *    - Kp = 0 (không khóa góc vị trí)
         *    - v_des = target_vel (rad/s)
         *    - Kd = VEL_TRACKING_KD (bù moment tự động theo sai lệch vận tốc)
         */
        if (cmd.velocity != 0.0f || cmd.yaw_rate != 0.0f)
        {
            /* Giới hạn an toàn vận tốc góc max [-20, +20 rad/s] */
            target_left_vel  = clampf(target_left_vel,  RS_EXT_V_MIN, RS_EXT_V_MAX);
            target_right_vel = clampf(target_right_vel, RS_EXT_V_MIN, RS_EXT_V_MAX);

            rs_ext_control_cmd(&hfdcan1, 1U, 0.0f, 0.0f, target_left_vel,  0.0f, VEL_TRACKING_KD);
            rs_ext_control_cmd(&hfdcan3, 1U, 0.0f, 0.0f, target_right_vel, 0.0f, VEL_TRACKING_KD);
        }
        else
        {
            /* Khi xe dừng: đặt target_vel = 0 kèm hệ số giảm chấn để triệt tiêu rung giật */
            rs_ext_control_cmd(&hfdcan1, 1U, 0.0f, 0.0f, 0.0f, 0.0f, VEL_IDLE_KD);
            rs_ext_control_cmd(&hfdcan3, 1U, 0.0f, 0.0f, 0.0f, 0.0f, VEL_IDLE_KD);
        }

        osDelay(10);   /* 100 Hz control loop */
    }
}

