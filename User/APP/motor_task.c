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

#define CONTROL_PERIOD_MS   10U   /* 100 Hz */

/* ── Shared state ─────────────────────────────────────────────────── */
volatile float g_cmd_velocity = 0.0f;  /* m/s   — last commanded, may be stale */
volatile float g_cmd_yaw_rate = 0.0f;  /* rad/s — last commanded, may be stale */
volatile uint8_t g_system_enabled = 1U;          /* state actually applied     */
osMutexId_t motor_state_mutex = NULL;

static volatile uint32_t s_cmd_last_tick = 0U;   /* when /cmd_vel last arrived */
static volatile uint8_t  s_enable_request = 1U;  /* written by the ROS callback */
static volatile uint8_t  s_cmd_timed_out  = 0U;

/* ── API for /cmd_vel callback ────────────────────────────────────── */
void robot_cmd_set(float vel, float yaw, uint8_t en)
{
    (void)en;
    g_cmd_velocity  = vel;
    g_cmd_yaw_rate  = yaw;
    s_cmd_last_tick = HAL_GetTick();
}

/* ── API for /motor_enable topic callback ─────────────────────────────
 * Runs in the micro-ROS executor task. It only records the request: sending
 * CAN frames from here would race with Motor_task on the same FDCAN handle,
 * and HAL_FDCAN_AddMessageToTxFifoQ() is not reentrant. Motor_task applies it.
 */
void motor_enable_set(uint8_t enable)
{
    s_enable_request = enable ? 1U : 0U;
}

uint8_t motor_cmd_timed_out(void)
{
    return s_cmd_timed_out;
}

/* ── Apply a pending enable/disable, from Motor_task context only ──── */
static void motor_apply_enable(uint8_t enable)
{
    if (enable)
    {
        printf("[MOTOR] Enabling FDCAN1 & FDCAN3 motors...\r\n");
        rs_ext_enable(&hfdcan1, MOTOR_ID_LEFT,  MASTER_ID);
        rs_ext_enable(&hfdcan3, MOTOR_ID_RIGHT, MASTER_ID);
    }
    else
    {
        printf("[MOTOR] Disabling FDCAN1 & FDCAN3 motors...\r\n");
        rs_ext_disable(&hfdcan1, MOTOR_ID_LEFT,  MASTER_ID, 0);
        rs_ext_disable(&hfdcan3, MOTOR_ID_RIGHT, MASTER_ID, 0);
    }
    g_system_enabled = enable;
    printf("[MOTOR] System state changed to: %s\r\n", enable ? "ENABLED" : "DISABLED");
}

uint8_t motor_get_enabled_state(void)
{
    return g_system_enabled;
}

/* ── Diagnostics ──────────────────────────────────────────────────────
 * Deliberately not called from Motor_task: even with the non-blocking console
 * these two lines are ~340 characters of formatting work, which has no place
 * in a 10 ms control period. LCD_Task_Entry drives this instead.
 */
void motor_diag_print(void)
{
    static uint32_t last_tick = 0U;
    uint32_t now = HAL_GetTick();

    if ((now - last_tick) < 2000U) return;
    last_tick = now;

    CAN_BusMetrics_t m;
    CAN_GetMetrics(&m);

    float p0, v0, t0, p1, v1, t1;
    uint8_t val0 = 0, val1 = 0;
    CAN_MotorFeedback_Get_Idx(0, &p0, &v0, &t0, &val0);
    CAN_MotorFeedback_Get_Idx(1, &p1, &v1, &t1, &val1);

    printf("[CAN_DIAG] CAN1(L): TX=%lu Err=%lu RX=%lu ID=0x%08lX (V=%.2f T=%.2f Fresh=%d) | CAN3(R): TX=%lu Err=%lu RX=%lu ID=0x%08lX (V=%.2f T=%.2f Fresh=%d)\r\n",
           m.can1_tx_cnt, m.can1_tx_err, m.can1_rx_cnt, m.can1_last_id, v0, t0, val0,
           m.can3_tx_cnt, m.can3_tx_err, m.can3_rx_cnt, m.can3_last_id, v1, t1, val1);

    /* Protocol layer: this is what tells apart "no node on the bus" (ACK),
       "TX never reaches the bus" (BIT0) and "wrong baudrate" (STUFF/FORM). */
    printf("[CAN_PHY ] CAN1: LEC=%-5s BusOff=%u ErrPassive=%u TEC=%u REC=%u | CAN3: LEC=%-5s BusOff=%u ErrPassive=%u TEC=%u REC=%u\r\n",
           CAN_LecToStr(m.can1_proto.lec), (unsigned)m.can1_proto.bus_off,
           (unsigned)m.can1_proto.err_passive, (unsigned)m.can1_proto.tec, (unsigned)m.can1_proto.rec,
           CAN_LecToStr(m.can3_proto.lec), (unsigned)m.can3_proto.bus_off,
           (unsigned)m.can3_proto.err_passive, (unsigned)m.can3_proto.tec, (unsigned)m.can3_proto.rec);

    CAN_PrintDebugStats();
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
    uint32_t next_wake;

    printf("[MOTOR] Motor_task started.\r\n");
    /* Enable the CAN transceivers (active-HIGH on this board).
       Level lives in robot_config.h as CAN_XCVR_NORMAL_MODE_LEVEL. */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15,
                      CAN_XCVR_NORMAL_MODE_LEVEL);
    osDelay(50);

    /* Khởi tạo cấu hình bộ lọc và ngắt FDCAN1 + FDCAN3 */
    FDCAN1_Config();
    FDCAN3_Config();
    CAN_SetExpectedMotorId(0U, MOTOR_ID_LEFT);
    CAN_SetExpectedMotorId(1U, MOTOR_ID_RIGHT);
    printf("[MOTOR] FDCAN1 & FDCAN3 Filters Configured.\r\n");
    osDelay(50);

    /* Enable tất cả motor lúc khởi động */
    motor_apply_enable(1U);
    osDelay(100);  /* Chờ motor vào trạng thái Run */

    next_wake = osKernelGetTickCount();

    for (;;)
    {
        /* Deterministic period: osDelayUntil() holds 100 Hz regardless of how
           long this iteration took, where osDelay() let the period drift.
           Re-anchor after an overrun so a late cycle cannot turn into a burst. */
        next_wake += CONTROL_PERIOD_MS;
        {
            uint32_t now_tick = osKernelGetTickCount();
            if ((int32_t)(next_wake - now_tick) <= 0) next_wake = now_tick + CONTROL_PERIOD_MS;
        }
        osDelayUntil(next_wake);

        /* A Bus_Off controller never rejoins on its own — check every cycle. */
        (void)CAN_BusOffRecover();

        /* Apply a pending /motor_enable request from the ROS callback. */
        uint8_t want_enable = s_enable_request;
        if (want_enable != g_system_enabled)
        {
            printf("[MOTOR] Enable state request: %u (current: %u)\r\n", want_enable, g_system_enabled);
            motor_apply_enable(want_enable);
        }

        float effective_vx = g_cmd_velocity;
        float effective_wz = g_cmd_yaw_rate;
        uint8_t effective_enable = g_system_enabled;

        /* ── /cmd_vel watchdog ──────────────────────────────────────
         * Hold the last velocity only while it is fresh. If the agent, the USB
         * link or the publisher dies, the robot must coast to a stop instead of
         * driving on the final command forever. */
        uint8_t cmd_stale = ((HAL_GetTick() - s_cmd_last_tick) > CMD_VEL_TIMEOUT_MS) ? 1U : 0U;
        if (cmd_stale)
        {
            effective_vx = 0.0f;
            effective_wz = 0.0f;
        }
        if (cmd_stale != s_cmd_timed_out)
        {
            s_cmd_timed_out = cmd_stale;
            printf("[MOTOR] /cmd_vel %s\r\n", cmd_stale ? "TIMEOUT — holding stop" : "resumed");
        }

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
            rs_ext_control_cmd(&hfdcan1, MOTOR_ID_LEFT,  0.0f, 0.0f, 0.0f, 0.0f, VEL_IDLE_KD);
            rs_ext_control_cmd(&hfdcan3, MOTOR_ID_RIGHT, 0.0f, 0.0f, 0.0f, 0.0f, VEL_IDLE_KD);
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

            rs_ext_control_cmd(&hfdcan1, MOTOR_ID_LEFT,  0.0f, 0.0f, target_left_vel,  0.0f, VEL_TRACKING_KD);
            rs_ext_control_cmd(&hfdcan3, MOTOR_ID_RIGHT, 0.0f, 0.0f, target_right_vel, 0.0f, VEL_TRACKING_KD);
        }
        else
        {
            /* Khi xe dừng: đặt target_vel = 0 kèm hệ số giảm chấn để triệt tiêu rung giật */
            rs_ext_control_cmd(&hfdcan1, MOTOR_ID_LEFT,  0.0f, 0.0f, 0.0f, 0.0f, VEL_IDLE_KD);
            rs_ext_control_cmd(&hfdcan3, MOTOR_ID_RIGHT, 0.0f, 0.0f, 0.0f, 0.0f, VEL_IDLE_KD);
        }

    }
}

