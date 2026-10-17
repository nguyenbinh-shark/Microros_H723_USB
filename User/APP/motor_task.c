/**
 * motor_task.c — Wheel control, one actuator run_mode per command topic
 *
 * Each ROS command topic owns one RobStride run_mode, and the topic that spoke
 * last decides which one the motors are in:
 *
 *   /cmd_vel  geometry_msgs/Twist         -> Velocity mode (run_mode 2)
 *       linear.x  -> robot forward speed (m/s)
 *       angular.z -> robot turn rate (rad/s)
 *     converted to wheel angular velocity (rad/s) via Kinematics:
 *       w_left  = (v - w * L/2) / R
 *       w_right = (v + w * L/2) / R
 *     and written to SPD_REF. The actuator closes the velocity loop itself with
 *     a PI controller, so commanded speed is held with zero droop, and ramps
 *     every step at VEL_MODE_ACC_RAD.
 *
 *   /cmd_mit  std_msgs/Float64MultiArray  -> Operation Control / MIT (run_mode 0)
 *       data = [p, v, kp, kd, tau]_left ++ [p, v, kp, kd, tau]_right
 *     forwarded to the actuator, which applies
 *       tau_out = tau + kp * (p - p_act) + kd * (v - v_act)
 *     The topic is in the wheel frame, like /motor_fb; the /cmd_mit callback
 *     applies MOTOR_*_SIGN, so this task only sees motor-shaft values.
 *     Raw actuator access, no kinematics.
 *
 *   /cmd_torque  std_msgs/Float64MultiArray  -> MIT mode, torque only
 *       data = [tau_left, tau_right]
 *     sent as an MIT setpoint with p = v = kp = kd = 0 and clamped to
 *     CMD_TORQUE_MAX_NM. Shares the MIT path, so it switches mode the same way.
 *
 * Switching mode costs a stop -> run_mode -> enable sequence on the bus, so a
 * host should drive one topic at a time rather than interleave them.
 *
 * CAN Bus: FDCAN1 = left motor (ID 1), FDCAN3 = right motor (ID 1)
 */

#include "motor_task.h"
#include "can_bsp.h"
#include "robstride_drv.h"
#include "fdcan.h"
#include "usart.h"  /* debug_uart_tx_dropped() */
#include "FreeRTOS.h"
#include "task.h"   /* taskENTER_CRITICAL — command hand-off from the ROS task */
#include "cmsis_os.h"
#include "robot_config.h"
#include "kinematics.h"
#include "bsp_sbus.h"
#include <math.h>
#include <stdio.h>

/* Motor CAN IDs and master ID */
#define MOTOR_ID_LEFT    1U
#define MOTOR_ID_RIGHT   1U
#define MASTER_ID        RS_DEFAULT_MASTER_ID   /* 0xFD */

/* Diagnostics only — the control path works in radians throughout. */
#define RAD_TO_DEG       57.2957795f

/* ── Diagnostic verbosity ─────────────────────────────────────
 * 1 = dump the full CAN/encoder detail on every tick, as this task used to.
 * Useful on the bench, wasteful in a running robot: see motor_diag_print(). */
#ifndef MOTOR_DIAG_VERBOSE
#define MOTOR_DIAG_VERBOSE   0
#endif

#define DIAG_PERIOD_MS       2000U    /* the one-line health beat */
#define DIAG_DETAIL_MIN_MS  10000U    /* floor between full dumps */

/* RobStride run_mode values written to RS_PARAM_RUN_MODE */
#define RS_RUN_MODE_MIT       0U
#define RS_RUN_MODE_VELOCITY  2U

/* ── Velocity mode (/cmd_vel) ─────────────────────────────────────────── */

/* Acceleration ramp applied inside the motor, rad/s^2. Converts to linear
 * acceleration as a = VEL_MODE_ACC_RAD * WHEEL_RADIUS, so 20 rad/s^2 with the
 * 0.05 m wheel is 1 m/s^2 — a step on /cmd_vel is ramped instead of slammed. */
#define VEL_MODE_ACC_RAD     20.0f

/* Phase current limit in amps, or 0.0f to leave the motor's own default alone.
 * Left at 0 deliberately: the right ceiling depends on the actuator model and
 * this project does not record which RobStride variant is fitted. Set it to the
 * datasheet rating once that is known — it is the cheapest protection there is
 * against a stalled wheel cooking a winding. */
#define VEL_MODE_CUR_LIMIT_A  0.0f

/* ── MIT mode (/cmd_mit) ──────────────────────────────────────────────── */

/* Damping used whenever MIT mode has to stop on its own — command timeout,
 * disarm, e-stop. Kp = 0, so the wheel resists motion without snapping back to
 * a position the host never asked for. */
#define MIT_STOP_KD      0.8f   /* Nm / (rad/s) — hệ số hãm khi dừng */

/* Control-loop period in FreeRTOS ticks (tick rate is 1 kHz, so 1 unit = 1 ms).
 * 2U = 500 Hz. Only rates of 1000/N Hz are expressible at this tick rate;
 * raise configTICK_RATE_HZ if you need a rate in between.
 *
 * What this rate does and does not buy:
 *   - It sets how fast a new command reaches the motor (worst case = one period).
 *   - It does NOT tighten tracking. Both loops run inside the RobStride FOC
 *     driver at tens of kHz; this task only streams setpoints.
 *
 * Cost check before lowering further: each cycle is 2 TX + 2 RX frames, ~150 us
 * each at 1 Mbit/s, so bus load is ~600 us / period — 6% at 100 Hz, 31% at 500 Hz. */
#define CONTROL_PERIOD_MS   2U   /* 500 Hz */

/* ── Shared state ─────────────────────────────────────────────────── */
volatile float g_cmd_velocity = 0.0f;  /* m/s   — last /cmd_vel, may be stale */
volatile float g_cmd_yaw_rate = 0.0f;  /* rad/s — last /cmd_vel, may be stale */
volatile uint8_t g_system_enabled = 0U;          /* state actually applied     */
osMutexId_t motor_state_mutex = NULL;

static MotorMitCmd_t s_mit_cmd[2];               /* last /cmd_mit, [0]=L [1]=R */

static volatile uint32_t   s_cmd_last_tick  = 0U;  /* last command, either topic */
static volatile uint8_t    s_enable_request = 0U;  /* written by the ROS callback */
static volatile uint8_t    s_cmd_timed_out  = 0U;
static volatile CtrlMode_t s_mode_request   = CTRL_MODE_VELOCITY; /* last topic */
static volatile CtrlMode_t s_mode_applied   = CTRL_MODE_VELOCITY; /* in the motor */

/* ── API for /cmd_vel callback ────────────────────────────────────── */
void robot_cmd_set(float vel, float yaw, uint8_t en)
{
    (void)en;
    taskENTER_CRITICAL();
    g_cmd_velocity  = vel;
    g_cmd_yaw_rate  = yaw;
    s_mode_request  = CTRL_MODE_VELOCITY;
    s_cmd_last_tick = HAL_GetTick();
    taskEXIT_CRITICAL();
}

/* ── API for /cmd_mit callback ────────────────────────────────────── */
void motor_mit_cmd_set(const MotorMitCmd_t *left, const MotorMitCmd_t *right)
{
    if (left == NULL || right == NULL) return;
    /* Ten floats cannot be written atomically; without the critical section
       Motor_task could send one wheel a pos from this message and a kp from the
       previous one. */
    taskENTER_CRITICAL();
    s_mit_cmd[0]    = *left;
    s_mit_cmd[1]    = *right;
    s_mode_request  = CTRL_MODE_MIT;
    s_cmd_last_tick = HAL_GetTick();
    taskEXIT_CRITICAL();
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

CtrlMode_t motor_get_control_mode(void)
{
    return s_mode_applied;
}

/* ── Motor mode configuration ─────────────────────────────────────────
 * Writes run_mode and its limits into motor RAM. Deliberately NOT followed by
 * rs_ext_save_param(): keeping the mode out of motor flash means a spare unit
 * from factory stock drops in and works, and it costs nothing to re-send a few
 * frames on every enable. The frames queue ahead of the enable frame on the
 * same bus, so the motor consumes them in order — no delay needed in between.
 */
static void motor_config_mode(CtrlMode_t mode)
{
    if (mode == CTRL_MODE_VELOCITY)
    {
        rs_ext_write_param_uint8(&hfdcan1, MOTOR_ID_LEFT,  MASTER_ID, RS_PARAM_RUN_MODE, RS_RUN_MODE_VELOCITY);
        rs_ext_write_param_uint8(&hfdcan3, MOTOR_ID_RIGHT, MASTER_ID, RS_PARAM_RUN_MODE, RS_RUN_MODE_VELOCITY);

        rs_ext_write_param_float(&hfdcan1, MOTOR_ID_LEFT,  MASTER_ID, RS_PARAM_ACC_RAD, VEL_MODE_ACC_RAD);
        rs_ext_write_param_float(&hfdcan3, MOTOR_ID_RIGHT, MASTER_ID, RS_PARAM_ACC_RAD, VEL_MODE_ACC_RAD);

        if (VEL_MODE_CUR_LIMIT_A > 0.0f)
        {
            rs_ext_write_param_float(&hfdcan1, MOTOR_ID_LEFT,  MASTER_ID, RS_PARAM_LIMIT_CUR, VEL_MODE_CUR_LIMIT_A);
            rs_ext_write_param_float(&hfdcan3, MOTOR_ID_RIGHT, MASTER_ID, RS_PARAM_LIMIT_CUR, VEL_MODE_CUR_LIMIT_A);
        }

        /* SPD_REF survives in motor RAM across a stop. Without this the motor
           would resume the speed it last held — possibly from long ago — for
           the one period until the first new setpoint lands. */
        rs_ext_write_param_float(&hfdcan1, MOTOR_ID_LEFT,  MASTER_ID, RS_PARAM_SPD_REF, 0.0f);
        rs_ext_write_param_float(&hfdcan3, MOTOR_ID_RIGHT, MASTER_ID, RS_PARAM_SPD_REF, 0.0f);

        printf("[MOTOR] run_mode=2 (Velocity, /cmd_vel), acc=%.1f rad/s^2, cur_limit=%s\r\n",
               VEL_MODE_ACC_RAD,
               (VEL_MODE_CUR_LIMIT_A > 0.0f) ? "set" : "motor default");
    }
    else
    {
        /* Written explicitly rather than relying on the factory default of 0:
           after a Velocity-mode session the motor RAM holds 2. */
        rs_ext_write_param_uint8(&hfdcan1, MOTOR_ID_LEFT,  MASTER_ID, RS_PARAM_RUN_MODE, RS_RUN_MODE_MIT);
        rs_ext_write_param_uint8(&hfdcan3, MOTOR_ID_RIGHT, MASTER_ID, RS_PARAM_RUN_MODE, RS_RUN_MODE_MIT);
        printf("[MOTOR] run_mode=0 (Operation Control / MIT, /cmd_mit or /cmd_torque)\r\n");
    }
}

/* ── Send one wheel's Velocity-mode setpoint, motor frame, rad/s ───── */
static void motor_send_velocity(hcan_t *hcan, uint8_t motor_id, float vel_rad_s)
{
    rs_ext_write_param_float(hcan, motor_id, MASTER_ID, RS_PARAM_SPD_REF, vel_rad_s);
}

/* ── Send one wheel's MIT setpoint, motor frame ────────────────────── */
static void motor_send_mit(hcan_t *hcan, uint8_t motor_id, const MotorMitCmd_t *c)
{
    rs_ext_control_cmd(hcan, motor_id, c->tau, c->pos, c->vel, c->kp, c->kd);
}

/* ── Hold both wheels still in whatever mode is applied ───────────────
 * Note the behavioural difference between the two modes. Velocity mode
 * commands 0 rad/s, which the PI loop *holds* — the robot actively fights being
 * pushed, up to the current limit. MIT mode applies damping only, so the wheels
 * resist motion but can still be pushed. If you would rather it free-wheel on
 * an e-stop, call rs_ext_disable() instead. */
static void motor_send_stop(CtrlMode_t mode)
{
    if (mode == CTRL_MODE_VELOCITY)
    {
        motor_send_velocity(&hfdcan1, MOTOR_ID_LEFT,  0.0f);
        motor_send_velocity(&hfdcan3, MOTOR_ID_RIGHT, 0.0f);
    }
    else
    {
        const MotorMitCmd_t stop = { 0.0f, 0.0f, 0.0f, MIT_STOP_KD, 0.0f };
        motor_send_mit(&hfdcan1, MOTOR_ID_LEFT,  &stop);
        motor_send_mit(&hfdcan3, MOTOR_ID_RIGHT, &stop);
    }
}

/* ── Apply a pending enable/disable, from Motor_task context only ──── */
static void motor_apply_enable(uint8_t enable)
{
    if (enable)
    {
        printf("[MOTOR] Enabling FDCAN1 & FDCAN3 motors...\r\n");
        /* A disable returns the motor to Reset state and the mode lives in RAM
           only, so re-assert it here rather than once at boot. */
        motor_config_mode(s_mode_applied);
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

/* ── Switch run_mode, from Motor_task context only ────────────────────
 * run_mode is only accepted while the motor is stopped (Reset state), so a
 * running motor goes stop -> run_mode -> enable. The wheels are unpowered for
 * the few frames in between. A disabled motor just records the new mode;
 * motor_apply_enable() writes it on the next enable. */
static void motor_apply_mode(CtrlMode_t mode)
{
    printf("[MOTOR] Control mode -> %s\r\n",
           (mode == CTRL_MODE_VELOCITY) ? "VELOCITY (/cmd_vel)" : "MIT (/cmd_mit or /cmd_torque)");
    s_mode_applied = mode;
    if (!g_system_enabled) return;

    rs_ext_disable(&hfdcan1, MOTOR_ID_LEFT,  MASTER_ID, 0);
    rs_ext_disable(&hfdcan3, MOTOR_ID_RIGHT, MASTER_ID, 0);
    motor_config_mode(mode);
    rs_ext_enable(&hfdcan1, MOTOR_ID_LEFT,  MASTER_ID);
    rs_ext_enable(&hfdcan3, MOTOR_ID_RIGHT, MASTER_ID);
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
/* ── Periodic diagnostics ────────────────────────────────────
 * Steady state is one short line every 2 s: is each motor answering, how fast
 * is it turning, how many encoder wraps has it counted, and are the RX
 * counters still climbing. That is the whole question while the robot works.
 *
 * The full dump — CAN error counters, protocol state, both encoder angles in
 * both units — costs ~620 characters and a dozen float conversions per shot.
 * It answers nothing when everything is fine and buries the one line that
 * does, so it now prints only on a fault, and no more often than
 * DIAG_DETAIL_MIN_MS: a motor left unplugged on the bench is a permanent
 * fault, and without that floor it would put the log straight back where it
 * started. Set MOTOR_DIAG_VERBOSE to 1 for the old unconditional behaviour.
 */
void motor_diag_print(void)
{
    static uint32_t last_tick   = 0U;
    static uint32_t last_detail = 0U;
    uint32_t now = HAL_GetTick();

    if ((now - last_tick) < DIAG_PERIOD_MS) return;
    last_tick = now;

    CAN_BusMetrics_t m;
    CAN_GetMetrics(&m);

    float p0, v0, t0, p1, v1, t1;
    uint8_t val0 = 0, val1 = 0;
    CAN_MotorFeedback_Get_Idx(0, &p0, &v0, &t0, &val0);
    CAN_MotorFeedback_Get_Idx(1, &p1, &v1, &t1, &val1);

    float raw0, cont0, raw1, cont1;
    int32_t turn0, turn1;
    CAN_MotorFeedback_GetAngle(0, &raw0, &cont0, &turn0, NULL);
    CAN_MotorFeedback_GetAngle(1, &raw1, &cont1, &turn1, NULL);

    /* A stale motor counts as a fault on purpose: unplugged, mis-addressed and
       bus-stopped all look the same from here, and all three want the detail. */
    uint8_t fault = (!val0 || !val1
                     || m.can1_tx_err          || m.can3_tx_err
                     || m.can1_proto.bus_off   || m.can3_proto.bus_off
                     || m.can1_proto.err_passive || m.can3_proto.err_passive) ? 1U : 0U;

    printf("[DIAG] L:%s v=%+5.2f n=%+ld | R:%s v=%+5.2f n=%+ld | RX=%lu/%lu%s\r\n",
           val0 ? "OK " : "OFF", (double)v0, (long)turn0,
           val1 ? "OK " : "OFF", (double)v1, (long)turn1,
           m.can1_rx_cnt, m.can3_rx_cnt,
           fault ? "  << FAULT" : "");

    uint8_t want_detail = (MOTOR_DIAG_VERBOSE != 0)
                          || (fault && (uint32_t)(now - last_detail) >= DIAG_DETAIL_MIN_MS);
    if (!want_detail) return;
    last_detail = now;

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

    /* Encoder angle in both forms. The pair is the diagnostic, not either value
       on its own: raw wraps inside [-4pi, +4pi] while cont has to stay
       continuous across those wraps, so a one-span step in cont, or a wrap
       count that climbs while the wheel is parked, points straight at dropped
       or mis-parsed feedback frames rather than at a real move. */
    printf("[CAN_ANG ] L: raw=%+6.2f cont=%+9.2f rad (%+9.1f deg) n=%+ld | R: raw=%+6.2f cont=%+9.2f rad (%+9.1f deg) n=%+ld\r\n",
           (double)raw0, (double)cont0, (double)(cont0 * RAD_TO_DEG), (long)turn0,
           (double)raw1, (double)cont1, (double)(cont1 * RAD_TO_DEG), (long)turn1);

    CAN_PrintDebugStats();

    /* Printed with the detail rather than the heartbeat: it only matters once
       you are reading a dump closely, and a non-zero count means the dump you
       are reading has holes in it. */
    printf("[UART7   ] debug log chars dropped since boot: %lu\r\n",
           debug_uart_tx_dropped());
}

/* ── Helper: clamp value ──────────────────────────────────────────── */
static inline float clampf(float val, float lo, float hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

/* Keep an MIT setpoint inside the 29-bit protocol's packing ranges. The driver
   would saturate anyway; clamping here makes the limit explicit at the call. */
static void mit_cmd_clamp(MotorMitCmd_t *c)
{
    c->pos = clampf(c->pos, RS_EXT_P_MIN,  RS_EXT_P_MAX);
    c->vel = clampf(c->vel, RS_EXT_V_MIN,  RS_EXT_V_MAX);
    c->kp  = clampf(c->kp,  RS_EXT_KP_MIN, RS_EXT_KP_MAX);
    c->kd  = clampf(c->kd,  RS_EXT_KD_MIN, RS_EXT_KD_MAX);
    c->tau = clampf(c->tau, RS_EXT_T_MIN,  RS_EXT_T_MAX);
}

/* ── Task body ────────────────────────────────────────────────────── */
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

    /* Require an explicit ROS or RC arm request after boot. */
    motor_apply_enable(0U);
    osDelay(100);

    uint8_t rc_arm_prev = 1U;  /* tracks sbus_is_armed() to fire on change only */

    next_wake = osKernelGetTickCount();

    for (;;)
    {
        /* Deterministic period: osDelayUntil() holds the rate regardless of how
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

        /* Switch E drives the same request the /motor_enable topic writes, so
           the two cannot fight over g_system_enabled. Only act on a change —
           otherwise the RC would pin the state and ROS could never enable or
           disable anything. */
        sbus_switch_update();
        if (sbus_is_online())
        {
            uint8_t rc_arm = sbus_is_armed() ? 1U : 0U;
            if (rc_arm != rc_arm_prev)
            {
                printf("[MOTOR] RC switch E -> %s\r\n", rc_arm ? "ENABLE" : "DISABLE");
                motor_enable_set(rc_arm);
                rc_arm_prev = rc_arm;
            }
        }

        /* Apply a pending /motor_enable request from the ROS callback. */
        uint8_t want_enable = s_enable_request;
        if (want_enable != g_system_enabled)
        {
            printf("[MOTOR] Enable state request: %u (current: %u)\r\n", want_enable, g_system_enabled);
            motor_apply_enable(want_enable);
        }

        /* Snapshot the latest command from whichever topic wrote last. */
        float effective_vx, effective_wz;
        MotorMitCmd_t mit[2];
        CtrlMode_t want_mode;
        uint32_t cmd_tick;
        taskENTER_CRITICAL();
        effective_vx = g_cmd_velocity;
        effective_wz = g_cmd_yaw_rate;
        mit[0]       = s_mit_cmd[0];
        mit[1]       = s_mit_cmd[1];
        want_mode    = s_mode_request;
        cmd_tick     = s_cmd_last_tick;
        taskEXIT_CRITICAL();

        uint8_t effective_enable = g_system_enabled;

        /* ── Command watchdog ───────────────────────────────────────
         * Hold the last command only while it is fresh. If the agent, the USB
         * link or the publisher dies, the robot must stop instead of driving on
         * the final command forever. Applies to whichever topic is active. */
        uint8_t cmd_stale = ((HAL_GetTick() - cmd_tick) > CMD_VEL_TIMEOUT_MS) ? 1U : 0U;
        if (cmd_stale != s_cmd_timed_out)
        {
            s_cmd_timed_out = cmd_stale;
            printf("[MOTOR] %s %s\r\n",
                   (want_mode == CTRL_MODE_VELOCITY) ? "/cmd_vel" : "/cmd_mit|/cmd_torque",
                   cmd_stale ? "TIMEOUT — holding stop" : "resumed");
        }

        /* ── SBUS Remote Controller Arbitration ─────────────────── */
        uint8_t rc_manual = 0U;
        RC_ControlMode_t rc_mode = sbus_get_control_mode();
        if (rc_mode == RC_MODE_EMERGENCY_STOP)
        {
            /* Switch F, or no RC link at all: nothing drives the wheels. */
            effective_enable = 0U;
        }
        else
        {
            if (rc_mode == RC_MODE_MANUAL_RC)
            {
                /* Manual Teleop: Max 1.0 m/s linear, 2.5 rad/s turn. The stick
                   gives a (v, w) pair, so it always runs in Velocity mode. */
                rc_manual = 1U;
                sbus_get_motion_cmd(1.0f, 2.5f, &effective_vx, &effective_wz);
                effective_enable = sbus_is_armed() ? 1U : 0U;
            }
            /* RC_MODE_AUTO_ROS2 keeps the ROS 2 command topics. */

            /* Switch E outranks the mode switch: disabled keeps both wheels
               dead whether the command came from RC or from ROS. */
            if (!sbus_is_armed())
            {
                effective_enable = 0U;
            }
        }

        CtrlMode_t mode = rc_manual ? CTRL_MODE_VELOCITY : want_mode;
        if (mode != s_mode_applied)
        {
            motor_apply_mode(mode);
        }

        /* Nếu hệ thống bị Disable hoặc mất lệnh: dừng giữ motor. */
        if (!effective_enable || (cmd_stale && !rc_manual))
        {
            motor_send_stop(mode);
            continue;
        }

        if (mode == CTRL_MODE_VELOCITY)
        {
            RobotCmd_t cmd;
            cmd.velocity = effective_vx;   /* m/s */
            cmd.yaw_rate = effective_wz;   /* rad/s */
            cmd.enable   = effective_enable;

            float target_left_vel  = 0.0f;   /* rad/s */
            float target_right_vel = 0.0f;   /* rad/s */

            /* 1. Động học vi sai (Kinematics): (v, w) -> (w_left, w_right), motor frame */
            robot_control_algo(&cmd, &target_left_vel, &target_right_vel);

            /* 2. Giới hạn an toàn [-20, +20 rad/s] — dải định mức của actuator.
                  Scale both wheels by the same factor instead of clamping each
                  one: clamping only the outer wheel in a fast turn changes the
                  L/R ratio, so the robot turns less than commanded and leaves
                  the path Nav2 planned. Scaling keeps the curvature w/v and
                  only slows the robot down.
                  Vòng kín vận tốc chạy TRONG motor (PI trên SPD_REF), sai số
                  tĩnh = 0, gia tốc giới hạn bởi VEL_MODE_ACC_RAD. */
            float peak = fmaxf(fabsf(target_left_vel), fabsf(target_right_vel));
            if (peak > RS_EXT_V_MAX)
            {
                float scale = RS_EXT_V_MAX / peak;
                target_left_vel  *= scale;
                target_right_vel *= scale;
            }
            /* Kept as a backstop for rounding; a no-op after the scale. */
            target_left_vel  = clampf(target_left_vel,  RS_EXT_V_MIN, RS_EXT_V_MAX);
            target_right_vel = clampf(target_right_vel, RS_EXT_V_MIN, RS_EXT_V_MAX);

            motor_send_velocity(&hfdcan1, MOTOR_ID_LEFT,  target_left_vel);
            motor_send_velocity(&hfdcan3, MOTOR_ID_RIGHT, target_right_vel);
        }
        else
        {
            mit_cmd_clamp(&mit[0]);
            mit_cmd_clamp(&mit[1]);
            motor_send_mit(&hfdcan1, MOTOR_ID_LEFT,  &mit[0]);
            motor_send_mit(&hfdcan3, MOTOR_ID_RIGHT, &mit[1]);
        }
    }
}

