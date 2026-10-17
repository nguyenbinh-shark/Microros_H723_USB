#ifndef ROBSTRIDE_DRV_H
#define ROBSTRIDE_DRV_H

#include "main.h"
#include "can_bsp.h"

/* -------------------------------------------------------------------------- */
/*  MIT 11-BIT MODE ranges. Belong to the classic MIT-Cheetah protocol and are */
/*  used ONLY by rs_mit_cmd/rs_enable/rs_disable/rs_parse_feedback at the      */
/*  bottom of this header. The firmware runs on 29-bit extended, so it scales  */
/*  against the RS_EXT_* block below instead. The two sets differ (V 33 vs 20, */
/*  KP 500 vs 5000, KD 5 vs 100) — each belongs to its own protocol, never     */
/*  reach for an RS_* constant when you mean RS_EXT_*.                          */
/* -------------------------------------------------------------------------- */
#define RS_P_MIN   (-12.57f)
#define RS_P_MAX   (12.57f)
#define RS_V_MIN   (-33.0f)
#define RS_V_MAX   (33.0f)
#define RS_KP_MIN  (0.0f)
#define RS_KP_MAX  (500.0f)
#define RS_KD_MIN  (0.0f)
#define RS_KD_MAX  (5.0f)
#define RS_T_MIN   (-60.0f)
#define RS_T_MAX   (60.0f)

#define RS_MODE_MIT 0x000u

/* -------------------------------------------------------------------------- */
/*                  RobStride 29-Bit Extended CAN Protocol                    */
/* -------------------------------------------------------------------------- */
/* 29-bit ID Bitfields: Bit 28..24 Mode (5bit), Bit 23..8 Data (16bit), Bit 7..0 ID (8bit) */
#define RS_CAN_TYPE_GET_ID       0x00u /* Get Device ID & MCU UID */
#define RS_CAN_TYPE_CONTROL      0x01u /* Operation control mode motor control instruction */
#define RS_CAN_TYPE_FEEDBACK     0x02u /* Motor feedback data */
#define RS_CAN_TYPE_ENABLE       0x03u /* Motor enable to run */
#define RS_CAN_TYPE_STOP         0x04u /* Motor stop running */
#define RS_CAN_TYPE_SET_ZERO     0x06u /* Set motor mechanical zero */
#define RS_CAN_TYPE_SET_CAN_ID   0x07u /* Set motor CAN_ID */
#define RS_CAN_TYPE_READ_PARAM   0x11u /* Single parameter read (17) */
#define RS_CAN_TYPE_WRITE_PARAM  0x12u /* Single parameter write (18) */
#define RS_CAN_TYPE_FAULT_FB     0x15u /* Fault feedback frame (21) */
#define RS_CAN_TYPE_SAVE_PARAM   0x16u /* Motor data save frame (22) */
#define RS_CAN_TYPE_SET_BAUD     0x17u /* Motor baud rate modification (23) */
#define RS_CAN_TYPE_ACTIVE_RPT   0x18u /* Motor active reporting frame (24) */
#define RS_CAN_TYPE_PROTOCOL_SW  0x19u /* Motor protocol modification frame (25) */
#define RS_CAN_TYPE_READ_VER     0x1Au /* Version number read frame (26) */

/* ACTIVE — 29-Bit Extended Protocol motion-control ranges (manual section 4).
   These are the ones rs_ext_control_cmd() scales against. */
#define RS_EXT_P_MIN   (-12.57f)
#define RS_EXT_P_MAX   (12.57f)
#define RS_EXT_V_MIN   (-20.0f)
#define RS_EXT_V_MAX   (20.0f)
#define RS_EXT_KP_MIN  (0.0f)
#define RS_EXT_KP_MAX  (5000.0f)
#define RS_EXT_KD_MIN  (0.0f)
#define RS_EXT_KD_MAX  (100.0f)
#define RS_EXT_T_MIN   (-60.0f)
#define RS_EXT_T_MAX   (60.0f)

/* Speed scale of the type-2 / type-24 FEEDBACK frame, separate from the command
   range above. The actuators fitted to RPBot report speed over +-30 rad/s
   (the CyberGear feedback range). Decoding it with the +-20 rad/s command range
   made /motor_fb velocity, and every wheel angle integrated from it, 0.67x too
   small: measured 2026-10-17 against LiDAR scan matching and the IMU yaw with
   the 0.060 m wheel. The command range stays at +-20 rad/s on purpose. */
#define RS_EXT_FB_V_MIN (-30.0f)
#define RS_EXT_FB_V_MAX (30.0f)

/* Continuous-angle accumulator (rs_ext_parse_feedback) tuning.
   RAIL_LSB    how close to code 0x0000 / 0xFFFF still counts as "the reported
               angle has saturated". 64 LSB is ~0.025 rad. It has to be wider
               than a couple of codes: a motor that clamps at exactly 4pi
               (12.56637) lands ~10 codes short of the top of this 12.57 scale,
               so a 1-2 LSB test would never see the rail at all.
   ACC_MAX_DT  ceiling in seconds on a single velocity-integration step, so a
               hole in the feedback stream cannot inject one large jump.
   ACC_VEL_DB  rad/s below which the shaft counts as parked, so velocity noise
               cannot creep the angle while it rests against a rail. */
#define RS_FB_RAIL_LSB    64u
#define RS_FB_ACC_MAX_DT  0.05f
#define RS_FB_ACC_VEL_DB  0.02f

/* Common Parameter Indexes for Read/Write Single Parameter (Type 17/18) */
#define RS_PARAM_RUN_MODE     0x7005u /* uint8_t: 0=Control/MIT, 1=PP, 2=Velocity, 3=Current, 5=CSP */
#define RS_PARAM_IQ_REF       0x7006u /* float: Target current in Current mode (A) */
#define RS_PARAM_SPD_REF      0x700Au /* float: Target speed in Velocity mode (rad/s) */
#define RS_PARAM_LIMIT_TORQUE 0x700Bu /* float: Max torque limit (Nm) */
#define RS_PARAM_CUR_KP       0x7010u /* float: Current loop Kp */
#define RS_PARAM_CUR_KI       0x7011u /* float: Current loop Ki */
#define RS_PARAM_ACC_RAD      0x7014u /* float: Acceleration in Velocity mode (rad/s^2) */
#define RS_PARAM_LOC_REF      0x7016u /* float: Target position in Position mode (rad) */
#define RS_PARAM_LIMIT_SPD    0x7017u /* float: Speed limit in CSP position mode (rad/s) */
#define RS_PARAM_LIMIT_CUR    0x7018u /* float: Current limit in Velocity mode (A) */
#define RS_PARAM_VEL_MAX      0x701Bu /* float: Max velocity in PP position mode */
#define RS_PARAM_ACC_SET      0x701Cu /* float: Acceleration in PP position mode */

/* Default Master CAN ID if none specified */
#define RS_DEFAULT_MASTER_ID  0xFD

/* -------------------------------------------------------------------------- */
/*                               Data Structures                              */
/* -------------------------------------------------------------------------- */

/* MIT 11-BIT MODE feedback structure — filled by rs_parse_feedback(). Unused
   while the firmware runs 29-bit; the active slot type is rs_ext_fb_t below. */
typedef struct
{
    uint8_t  id;          /* Motor CAN ID from feedback byte0 */
    uint8_t  state;       /* Not provided in MIT feedback; kept for compatibility */
    uint16_t p_raw;
    uint16_t v_raw;
    uint16_t t_raw;
    float    pos_raw;     /* Single-turn raw position rad [-4π, 4π] */
    float    pos;         /* Continuous multi-turn accumulated angle rad */
    float    last_raw_pos;/* Last received raw position for delta */
    int32_t  round_cnt;   /* Number of full multi-turn wraps */
    uint8_t  init_done;   /* 1 = First sample initialized */
    float    vel;         /* rad/s */
    float    tor;         /* Nm */
    float    temp;        /* degC, from bytes6-7 / 10 */
} rs_fb_t;

/* ACTIVE — 29-bit extended feedback structure; this is what can_bsp.c stores
   per motor slot and what the micro-ROS odometry is published from. */
typedef struct
{
    uint8_t  motor_id;    /* Motor CAN ID (Bit 15..8 of CAN ID) */
    uint8_t  master_id;   /* Host CAN ID (Bit 7..0 of CAN ID) */
    uint8_t  mode_stat;   /* 0: Reset, 1: Cali, 2: Run */
    uint8_t  fault_code;  /* Bitfield of motor faults */
    float    pos_raw;     /* Reported angle rad, exactly as it came off the wire,
                             confined to [-4π, 4π] */
    float    pos;         /* Continuous multi-turn angle rad, no [-4π, 4π] ceiling */
    float    last_raw_pos;/* Previous raw sample. Diagnostics only now that the
                             accumulator re-anchors instead of summing deltas */
    int32_t  round_cnt;   /* Whole [-4π, 4π] spans (4 revolutions each) folded into pos */
    uint8_t  init_done;   /* 1 = first sample seen, accumulator anchored */
    uint8_t  at_rail;     /* 1 = last sample sat on an end code of the reported
                             range, so pos is riding the reported velocity until
                             the shaft comes back inside it */
    float    vel;         /* rad/s [-20, 20] */
    float    tor;         /* Nm [-60, 60] */
    float    temp;        /* °C */
    uint32_t last_rx_tick;/* HAL Tick of last received frame */
} rs_ext_fb_t;

/* -------------------------------------------------------------------------- */
/*                            Function Prototypes                             */
/* -------------------------------------------------------------------------- */

/* Helper to build 29-bit CAN ID */
uint32_t rs_ext_build_id(uint8_t comm_type, uint16_t data2, uint8_t target_id);

/* 29-bit CAN Protocol Motor Commands */
int  rs_ext_enable(hcan_t *hcan, uint8_t target_id, uint8_t master_id);
int  rs_ext_disable(hcan_t *hcan, uint8_t target_id, uint8_t master_id, uint8_t clear_fault);
int  rs_ext_set_zero(hcan_t *hcan, uint8_t target_id, uint8_t master_id);
int  rs_ext_control_cmd(hcan_t *hcan, uint8_t target_id, float torque, float pos, float vel, float kp, float kd);

/* Parameter Read/Write & Save Functions */
int  rs_ext_write_param_float(hcan_t *hcan, uint8_t target_id, uint8_t master_id, uint16_t index, float val);
int  rs_ext_write_param_uint8(hcan_t *hcan, uint8_t target_id, uint8_t master_id, uint16_t index, uint8_t val);
int  rs_ext_read_param(hcan_t *hcan, uint8_t target_id, uint8_t master_id, uint16_t index);
int  rs_ext_save_param(hcan_t *hcan, uint8_t target_id, uint8_t master_id);
int  rs_ext_get_device_id(hcan_t *hcan, uint8_t target_id, uint8_t master_id);

/* Feedback parsing for 29-bit CAN */
int  rs_ext_parse_feedback(rs_ext_fb_t *fb, uint32_t rx_ext_id, const uint8_t *rx_data, uint32_t len);

/* MIT 11-BIT MODE — complete alternative protocol (classic MIT-Cheetah
   framing: 11-bit standard ID, P16|V12|Kp12|Kd12|T12 packing, FF..FC / FF..FD
   enable/disable). Working code, simply not called while the firmware drives
   the wheels over 29-bit extended. For MIT-Cheetah / Damiao actuators, or
   RobStride units switched into MIT mode.
   Sole users of canx_send_data(), rs_fb_t and the RS_* ranges at the top of
   this header — if this block ever goes, those go with it. */
int  rs_mit_cmd(hcan_t *hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq);
void rs_enable(hcan_t *hcan, uint16_t motor_id);
void rs_disable(hcan_t *hcan, uint16_t motor_id);
void rs_parse_feedback(rs_fb_t *fb, const uint8_t *rx_data, uint32_t len);

#endif /* ROBSTRIDE_DRV_H */
