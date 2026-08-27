#ifndef ROBSTRIDE_DRV_H
#define ROBSTRIDE_DRV_H

#include "main.h"
#include "can_bsp.h"

/* -------------------------------------------------------------------------- */
/*                        RobStride MIT Protocol Ranges                       */
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

/* 29-Bit Extended Protocol Motion Control Ranges (from manual section 4) */
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

/* Classic 11-bit Feedback structure */
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

/* 29-bit Extended CAN Feedback structure */
typedef struct
{
    uint8_t  motor_id;    /* Motor CAN ID (Bit 15..8 of CAN ID) */
    uint8_t  master_id;   /* Host CAN ID (Bit 7..0 of CAN ID) */
    uint8_t  mode_stat;   /* 0: Reset, 1: Cali, 2: Run */
    uint8_t  fault_code;  /* Bitfield of motor faults */
    float    pos_raw;     /* Single-turn raw position rad [-4π, 4π] */
    float    pos;         /* Continuous multi-turn accumulated angle rad */
    float    last_raw_pos;/* Last received raw position for delta */
    int32_t  round_cnt;   /* Number of full multi-turn wraps */
    uint8_t  init_done;   /* 1 = First sample initialized */
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

/* Classic 11-bit MIT Functions (backward compatibility) */
int  rs_mit_cmd(hcan_t *hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq);
void rs_enable(hcan_t *hcan, uint16_t motor_id);
void rs_disable(hcan_t *hcan, uint16_t motor_id);
void rs_parse_feedback(rs_fb_t *fb, const uint8_t *rx_data, uint32_t len);

#endif /* ROBSTRIDE_DRV_H */
