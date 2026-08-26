#include "robstride_drv.h"
#include "fdcan.h"
#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1; /* Provided by fdcan.c */
extern FDCAN_HandleTypeDef hfdcan3; /* Provided by fdcan.c */

/* -------------------------------------------------------------------------- */
/*                              Helper Functions                              */
/* -------------------------------------------------------------------------- */

static inline int float_to_uint(float x, float x_min, float x_max, int bits)
{
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static inline float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

uint32_t rs_ext_build_id(uint8_t comm_type, uint16_t data2, uint8_t target_id)
{
    return (((uint32_t)comm_type & 0x1F) << 24) |
           (((uint32_t)data2 & 0xFFFF) << 8) |
           ((uint32_t)target_id & 0xFF);
}

/* -------------------------------------------------------------------------- */
/*                  29-Bit Extended CAN Protocol Implementation               */
/* -------------------------------------------------------------------------- */

/**
  * @brief  Enable motor in 29-bit CAN protocol (Communication Type 3)
  */
int rs_ext_enable(hcan_t *hcan, uint8_t target_id, uint8_t master_id)
{
    if (!hcan) return -1;
    uint8_t data[8] = {0};
    uint32_t ext_id = rs_ext_build_id(RS_CAN_TYPE_ENABLE, master_id, target_id);
    return canx_send_ext_data(hcan, ext_id, data, 8);
}

/**
  * @brief  Stop / Disable motor in 29-bit CAN protocol (Communication Type 4)
  */
int rs_ext_disable(hcan_t *hcan, uint8_t target_id, uint8_t master_id, uint8_t clear_fault)
{
    if (!hcan) return -1;
    uint8_t data[8] = {0};
    data[0] = clear_fault ? 1 : 0;
    uint32_t ext_id = rs_ext_build_id(RS_CAN_TYPE_STOP, master_id, target_id);
    return canx_send_ext_data(hcan, ext_id, data, 8);
}

/**
  * @brief  Set motor mechanical zero in 29-bit CAN protocol (Communication Type 6)
  */
int rs_ext_set_zero(hcan_t *hcan, uint8_t target_id, uint8_t master_id)
{
    if (!hcan) return -1;
    uint8_t data[8] = {0};
    data[0] = 1;
    uint32_t ext_id = rs_ext_build_id(RS_CAN_TYPE_SET_ZERO, master_id, target_id);
    return canx_send_ext_data(hcan, ext_id, data, 8);
}

/**
  * @brief  Send operation control mode instruction (Communication Type 1)
  *         t_ref = Kd * (v_set - v_act) + Kp * (p_set - p_act) + t_ff
  */
int rs_ext_control_cmd(hcan_t *hcan, uint8_t target_id, float torque, float pos, float vel, float kp, float kd)
{
    if (!hcan) return -1;

    uint16_t t_raw   = (uint16_t)float_to_uint(torque, RS_EXT_T_MIN,  RS_EXT_T_MAX,  16);
    uint16_t pos_raw = (uint16_t)float_to_uint(pos,    RS_EXT_P_MIN,  RS_EXT_P_MAX,  16);
    uint16_t vel_raw = (uint16_t)float_to_uint(vel,    RS_EXT_V_MIN,  RS_EXT_V_MAX,  16);
    uint16_t kp_raw  = (uint16_t)float_to_uint(kp,     RS_EXT_KP_MIN, RS_EXT_KP_MAX, 16);
    uint16_t kd_raw  = (uint16_t)float_to_uint(kd,     RS_EXT_KD_MIN, RS_EXT_KD_MAX, 16);

    /* Build 29-bit CAN ID with target torque in bit23..8 */
    uint32_t ext_id = rs_ext_build_id(RS_CAN_TYPE_CONTROL, t_raw, target_id);

    uint8_t data[8];
    data[0] = (uint8_t)(pos_raw >> 8);
    data[1] = (uint8_t)(pos_raw & 0xFF);
    data[2] = (uint8_t)(vel_raw >> 8);
    data[3] = (uint8_t)(vel_raw & 0xFF);
    data[4] = (uint8_t)(kp_raw >> 8);
    data[5] = (uint8_t)(kp_raw & 0xFF);
    data[6] = (uint8_t)(kd_raw >> 8);
    data[7] = (uint8_t)(kd_raw & 0xFF);

    return canx_send_ext_data(hcan, ext_id, data, 8);
}

/**
  * @brief  Write single float parameter (Communication Type 18)
  */
int rs_ext_write_param_float(hcan_t *hcan, uint8_t target_id, uint8_t master_id, uint16_t index, float val)
{
    if (!hcan) return -1;
    uint8_t data[8] = {0};

    data[0] = (uint8_t)(index & 0xFF);
    data[1] = (uint8_t)((index >> 8) & 0xFF);
    data[2] = 0x00;
    data[3] = 0x00;

    memcpy(&data[4], &val, 4);

    uint32_t ext_id = rs_ext_build_id(RS_CAN_TYPE_WRITE_PARAM, master_id, target_id);
    return canx_send_ext_data(hcan, ext_id, data, 8);
}

/**
  * @brief  Write single uint8 parameter (Communication Type 18, e.g. RS_PARAM_RUN_MODE)
  */
int rs_ext_write_param_uint8(hcan_t *hcan, uint8_t target_id, uint8_t master_id, uint16_t index, uint8_t val)
{
    if (!hcan) return -1;
    uint8_t data[8] = {0};

    data[0] = (uint8_t)(index & 0xFF);
    data[1] = (uint8_t)((index >> 8) & 0xFF);
    data[2] = 0x00;
    data[3] = 0x00;
    data[4] = val;

    uint32_t ext_id = rs_ext_build_id(RS_CAN_TYPE_WRITE_PARAM, master_id, target_id);
    return canx_send_ext_data(hcan, ext_id, data, 8);
}

/**
  * @brief  Read single parameter (Communication Type 17)
  */
int rs_ext_read_param(hcan_t *hcan, uint8_t target_id, uint8_t master_id, uint16_t index)
{
    if (!hcan) return -1;
    uint8_t data[8] = {0};

    data[0] = (uint8_t)(index & 0xFF);
    data[1] = (uint8_t)((index >> 8) & 0xFF);

    uint32_t ext_id = rs_ext_build_id(RS_CAN_TYPE_READ_PARAM, master_id, target_id);
    return canx_send_ext_data(hcan, ext_id, data, 8);
}

/**
  * @brief  Save parameters to Flash (Communication Type 22)
  */
int rs_ext_save_param(hcan_t *hcan, uint8_t target_id, uint8_t master_id)
{
    if (!hcan) return -1;
    uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    uint32_t ext_id = rs_ext_build_id(RS_CAN_TYPE_SAVE_PARAM, master_id, target_id);
    return canx_send_ext_data(hcan, ext_id, data, 8);
}

/**
  * @brief  Get Device ID (Communication Type 0)
  */
int rs_ext_get_device_id(hcan_t *hcan, uint8_t target_id, uint8_t master_id)
{
    if (!hcan) return -1;
    uint8_t data[8] = {0};
    uint32_t ext_id = rs_ext_build_id(RS_CAN_TYPE_GET_ID, master_id, target_id);
    return canx_send_ext_data(hcan, ext_id, data, 8);
}

/**
  * @brief  Parse motor feedback frame (Communication Type 2)
  */
int rs_ext_parse_feedback(rs_ext_fb_t *fb, uint32_t rx_ext_id, const uint8_t *rx_data, uint32_t len)
{
    if (!fb || !rx_data || len != FDCAN_DLC_BYTES_8)
    {
        return -1;
    }

    uint8_t comm_type = (rx_ext_id >> 24) & 0x1F;
    if (comm_type != RS_CAN_TYPE_FEEDBACK)
    {
        return -2; /* Not a Type 2 feedback frame */
    }

    fb->mode_stat  = (rx_ext_id >> 22) & 0x03;
    fb->fault_code = (rx_ext_id >> 16) & 0x3F;
    fb->motor_id   = (rx_ext_id >> 8) & 0xFF;
    fb->master_id  = rx_ext_id & 0xFF;

    uint16_t pos_raw = ((uint16_t)rx_data[0] << 8) | rx_data[1];
    uint16_t vel_raw = ((uint16_t)rx_data[2] << 8) | rx_data[3];
    uint16_t tor_raw = ((uint16_t)rx_data[4] << 8) | rx_data[5];
    uint16_t tmp_raw = ((uint16_t)rx_data[6] << 8) | rx_data[7];

    fb->pos  = uint_to_float(pos_raw, RS_EXT_P_MIN, RS_EXT_P_MAX, 16);
    fb->vel  = uint_to_float(vel_raw, RS_EXT_V_MIN, RS_EXT_V_MAX, 16);
    fb->tor  = uint_to_float(tor_raw, RS_EXT_T_MIN, RS_EXT_T_MAX, 16);
    fb->temp = ((float)tmp_raw) / 10.0f;
    fb->last_rx_tick = HAL_GetTick();

    return 0;
}

/* -------------------------------------------------------------------------- */
/*                  Classic 11-Bit MIT Functions (Backward Compatibility)    */
/* -------------------------------------------------------------------------- */

void rs_parse_feedback(rs_fb_t *fb, const uint8_t *rx_data, uint32_t len)
{
    if (!fb || !rx_data || len != FDCAN_DLC_BYTES_8)
    {
        return;
    }

    fb->id    = rx_data[0];
    fb->state = 0;

    fb->p_raw = ((uint16_t)rx_data[1] << 8) | rx_data[2];
    fb->v_raw = ((uint16_t)rx_data[3] << 4) | (rx_data[4] >> 4);
    fb->t_raw = (((uint16_t)(rx_data[4] & 0x0Fu)) << 8) | rx_data[5];

    fb->pos = uint_to_float(fb->p_raw, RS_P_MIN, RS_P_MAX, 16);
    fb->vel = uint_to_float(fb->v_raw, RS_V_MIN, RS_V_MAX, 12);
    fb->tor = uint_to_float(fb->t_raw, RS_T_MIN, RS_T_MAX, 12);

    uint16_t temp_raw = ((uint16_t)rx_data[6] << 8) | rx_data[7];
    fb->temp = ((float)temp_raw) / 10.0f;
}

int rs_mit_cmd(hcan_t *hcan, uint16_t motor_id, float pos, float vel, float kp, float kd, float torq)
{
    if (!hcan)
    {
        return -1;
    }

    uint8_t data[8];
    uint16_t pos_tmp = (uint16_t)float_to_uint(pos, RS_P_MIN, RS_P_MAX, 16);
    uint16_t vel_tmp = (uint16_t)float_to_uint(vel, RS_V_MIN, RS_V_MAX, 12);
    uint16_t kp_tmp  = (uint16_t)float_to_uint(kp,  RS_KP_MIN, RS_KP_MAX, 12);
    uint16_t kd_tmp  = (uint16_t)float_to_uint(kd,  RS_KD_MIN, RS_KD_MAX, 12);
    uint16_t tor_tmp = (uint16_t)float_to_uint(torq,RS_T_MIN, RS_T_MAX, 12);

    data[0] = (uint8_t)(pos_tmp >> 8);
    data[1] = (uint8_t)(pos_tmp);
    data[2] = (uint8_t)(vel_tmp >> 4);
    data[3] = (uint8_t)(((vel_tmp & 0x0Fu) << 4) | (kp_tmp >> 8));
    data[4] = (uint8_t)(kp_tmp);
    data[5] = (uint8_t)(kd_tmp >> 4);
    data[6] = (uint8_t)(((kd_tmp & 0x0Fu) << 4) | (tor_tmp >> 8));
    data[7] = (uint8_t)(tor_tmp);

    uint16_t id = motor_id + RS_MODE_MIT;
    return canx_send_data(hcan, id, data, 8);
}

void rs_enable(hcan_t *hcan, uint16_t motor_id)
{
    if (!hcan) return;
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    uint16_t id = motor_id + RS_MODE_MIT;
    canx_send_data(hcan, id, data, 8);
}

void rs_disable(hcan_t *hcan, uint16_t motor_id)
{
    if (!hcan) return;
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    uint16_t id = motor_id + RS_MODE_MIT;
    canx_send_data(hcan, id, data, 8);
}

