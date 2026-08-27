#include "motor_drv.h"

#include <string.h>

static uint32_t cybergear_build_extid(uint8_t comm_type, uint16_t data_area, uint8_t motor_id)
{
    return (((uint32_t)comm_type & 0x1FU) << 24) |
           (((uint32_t)data_area & 0xFFFFU) << 8) |
           ((uint32_t)motor_id & 0xFFU);
}

static int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x_float > x_max)
    {
        x_float = x_max;
    }
    else if (x_float < x_min)
    {
        x_float = x_min;
    }
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void cybergear_fbdata(Wheel_Motor_t *motor, uint8_t *rx_data, uint32_t data_len)
{
    if (data_len == FDCAN_DLC_BYTES_8)
    {
        /* CyberGear feedback frame (comm type 2), manual §4.1.3:
         *   bytes 0-1: position (16-bit, ±4π rad)
         *   bytes 2-3: velocity (16-bit, ±30 rad/s)
         *   bytes 4-5: torque   (16-bit, ±12 Nm)
         *   bytes 6-7: temperature (16-bit signed, °C × 10)
         * motor id / fault / mode are carried in the ExtID, parsed in can_bsp.c */
        motor->para.p_int = (rx_data[0] << 8) | rx_data[1];
        motor->para.v_int = (rx_data[2] << 8) | rx_data[3];
        motor->para.t_int = (rx_data[4] << 8) | rx_data[5];

        motor->para.pos = uint_to_float(motor->para.p_int, CYBERGEAR_FB_P_MIN, CYBERGEAR_FB_P_MAX, 16);
        motor->para.vel = uint_to_float(motor->para.v_int, CYBERGEAR_FB_V_MIN, CYBERGEAR_FB_V_MAX, 16);
        motor->para.tor = uint_to_float(motor->para.t_int, CYBERGEAR_FB_T_MIN, CYBERGEAR_FB_T_MAX, 16);

        int16_t temp_raw = (int16_t)((rx_data[6] << 8) | rx_data[7]);
        motor->para.Tmos  = ((float)temp_raw) * 0.1f;   /* °C */
        motor->para.Tcoil = motor->para.Tmos;           /* only one temperature field in feedback */
    }
}

void enable_motor_mode(hcan_t *hcan, uint32_t motor_id, uint32_t mode_id)
{
    uint8_t data[8];
    uint32_t id = cybergear_build_extid(CYBERGEAR_COMM_ENABLE, CYBERGEAR_MASTER_ID, (uint8_t)motor_id);
    (void)mode_id;

    memset(data, 0, sizeof(data));
    canx_send_data(hcan, id, data, 8);
}

void disable_motor_mode(hcan_t *hcan, uint32_t motor_id, uint32_t mode_id)
{
    uint8_t data[8];
    uint32_t id = cybergear_build_extid(CYBERGEAR_COMM_STOP, CYBERGEAR_MASTER_ID, (uint8_t)motor_id);
    (void)mode_id;

    memset(data, 0, sizeof(data));
    canx_send_data(hcan, id, data, 8);
}

void mit_ctrl(hcan_t *hcan, uint32_t motor_id, float pos, float vel, float kp, float kd, float torq)
{
    /* CyberGear operation-control frame (comm type 1), manual §4.1.2 / §4.2.1:
     *   ExtID data-area-2 (bits 23..8) = torque (16-bit, -12..12 Nm)
     *   Data bytes: position(16) | velocity(16) | Kp(16) | Kd(16), big-endian
     * (NOT the MIT-Cheetah P16/V12/Kp12/Kd12/T12 packing.) */
    uint8_t  data[8];
    uint16_t pos_tmp;
    uint16_t vel_tmp;
    uint16_t kp_tmp;
    uint16_t kd_tmp;
    uint16_t tor_tmp;
    uint32_t id;

    pos_tmp = (uint16_t)float_to_uint(pos,  CYBERGEAR_P_MIN,  CYBERGEAR_P_MAX,  16);
    vel_tmp = (uint16_t)float_to_uint(vel,  CYBERGEAR_V_MIN,  CYBERGEAR_V_MAX,  16);
    kp_tmp  = (uint16_t)float_to_uint(kp,   CYBERGEAR_KP_MIN, CYBERGEAR_KP_MAX, 16);
    kd_tmp  = (uint16_t)float_to_uint(kd,   CYBERGEAR_KD_MIN, CYBERGEAR_KD_MAX, 16);
    tor_tmp = (uint16_t)float_to_uint(torq, CYBERGEAR_T_MIN,  CYBERGEAR_T_MAX,  16);

    /* torque is carried in the ExtID data-area-2 (bits 23..8), not in data bytes */
    id = cybergear_build_extid(CYBERGEAR_COMM_OPERATION_CTRL, tor_tmp, (uint8_t)motor_id);

    data[0] = (uint8_t)(pos_tmp >> 8);
    data[1] = (uint8_t)(pos_tmp);
    data[2] = (uint8_t)(vel_tmp >> 8);
    data[3] = (uint8_t)(vel_tmp);
    data[4] = (uint8_t)(kp_tmp >> 8);
    data[5] = (uint8_t)(kp_tmp);
    data[6] = (uint8_t)(kd_tmp >> 8);
    data[7] = (uint8_t)(kd_tmp);

    canx_send_data(hcan, id, data, 8);
}
