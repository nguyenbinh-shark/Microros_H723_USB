/**
 * @file  cybergear_drv.h
 * @brief Xiaomi CyberGear actuator driver — REFERENCE ONLY, NOT USED BY THE FIRMWARE.
 *
 * The robot drives both wheels through robstride_drv.c (RobStride, 29-bit
 * extended protocol). Nothing in the project includes this header — it is kept
 * as a worked example of the CyberGear frame layout for anyone porting to that
 * actuator. The .c still builds, but -ffunction-sections + --gc-sections drop
 * every symbol from the final image, so it costs no flash.
 *
 * @note CyberGear speaks 29-bit extended frames EXCLUSIVELY. The protocol has no
 *       11-bit / standard-ID mode: comm type, host ID and target ID are all packed
 *       into the extended identifier itself (see cybergear_build_extid below).
 *       MIT 11-bit framing is a different protocol and lives in its own block in
 *       robstride_drv.c — the two are unrelated, do not mix them up.
 *
 * @warning Do not wire this in as-is. The senders here build the 29-bit ID
 *          correctly, then hand it to canx_send_data(), which transmits with
 *          FDCAN_STANDARD_ID and takes `id` as uint16_t. The identifier is
 *          truncated and the frame leaves as a standard frame no CyberGear will
 *          ever answer. Point those three calls at canx_send_ext_data() instead,
 *          the way robstride_drv.c does.
 *
 * @note All public symbols carry the cybergear_ prefix so they never read as the
 *       active wheel-motor API. See docs/porting.md for the porting procedure.
 */
#ifndef __CYBERGEAR_DRV_H__
#define __CYBERGEAR_DRV_H__
#include "main.h"
#include "fdcan.h"
#include "can_bsp.h"

#define MIT_MODE 0x000

#define CYBERGEAR_COMM_OPERATION_CTRL   0x01U
#define CYBERGEAR_COMM_FEEDBACK         0x02U
#define CYBERGEAR_COMM_ENABLE           0x03U
#define CYBERGEAR_COMM_STOP             0x04U
#define CYBERGEAR_MASTER_ID             0x00FDU

/* CyberGear MIT command ranges (OpenELAB guide, section 4.1/4.2). */
#define CYBERGEAR_P_MIN (-12.566371f)  /* -4pi rad */
#define CYBERGEAR_P_MAX (12.566371f)   /* +4pi rad */
#define CYBERGEAR_V_MIN (-30.0f)       /* rad/s */
#define CYBERGEAR_V_MAX (30.0f)        /* rad/s */
#define CYBERGEAR_KP_MIN (0.0f)
#define CYBERGEAR_KP_MAX (500.0f)
#define CYBERGEAR_KD_MIN (0.0f)
#define CYBERGEAR_KD_MAX (5.0f)
#define CYBERGEAR_T_MIN (-12.0f)       /* Nm */
#define CYBERGEAR_T_MAX (12.0f)        /* Nm */

/* CyberGear feedback ranges in MIT feedback frame. */
#define CYBERGEAR_FB_P_MIN (-12.566371f)
#define CYBERGEAR_FB_P_MAX (12.566371f)
#define CYBERGEAR_FB_V_MIN (-30.0f)
#define CYBERGEAR_FB_V_MAX (30.0f)
#define CYBERGEAR_FB_T_MIN (-12.0f)
#define CYBERGEAR_FB_T_MAX (12.0f)

typedef struct 
{
	uint32_t id;
	uint16_t state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos;
	float vel;
	float tor;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
} cybergear_fbpara_t;

typedef struct
{
	uint16_t mode;
	cybergear_fbpara_t para;	
} cybergear_motor_t;

void cybergear_fbdata(cybergear_motor_t *motor, uint8_t *rx_data, uint32_t data_len);
void cybergear_enable_motor_mode(hcan_t *hcan, uint32_t motor_id, uint32_t mode_id);
void cybergear_disable_motor_mode(hcan_t *hcan, uint32_t motor_id, uint32_t mode_id);
void cybergear_mit_ctrl(hcan_t *hcan, uint32_t motor_id, float pos, float vel, float kp, float kd, float torq);

#endif /* __CYBERGEAR_DRV_H__ */

