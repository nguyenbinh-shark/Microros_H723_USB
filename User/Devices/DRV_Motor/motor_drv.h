#ifndef __MOTOR_DRV_H__
#define __MOTOR_DRV_H__
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
} motor_fbpara_t;

typedef struct
{
	uint16_t mode;
	motor_fbpara_t para;	
} Wheel_Motor_t;

void cybergear_fbdata(Wheel_Motor_t *motor, uint8_t *rx_data, uint32_t data_len);
void enable_motor_mode(hcan_t *hcan, uint32_t motor_id, uint32_t mode_id);
void disable_motor_mode(hcan_t *hcan, uint32_t motor_id, uint32_t mode_id);
void mit_ctrl(hcan_t *hcan, uint32_t motor_id, float pos, float vel, float kp, float kd, float torq);

#endif /* __MOTOR_DRV_H__ */

