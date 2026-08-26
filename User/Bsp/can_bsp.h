#ifndef _CAN_BSP_H
#define _CAN_BSP_H

#include "main.h"
#include "fdcan.h"

/* Alias used by drivers to stay portable across projects */
typedef FDCAN_HandleTypeDef hcan_t;

/* Basic TX helper (classic CAN, 11-bit standard ID) */
uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);

/* Basic TX helper (classic CAN, 29-bit extended ID) */
uint8_t canx_send_ext_data(FDCAN_HandleTypeDef *hcan, uint32_t ext_id, uint8_t *data, uint32_t len);

/* FDCAN filter + notification + start helpers */
void FDCAN1_Config(void);
void FDCAN2_Config(void);
void FDCAN3_Config(void);

/* Motor feedback getters (for micro-ROS publish) */
void CAN_MotorFeedback_Get(float *pos, float *vel, uint8_t *motor_id, uint8_t *valid);
void CAN_MotorFeedback_Get_Idx(uint8_t idx, float *pos, float *vel, float *tor, uint8_t *valid);
float CAN_GetMotorVel(uint8_t id);   /* id=1 left, id=2 right. Returns 0 if stale. */

#endif
