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
void CAN_MotorFeedback_ResetAccumulated(uint8_t idx);
float CAN_GetMotorVel(uint8_t id);   /* id=1 left, id=2 right. Returns 0 if stale. */

/* CAN ID a slot accepts feedback from (idx 0 = left/FDCAN1, 1 = right/FDCAN3) */
void CAN_SetExpectedMotorId(uint8_t idx, uint8_t motor_id);

/* Per-bus CAN protocol-layer status (from FDCAN_PSR / FDCAN_ECR) */
typedef struct {
    uint8_t  lec;        /* Last Error Code: 0=none 1=stuff 2=form 3=ack 4=bit1 5=bit0 6=crc 7=no change */
    uint8_t  bus_off;    /* 1 = controller has left the bus (TEC reached 255)   */
    uint8_t  err_passive;/* 1 = error-passive state                             */
    uint8_t  warning;    /* 1 = TEC or REC has reached 96                       */
    uint8_t  tec;        /* Transmit error counter [0..255]                     */
    uint8_t  rec;        /* Receive error counter  [0..127]                     */
} CAN_ProtoStatus_t;

/* CAN bus diagnostic metrics */
typedef struct {
    uint32_t can1_tx_cnt;
    uint32_t can1_tx_err;
    uint32_t can1_rx_cnt;
    uint32_t can1_last_id;
    uint32_t can3_tx_cnt;
    uint32_t can3_tx_err;
    uint32_t can3_rx_cnt;
    uint32_t can3_last_id;
    CAN_ProtoStatus_t can1_proto;
    CAN_ProtoStatus_t can3_proto;
} CAN_BusMetrics_t;

void CAN_GetMetrics(CAN_BusMetrics_t *metrics);

/* Human-readable name for CAN_ProtoStatus_t.lec */
const char *CAN_LecToStr(uint8_t lec);

/* Restart any bus that has fallen into Bus_Off. Returns bitmask: b0=CAN1, b1=CAN3. */
uint8_t CAN_BusOffRecover(void);

/* Debug: Print CAN feedback statistics */
void CAN_PrintDebugStats(void);

#endif
