#include "can_bsp.h"
#include "fdcan.h"
#include "robstride_drv.h"
#include "string.h"

/* ── RX buffers ──────────────────────────────────────────────────── */
FDCAN_RxHeaderTypeDef RxHeader1;
uint8_t g_Can1RxData[64];

FDCAN_RxHeaderTypeDef RxHeader2;
uint8_t g_Can2RxData[64];

FDCAN_RxHeaderTypeDef RxHeader3;
uint8_t g_Can3RxData[64];

/* ── Motor feedback slots ────────────────────────────────────────── */
#define CAN_FB_TIMEOUT_MS  (120U)
#define CAN_MOTOR_COUNT    (2U)   /* slot 0 = left (FDCAN1), slot 1 = right (FDCAN3) */

typedef struct {
    rs_ext_fb_t fb;
    volatile uint8_t  valid;
    volatile uint32_t last_ms;
} MotorSlot_t;

static MotorSlot_t g_motors[CAN_MOTOR_COUNT];  /* index 0 → FDCAN1, index 1 → FDCAN3 */

/* ── Motor feedback parse (called from RX ISR) ─────────────────── */
static void CAN_ParseMotorFeedback(const FDCAN_RxHeaderTypeDef *rx_header,
                                    const uint8_t *rx_data, uint8_t idx)
{
    if (rx_header->IdType != FDCAN_EXTENDED_ID) return;
    if (rx_header->DataLength != FDCAN_DLC_BYTES_8) return;
    if (idx >= CAN_MOTOR_COUNT) return;

    if (rs_ext_parse_feedback(&g_motors[idx].fb, rx_header->Identifier, rx_data, rx_header->DataLength) == 0)
    {
        g_motors[idx].valid   = 1U;
        g_motors[idx].last_ms = HAL_GetTick();
    }
}

/* ── Legacy API — returns motor slot 0 (left/FDCAN1) ─────────── */
void CAN_MotorFeedback_Get(float *pos, float *vel, uint8_t *motor_id, uint8_t *valid)
{
    uint32_t now = HAL_GetTick();
    MotorSlot_t *s = &g_motors[0];
    uint8_t fresh = (s->valid && ((uint32_t)(now - s->last_ms) <= CAN_FB_TIMEOUT_MS)) ? 1U : 0U;

    if (pos)      *pos      = fresh ? s->fb.pos : 0.0f;
    if (vel)      *vel      = fresh ? s->fb.vel : 0.0f;
    if (motor_id) *motor_id = 1U;
    if (valid)    *valid    = fresh;
}

/* ── Extended API — returns feedback for a specific motor index ─────────── */
void CAN_MotorFeedback_Get_Idx(uint8_t idx, float *pos, float *vel, float *tor, uint8_t *valid)
{
    if (idx >= CAN_MOTOR_COUNT) return;
    uint32_t now = HAL_GetTick();
    MotorSlot_t *s = &g_motors[idx];
    uint8_t fresh = (s->valid && ((uint32_t)(now - s->last_ms) <= CAN_FB_TIMEOUT_MS)) ? 1U : 0U;

    if (pos)   *pos   = fresh ? s->fb.pos : 0.0f;
    if (vel)   *vel   = fresh ? s->fb.vel : 0.0f;
    if (tor)   *tor   = fresh ? s->fb.tor : 0.0f;
    if (valid) *valid = fresh;
}

/* Per-motor velocity — used by observe_task */
float CAN_GetMotorVel(uint8_t id)
{
    uint32_t now = HAL_GetTick();
    uint8_t idx;
    MotorSlot_t *s;

    if (id < 1U || id > CAN_MOTOR_COUNT) return 0.0f;
    idx = id - 1U;
    s   = &g_motors[idx];
    if (!s->valid || (uint32_t)(now - s->last_ms) > CAN_FB_TIMEOUT_MS) return 0.0f;
    return s->fb.vel;
}

/* ── FDCAN1 Config (Filter + Notification + Start) ────────────── */
void FDCAN1_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    HAL_FDCAN_Stop(&hfdcan1);

    /* Accept all Standard ID frames to RX FIFO0 */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x000;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

    /* Accept all Extended ID frames to RX FIFO0 */
    FDCAN_FilterTypeDef sExtFilterConfig;
    sExtFilterConfig.IdType = FDCAN_EXTENDED_ID;
    sExtFilterConfig.FilterIndex = 0;
    sExtFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sExtFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sExtFilterConfig.FilterID1 = 0x00000000;
    sExtFilterConfig.FilterID2 = 0x00000000;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sExtFilterConfig);

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                  FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan1);
}

#if 0
/* ── FDCAN2 Config (Filter + Notification + Start) ────────────── */
void FDCAN2_Config(void)
{
    /* FDCAN2 not used on this target */
}
#endif

/* ── FDCAN3 Config (Filter + Notification + Start) ────────────── */
void FDCAN3_Config(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    HAL_FDCAN_Stop(&hfdcan3);

    /* Accept all Standard ID frames to RX FIFO0 */
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x000;
    HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig);

    /* Accept all Extended ID frames to RX FIFO0 */
    FDCAN_FilterTypeDef sExtFilterConfig;
    sExtFilterConfig.IdType = FDCAN_EXTENDED_ID;
    sExtFilterConfig.FilterIndex = 0;
    sExtFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sExtFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sExtFilterConfig.FilterID1 = 0x00000000;
    sExtFilterConfig.FilterID2 = 0x00000000;
    HAL_FDCAN_ConfigFilter(&hfdcan3, &sExtFilterConfig);

    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                  FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
    HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
    HAL_FDCAN_Start(&hfdcan3);
}

static uint32_t CAN_LenToDLC(uint32_t len)
{
    switch (len)
    {
        case 0:  return FDCAN_DLC_BYTES_0;
        case 1:  return FDCAN_DLC_BYTES_1;
        case 2:  return FDCAN_DLC_BYTES_2;
        case 3:  return FDCAN_DLC_BYTES_3;
        case 4:  return FDCAN_DLC_BYTES_4;
        case 5:  return FDCAN_DLC_BYTES_5;
        case 6:  return FDCAN_DLC_BYTES_6;
        case 7:  return FDCAN_DLC_BYTES_7;
        case 8:  return FDCAN_DLC_BYTES_8;
        case 12: return FDCAN_DLC_BYTES_12;
        case 16: return FDCAN_DLC_BYTES_16;
        case 20: return FDCAN_DLC_BYTES_20;
        case 24: return FDCAN_DLC_BYTES_24;
        case 32: return FDCAN_DLC_BYTES_32;
        case 48: return FDCAN_DLC_BYTES_48;
        case 64: return FDCAN_DLC_BYTES_64;
        default: return (len <= 8) ? len : FDCAN_DLC_BYTES_8;
    }
}

/* ── CAN TX: 11-bit Standard ID ──────────────────────────────── */
uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef TxHeader;

    TxHeader.Identifier = id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = CAN_LenToDLC(len);

    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data) != HAL_OK)
        return 2;
    return 0;
}

/* ── CAN TX: 29-bit Extended ID ──────────────────────────────── */
uint8_t canx_send_ext_data(FDCAN_HandleTypeDef *hcan, uint32_t ext_id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef TxHeader;

    TxHeader.Identifier = ext_id;
    TxHeader.IdType = FDCAN_EXTENDED_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = CAN_LenToDLC(len);

    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data) != HAL_OK)
        return 2;
    return 0;
}

/* ── RX Callbacks ────────────────────────────────────────────── */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET)
    {
        while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0)
        {
            if (hfdcan->Instance == FDCAN1)
            {
                memset(g_Can1RxData, 0, sizeof(g_Can1RxData));
                HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader1, g_Can1RxData);
                CAN_ParseMotorFeedback(&RxHeader1, g_Can1RxData, 0U);   /* FDCAN1 = left (slot 0) */
            }
            else if (hfdcan->Instance == FDCAN2)
            {
                memset(g_Can2RxData, 0, sizeof(g_Can2RxData));
                HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader2, g_Can2RxData);
                CAN_ParseMotorFeedback(&RxHeader2, g_Can2RxData, 1U);   /* FDCAN2 = right/aux (slot 1) */
            }
            else if (hfdcan->Instance == FDCAN3)
            {
                memset(g_Can3RxData, 0, sizeof(g_Can3RxData));
                HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader3, g_Can3RxData);
                CAN_ParseMotorFeedback(&RxHeader3, g_Can3RxData, 1U);   /* FDCAN3 = right (slot 1) */
            }
        }
    }
}
