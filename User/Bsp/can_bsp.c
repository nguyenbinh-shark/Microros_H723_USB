#include "can_bsp.h"
#include "fdcan.h"
#include "robstride_drv.h"
#include <string.h>
#include <stdio.h>

/* ── RX buffers ──────────────────────────────────────────────────── */
FDCAN_RxHeaderTypeDef RxHeader1;
uint8_t g_Can1RxData[64];

FDCAN_RxHeaderTypeDef RxHeader2;
uint8_t g_Can2RxData[64];

FDCAN_RxHeaderTypeDef RxHeader3;
uint8_t g_Can3RxData[64];

/* ── Motor feedback slots ────────────────────────────────────────── */
#define CAN_FB_TIMEOUT_MS  (500U)
#define CAN_MOTOR_COUNT    (2U)   /* slot 0 = left (FDCAN1), slot 1 = right (FDCAN3) */

typedef struct {
    rs_ext_fb_t fb;
    volatile uint8_t  valid;
    volatile uint32_t last_ms;
} MotorSlot_t;

static MotorSlot_t g_motors[CAN_MOTOR_COUNT];  /* index 0 → FDCAN1, index 1 → FDCAN3 */
static CAN_BusMetrics_t g_can_metrics = {0};

/* CAN ID each slot is allowed to accept feedback from. Both wheels are ID 1
   because they sit on separate buses; kept explicit so a mis-addressed motor
   shows up as "no feedback" instead of feeding the wrong wheel. */
static uint8_t g_motor_expected_id[CAN_MOTOR_COUNT] = { 1U, 1U };

void CAN_SetExpectedMotorId(uint8_t idx, uint8_t motor_id)
{
    if (idx < CAN_MOTOR_COUNT) g_motor_expected_id[idx] = motor_id;
}

/* Read PSR + ECR of one bus.
 *
 * Reading PSR clears LEC to 7 ("no change"), and more than one caller polls
 * these metrics — the ROS status publisher at 5 Hz and the console log every
 * 2 s. If the raw value were stored, whichever call came first would swallow
 * the error code and the log would only ever print "-". So a real error code
 * is latched and only replaced by another real one. */
static void CAN_ReadProtoStatus(FDCAN_HandleTypeDef *hcan, CAN_ProtoStatus_t *out)
{
    FDCAN_ProtocolStatusTypeDef ps;
    FDCAN_ErrorCountersTypeDef  ec;

    HAL_FDCAN_GetProtocolStatus(hcan, &ps);
    HAL_FDCAN_GetErrorCounters(hcan, &ec);

    if (ps.LastErrorCode != FDCAN_PROTOCOL_ERROR_NO_CHANGE)
    {
        out->lec = (uint8_t)ps.LastErrorCode;
    }
    out->bus_off     = (uint8_t)ps.BusOff;
    out->err_passive = (uint8_t)ps.ErrorPassive;
    out->warning     = (uint8_t)ps.Warning;
    out->tec         = (uint8_t)ec.TxErrorCnt;
    out->rec         = (uint8_t)ec.RxErrorCnt;
}

void CAN_GetMetrics(CAN_BusMetrics_t *metrics)
{
    if (metrics) {
        CAN_ReadProtoStatus(&hfdcan1, &g_can_metrics.can1_proto);
        CAN_ReadProtoStatus(&hfdcan3, &g_can_metrics.can3_proto);
        *metrics = g_can_metrics;
    }
}

const char *CAN_LecToStr(uint8_t lec)
{
    switch (lec)
    {
        case FDCAN_PROTOCOL_ERROR_NONE:      return "OK";
        case FDCAN_PROTOCOL_ERROR_STUFF:     return "STUFF";   /* wrong baudrate / noise    */
        case FDCAN_PROTOCOL_ERROR_FORM:      return "FORM";    /* wrong baudrate / noise    */
        case FDCAN_PROTOCOL_ERROR_ACK:       return "ACK";     /* nobody else on the bus    */
        case FDCAN_PROTOCOL_ERROR_BIT1:      return "BIT1";
        case FDCAN_PROTOCOL_ERROR_BIT0:      return "BIT0";    /* TX not reaching the bus   */
        case FDCAN_PROTOCOL_ERROR_CRC:       return "CRC";
        case FDCAN_PROTOCOL_ERROR_NO_CHANGE: return "-";       /* no error since last read  */
        default:                             return "?";
    }
}

/* Bosch M_CAN sets CCCR.INIT on Bus_Off and stays there until software clears
   it. Without this the node never comes back, even after the wiring is fixed.
   Reads the register directly so it is cheap enough to poll every control
   cycle, independent of the diagnostic snapshot. */
uint8_t CAN_BusOffRecover(void)
{
    uint8_t recovered = 0U;

    if ((hfdcan1.Instance->PSR & FDCAN_PSR_BO) != 0U)
    {
        if (HAL_FDCAN_Start(&hfdcan1) == HAL_OK) recovered |= 0x01U;
    }
    if ((hfdcan3.Instance->PSR & FDCAN_PSR_BO) != 0U)
    {
        if (HAL_FDCAN_Start(&hfdcan3) == HAL_OK) recovered |= 0x02U;
    }
    return recovered;
}

/* ── Motor feedback parse (called from RX ISR) ─────────────────── */
static uint32_t s_debug_frame_count = 0;
static uint32_t s_debug_parse_fail_count = 0;

static void CAN_ParseMotorFeedback(const FDCAN_RxHeaderTypeDef *rx_header,
                                    const uint8_t *rx_data, uint8_t idx)
{
    if (idx >= CAN_MOTOR_COUNT) return;
    if (rx_header->DataLength != FDCAN_DLC_BYTES_8 && rx_header->DataLength != 8) return;

    s_debug_frame_count++;

    /* Extended (29-bit) frames only. The old 11-bit fallback decoded *any*
       standard frame on the bus as an MIT feedback and marked the slot valid,
       so unrelated traffic silently corrupted the wheel velocities. */
    if (rx_header->IdType != FDCAN_EXTENDED_ID) {
        s_debug_parse_fail_count++;
        return;
    }

    rs_ext_fb_t tmp = g_motors[idx].fb;   /* keep multi-turn accumulator state */

    if (rs_ext_parse_feedback(&tmp, rx_header->Identifier, rx_data, rx_header->DataLength) != 0)
    {
        s_debug_parse_fail_count++;
        return;
    }

    /* Only accept frames addressed to us and coming from the expected motor. */
    if (tmp.master_id != RS_DEFAULT_MASTER_ID) return;
    if (tmp.motor_id  != g_motor_expected_id[idx]) return;

    g_motors[idx].fb      = tmp;
    g_motors[idx].valid   = 1U;
    g_motors[idx].last_ms = HAL_GetTick();
    printf("[CAN_FB] Motor[%d] vel=%+6.2f tor=%+5.2f pos=%+7.1f (tick=%lu)\r\n", idx, tmp.vel, tmp.tor, tmp.pos, HAL_GetTick());
}

void CAN_PrintDebugStats(void)
{
    printf("[CAN_DBG] Total frames: %lu | Parse fails: %lu | Motor[0] valid=%d last=%lums | Motor[1] valid=%d last=%lums\r\n",
           s_debug_frame_count, s_debug_parse_fail_count,
           g_motors[0].valid, (uint32_t)(HAL_GetTick() - g_motors[0].last_ms),
           g_motors[1].valid, (uint32_t)(HAL_GetTick() - g_motors[1].last_ms));
}

/* ── Legacy API — returns motor slot 0 (left/FDCAN1) ─────────── */
void CAN_MotorFeedback_Get(float *pos, float *vel, uint8_t *motor_id, uint8_t *valid)
{
    uint32_t now = HAL_GetTick();
    MotorSlot_t *s = &g_motors[0];
    uint8_t fresh = (s->valid && ((uint32_t)(now - s->last_ms) <= CAN_FB_TIMEOUT_MS)) ? 1U : 0U;

    if (pos)      *pos      = s->valid ? s->fb.pos : 0.0f;
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

    if (pos)   *pos   = s->valid ? s->fb.pos : 0.0f;
    if (vel)   *vel   = fresh ? s->fb.vel : 0.0f;
    if (tor)   *tor   = fresh ? s->fb.tor : 0.0f;
    if (valid) *valid = fresh;
}

/* ── Reset accumulated continuous angle to 0 ──────────────────────── */
void CAN_MotorFeedback_ResetAccumulated(uint8_t idx)
{
    if (idx >= CAN_MOTOR_COUNT) return;
    g_motors[idx].fb.pos          = 0.0f;
    g_motors[idx].fb.round_cnt    = 0;
    g_motors[idx].fb.last_raw_pos = g_motors[idx].fb.pos_raw;
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
    HAL_FDCAN_ConfigInterruptLines(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0);
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
    HAL_FDCAN_ConfigInterruptLines(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0);
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

/* Cyclic setpoints are only useful while fresh. If the queue is backing up,
   drop this frame rather than shipping a stale command several periods late. */
static uint8_t CAN_TxQueueStalled(FDCAN_HandleTypeDef *hcan)
{
    if (HAL_FDCAN_GetTxFifoFreeLevel(hcan) != 0U) return 0U;

    if (hcan->Instance == FDCAN1) g_can_metrics.can1_tx_err++;
    else if (hcan->Instance == FDCAN3) g_can_metrics.can3_tx_err++;
    return 1U;
}

/* ── CAN TX: 11-bit Standard ID ──────────────────────────────── */
uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef TxHeader;

    if (CAN_TxQueueStalled(hcan)) return 1;

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
    {
        if (hcan->Instance == FDCAN1) g_can_metrics.can1_tx_err++;
        else if (hcan->Instance == FDCAN3) g_can_metrics.can3_tx_err++;
        return 2;
    }

    if (hcan->Instance == FDCAN1) g_can_metrics.can1_tx_cnt++;
    else if (hcan->Instance == FDCAN3) g_can_metrics.can3_tx_cnt++;
    return 0;
}

/* ── CAN TX: 29-bit Extended ID ──────────────────────────────── */
uint8_t canx_send_ext_data(FDCAN_HandleTypeDef *hcan, uint32_t ext_id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef TxHeader;

    if (CAN_TxQueueStalled(hcan)) return 1;

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
    {
        if (hcan->Instance == FDCAN1) g_can_metrics.can1_tx_err++;
        else if (hcan->Instance == FDCAN3) g_can_metrics.can3_tx_err++;
        return 2;
    }

    if (hcan->Instance == FDCAN1) g_can_metrics.can1_tx_cnt++;
    else if (hcan->Instance == FDCAN3) g_can_metrics.can3_tx_cnt++;
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
                g_can_metrics.can1_rx_cnt++;
                g_can_metrics.can1_last_id = RxHeader1.Identifier;
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
                g_can_metrics.can3_rx_cnt++;
                g_can_metrics.can3_last_id = RxHeader3.Identifier;
                CAN_ParseMotorFeedback(&RxHeader3, g_Can3RxData, 1U);   /* FDCAN3 = right (slot 1) */
            }
        }
    }
}
