#ifndef __LCD_TASK_H__
#define __LCD_TASK_H__

#include <stdint.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t cmd_rx_count;     /* Total /cmd_vel messages received */
    uint32_t pub_tx_count;     /* Total /motor_fb messages published */
    uint32_t cmd_last_tick;    /* HAL_GetTick() when last cmd was received */
    uint32_t cmd_age_ms;       /* Time elapsed since last cmd (ms) */
    float    cmd_rate_hz;      /* Frequency of incoming commands (Hz) */
    uint16_t ping_rtt_ms;      /* Round-trip time latency to Agent (ms) */
    uint16_t rx_pending_bytes; /* Unprocessed bytes pending in USB RX buffer */
} CommMetrics_t;

void comm_metrics_get(CommMetrics_t *metrics);

/**
 * @brief Initialize LCD hardware & draw the debug dashboard layout
 */
void LCD_Debug_Init(void);

/**
 * @brief Periodic UI update (refresh rates: 5 - 10 Hz)
 */
void LCD_Debug_Update(void);

/**
 * @brief FreeRTOS task entry for LCD display
 */
void LCD_Task_Entry(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* __LCD_TASK_H__ */

