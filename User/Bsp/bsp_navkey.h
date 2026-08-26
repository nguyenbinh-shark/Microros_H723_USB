#ifndef __BSP_NAVKEY_H__
#define __BSP_NAVKEY_H__

#include <stdint.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    NAV_KEY_NONE = 0,
    NAV_KEY_UP,
    NAV_KEY_DOWN,
    NAV_KEY_LEFT,
    NAV_KEY_RIGHT,
    NAV_KEY_PRESS
} NavKey_t;

/**
 * @brief Initialize ADC1 on PA5 (ADC_CHANNEL_19) for 5-way analog navigation key
 */
void NavKey_Init(void);

/**
 * @brief Scan 5-way navigation key with edge-triggered single press detection
 * @return NavKey_t Key event (returns non-zero only once per press)
 */
NavKey_t NavKey_Scan(void);

/**
 * @brief Get raw 16-bit ADC value of PA5
 */
uint16_t NavKey_GetRawADC(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_NAVKEY_H__ */

