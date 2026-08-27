#include "bsp_sbus.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>   /* abs() — was implicitly declared, returning int */
#include <math.h>

sbus_t sbus = {0};
extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;

static uint8_t sbus_rx[SBUS_RX_BUFSIZE];

/* Restart DMA reception for next frame */
static void sbus_start_dma(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, sbus_rx, SBUS_RX_BUFSIZE);
    if (huart5.hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
    }
}

void sbus_bsp_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    memset(&sbus, 0, sizeof(sbus));
    for (int i = 0; i < 16; i++) {
        sbus.ch[i] = SBUS_CHANNEL_MID;
    }

    /* 1. UART5 Peripheral Clock Selection */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART5;
    PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
    (void)HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

    /* 2. Enable Clocks */
    __HAL_RCC_UART5_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_DMA1_CLK_ENABLE();

    /**UART5 GPIO Configuration
    PD2     ------> UART5_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART5;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* 3. Configure UART5 for 100000 8E2 (SBUS Standard) */
    huart5.Instance = UART5;
    huart5.Init.BaudRate = 100000;
    huart5.Init.WordLength = UART_WORDLENGTH_9B; /* 8 Data + 1 Parity */
    huart5.Init.StopBits = UART_STOPBITS_2;
    huart5.Init.Parity = UART_PARITY_EVEN;
    huart5.Init.Mode = UART_MODE_RX;
    huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart5.Init.OverSampling = UART_OVERSAMPLING_16;
    huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart5) != HAL_OK)
    {
        printf("[SBUS_ERR] UART5 Init Failed!\r\n");
        return;
    }

    /* 4. Configure DMA1 Stream 4 for UART5_RX */
    hdma_uart5_rx.Instance = DMA1_Stream4;
    hdma_uart5_rx.Init.Request = DMA_REQUEST_UART5_RX;
    hdma_uart5_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart5_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart5_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart5_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart5_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart5_rx.Init.Mode = DMA_NORMAL;
    hdma_uart5_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_uart5_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(&hdma_uart5_rx) != HAL_OK)
    {
        printf("[SBUS_ERR] DMA Init Failed!\r\n");
        return;
    }

    __HAL_LINKDMA(&huart5, hdmarx, hdma_uart5_rx);

    /* 5. Enable Interrupts */
    HAL_NVIC_SetPriority(UART5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

    /* 6. Start Reception */
    sbus_start_dma();
    printf("[BOOT] SBUS / DBUS Receiver Initialized on UART5 (PD2) @ 100k 8E2\r\n");
}

static void sbus_decode_frame(const uint8_t *f)
{
    sbus.ch[0]  = (uint16_t)((f[1]        | f[2]  << 8)               & 0x07FF);
    sbus.ch[1]  = (uint16_t)((f[2]  >> 3  | f[3]  << 5)               & 0x07FF);
    sbus.ch[2]  = (uint16_t)((f[3]  >> 6  | f[4]  << 2 | f[5]  << 10) & 0x07FF);
    sbus.ch[3]  = (uint16_t)((f[5]  >> 1  | f[6]  << 7)               & 0x07FF);
    sbus.ch[4]  = (uint16_t)((f[6]  >> 4  | f[7]  << 4)               & 0x07FF);
    sbus.ch[5]  = (uint16_t)((f[7]  >> 7  | f[8]  << 1 | f[9]  << 9)  & 0x07FF);
    sbus.ch[6]  = (uint16_t)((f[9]  >> 2  | f[10] << 6)               & 0x07FF);
    sbus.ch[7]  = (uint16_t)((f[10] >> 5  | f[11] << 3)               & 0x07FF);
    sbus.ch[8]  = (uint16_t)((f[12]       | f[13] << 8)               & 0x07FF);
    sbus.ch[9]  = (uint16_t)((f[13] >> 3  | f[14] << 5)               & 0x07FF);
    sbus.ch[10] = (uint16_t)((f[14] >> 6  | f[15] << 2 | f[16] << 10) & 0x07FF);
    sbus.ch[11] = (uint16_t)((f[16] >> 1  | f[17] << 7)               & 0x07FF);
    sbus.ch[12] = (uint16_t)((f[17] >> 4  | f[18] << 4)               & 0x07FF);
    sbus.ch[13] = (uint16_t)((f[18] >> 7  | f[19] << 1 | f[20] << 9)  & 0x07FF);
    sbus.ch[14] = (uint16_t)((f[20] >> 2  | f[21] << 6)               & 0x07FF);
    sbus.ch[15] = (uint16_t)((f[21] >> 5  | f[22] << 3)               & 0x07FF);

    sbus.ch17       = (f[23] & 0x01) ? 1 : 0;
    sbus.ch18       = (f[23] & 0x02) ? 1 : 0;
    sbus.frame_lost = (f[23] & 0x04) ? 1 : 0;
    sbus.failsafe   = (f[23] & 0x08) ? 1 : 0;
    sbus.online     = (sbus.failsafe == 0) ? 1 : 0;
    sbus.last_rx_tick = HAL_GetTick();
}

/* Callback from UART IDLE Line Event */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART5)
    {
        for (uint16_t i = 0; i + SBUS_FRAME_LEN <= Size; i++)
        {
            if (sbus_rx[i] == SBUS_HEADER && sbus_rx[i + SBUS_FRAME_LEN - 1] == SBUS_FOOTER)
            {
                sbus_decode_frame(&sbus_rx[i]);
                break;
            }
        }
        sbus_start_dma();
    }
}

extern void debug_uart_tx_reset(void);

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        sbus_start_dma();
    }
    else if (huart->Instance == UART7)
    {
        /* Debug console: clear the stuck TX so the ring keeps draining. */
        debug_uart_tx_reset();
    }
}

bool sbus_is_online(void)
{
    uint32_t now = HAL_GetTick();
    if ((now - sbus.last_rx_tick) > 300U || sbus.failsafe != 0 || sbus.online == 0)
    {
        sbus.online = 0;
        return false;
    }
    return true;
}

RC_ControlMode_t sbus_get_control_mode(void)
{
    if (!sbus_is_online())
    {
        return RC_MODE_AUTO_ROS2; /* Default to ROS2 /cmd_vel when transmitter is off */
    }

    /* Switch A / Channel 5 (ch[4]):
     * Range: ~364 (Up) / ~1024 (Mid) / ~1684 (Down)
     */
    if (sbus.ch[4] < 700)
    {
        return RC_MODE_MANUAL_RC;      /* Switch Up: Manual RC Joy Control */
    }
    else if (sbus.ch[4] >= 700 && sbus.ch[4] <= 1350)
    {
        return RC_MODE_AUTO_ROS2;      /* Switch Mid: Auto ROS 2 /cmd_vel Control */
    }
    else
    {
        return RC_MODE_EMERGENCY_STOP; /* Switch Down: E-STOP */
    }
}

void sbus_get_motion_cmd(float max_vx, float max_wz, float *out_vx, float *out_wz)
{
    if (out_vx == NULL || out_wz == NULL) return;

    if (!sbus_is_online())
    {
        *out_vx = 0.0f;
        *out_wz = 0.0f;
        return;
    }

    /* 
     * Channel 2 (CH2, Pitch Stick / Forward-Backward):
     * Range 364..1684, Center 1024
     */
    int32_t raw_vx = (int32_t)sbus.ch[1] - (int32_t)SBUS_CHANNEL_MID;
    if (labs(raw_vx) < 35) raw_vx = 0; /* Deadband ±35 */
    *out_vx = ((float)raw_vx / 660.0f) * max_vx;

    /*
     * Channel 1 (CH1, Roll / Yaw Stick / Steering):
     * Range 364..1684, Center 1024 (Left/Right)
     */
    int32_t raw_wz = (int32_t)sbus.ch[0] - (int32_t)SBUS_CHANNEL_MID;
    if (labs(raw_wz) < 35) raw_wz = 0; /* Deadband ±35 */
    *out_wz = -((float)raw_wz / 660.0f) * max_wz; /* Inverted for standard ROS CCW positive */
}

