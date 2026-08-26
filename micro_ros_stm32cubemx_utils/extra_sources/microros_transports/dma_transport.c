#include <uxr/client/transport.h>

#include <rmw_microxrcedds_c/config.h>

#include "main.h"
#include "cmsis_os.h"
#include "usart.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

// --- micro-ROS Transports ---
#define UART_DMA_BUFFER_SIZE 2048

static uint8_t dma_buffer[UART_DMA_BUFFER_SIZE];
static size_t dma_head = 0, dma_tail = 0;

bool cubemx_transport_open(struct uxrCustomTransport * transport){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
    dma_head = 0;
    dma_tail = 0;
    HAL_UART_Receive_DMA(uart, dma_buffer, UART_DMA_BUFFER_SIZE);
    return true;
}

bool cubemx_transport_close(struct uxrCustomTransport * transport){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;
    HAL_UART_DMAStop(uart);
    return true;
}

size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;

    /* Chờ truyền trước hoàn tất (timeout 50ms), không làm rớt gói XRCE-DDS */
    uint32_t wait_ready_ms = 50;
    while (uart->gState != HAL_UART_STATE_READY && wait_ready_ms > 0){
        osDelay(1);
        wait_ready_ms--;
    }

    if (uart->gState != HAL_UART_STATE_READY){
        HAL_UART_DMAStop(uart);
        uart->gState = HAL_UART_STATE_READY;
    }

    HAL_StatusTypeDef ret = HAL_UART_Transmit_DMA(uart, (uint8_t *)buf, len);
    if (ret != HAL_OK){
        return 0;
    }

    /* Chờ truyền xong gói hiện tại */
    uint32_t timeout_ms = 50;
    while (uart->gState != HAL_UART_STATE_READY && timeout_ms > 0){
        osDelay(1);
        timeout_ms--;
    }

    if (timeout_ms == 0) {
        HAL_UART_DMAStop(uart);
        uart->gState = HAL_UART_STATE_READY;
        return 0;
    }

    return len;
}

size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){
    UART_HandleTypeDef * uart = (UART_HandleTypeDef*) transport->args;

    /* Tự động xóa cờ lỗi ORE/NE/FE và hồi phục DMA RX nếu có lỗi đường truyền */
    if (__HAL_UART_GET_FLAG(uart, UART_FLAG_ORE) ||
        __HAL_UART_GET_FLAG(uart, UART_FLAG_NE)  ||
        __HAL_UART_GET_FLAG(uart, UART_FLAG_FE)  ||
        __HAL_UART_GET_FLAG(uart, UART_FLAG_PE))
    {
        __HAL_UART_CLEAR_FLAG(uart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
        if (uart->RxState == HAL_UART_STATE_READY || uart->hdmarx->State != HAL_DMA_STATE_BUSY)
        {
            HAL_UART_DMAStop(uart);
            dma_head = 0;
            dma_tail = 0;
            HAL_UART_Receive_DMA(uart, dma_buffer, UART_DMA_BUFFER_SIZE);
        }
    }

    int ms_used = 0;
    do
    {
        dma_tail = UART_DMA_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(uart->hdmarx);
        if (dma_head != dma_tail) {
            break;
        }
        ms_used++;
        osDelay(portTICK_RATE_MS);
    } while (ms_used < timeout);
    
    size_t wrote = 0;
    while ((dma_head != dma_tail) && (wrote < len)){
        buf[wrote] = dma_buffer[dma_head];
        dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
        wrote++;
    }
    
    return wrote;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7)
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
        HAL_UART_DMAStop(huart);
        dma_head = 0;
        dma_tail = 0;
        HAL_UART_Receive_DMA(huart, dma_buffer, UART_DMA_BUFFER_SIZE);
    }
}

#endif //RMW_UXRCE_TRANSPORT_CUSTOM