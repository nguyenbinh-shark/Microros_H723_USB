#include <rmw_microros/rmw_microros.h>
#include "main.h"
#include "cmsis_os.h"
#include "task.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"
#include <string.h>
#include <stdbool.h>

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_HS;
extern USBD_HandleTypeDef hUsbDeviceHS;

#define USB_BUFFER_SIZE 2048U
#define WRITE_TIMEOUT_MS 100U
/* Framing can escape every byte. Keep the buffer alive even if completion
 * times out: USB may still be reading it after the caller returns. */
#define USB_TX_BUFFER_SIZE (2U * RMW_UXRCE_MAX_TRANSPORT_MTU + 32U)

static uint8_t line_coding[7] = {0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08};
static uint8_t storage_buffer[USB_BUFFER_SIZE];
static uint8_t tx_buffer[USB_TX_BUFFER_SIZE];
static volatile size_t it_head;
static volatile size_t it_tail;
static volatile bool rx_overflow;
static volatile bool g_write_complete;
static bool initialized;

static int8_t CDC_TransmitCplt_HS(uint8_t *buf, uint32_t *len, uint8_t epnum)
{
    (void)buf;
    (void)len;
    (void)epnum;
    g_write_complete = true;
    return USBD_OK;
}

static int8_t CDC_Control_HS(uint8_t cmd, uint8_t *buf, uint16_t length)
{
    if ((cmd == CDC_SET_LINE_CODING || cmd == CDC_GET_LINE_CODING) &&
        length >= sizeof(line_coding))
    {
        if (cmd == CDC_SET_LINE_CODING)
            memcpy(line_coding, buf, sizeof(line_coding));
        else
            memcpy(buf, line_coding, sizeof(line_coding));
    }
    return USBD_OK;
}

static int8_t CDC_Receive_HS(uint8_t *buf, uint32_t *len)
{
    size_t tail = it_tail;
    size_t head = it_head;
    size_t used = (tail + USB_BUFFER_SIZE - head) % USB_BUFFER_SIZE;
    size_t free_bytes = USB_BUFFER_SIZE - 1U - used;

    /* Reserve one slot to distinguish full from empty. Drop an overflowing
     * packet as a whole rather than silently overwriting unread XRCE bytes. */
    if (*len > free_bytes)
    {
        rx_overflow = true;
    }
    else
    {
        for (uint32_t i = 0; i < *len; ++i)
        {
            storage_buffer[tail] = buf[i];
            tail = (tail + 1U) % USB_BUFFER_SIZE;
        }
        __DMB();
        it_tail = tail;
    }
    USBD_CDC_SetRxBuffer(&hUsbDeviceHS, buf);
    USBD_CDC_ReceivePacket(&hUsbDeviceHS);
    return USBD_OK;
}

static void reset_rx(void)
{
    taskENTER_CRITICAL();
    it_head = 0U;
    it_tail = 0U;
    rx_overflow = false;
    taskEXIT_CRITICAL();
}

bool cubemx_transport_open(struct uxrCustomTransport *transport)
{
    (void)transport;
    if (!initialized)
    {
        USBD_Interface_fops_HS.Control = CDC_Control_HS;
        USBD_Interface_fops_HS.Receive = CDC_Receive_HS;
        USBD_Interface_fops_HS.TransmitCplt = CDC_TransmitCplt_HS;
        initialized = true;
    }
    reset_rx();
    return true;
}

bool cubemx_transport_close(struct uxrCustomTransport *transport)
{
    (void)transport;
    reset_rx();
    /* Do not modify tx_buffer: an outstanding USB transfer owns it. */
    return true;
}

static bool usb_tx_idle(void)
{
    USBD_CDC_HandleTypeDef *cdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceHS.pClassData;
    return cdc != NULL && cdc->TxState == 0U;
}

size_t cubemx_transport_write(struct uxrCustomTransport *transport,
                             const uint8_t *buf, size_t len, uint8_t *err)
{
    (void)transport;
    if (err != NULL) *err = 0U;
    if (len == 0U) return 0U;
    if (buf == NULL || len > sizeof(tx_buffer))
    {
        if (err != NULL) *err = 1U;
        return 0U;
    }

    int64_t start = uxr_millis();
    uint8_t ret = USBD_BUSY;
    do
    {
        if (hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED) break;
        if (usb_tx_idle())
        {
            memcpy(tx_buffer, buf, len);
            g_write_complete = false;
            ret = CDC_Transmit_HS(tx_buffer, (uint16_t)len);
            if (ret != USBD_BUSY) break;
        }
        osDelay(1U);
    } while ((uxr_millis() - start) < WRITE_TIMEOUT_MS);

    if (ret == USBD_OK)
    {
        while (!g_write_complete && !usb_tx_idle() &&
               hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED &&
               (uxr_millis() - start) < WRITE_TIMEOUT_MS)
        {
            osDelay(1U);
        }
        if (hUsbDeviceHS.dev_state == USBD_STATE_CONFIGURED &&
            (g_write_complete || usb_tx_idle())) return len;
    }

    if (err != NULL) *err = 1U;
    return 0U;
}

size_t cubemx_transport_read(struct uxrCustomTransport *transport,
                            uint8_t *buf, size_t len, int timeout, uint8_t *err)
{
    (void)transport;
    if (err != NULL) *err = 0U;
    if (buf == NULL || len == 0U) return 0U;

    int64_t start = uxr_millis();
    do
    {
        /* USB interrupt is the sole producer; this task is the sole consumer.
         * Publish head only after copying out each available batch. */
        size_t head = it_head;
        size_t tail = it_tail;
        __DMB();
        size_t count = 0U;
        while (head != tail && count < len)
        {
            buf[count++] = storage_buffer[head];
            head = (head + 1U) % USB_BUFFER_SIZE;
        }
        __DMB();
        it_head = head;

        taskENTER_CRITICAL();
        bool overflow = rx_overflow;
        rx_overflow = false;
        taskEXIT_CRITICAL();
        if (overflow && err != NULL) *err = 1U;
        if (count > 0U || overflow) return count;
        if (timeout <= 0) break;
        osDelay(1U);
    } while ((uxr_millis() - start) < timeout);
    return 0U;
}

size_t cubemx_transport_get_pending(void)
{
    size_t head = it_head;
    size_t tail = it_tail;
    return (tail + USB_BUFFER_SIZE - head) % USB_BUFFER_SIZE;
}

#endif
