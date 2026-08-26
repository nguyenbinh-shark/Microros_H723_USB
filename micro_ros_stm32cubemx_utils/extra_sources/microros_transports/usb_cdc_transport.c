#include <rmw_microros/rmw_microros.h>

#include "main.h"
#include "cmsis_os.h"
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#ifdef RMW_UXRCE_TRANSPORT_CUSTOM

// --- USB CDC Handles ---
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_HS;
extern USBD_HandleTypeDef hUsbDeviceHS;

// --- Reimplemented USB CDC callbacks ---
static int8_t CDC_TransmitCplt_HS(uint8_t *Buf, uint32_t *Len, uint8_t epnum);
static int8_t CDC_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_HS(uint8_t* Buf, uint32_t *Len);

// Line coding: Rate: 115200bps; CharFormat: 1 Stop bit; Parity: None; Data: 8 bits
static uint8_t line_coding[7] = {0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08};

// --- micro-ROS Transports ---
#define USB_BUFFER_SIZE 2048
#define WRITE_TIMEOUT_MS 20U

volatile uint8_t storage_buffer[USB_BUFFER_SIZE] = {0};
volatile size_t it_head = 0;
volatile size_t it_tail = 0;
volatile bool g_write_complete = false;
bool initialized = false;

// Transmission completed callback
static int8_t CDC_TransmitCplt_HS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
    (void) Buf;
    (void) Len;
    (void) epnum;

    g_write_complete = true;
    return USBD_OK;
}

// USB CDC requests callback
static int8_t CDC_Control_HS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
    switch(cmd)
    {
        case CDC_SET_LINE_CODING:
        memcpy(line_coding, pbuf, sizeof(line_coding));
        break;

        case CDC_GET_LINE_CODING:
        memcpy(pbuf, line_coding, sizeof(line_coding));
        break;

        case CDC_SEND_ENCAPSULATED_COMMAND:
        case CDC_GET_ENCAPSULATED_RESPONSE:
        case CDC_SET_COMM_FEATURE:
        case CDC_GET_COMM_FEATURE:
        case CDC_CLEAR_COMM_FEATURE:
        case CDC_SET_CONTROL_LINE_STATE:
        case CDC_SEND_BREAK:
        default:
            break;
    }

    return USBD_OK;
}

// Data received callback
static int8_t CDC_Receive_HS(uint8_t* Buf, uint32_t *Len)
{
	USBD_CDC_SetRxBuffer(&hUsbDeviceHS, &Buf[0]);

    // Circular buffer
    if ((it_tail + *Len) > USB_BUFFER_SIZE)
	{
        size_t first_section = USB_BUFFER_SIZE - it_tail;
        size_t second_section = *Len - first_section;

		memcpy((void*) &storage_buffer[it_tail] , Buf, first_section);
		memcpy((void*) &storage_buffer[0] , Buf, second_section);
        it_tail = second_section;
	}
    else
    {
		memcpy((void*) &storage_buffer[it_tail] , Buf, *Len);
		it_tail += *Len;
    }

	USBD_CDC_ReceivePacket(&hUsbDeviceHS);

	return (USBD_OK);
}

bool cubemx_transport_open(struct uxrCustomTransport * transport){

    if (!initialized)
    {
        // USB is initialized on generated main code: Replace default callbacks here
        USBD_Interface_fops_HS.Control = CDC_Control_HS;
        USBD_Interface_fops_HS.Receive = CDC_Receive_HS;
        USBD_Interface_fops_HS.TransmitCplt = CDC_TransmitCplt_HS;
        initialized = true;
    }

    return true;
}

bool cubemx_transport_close(struct uxrCustomTransport * transport){
    return true;
}

size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err){
	uint8_t ret = CDC_Transmit_HS((uint8_t*)buf, len);

	if (USBD_OK != ret)
	{
		return 0;
	}

    int64_t start = uxr_millis();
    while(!g_write_complete && (uxr_millis() -  start) < WRITE_TIMEOUT_MS)
    {
    	vTaskDelay( 1 / portTICK_PERIOD_MS);
    }

    size_t writed = g_write_complete ? len : 0;
    g_write_complete = false;

	return writed;
}

size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err){

    int64_t start = uxr_millis();
    size_t readed = 0;

    do
    {
        if (it_head != it_tail)
        {
            while ((it_head != it_tail) && (readed < len)){
                buf[readed] = storage_buffer[it_head];
                it_head = (it_head + 1) % USB_BUFFER_SIZE;
                readed++;
            }

            break;
        }

       vTaskDelay( 1 / portTICK_PERIOD_MS );
    } while ((uxr_millis() -  start) < timeout);

    return readed;
}

size_t cubemx_transport_get_pending(void)
{
    size_t head = it_head;
    size_t tail = it_tail;
    if (tail >= head) {
        return tail - head;
    } else {
        return USB_BUFFER_SIZE - head + tail;
    }
}


#endif