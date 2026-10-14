"""Compile and exercise the actual firmware USB CDC transport with host mocks.

No board traffic or production files are changed. Generated fixtures/executable
stay in ignored build/usb_cdc_host. The assertions target stream corruption and
asynchronous USB buffer ownership, not ROS agent or hardware validation.
"""
from pathlib import Path
import os
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "build" / "usb_cdc_host"
OUT.mkdir(parents=True, exist_ok=True)

HEADERS = {
    "rmw_microros/rmw_microros.h": r"""
#pragma once
#include <stdint.h>
#include <stddef.h>
#define RMW_UXRCE_TRANSPORT_CUSTOM 1
#define RMW_UXRCE_MAX_TRANSPORT_MTU 512U
struct uxrCustomTransport { int unused; };
int64_t uxr_millis(void);
""",
    "main.h": r"""
#pragma once
#include <stdint.h>
#define __DMB() do {} while (0)
""",
    "cmsis_os.h": r"""
#pragma once
#include <stdint.h>
void osDelay(uint32_t ms);
""",
    "task.h": r"""
#pragma once
#define taskENTER_CRITICAL() do {} while (0)
#define taskEXIT_CRITICAL() do {} while (0)
""",
    "usbd_cdc_if.h": r"""
#pragma once
#include "usbd_cdc.h"
uint8_t CDC_Transmit_HS(uint8_t *buf, uint16_t len);
""",
    "usbd_cdc.h": r"""
#pragma once
#include <stdint.h>
#define USBD_OK 0U
#define USBD_BUSY 1U
#define USBD_FAIL 2U
#define USBD_STATE_CONFIGURED 3U
#define CDC_SET_LINE_CODING 0x20U
#define CDC_GET_LINE_CODING 0x21U
typedef struct { volatile uint32_t TxState; } USBD_CDC_HandleTypeDef;
typedef struct { void *pClassData; uint8_t dev_state; } USBD_HandleTypeDef;
typedef struct {
    int8_t (*Control)(uint8_t, uint8_t *, uint16_t);
    int8_t (*Receive)(uint8_t *, uint32_t *);
    int8_t (*TransmitCplt)(uint8_t *, uint32_t *, uint8_t);
} USBD_CDC_ItfTypeDef;
void USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *dev, uint8_t *buf);
void USBD_CDC_ReceivePacket(USBD_HandleTypeDef *dev);
""",
}
for rel, body in HEADERS.items():
    target = OUT / rel
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(body, encoding="utf-8")

SOURCE = ROOT / "micro_ros_stm32cubemx_utils/extra_sources/microros_transports/usb_cdc_transport.c"
HARNESS = r"""
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "usbd_cdc.h"
#include "rmw_microros/rmw_microros.h"

USBD_CDC_ItfTypeDef USBD_Interface_fops_HS;
USBD_HandleTypeDef hUsbDeviceHS;
static USBD_CDC_HandleTypeDef mock_cdc;
static int64_t clock_ms;
static int64_t completion_at = -1;
static int64_t disconnect_at = -1;
static int auto_completion_ms = 1;
static unsigned delay_calls, busy_left, transmit_calls, transfers_started;
static uint8_t *inflight;
static uint16_t inflight_length;

int64_t uxr_millis(void) { return clock_ms; }
static void complete(void) {
    uint32_t length = inflight_length;
    mock_cdc.TxState = 0U;
    completion_at = -1;
    if (USBD_Interface_fops_HS.TransmitCplt)
        USBD_Interface_fops_HS.TransmitCplt(inflight, &length, 0U);
}
void osDelay(uint32_t ms) {
    ++delay_calls;
    clock_ms += ms;
    if (disconnect_at >= 0 && clock_ms >= disconnect_at)
        hUsbDeviceHS.dev_state = 0U;
    if (completion_at >= 0 && clock_ms >= completion_at) complete();
}
void USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *dev, uint8_t *buf) {
    (void)dev; (void)buf;
}
void USBD_CDC_ReceivePacket(USBD_HandleTypeDef *dev) { (void)dev; }
uint8_t CDC_Transmit_HS(uint8_t *buf, uint16_t len) {
    ++transmit_calls;
    if (busy_left) { --busy_left; return USBD_BUSY; }
    if (mock_cdc.TxState) return USBD_BUSY;
    inflight = buf;
    inflight_length = len;
    mock_cdc.TxState = 1U;
    ++transfers_started;
    if (auto_completion_ms >= 0) {
        completion_at = clock_ms + auto_completion_ms;
        if (auto_completion_ms == 0) complete();
    }
    return USBD_OK;
}

#include "@SOURCE@"

#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #expr); exit(1); \
} } while (0)
static void reset(void) {
    mock_cdc.TxState = 0U;
    hUsbDeviceHS.pClassData = &mock_cdc;
    hUsbDeviceHS.dev_state = USBD_STATE_CONFIGURED;
    clock_ms = 0;
    completion_at = disconnect_at = -1;
    auto_completion_ms = 1;
    delay_calls = busy_left = transmit_calls = transfers_started = 0U;
    inflight = NULL;
    inflight_length = 0U;
    cubemx_transport_close(NULL);
    CHECK(cubemx_transport_open(NULL));
}
static void receive(uint8_t *data, uint32_t len) {
    CHECK(USBD_Interface_fops_HS.Receive != NULL);
    CHECK(USBD_Interface_fops_HS.Receive(data, &len) == USBD_OK);
}
static void pattern(uint8_t *data, size_t len, unsigned salt) {
    for (size_t i = 0; i < len; ++i) data[i] = (uint8_t)(i * 37U + salt);
}

static void test_boundary(void) {
    uint8_t data[2000], tail[48], result[2000], err = 9U;
    reset(); pattern(data, sizeof(data), 7U); pattern(tail, sizeof(tail), 101U);
    receive(data, sizeof(data));
    CHECK(cubemx_transport_read(NULL, result, sizeof(result), 0, &err) == sizeof(data));
    CHECK(err == 0U && memcmp(data, result, sizeof(data)) == 0);
    receive(tail, sizeof(tail)); /* Write index reaches exactly 2048. */
    CHECK(cubemx_transport_get_pending() == sizeof(tail));
    CHECK(cubemx_transport_read(NULL, result, sizeof(result), 0, &err) == sizeof(tail));
    CHECK(memcmp(tail, result, sizeof(tail)) == 0);
    CHECK(cubemx_transport_get_pending() == 0U);
    CHECK(cubemx_transport_read(NULL, result, sizeof(result), 0, &err) == 0U);
    puts("PASS exact RX boundary wraps to an empty ring");
}

static void test_overflow(void) {
    uint8_t old[2047], excess[64], result[2048], err = 0U;
    reset(); pattern(old, sizeof(old), 19U); pattern(excess, sizeof(excess), 211U);
    receive(old, sizeof(old)); receive(excess, sizeof(excess));
    CHECK(cubemx_transport_get_pending() == sizeof(old));
    CHECK(cubemx_transport_read(NULL, result, sizeof(result), 0, &err) == sizeof(old));
    CHECK(err != 0U && memcmp(old, result, sizeof(old)) == 0);
    CHECK(cubemx_transport_get_pending() == 0U);
    receive(excess, sizeof(excess));
    CHECK(cubemx_transport_read(NULL, result, sizeof(result), 0, &err) == sizeof(excess));
    CHECK(err == 0U && memcmp(excess, result, sizeof(excess)) == 0);
    puts("PASS RX overflow preserves unread bytes and recovers");
}

static void test_busy_and_stale_flag(void) {
    uint8_t data[33], err = 9U;
    reset(); pattern(data, sizeof(data), 9U); busy_left = 3U;
    CHECK(cubemx_transport_write(NULL, data, sizeof(data), &err) == sizeof(data));
    CHECK(err == 0U && transmit_calls == 4U && transfers_started == 1U);
    CHECK(memcmp(inflight, data, sizeof(data)) == 0);
    reset(); complete(); /* A previous completion before this transfer starts. */
    auto_completion_ms = -1;
    CHECK(cubemx_transport_write(NULL, data, sizeof(data), &err) == 0U);
    CHECK(err != 0U && clock_ms == WRITE_TIMEOUT_MS && mock_cdc.TxState == 1U);
    puts("PASS USB busy retries and stale completion cannot fake success");
}

static void test_timeout_ownership(void) {
    uint8_t first[80], expected[80], second[80], err = 9U;
    reset(); pattern(first, sizeof(first), 39U); memcpy(expected, first, sizeof(first));
    pattern(second, sizeof(second), 88U); auto_completion_ms = -1;
    CHECK(cubemx_transport_write(NULL, first, sizeof(first), &err) == 0U);
    CHECK(err != 0U && transfers_started == 1U && inflight != first);
    memset(first, 0xEE, sizeof(first)); /* Caller releases/changes its buffer. */
    CHECK(memcmp(inflight, expected, sizeof(expected)) == 0);
    CHECK(cubemx_transport_write(NULL, second, sizeof(second), &err) == 0U);
    CHECK(err != 0U && transfers_started == 1U);
    CHECK(memcmp(inflight, expected, sizeof(expected)) == 0);
    CHECK(cubemx_transport_close(NULL)); CHECK(cubemx_transport_open(NULL));
    CHECK(memcmp(inflight, expected, sizeof(expected)) == 0);
    complete(); auto_completion_ms = 1;
    CHECK(cubemx_transport_write(NULL, second, sizeof(second), &err) == sizeof(second));
    CHECK(err == 0U && transfers_started == 2U);
    CHECK(memcmp(inflight, second, sizeof(second)) == 0);
    puts("PASS timed-out TX retains USB buffer through mutation, next write, and reopen");
}

static void test_disconnect_and_reopen(void) {
    uint8_t data[80], out[80], err = 9U;
    reset(); pattern(data, sizeof(data), 61U); receive(data, sizeof(data));
    CHECK(cubemx_transport_close(NULL)); CHECK(cubemx_transport_open(NULL));
    CHECK(cubemx_transport_get_pending() == 0U);
    CHECK(cubemx_transport_read(NULL, out, sizeof(out), 0, &err) == 0U);
    CHECK(err == 0U && delay_calls == 0U);
    auto_completion_ms = -1; disconnect_at = 4;
    CHECK(cubemx_transport_write(NULL, data, sizeof(data), &err) == 0U);
    CHECK(err != 0U && clock_ms == 4);
    reset(); hUsbDeviceHS.dev_state = 0U;
    CHECK(cubemx_transport_write(NULL, data, sizeof(data), &err) == 0U);
    CHECK(err != 0U && transfers_started == 0U && delay_calls == 0U);
    reset(); hUsbDeviceHS.pClassData = NULL;
    CHECK(cubemx_transport_write(NULL, data, sizeof(data), &err) == 0U);
    CHECK(err != 0U && clock_ms == WRITE_TIMEOUT_MS);
    puts("PASS disconnect/absent class fail and reopen resets RX");
}

static void test_nonblocking_and_guards(void) {
    uint8_t data[8], out[8], huge[USB_TX_BUFFER_SIZE + 1U], err = 9U;
    reset(); pattern(data, sizeof(data), 17U);
    CHECK(cubemx_transport_read(NULL, out, sizeof(out), 0, &err) == 0U);
    CHECK(err == 0U && delay_calls == 0U);
    receive(data, sizeof(data));
    CHECK(cubemx_transport_read(NULL, out, sizeof(out), 0, &err) == sizeof(data));
    CHECK(memcmp(data, out, sizeof(data)) == 0 && delay_calls == 0U);
    CHECK(cubemx_transport_write(NULL, huge, sizeof(huge), &err) == 0U && err != 0U);
    CHECK(cubemx_transport_write(NULL, NULL, 8U, &err) == 0U && err != 0U);
    CHECK(cubemx_transport_write(NULL, NULL, 0U, &err) == 0U && err == 0U);
    CHECK(cubemx_transport_read(NULL, NULL, 0U, 1, NULL) == 0U);
    CHECK(transfers_started == 0U && delay_calls == 0U);
    puts("PASS timeout-zero read and invalid/oversized TX guards");
}
int main(void) {
    test_boundary(); test_overflow(); test_busy_and_stale_flag();
    test_timeout_ownership(); test_disconnect_and_reopen(); test_nonblocking_and_guards();
    puts("All USB CDC host regression tests passed.");
    return 0;
}
""".replace("@SOURCE@", SOURCE.as_posix())
(OUT / "test_usb_cdc.c").write_text(HARNESS, encoding="utf-8")
gcc = shutil.which("gcc") or r"C:\msys64\ucrt64\bin\gcc.exe"
exe = OUT / "test_usb_cdc.exe"
environment = os.environ.copy()
environment["PATH"] = str(Path(gcc).parent) + os.pathsep + environment["PATH"]
subprocess.run([gcc, "-std=c11", "-O1", "-Wall", "-Wextra", "-Werror",
                "-I", str(OUT), str(OUT / "test_usb_cdc.c"), "-o", str(exe)],
               check=True, env=environment)
subprocess.run([str(exe)], check=True, env=environment)
