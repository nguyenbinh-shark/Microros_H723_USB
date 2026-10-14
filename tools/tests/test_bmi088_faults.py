"""Host fault injection against the actual BMI088 driver and SPI middleware."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
driver = root / "User/Devices/BMI088"

main_h = r'''
#ifndef MAIN_H
#define MAIN_H
#include <stddef.h>
#include <stdint.h>
typedef struct { int unused; } SPI_HandleTypeDef;
typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;
typedef enum { GPIO_PIN_RESET = 0, GPIO_PIN_SET = 1 } GPIO_PinState;
#define ACC_CS_GPIO_Port ((void *)1)
#define GYRO_CS_GPIO_Port ((void *)2)
#define ACC_CS_Pin 1
#define GYRO_CS_Pin 2
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *, uint8_t *, uint8_t *, uint16_t, uint32_t);
void HAL_GPIO_WritePin(void *, uint16_t, GPIO_PinState);
void HAL_Delay(uint32_t);
#endif
'''

test_c = r'''
#include "BMI088driver.h"
#include "BMI088Middleware.h"
#include "BMI088reg.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

extern uint8_t caliOffset;
extern float BMI088_ACCEL_SEN, BMI088_GYRO_SEN;
static uint8_t accel_registers[256], gyro_registers[256];
static unsigned calls, fail_at, phase, selected, reg_address, last_timeout;
static SPI_HandleTypeDef spi;

void HAL_GPIO_WritePin(void *port, uint16_t pin, GPIO_PinState state)
{
    (void)pin;
    if (state == GPIO_PIN_RESET) { selected = (unsigned)(uintptr_t)port; phase = 0; }
    else { selected = 0; }
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *handle, uint8_t *tx,
                                         uint8_t *rx, uint16_t len, uint32_t timeout)
{
    assert(handle == &spi && len == 1 && selected != 0);
    calls++;
    last_timeout = timeout;
    if (fail_at == calls) { *rx = 0xff; return HAL_TIMEOUT; }
    if (phase == 0) { reg_address = *tx & 0x7f; *rx = 0; }
    else if (selected == 1 && phase == 1) { *rx = 0; } /* accel SPI dummy */
    else {
        unsigned offset = phase - (selected == 1 ? 2U : 1U);
        *rx = (selected == 1 ? accel_registers : gyro_registers)[reg_address + offset];
    }
    phase++;
    return HAL_OK;
}

void HAL_Delay(uint32_t ms) { (void)ms; }
void DWT_Delay(float seconds) { (void)seconds; }
float DWT_GetTimeline_s(void) { static float t; t += 1; return t; }

static void reset_fixture(void)
{
    calls = fail_at = 0;
    BMI088_SPI = &spi;
    BMI088.gNorm = 9.81f;
    BMI088.AccelScale = 1.0f;
    caliOffset = 0;
    for (unsigned i = 0; i < 3; i++) {
        BMI088.Accel[i] = 100.0f + i;
        BMI088.Gyro[i] = 200.0f + i;
        BMI088.AccelOffset[i] = BMI088.GyroOffset[i] = 0;
    }
    BMI088.Temperature = 321.0f;
    accel_registers[BMI088_ACC_CHIP_ID] = BMI088_ACC_CHIP_ID_VALUE;
    gyro_registers[BMI088_GYRO_CHIP_ID] = BMI088_GYRO_CHIP_ID_VALUE;
    accel_registers[BMI088_ACCEL_XOUT_L + 4] = 5469 & 0xff;
    accel_registers[BMI088_ACCEL_XOUT_L + 5] = 5469 >> 8;
    gyro_registers[2] = 1111 & 0xff;
    gyro_registers[3] = 1111 >> 8;
}

static void assert_old_sample(void)
{
    assert(!BMI088_ReadIsValid());
    for (unsigned i = 0; i < 3; i++) {
        assert(BMI088.Accel[i] == 100.0f + i);
        assert(BMI088.Gyro[i] == 200.0f + i);
    }
    assert(BMI088.Temperature == 321.0f);
}

int main(void)
{
    reset_fixture();
    assert(!BMI088_ReadIsValid());
    BMI088_Read(&BMI088);
    assert(BMI088_ReadIsValid());
    unsigned normal_calls = calls;
    assert(normal_calls == 24 && last_timeout == 1);
    assert(fabsf(BMI088.Accel[2] - 5469 * BMI088_ACCEL_SEN) < 1e-5f);
    assert(fabsf(BMI088.Gyro[0] - 1111 * BMI088_GYRO_SEN) < 1e-5f);
    assert(BMI088.Temperature == 23.0f);

    for (unsigned byte = 1; byte <= normal_calls; byte++) {
        reset_fixture();
        fail_at = byte;
        BMI088_Read(&BMI088);
        assert_old_sample();
        assert(calls == byte && !BMI088_SPI_IsValid());
        calls = fail_at = 0;
        BMI088_Read(&BMI088);
        assert(BMI088_ReadIsValid() && calls == normal_calls);
    }

    reset_fixture();
    accel_registers[BMI088_ACC_CHIP_ID] = 0xff;
    BMI088_Read(&BMI088);
    assert_old_sample(); assert(calls == 3);
    reset_fixture();
    gyro_registers[BMI088_GYRO_CHIP_ID] = 0;
    BMI088_Read(&BMI088);
    assert_old_sample(); assert(calls == 20);

    reset_fixture(); BMI088.gNorm = 0;
    BMI088_Read(&BMI088); assert_old_sample(); assert(calls == 0);
    reset_fixture(); BMI088.gNorm = NAN;
    BMI088_Read(&BMI088); assert_old_sample(); assert(calls == 0);
    reset_fixture(); BMI088.AccelScale = INFINITY;
    BMI088_Read(&BMI088); assert_old_sample(); assert(calls == 0);
    reset_fixture(); caliOffset = 1; BMI088.GyroOffset[1] = NAN;
    BMI088_Read(&BMI088); assert_old_sample();
    reset_fixture(); caliOffset = 1; BMI088.AccelOffset[0] = INFINITY;
    BMI088_Read(&BMI088); assert_old_sample();
    reset_fixture(); BMI088_SPI = NULL;
    BMI088_Read(&BMI088); assert_old_sample(); assert(calls == 0);
    reset_fixture();
    BMI088_Read(NULL); assert_old_sample(); assert(calls == 0);
    puts("PASS: all 24 SPI-byte faults, atomic sample, chip IDs, finite calibration, nulls and recovery");
    return 0;
}
'''

with tempfile.TemporaryDirectory(prefix="bmi088-faults-") as temp_name:
    temp = Path(temp_name)
    (temp / "main.h").write_text(main_h)
    (temp / "test.c").write_text(test_c)
    exe = temp / "test.exe"
    subprocess.run([
        "gcc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-parentheses",
        "-I", str(temp), "-I", str(driver), "-I", str(root / "User/Bsp"),
        str(temp / "test.c"), str(driver / "BMI088driver.c"),
        str(driver / "BMI088Middleware.c"), "-lm", "-o", str(exe),
    ], check=True)
    subprocess.run([str(exe)], check=True)
