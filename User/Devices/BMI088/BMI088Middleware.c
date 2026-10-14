#include "BMI088Middleware.h"
#include "main.h"

SPI_HandleTypeDef *BMI088_SPI;

/* One byte normally takes microseconds; keep a failed runtime read bounded. */
#define BMI088_SPI_TIMEOUT_MS 1U
static uint8_t bmi088_spi_valid = 1U;

void BMI088_SPI_ResetStatus(void)
{
    bmi088_spi_valid = 1U;
}

uint8_t BMI088_SPI_IsValid(void)
{
    return bmi088_spi_valid;
}

void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data = 0U;
    if (!bmi088_spi_valid)
        return 0U;

    if (BMI088_SPI == NULL ||
        HAL_SPI_TransmitReceive(BMI088_SPI, &txdata, &rx_data, 1,
                                BMI088_SPI_TIMEOUT_MS) != HAL_OK)
    {
        bmi088_spi_valid = 0U;
        return 0U;
    }
    return rx_data;
}
