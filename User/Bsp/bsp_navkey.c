#include "bsp_navkey.h"
#include <stdio.h>

ADC_HandleTypeDef hadc1;

static uint16_t s_last_adc_val = 0;
static NavKey_t s_last_key = NAV_KEY_NONE;
static uint8_t  s_debounce_count = 0;

void NavKey_Init(void)
{
    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    /* 1. Configure ADC Clock via PLL2 */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInitStruct.PLL2.PLL2M = 2;
    PeriphClkInitStruct.PLL2.PLL2N = 16;
    PeriphClkInitStruct.PLL2.PLL2P = 2;
    PeriphClkInitStruct.PLL2.PLL2Q = 2;
    PeriphClkInitStruct.PLL2.PLL2R = 2;
    PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
    PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
    PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
    (void)HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

    /* 2. Clock & GPIO Enable */
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA5 -> ADC1_INP19 */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* 3. Configure ADC1 Instance (Single conversion polling) */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV64;
    hadc1.Init.Resolution = ADC_RESOLUTION_16B;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
    hadc1.Init.OversamplingMode = DISABLE;

    if (HAL_ADC_Init(&hadc1) == HAL_OK)
    {
        multimode.Mode = ADC_MODE_INDEPENDENT;
        (void)HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);

        sConfig.Channel = ADC_CHANNEL_19;
        sConfig.Rank = ADC_REGULAR_RANK_1;
        sConfig.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;
        sConfig.SingleDiff = ADC_SINGLE_ENDED;
        sConfig.OffsetNumber = ADC_OFFSET_NONE;
        sConfig.Offset = 0;
        sConfig.OffsetSignedSaturation = DISABLE;
        (void)HAL_ADC_ConfigChannel(&hadc1, &sConfig);

        /* Run ADC calibration */
        (void)HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
    }
}

uint16_t NavKey_GetRawADC(void)
{
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK)
    {
        s_last_adc_val = (uint16_t)HAL_ADC_GetValue(&hadc1);
        return s_last_adc_val;
    }
    return s_last_adc_val;
}

static NavKey_t Decode_ADC_To_Key(uint16_t adc)
{
    /* 
     * Typical 5-Way Analog Key Resistor Divider Thresholds (16-bit 0-65535):
     * Idle (released): < 2000 or > 62000 (depending on hardware pull)
     */
    if (adc < 3000 || adc > 63000)
    {
        return NAV_KEY_NONE;
    }
    else if (adc >= 3000 && adc < 16000)
    {
        return NAV_KEY_LEFT;
    }
    else if (adc >= 16000 && adc < 29000)
    {
        return NAV_KEY_DOWN;
    }
    else if (adc >= 29000 && adc < 42000)
    {
        return NAV_KEY_RIGHT;
    }
    else if (adc >= 42000 && adc < 54000)
    {
        return NAV_KEY_UP;
    }
    else
    {
        return NAV_KEY_PRESS;
    }
}

NavKey_t NavKey_Scan(void)
{
    uint16_t adc = NavKey_GetRawADC();
    NavKey_t current_raw_key = Decode_ADC_To_Key(adc);

    NavKey_t event = NAV_KEY_NONE;

    if (current_raw_key != NAV_KEY_NONE)
    {
        if (s_last_key == NAV_KEY_NONE)
        {
            s_debounce_count++;
            if (s_debounce_count >= 2)
            {
                s_last_key = current_raw_key;
                event = current_raw_key; /* Single edge-triggered event */
            }
        }
    }
    else
    {
        s_debounce_count = 0;
        s_last_key = NAV_KEY_NONE;
    }

    return event;
}

