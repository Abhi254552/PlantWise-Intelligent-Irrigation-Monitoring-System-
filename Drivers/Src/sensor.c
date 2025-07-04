#include "main.h"
#include "stm32f4xx_hal.h"

ADC_HandleTypeDef hadc1;
static ADC_ChannelConfTypeDef sConfig = {0};

void sensor_init(void) {
    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;  // 42 MHz
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode          = ENABLE;
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.NbrOfConversion       = ADC_NUM_CHANNELS;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIOA_Init = {0};
    GPIOA_Init.Pin  = GPIO_PIN_0 | GPIO_PIN_1;
    GPIOA_Init.Mode = GPIO_MODE_ANALOG;
    GPIOA_Init.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIOA_Init);

    HAL_NVIC_SetPriority(ADC_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);

    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel      = ADC_CHANNEL_SOIL;
    sConfig.Rank         = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    sConfig.Channel      = ADC_CHANNEL_LIGHT;
    sConfig.Rank         = 2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}

void sensor_read_all(void) {
    if (HAL_ADC_Start_IT(&hadc1) != HAL_OK) {
        Error_Handler();
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc->Instance == ADC1) {
        moisture_raw = HAL_ADC_GetValue(&hadc1);
        light_raw = HAL_ADC_GetValue(&hadc1);
        m_flag = true;
        HAL_ADC_Stop_IT(&hadc1);
    }
}

float Sensor_ConvertToPercentage(uint32_t raw_value) {
    if (raw_value > 4095) raw_value = 4095;
    float percent = ((float)raw_value / 4095.0f) * 100.0f;
    return percent;
}

float CalibratedMoisture(uint32_t raw_value) {
    const float RAW_WET  =  500.0f;   
    const float RAW_DRY  = 3800.0f;   
    if (raw_value < RAW_WET)  raw_value = RAW_WET;
    if (raw_value > RAW_DRY)  raw_value = RAW_DRY;
    return ((RAW_DRY - (float)raw_value) / (RAW_DRY - RAW_WET)) * 100.0f;
}

void ADC_IRQHandler(void) {
    HAL_ADC_IRQHandler(&hadc1);
}
