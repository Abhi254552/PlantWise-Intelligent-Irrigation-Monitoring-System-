#include "stm32f4xx_hal.h"
#include "main.h"  // if you keep your ADC_CHANNEL_* defines there

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hadc->Instance == ADC1)
    {
        // 1) Clock enable
        __HAL_RCC_ADC1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        // 2) PA0 = ADC1_IN0, PA1 = ADC1_IN1
        GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        // 3) Enable interrupt if you use HAL_ADC_Start_IT()
        HAL_NVIC_SetPriority(ADC_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(ADC_IRQn);
    }
    // If you still use ADC3 anywhere, you can also init it here:
    // else if (hadc->Instance == ADC3) { …your existing ADC3 init… }
}
