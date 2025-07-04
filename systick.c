#include "main.h"      // for SENSOR_UPDATE_MS, IRRIGATION_CHECK_MS
#include "stm32f4xx_hal.h"

extern volatile bool s500_flag, h_flag;

void SysTick_Handler(void)
{
    // 1) keep HAL_Delay() working
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();

    // 2) your 500 ms / 1 h counters
    static uint32_t ms500 = 0, msH = 0;
    if (++ms500 >= SENSOR_UPDATE_MS) {
        ms500 = 0;
        s500_flag = true;
    }
    if (++msH >= IRRIGATION_CHECK_MS) {
        msH = 0;
        h_flag = true;
    }
}
