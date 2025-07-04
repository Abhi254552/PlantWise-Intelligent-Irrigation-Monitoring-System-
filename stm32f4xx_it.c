#include "main.h"
#include "stm32f4xx_hal.h"
#include "servo.h"

extern volatile bool s500_flag, h_flag;
extern volatile uint32_t pump_timer_ms;
extern volatile bool     manual_stop_flag;

void TIM3_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim3);
}

void SysTick_Handler(void) {
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();

    static uint32_t ms500 = 0, msH = 0;
    if (++ms500 >= SENSOR_UPDATE_MS) {
        ms500 = 0;
        s500_flag = true;
    }
    if (++msH >= IRRIGATION_CHECK_MS) {
        msH = 0;
        h_flag = true;
    }
		if (pump_timer_ms) {
    if (--pump_timer_ms == 0) {
        manual_stop_flag = true;
    }
	}
}
