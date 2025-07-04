// Core/delay.c
#include "delay.h"
#include "main.h"      // pulls in extern htim1 and __HAL_TIM_GET_COUNTER/etc.

void delay_us(uint32_t us) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    // busy-wait until TIM1 count reaches the requested microseconds
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

void delay_ms(uint32_t ms) {
    while (ms--) {
        delay_us(1000);
    }
}