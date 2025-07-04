#include "delay.h"
#include "main.h"      

void delay_us(uint32_t us) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

void delay_ms(uint32_t ms) {
    while (ms--) {
        delay_us(1000);
    }
}