#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"
#include "stm32f4xx_hal_tim.h"

extern TIM_HandleTypeDef htim3;

void servo_init(void);
void servo_start_sweep(void);
void servo_stop(void);

#endif
