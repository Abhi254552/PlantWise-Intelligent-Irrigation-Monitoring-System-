#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "stm32f4xx_hal.h"
#include <stdint.h>

extern ADC_HandleTypeDef hadc1;

void sensor_init(void);

void sensor_read_all(void);

float Sensor_ConvertToPercentage(uint32_t raw_value);

float CalibratedMoisture(uint32_t raw_value);
#endif 
