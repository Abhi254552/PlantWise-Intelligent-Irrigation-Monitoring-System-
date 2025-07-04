#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define ADC_CHANNEL_SOIL    ADC_CHANNEL_0
#define ADC_CHANNEL_LIGHT   ADC_CHANNEL_1
#define ADC_NUM_CHANNELS    2

#define SENSOR_UPDATE_MS    500        
#define IRRIGATION_CHECK_MS (60 * 60 * 1000) 

#define PUMP_MAX_RUNTIME_MS 60000  

#define WIFI_SSID       "Shiven"
#define WIFI_PASSWORD   "shiven424"
#define MQTT_BROKER_IP  "192.168.106.42"
#define MQTT_BROKER_PORT 1883
#define MQTT_CLIENT_ID  "STM32PlantNode"
#define MQTT_USER       ""
#define MQTT_PASS       ""

#define TOPIC_SENSORS       "plant/sensors"
#define TOPIC_CONTROL_APP   "plant/control/app"
#define TOPIC_CONFIG        "plant/config"
#define TOPIC_NOTIFICATION  "plant/notification"
#define TOPIC_CONFIG_ACK    "plant/config/ack"

#define LED_GPIO_PORT  GPIOD
#define LED_PIN        GPIO_PIN_12

#define DEBUG_OFF

extern volatile bool    s500_flag;
extern volatile bool    h_flag;
extern volatile bool    m_flag;
extern volatile bool    config_flag;
extern volatile bool    manual_flag;
extern volatile bool    manual_stop_flag;
extern volatile bool    watering;

extern volatile uint32_t moisture_raw;
extern volatile uint32_t light_raw;
extern volatile float    moisture_percent;
extern volatile float    light_percent;
extern volatile uint8_t  dht_temp;
extern volatile uint8_t  dht_hum;
extern volatile uint32_t moisture_threshold;
extern volatile uint32_t pump_timer_ms;

extern ADC_HandleTypeDef hadc1;

extern TIM_HandleTypeDef htim1;

void delayuS(uint32_t us);
void delayMS(uint32_t ms);

void mqtt_process_uart(void);

void SystemClock_Config(void);
void Error_Handler(void);

void sensor_init(void);
void relay_init(void);
void servo_init(void);
void wifi_init(void);
void mqtt_init(void);
void timer_init(void);

#endif
