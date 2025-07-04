#include "main.h"
#include "sensor.h"
#include "dht11.h"
#include "	relay.h"
#include "servo.h"
#include "mqtt.h"
#include "timer.h"
#include "delay.h"
#include "config.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "stm32f4xx_hal_pwr.h"

TIM_HandleTypeDef htim1;

volatile bool    s500_flag        = false;
volatile bool    h_flag           = false;
volatile bool    m_flag           = false;
volatile bool    config_flag      = false;
volatile bool    manual_flag      = false;
volatile bool    manual_stop_flag = false;

volatile bool     watering        = false;
volatile uint32_t pump_timer_ms   = 0;   

volatile uint32_t moisture_raw      = 0;
volatile uint32_t light_raw         = 0;
volatile float    moisture_percent  = 0.0f;
volatile float    light_percent     = 0.0f;
volatile uint8_t  dht_temp          = 0;
volatile uint8_t  dht_hum           = 0;
volatile uint32_t moisture_threshold = 50;  

extern ADC_HandleTypeDef  hadc1;
extern UART_HandleTypeDef huart2;

#ifdef DEBUG_OFF
#define printf_uart(...)  ((void)0)
#else
static void printf_uart(const char *fmt, ...)
{
    extern volatile bool esp_raw_busy;       
    if (esp_raw_busy) return;

    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);

    if (n > 0) {
        if (n > (int)sizeof buf) n = sizeof buf;
        HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n, HAL_MAX_DELAY);
    }
}
#endif

static void idle_sleep(void)
{
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
    HAL_SuspendTick();                    
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
    HAL_ResumeTick();                  

    SystemClock_Config();
}

static void led_init(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin  = LED_PIN;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(LED_GPIO_PORT, &g);
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET);
}

static void MX_TIM1_Init(void)
{
    __HAL_RCC_TIM1_CLK_ENABLE();
    htim1.Instance           = TIM1;
    htim1.Init.Prescaler     = (SystemCoreClock / 1000000) - 1;
    htim1.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim1.Init.Period        = 0xFFFF;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim1);
    HAL_TIM_Base_Start(&htim1);
}

int main(void)
{
    HAL_Init();

    cfg_load_uint32(&moisture_threshold);
    if (moisture_threshold == 0xFFFFFFFF || moisture_threshold == 0)
        moisture_threshold = 50;
		
    SystemClock_Config();

    DHT11_Init();
    relay_init();              
    servo_init();              
    wifi_init();
    mqtt_init();
    timer_init();                
    MX_TIM1_Init();
    led_init();

    mqtt_publish(TOPIC_NOTIFICATION, "online");

    bool adc_ready = false;
    while (1) {
        if (s500_flag) {
            s500_flag = false;
            HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);

            if (adc_ready) {              
                HAL_ADC_Stop(&hadc1);
                HAL_ADC_DeInit(&hadc1);
                __HAL_RCC_ADC1_CLK_DISABLE();
                adc_ready = false;
            }
            static uint8_t tick = 0;
            if (++tick >= (1000 / SENSOR_UPDATE_MS)) {
                tick = 0;
                uint8_t T = 0, H = 0;
                if (DHT11_Read(&T, &H) == DHT11_OK) {
                    dht_temp = 2*T;
                    dht_hum  = H/2;
                }
            }

            if (!adc_ready) {
                __HAL_RCC_ADC1_CLK_ENABLE();
                sensor_init();
                adc_ready = true;
            }
            HAL_ADC_Start(&hadc1);
            if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
                moisture_raw = HAL_ADC_GetValue(&hadc1);
            if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK)
                light_raw = HAL_ADC_GetValue(&hadc1);
            HAL_ADC_Stop(&hadc1);

            moisture_percent = CalibratedMoisture(moisture_raw);
            light_percent    = Sensor_ConvertToPercentage(light_raw);
            printf_uart("DBG: thresh=%lu   moist=%.1f%%   timer=%lu\r\n",
                        moisture_threshold, moisture_percent, pump_timer_ms);

            {
                char payload[128];
                int n = snprintf(payload, sizeof payload,
                    "{\"moisture\":%.1f,\"light\":%.1f,\"temp\":%u,\"hum\":%u}",
                    moisture_percent, light_percent, dht_temp, dht_hum);
                if (n > 0 && n < (int)sizeof(payload))
                    mqtt_publish_raw(TOPIC_SENSORS, payload, 0);
            }

            if (pump_timer_ms == 0) {
							if (moisture_percent < (float)moisture_threshold) {
									if (!watering) {
											watering = true;
											HAL_Delay(500);
											mqtt_publish(TOPIC_NOTIFICATION, "watering_start");
											HAL_Delay(500);
											relay_set(true);           
											servo_start_sweep();
									}
							} else {
									if (watering) {
											watering = false;
											mqtt_publish(TOPIC_NOTIFICATION,"watering_stop");
											relay_set(false);
											servo_stop();
									}
							}
					}
			} 
				
        if (h_flag) {
					h_flag = false;
					if (pump_timer_ms == 0) {
							if (moisture_percent < (float)moisture_threshold) {
									if (!watering) {
											watering = true;
											HAL_Delay(500);
											mqtt_publish(TOPIC_NOTIFICATION, "watering_start");
											HAL_Delay(500);
											relay_set(true);               
											servo_start_sweep();
									}
							} else {
									if (watering) {
											watering = false;
											mqtt_publish(TOPIC_NOTIFICATION,"watering_stop");
											relay_set(false);
											servo_stop();
									}
							}
					}
			}

        if (config_flag) {
            config_flag = false;
            printf_uart("[main] cfg_flag handled, thresh=%lu\r\n",
                        moisture_threshold);

            cfg_store_uint32(moisture_threshold); 

            char ack[64];
            snprintf(ack, sizeof ack,
                     "{\"threshold_ack\":%lu}", moisture_threshold);
            mqtt_publish_raw(TOPIC_CONFIG_ACK, ack, 1);
        }

        if (manual_flag) {
							manual_flag = false;
							if (!watering) {
									watering = true;
									HAL_Delay(500);
									mqtt_publish(TOPIC_NOTIFICATION, "watering_start");
									HAL_Delay(500);
									relay_set(true);                       
									servo_start_sweep();
							}
					}

					if (manual_stop_flag) {
							manual_stop_flag = false;
							if (watering) {
									watering = false;
									mqtt_publish(TOPIC_NOTIFICATION,"watering_stop");
									relay_set(false);
									servo_stop();
							}
							pump_timer_ms = 0;
					}

        mqtt_process_uart();
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 8;
    RCC_OscInitStruct.PLL.PLLN       = 336;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_SYSCLK   |
                                       RCC_CLOCKTYPE_HCLK     |
                                       RCC_CLOCKTYPE_PCLK1    |
                                       RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
        Error_Handler();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {
			
    }
}
