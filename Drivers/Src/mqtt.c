#include "mqtt.h"
#include "main.h"
#include "config.h"          /* flash helpers */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

UART_HandleTypeDef huart2;

static void printf_uart(const char *fmt, ...)
{
    char buf[128];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);

    if (n > 0) {
        if (n > (int)sizeof buf) n = sizeof buf;
        HAL_UART_Transmit(&huart2, (uint8_t *)buf,
                          (uint16_t)n, HAL_MAX_DELAY);
    }
}

#define LED_GPIO_PORT  GPIOD
#define LED_PIN        GPIO_PIN_12

extern volatile uint32_t pump_timer_ms;
volatile bool esp_raw_busy = false;
volatile bool esp_busy = false;
volatile bool esp_prompt_ready = false;

void esp_send(const char *cmd)
{
    HAL_UART_Transmit(&huart2,(uint8_t*)cmd,strlen(cmd),HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart2,(uint8_t*)"\r\n",2,HAL_MAX_DELAY);
}

void esp_wifi_lowpower(bool enable) 
{
    if (enable) {
        esp_send("AT+RFPOWER=40");   
        esp_send("AT+SLEEP=2");    
    } else {
        esp_send("AT+SLEEP=0");     
        esp_send("AT+RFPOWER=82");   
    }
    HAL_Delay(50);
}

#define RX_BUF 256
static char     rx[RX_BUF];
static uint16_t idx = 0;

static bool wait_prompt(uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();
    while (HAL_GetTick() - t0 < ms) {
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) &&
            (char)(huart2.Instance->DR & 0xFF) == '>')
            return true;
    }
    return false;
}

static bool wait_ok(uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();
    char last[2] = {0};
    while (HAL_GetTick() - t0 < ms) {
        if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
            last[0] = last[1];
            last[1] = (char)(huart2.Instance->DR & 0xFF);
            if (last[0] == 'O' && last[1] == 'K') return true;
        }
    }
    return false;
}

void mqtt_publish_raw(const char *topic,const char *pay,uint8_t retain)
{
    if (esp_busy) return;

		uint8_t qos = (strcmp(topic, TOPIC_CONFIG_ACK) == 0) ? 2 : 0;
	
   char cmd[64];
   int n = snprintf(cmd,sizeof cmd,
        "AT+MQTTPUBRAW=0,\"%s\",%u,%u,%u\r\n",
        topic,(unsigned)strlen(pay),qos,retain);
    HAL_UART_Transmit(&huart2,(uint8_t*)cmd,(uint16_t)n,HAL_MAX_DELAY);

    esp_prompt_ready = false;                    
    uint32_t t0 = HAL_GetTick();
    while (!esp_prompt_ready && HAL_GetTick()-t0 < 500) { }

    if (!esp_prompt_ready) return;          

    esp_busy = true;
    HAL_UART_Transmit(&huart2,(uint8_t*)pay,
                      (uint16_t)strlen(pay),HAL_MAX_DELAY);
}

void mqtt_publish(const char *topic,const char *pay)
{
    if (esp_busy) return;

    char esc[256]; size_t j = 0;
    for (size_t i=0; pay[i]&&j<sizeof esc-2; ++i) {
        if (pay[i]=='"') esc[j++]='\\';
        esc[j++]=pay[i];
    }
    esc[j]='\0';

    int qos =
				(strcmp(topic, TOPIC_SENSORS)      == 0) ? 0 :
				(strcmp(topic, TOPIC_CONFIG_ACK)   == 0) ? 2 : 1;
    char cmd[384];
    int n = snprintf(cmd,sizeof cmd,
        "AT+MQTTPUB=0,\"%s\",\"%s\",%d,0\r\n",topic,esc,qos);
    HAL_UART_Transmit(&huart2,(uint8_t*)cmd,(uint16_t)n,HAL_MAX_DELAY);

    esp_busy = true;
}

void wifi_init(void)
{
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_2 | GPIO_PIN_3;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_PULLUP;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA,&g);

    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
		
		HAL_NVIC_SetPriority(USART2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
		
    HAL_UART_Init(&huart2);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

void mqtt_init(void)
{
		esp_send("ATE0");              
    HAL_Delay(50);
    esp_send("AT+RST");                 HAL_Delay(2000);
    esp_send("AT+GMR");                 HAL_Delay(200);
    esp_send("AT+CWMODE=1");            HAL_Delay(100);

    char cmd[128];
    snprintf(cmd,sizeof cmd,"AT+CWJAP=\"%s\",\"%s\"",
             WIFI_SSID,WIFI_PASSWORD);
    esp_send(cmd);                      HAL_Delay(5000);

    snprintf(cmd,sizeof cmd,
             "AT+MQTTUSERCFG=0,1,\"%s\",\"%s\",\"%s\",0,0,\"\"",
             MQTT_CLIENT_ID,MQTT_USER,MQTT_PASS);
    esp_send(cmd);                      HAL_Delay(200);

    esp_send("AT+MQTTCONNCFG=0,120,1,\"plant/notification\",\"offline\",1,1"); HAL_Delay(200);

    snprintf(cmd,sizeof cmd,"AT+MQTTCONN=0,\"%s\",%d,1",
             MQTT_BROKER_IP,MQTT_BROKER_PORT);
    esp_send(cmd);                      HAL_Delay(1000);

    snprintf(cmd,sizeof cmd,"AT+MQTTSUB=0,\"%s\",1",TOPIC_CONTROL_APP);
    esp_send(cmd);                      HAL_Delay(100);
    snprintf(cmd,sizeof cmd,"AT+MQTTSUB=0,\"%s\",1",TOPIC_CONFIG);
    esp_send(cmd);                      HAL_Delay(100);
}

void USART2_IRQHandler(void)
{
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
        char c = (char)(huart2.Instance->DR & 0xFF);

        if (c == '>') {                         
            esp_prompt_ready = true;
            __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_RXNE);
            return;
        }

        if (c == '\r' || c == '\n') {
            if (idx) {
                rx[idx]='\0';
                mqtt_process_line(rx);
                idx=0;
            }
        } else if (idx < RX_BUF-1) {
            rx[idx++]=c;
        }
        __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_RXNE);
    }
}

void mqtt_process_line(char *ln)
{
    if (strstr(ln,"+MQTTPUB:OK") || strstr(ln,"OK") || strstr(ln,"ERROR"))
        esp_busy = false;

    if (strstr(ln, "\"plant/control/app\"")) {

				if (strstr(ln, ",start")) { manual_flag = true;  manual_stop_flag = false; return; }
				if (strstr(ln, ",stop"))  { manual_stop_flag = true;                       return; }

				if (strstr(ln, "\"ON\"") || strstr(ln, "\"on\"")) {
						pump_timer_ms    = 5000;   
						manual_flag      = true;   
						manual_stop_flag = false;
						return;
				}
				if (strstr(ln, "\"OFF\"") || strstr(ln, "\"off\"")) {
						manual_stop_flag = true; 
						return;
				}
		}

    if (!strstr(ln,"\"plant/config\"")) return;
    char *json=strrchr(ln,'{'); if(!json) return;
    char *col=strchr(json,':'); if(!col)  return;
    while(*++col==' '||*col=='\t'||*col=='\"');
    uint32_t v=strtoul(col,NULL,10); if(v<1||v>100) return;
    moisture_threshold=v; config_flag=true; cfg_store_uint32(v);
}

void mqtt_process_uart(void) {}