#include "dht11.h"
#include "delay.h"
#include "stm32f4xx_hal.h"

#define DHT_PORT        GPIOA
#define DHT_PIN         GPIO_PIN_7
#define DHT_TIMEOUT_US  1000

static void set_output(void) {
    GPIO_InitTypeDef cfg = {0};
    cfg.Pin   = DHT_PIN;
    cfg.Mode  = GPIO_MODE_OUTPUT_OD;
    cfg.Pull  = GPIO_PULLUP;
    cfg.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT_PORT, &cfg);
}

static void set_input(void) {
    GPIO_InitTypeDef cfg = {0};
    cfg.Pin  = DHT_PIN;
    cfg.Mode = GPIO_MODE_INPUT;
    cfg.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(DHT_PORT, &cfg);
}

void DHT11_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    set_output();
    HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_SET);
}

static uint8_t DHT11_CheckResponse(void) {
    uint32_t t = 0;
    while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_SET) {
        if (++t > DHT_TIMEOUT_US) return 0;
        delay_us(1);
    }
    t = 0;
    while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_RESET) {
        if (++t > DHT_TIMEOUT_US) return 0;
        delay_us(1);
    }
    return 1;
}

static uint8_t DHT11_ReadByte(void) {
    uint8_t val = 0;
    for (int i = 0; i < 8; i++) {
        uint32_t t = 0;
        while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_RESET) {
            if (++t > DHT_TIMEOUT_US) break;
            delay_us(1);
        }
        delay_us(40);
        val <<= 1;
        if (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_SET) {
            val |= 1;
        }
        t = 0;
        while (HAL_GPIO_ReadPin(DHT_PORT, DHT_PIN) == GPIO_PIN_SET) {
            if (++t > DHT_TIMEOUT_US) break;
            delay_us(1);
        }
    }
    return val;
}

DHT11_Status DHT11_Read(uint8_t *temperature, uint8_t *humidity) {
    uint8_t rh_i, rh_d, t_i, t_d, sum;

    set_output();
    HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_RESET);
    delay_ms(18);
    HAL_GPIO_WritePin(DHT_PORT, DHT_PIN, GPIO_PIN_SET);
    delay_us(40);

    set_input();
    if (!DHT11_CheckResponse()) {
        DHT11_Init();
        return DHT11_ERR;
    }

    rh_i = DHT11_ReadByte();
    rh_d = DHT11_ReadByte();
    t_i  = DHT11_ReadByte();
    t_d  = DHT11_ReadByte();
    sum  = DHT11_ReadByte();

    if ((uint8_t)(rh_i + rh_d + t_i + t_d) != sum) {
        DHT11_Init();
        return DHT11_ERR;
    }

    *humidity    = rh_i;
    *temperature = t_i;
    DHT11_Init();
    return DHT11_OK;
}
