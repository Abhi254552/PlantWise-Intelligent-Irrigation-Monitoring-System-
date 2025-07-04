// servo.c
#include "servo.h"
#include "stm32f4xx_hal.h"

TIM_HandleTypeDef htim3; 

#define SERVO_TIM         TIM3
#define SERVO_CH          TIM_CHANNEL_1
#define SERVO_GPIO_PORT   GPIOA
#define SERVO_GPIO_PIN    GPIO_PIN_6 

#define SERVO_FREQ_HZ     50     
#define PULSE_MIN_US      500     
#define PULSE_MAX_US      1000    

static bool   sweeping = false;
static uint8_t direction = 0;   
static uint32_t pulse_us = PULSE_MIN_US;

void servo_init(void) {
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    HAL_NVIC_SetPriority(TIM3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);

    GPIO_InitTypeDef g = {0};
    g.Pin       = SERVO_GPIO_PIN;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_LOW;
    g.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(SERVO_GPIO_PORT, &g);

    uint32_t tim_clk = HAL_RCC_GetPCLK1Freq() * 2;  
    htim3.Instance = SERVO_TIM;
    htim3.Init.Prescaler = (tim_clk/1000000) - 1;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = (1000000 / SERVO_FREQ_HZ) - 1;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim3);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode     = TIM_OCMODE_PWM1;
    oc.Pulse      = PULSE_MIN_US;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    HAL_TIM_PWM_ConfigChannel(&htim3, &oc, SERVO_CH);
    HAL_TIM_PWM_Start(&htim3, SERVO_CH);
}

void servo_start_sweep(void) {
    sweeping = true;
    direction = 0;
    pulse_us = PULSE_MIN_US;
    __HAL_TIM_SET_COMPARE(&htim3, SERVO_CH, pulse_us);
    __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
}

void servo_stop(void) {
    sweeping = false;
    __HAL_TIM_DISABLE_IT(&htim3, TIM_IT_UPDATE);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *ht) {
    if (ht->Instance == SERVO_TIM && sweeping) {
        if (direction == 0) {
            pulse_us += 20;
            if (pulse_us >= PULSE_MAX_US) {
                pulse_us = PULSE_MAX_US;
                direction = 1;
            }
        } else {
            pulse_us -= 20;
            if (pulse_us <= PULSE_MIN_US) {
                pulse_us = PULSE_MIN_US;
                direction = 0;
            }
        }
        __HAL_TIM_SET_COMPARE(&htim3, SERVO_CH, pulse_us);
    }
}
