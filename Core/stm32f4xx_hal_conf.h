#ifndef __STM32F4xx_HAL_CONF_H
#define __STM32F4xx_HAL_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* ########################## Module Selection ############################## */
#define HAL_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
#define HAL_EXTI_MODULE_ENABLED
// #define HAL_ETH_MODULE_ENABLED   // Uncomment only if using Ethernet
// #define HAL_SPI_MODULE_ENABLED   // Uncomment only if using SPI

/* ########################## Oscillator Values adaptation ################## */
#define HSE_VALUE               ((uint32_t)8000000U)     /*!< External crystal osc freq */
#define HSE_STARTUP_TIMEOUT     ((uint32_t)100U)         /*!< HSE startup timeout */
#define HSI_VALUE               ((uint32_t)16000000U)    /*!< Internal RC osc freq */
#define LSI_VALUE               ((uint32_t)32000U)
#define LSE_VALUE               ((uint32_t)32768U)
#define LSE_STARTUP_TIMEOUT     ((uint32_t)5000U)
#define EXTERNAL_CLOCK_VALUE    ((uint32_t)12288000U)

/* ########################### System Configuration ######################### */
#define  VDD_VALUE                    ((uint32_t)3300U)   /*!< VDD in mV */
#define  TICK_INT_PRIORITY            ((uint32_t)0x0FU)   /*!< Lowest priority */
#define  USE_RTOS                     0U
#define  PREFETCH_ENABLE              1U
#define  INSTRUCTION_CACHE_ENABLE     1U
#define  DATA_CACHE_ENABLE            1U

/* ################## Assert Selection ############################## */
#define assert_param(expr) ((void)0U)

/* ################## Register Callbacks Configuration ###################### */
#define  USE_HAL_ADC_REGISTER_CALLBACKS        0U
#define  USE_HAL_TIM_REGISTER_CALLBACKS        0U
#define  USE_HAL_UART_REGISTER_CALLBACKS       0U

/* ################## Ethernet peripheral configuration ##################### */
#define ETH_RX_BUF_SIZE                ETH_MAX_PACKET_SIZE
#define ETH_TX_BUF_SIZE                ETH_MAX_PACKET_SIZE
#define ETH_RXBUFNB                    ((uint32_t)4)
#define ETH_TXBUFNB                    ((uint32_t)4)

/* ################## SPI peripheral configuration ########################## */
#define SPI_DATASIZE_16BIT            ((uint32_t)0x0000000FU)

/* ################## Includes ################################################ */
#include "stm32f4xx_hal_def.h"

#ifdef HAL_RCC_MODULE_ENABLED
  #include "stm32f4xx_hal_rcc.h"
  #include "stm32f4xx_hal_rcc_ex.h"
#endif

#ifdef HAL_GPIO_MODULE_ENABLED
  #include "stm32f4xx_hal_gpio.h"
#endif

#ifdef HAL_DMA_MODULE_ENABLED
  #include "stm32f4xx_hal_dma.h"
#endif

#ifdef HAL_CORTEX_MODULE_ENABLED
  #include "stm32f4xx_hal_cortex.h"
#endif

#ifdef HAL_ADC_MODULE_ENABLED
  #include "stm32f4xx_hal_adc.h"
  #include "stm32f4xx_hal_adc_ex.h"
#endif

#ifdef HAL_TIM_MODULE_ENABLED
  #include "stm32f4xx_hal_tim.h"
  #include "stm32f4xx_hal_tim_ex.h"
#endif

#ifdef HAL_UART_MODULE_ENABLED
  #include "stm32f4xx_hal_uart.h"
#endif

#ifdef HAL_FLASH_MODULE_ENABLED
  #include "stm32f4xx_hal_flash.h"
  #include "stm32f4xx_hal_flash_ex.h"
#endif

#ifdef HAL_PWR_MODULE_ENABLED
  #include "stm32f4xx_hal_pwr.h"
  #include "stm32f4xx_hal_pwr_ex.h"
#endif

#ifdef HAL_EXTI_MODULE_ENABLED
  #include "stm32f4xx_hal_exti.h"
#endif

#include "stm32f4xx_hal.h" // this must come last

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_HAL_CONF_H */
