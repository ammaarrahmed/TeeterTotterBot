#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define HAL_MODULE_ENABLED
#define HAL_RCC_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_CORTEX_MODULE_ENABLED
#define HAL_FLASH_MODULE_ENABLED
#define HAL_PWR_MODULE_ENABLED
#define HAL_I2C_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_ADC_MODULE_ENABLED

#ifndef HSI_VALUE
#define HSI_VALUE ((uint32_t)16000000U)
#endif

#ifndef HSE_VALUE
#define HSE_VALUE ((uint32_t)25000000U)
#endif

#ifndef LSI_VALUE
#define LSI_VALUE ((uint32_t)32000U)
#endif

#ifndef LSE_VALUE
#define LSE_VALUE ((uint32_t)32768U)
#endif

#define  VDD_VALUE                    ((uint32_t)3300U)
#define  TICK_INT_PRIORITY            ((uint32_t)0x0FU)
#define  USE_RTOS                     0U
#define  PREFETCH_ENABLE              1U
#define  INSTRUCTION_CACHE_ENABLE     1U
#define  DATA_CACHE_ENABLE            1U

#define  USE_HAL_ADC_REGISTER_CALLBACKS        0U
#define  USE_HAL_I2C_REGISTER_CALLBACKS        0U
#define  USE_HAL_TIM_REGISTER_CALLBACKS        0U
#define  USE_HAL_UART_REGISTER_CALLBACKS       0U

#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_cortex.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_pwr.h"
#include "stm32f4xx_hal_i2c.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_hal_adc.h"

#include <stddef.h>

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line);
#endif

#ifdef __cplusplus
}
#endif