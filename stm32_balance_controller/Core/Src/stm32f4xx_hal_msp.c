#include "main.h"

void HAL_MspInit(void)
{
	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_RCC_PWR_CLK_ENABLE();
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C1) {
		GPIO_InitTypeDef gpio_init = {0};

		__HAL_RCC_GPIOB_CLK_ENABLE();
		gpio_init.Pin = GPIO_PIN_6 | GPIO_PIN_7;
		gpio_init.Mode = GPIO_MODE_AF_OD;
		gpio_init.Pull = GPIO_PULLUP;
		gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		gpio_init.Alternate = GPIO_AF4_I2C1;
		HAL_GPIO_Init(GPIOB, &gpio_init);

		__HAL_RCC_I2C1_CLK_ENABLE();
	}
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c)
{
	if (hi2c->Instance == I2C1) {
		__HAL_RCC_I2C1_CLK_DISABLE();
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7);
	}
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		GPIO_InitTypeDef gpio_init = {0};

		__HAL_RCC_GPIOA_CLK_ENABLE();
		gpio_init.Pin = GPIO_PIN_9 | GPIO_PIN_10;
		gpio_init.Mode = GPIO_MODE_AF_PP;
		gpio_init.Pull = GPIO_PULLUP;
		gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		gpio_init.Alternate = GPIO_AF7_USART1;
		HAL_GPIO_Init(GPIOA, &gpio_init);

		__HAL_RCC_USART1_CLK_ENABLE();
	}
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART1) {
		__HAL_RCC_USART1_CLK_DISABLE();
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9 | GPIO_PIN_10);
	}
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2) {
		__HAL_RCC_TIM2_CLK_ENABLE();
	}

	if (htim->Instance == TIM3) {
		__HAL_RCC_TIM3_CLK_ENABLE();
	}
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2) {
		GPIO_InitTypeDef gpio_init = {0};

		__HAL_RCC_GPIOA_CLK_ENABLE();
		gpio_init.Pin = GPIO_PIN_0 | GPIO_PIN_1;
		gpio_init.Mode = GPIO_MODE_AF_PP;
		gpio_init.Pull = GPIO_NOPULL;
		gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		gpio_init.Alternate = GPIO_AF1_TIM2;
		HAL_GPIO_Init(GPIOA, &gpio_init);
	}

	if (htim->Instance == TIM3) {
		GPIO_InitTypeDef gpio_init = {0};

		__HAL_RCC_GPIOB_CLK_ENABLE();
		gpio_init.Pin = GPIO_PIN_0 | GPIO_PIN_1;
		gpio_init.Mode = GPIO_MODE_AF_PP;
		gpio_init.Pull = GPIO_NOPULL;
		gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		gpio_init.Alternate = GPIO_AF2_TIM3;
		HAL_GPIO_Init(GPIOB, &gpio_init);
	}
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2) {
		__HAL_RCC_TIM2_CLK_DISABLE();
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1);
	}

	if (htim->Instance == TIM3) {
		__HAL_RCC_TIM3_CLK_DISABLE();
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0 | GPIO_PIN_1);
	}
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == ADC1 && ROBOT_USE_ALT_ENABLE_PWM_PINS == 0u) {
		GPIO_InitTypeDef gpio_init = {0};

		__HAL_RCC_GPIOA_CLK_ENABLE();
		gpio_init.Pin = GPIO_PIN_0;
		gpio_init.Mode = GPIO_MODE_ANALOG;
		gpio_init.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &gpio_init);

		__HAL_RCC_ADC1_CLK_ENABLE();
	}
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == ADC1 && ROBOT_USE_ALT_ENABLE_PWM_PINS == 0u) {
		__HAL_RCC_ADC1_CLK_DISABLE();
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);
	}
}