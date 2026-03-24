#include "main.h"

#include "freertos_tasks.h"

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
ADC_HandleTypeDef hadc1;

static void MX_NVIC_Init(void);
static void MX_MotorEnablePinsAsGpioOutputs(void);
static void MX_StartMotorPwmOutputs(void);

int main(void)
{
	HAL_Init();
	SystemClock_Config();

	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_ADC1_Init();
	MX_NVIC_Init();

	if (ROBOT_FORCE_ENABLE_PINS_HIGH_FOR_TEST != 0u) {
		MX_MotorEnablePinsAsGpioOutputs();
		HAL_GPIO_WritePin(ROBOT_LEFT_MOTOR_ENABLE_GPIO_Port, ROBOT_LEFT_MOTOR_ENABLE_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ROBOT_RIGHT_MOTOR_ENABLE_GPIO_Port, ROBOT_RIGHT_MOTOR_ENABLE_Pin, GPIO_PIN_SET);
	} else if (ROBOT_USE_SOFTWARE_PWM_ENABLE_PINS != 0u) {
		MX_MotorEnablePinsAsGpioOutputs();
		if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) {
			Error_Handler();
		}
	} else {
		MX_StartMotorPwmOutputs();
	}

	robot_runtime_init();
	robot_uart_start_receive_it();

	while (1) {
		robot_runtime_poll();
	}
}

void MX_TIM2_Init(void)
{
	TIM_ClockConfigTypeDef clock_source_config = {0};
	TIM_MasterConfigTypeDef master_config = {0};
	TIM_OC_InitTypeDef oc_config = {0};

	htim2.Instance = TIM2;
	if (ROBOT_USE_SOFTWARE_PWM_ENABLE_PINS != 0u) {
		__HAL_RCC_TIM2_CLK_ENABLE();
		htim2.Init.Prescaler = ROBOT_SOFTWARE_PWM_TIMER_PRESCALER;
		htim2.Init.Period = ROBOT_SOFTWARE_PWM_TIMER_PERIOD_TICKS;
	} else {
		htim2.Init.Prescaler = ROBOT_PWM_TIMER_PRESCALER;
		htim2.Init.Period = ROBOT_PWM_PERIOD_TICKS;
	}
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}

	clock_source_config.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &clock_source_config) != HAL_OK) {
		Error_Handler();
	}

	if (ROBOT_USE_SOFTWARE_PWM_ENABLE_PINS == 0u) {
		if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
			Error_Handler();
		}

		master_config.MasterOutputTrigger = TIM_TRGO_RESET;
		master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
		if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &master_config) != HAL_OK) {
			Error_Handler();
		}

		oc_config.OCMode = TIM_OCMODE_PWM1;
		oc_config.Pulse = 0;
		oc_config.OCPolarity = TIM_OCPOLARITY_HIGH;
		oc_config.OCFastMode = TIM_OCFAST_DISABLE;

		if (HAL_TIM_PWM_ConfigChannel(&htim2, &oc_config, TIM_CHANNEL_1) != HAL_OK) {
			Error_Handler();
		}

		if (HAL_TIM_PWM_ConfigChannel(&htim2, &oc_config, TIM_CHANNEL_2) != HAL_OK) {
			Error_Handler();
		}

		HAL_TIM_MspPostInit(&htim2);
	}
}

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef osc_init = {0};
	RCC_ClkInitTypeDef clk_init = {0};

	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	osc_init.HSIState = RCC_HSI_ON;
	osc_init.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	osc_init.PLL.PLLState = RCC_PLL_ON;
	osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	osc_init.PLL.PLLM = 16;
	osc_init.PLL.PLLN = 336;
	osc_init.PLL.PLLP = RCC_PLLP_DIV4;
	osc_init.PLL.PLLQ = 7;

	if (HAL_RCC_OscConfig(&osc_init) != HAL_OK) {
		Error_Handler();
	}

	clk_init.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
	clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
	clk_init.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef gpio_init = {0};

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	HAL_GPIO_WritePin(
		GPIOB,
		ROBOT_LEFT_MOTOR_IN1_Pin | ROBOT_LEFT_MOTOR_IN2_Pin | ROBOT_RIGHT_MOTOR_IN1_Pin | ROBOT_RIGHT_MOTOR_IN2_Pin,
		GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ROBOT_STATUS_LED_GPIO_Port, ROBOT_STATUS_LED_Pin, GPIO_PIN_SET);

	gpio_init.Pin = ROBOT_LEFT_MOTOR_IN1_Pin | ROBOT_LEFT_MOTOR_IN2_Pin | ROBOT_RIGHT_MOTOR_IN1_Pin | ROBOT_RIGHT_MOTOR_IN2_Pin;
	gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init.Pull = GPIO_NOPULL;
	gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &gpio_init);

	gpio_init.Pin = ROBOT_STATUS_LED_Pin;
	gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init.Pull = GPIO_NOPULL;
	gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ROBOT_STATUS_LED_GPIO_Port, &gpio_init);
}

void MX_I2C1_Init(void)
{
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
}

void MX_USART1_UART_Init(void)
{
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;

	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
}

void MX_TIM3_Init(void)
{
	TIM_ClockConfigTypeDef clock_source_config = {0};
	TIM_MasterConfigTypeDef master_config = {0};
	TIM_OC_InitTypeDef oc_config = {0};

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = ROBOT_PWM_TIMER_PRESCALER;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = ROBOT_PWM_PERIOD_TICKS;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}

	clock_source_config.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &clock_source_config) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}

	master_config.MasterOutputTrigger = TIM_TRGO_RESET;
	master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &master_config) != HAL_OK) {
		Error_Handler();
	}

	oc_config.OCMode = TIM_OCMODE_PWM1;
	oc_config.Pulse = 0;
	oc_config.OCPolarity = TIM_OCPOLARITY_HIGH;
	oc_config.OCFastMode = TIM_OCFAST_DISABLE;

	if (HAL_TIM_PWM_ConfigChannel(&htim3, &oc_config, ROBOT_LEFT_MOTOR_PWM_CHANNEL) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim3, &oc_config, ROBOT_RIGHT_MOTOR_PWM_CHANNEL) != HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim3);
}

void MX_ADC1_Init(void)
{
	ADC_ChannelConfTypeDef adc_config = {0};

	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

	if (ROBOT_USE_ALT_ENABLE_PWM_PINS == 0u) {
		if (HAL_ADC_Init(&hadc1) != HAL_OK) {
			Error_Handler();
		}

		adc_config.Channel = ADC_CHANNEL_0;
		adc_config.Rank = 1;
		adc_config.SamplingTime = ADC_SAMPLETIME_84CYCLES;

		if (HAL_ADC_ConfigChannel(&hadc1, &adc_config) != HAL_OK) {
			Error_Handler();
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	robot_uart_rx_complete_callback(huart);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (ROBOT_USE_SOFTWARE_PWM_ENABLE_PINS != 0u && htim->Instance == TIM2) {
		robot_motor_pwm_tick_callback();
	}
}

void Error_Handler(void)
{
	__disable_irq();
	while (1) {
		HAL_GPIO_TogglePin(ROBOT_STATUS_LED_GPIO_Port, ROBOT_STATUS_LED_Pin);
		HAL_Delay(100);
	}
}

static void MX_NVIC_Init(void)
{
	if (ROBOT_USE_SOFTWARE_PWM_ENABLE_PINS != 0u) {
		HAL_NVIC_SetPriority(TIM2_IRQn, 5, 0);
		HAL_NVIC_EnableIRQ(TIM2_IRQn);
	}

	HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(USART1_IRQn);
}

static void MX_MotorEnablePinsAsGpioOutputs(void)
{
	GPIO_InitTypeDef gpio_init = {0};

	HAL_TIM_PWM_Stop(&ROBOT_LEFT_MOTOR_PWM_TIMER, ROBOT_LEFT_MOTOR_PWM_CHANNEL);
	HAL_TIM_PWM_Stop(&ROBOT_RIGHT_MOTOR_PWM_TIMER, ROBOT_RIGHT_MOTOR_PWM_CHANNEL);
	HAL_GPIO_DeInit(ROBOT_LEFT_MOTOR_ENABLE_GPIO_Port, ROBOT_LEFT_MOTOR_ENABLE_Pin | ROBOT_RIGHT_MOTOR_ENABLE_Pin);

	gpio_init.Pin = ROBOT_LEFT_MOTOR_ENABLE_Pin | ROBOT_RIGHT_MOTOR_ENABLE_Pin;
	gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init.Pull = GPIO_NOPULL;
	gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(ROBOT_LEFT_MOTOR_ENABLE_GPIO_Port, &gpio_init);
}

static void MX_StartMotorPwmOutputs(void)
{
	__HAL_TIM_SET_COMPARE(&ROBOT_LEFT_MOTOR_PWM_TIMER, ROBOT_LEFT_MOTOR_PWM_CHANNEL, 0u);
	__HAL_TIM_SET_COMPARE(&ROBOT_RIGHT_MOTOR_PWM_TIMER, ROBOT_RIGHT_MOTOR_PWM_CHANNEL, 0u);

	if (HAL_TIM_Base_Start(&ROBOT_LEFT_MOTOR_PWM_TIMER) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_TIM_PWM_Start(&ROBOT_LEFT_MOTOR_PWM_TIMER, ROBOT_LEFT_MOTOR_PWM_CHANNEL) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_TIM_PWM_Start(&ROBOT_RIGHT_MOTOR_PWM_TIMER, ROBOT_RIGHT_MOTOR_PWM_CHANNEL) != HAL_OK) {
		Error_Handler();
	}
}
