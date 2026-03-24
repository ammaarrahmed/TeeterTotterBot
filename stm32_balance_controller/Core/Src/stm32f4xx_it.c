#include "main.h"

void NMI_Handler(void)
{
	while (1) {
	}
}

void HardFault_Handler(void)
{
	while (1) {
	}
}

void MemManage_Handler(void)
{
	while (1) {
	}
}

void BusFault_Handler(void)
{
	while (1) {
	}
}

void UsageFault_Handler(void)
{
	while (1) {
	}
}

void DebugMon_Handler(void)
{
}

void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim2);
}

void USART1_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart1);
}
