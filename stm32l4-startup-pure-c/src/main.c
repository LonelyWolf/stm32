#include "main.h"

// Test IRQ handler
void TIM7_IRQHandler(void) {
	TIM7->SR = ~TIM_SR_UIF;
	GPIO_PIN_INVERT(GPIOA, GPIO_PIN_5);
}

int main(void) {
	// Initialize the PA5 pin (LED on the Nucleo board)
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	GPIO_set_mode(GPIOA, GPIO_Mode_OUT, GPIO_PUPD_NONE, GPIO_PIN_5);
	GPIO_out_cfg(GPIOA, GPIO_OT_PP, GPIO_SPD_LOW, GPIO_PIN_5);

	// Initialize TIM7 (triggers twice per second)
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM7EN;
	TIM7->CR1 = TIM_CR1_ARPE;
	TIM7->ARR = 999U;
	TIM7->PSC = SystemCoreClock / ((TIM7->ARR + 1U) << 1);
	TIM7->DIER |= TIM_DIER_UIE;
	TIM7->EGR = TIM_EGR_UG;
	TIM7->SR = ~TIM_SR_UIF;
	TIM7->CR1 |= TIM_CR1_CEN;
	NVIC_EnableIRQ(TIM7_IRQn);
	NVIC_SetPriority(TIM7_IRQn, 14U);

	// The main loop
	while (1) {
		PWR_EnterSLEEPMode(PWR_SENTRY_WFI);
	}
}
