// Delay functions using timer


#include "delay.h"


funcCallback_TypeDef delay_CallBack;


// TIM6 IRQ handler
void TIM6_IRQHandler(void) {
	DELAY_TIM->SR = 0; // Clear the TIMx's interrupt pending bit (TIM6 rises only UPDATE IT)
	if (delay_CallBack) delay_CallBack(); // Call callback function if it non NULL
}

// Initialize delay timer
// input:
//   func_CallBack - callback function which will be called on every timer counter overflow
//                   or NULL value if no callback needed
void Delay_Init(funcCallback_TypeDef func_CallBack) {
	// Configure timer counter to overflow every half second

	// One timer tick = 0.00005s = 0.05ms = 50us
	DELAY_TIM_APB  |= DELAY_TIM_PERIPH; // Enable the TIMx peripheral
	DELAY_TIM->CR1 |= TIM_CR1_ARPE; // Auto-preload enable
	DELAY_TIM->PSC  = SystemCoreClock / 20000; // Delay timer prescaler, must be 1600
	DELAY_TIM->ARR  = 9999; // Delay timer auto reload value (20000 ticks per second)
	DELAY_TIM->EGR  = TIM_EGR_UG; // Generate an update event to reload the prescaler value immediately
	DELAY_TIM->SR   = 0; // Clear timer flags

	// Set callback function and enable interrupt if it not NULL
	delay_CallBack = func_CallBack;
	if (delay_CallBack) {
		DELAY_TIM->DIER |= TIM_DIER_UIE;
		NVIC_EnableIRQ(DELAY_TIM_IRQN);
	}

	// Timer counter enable
	DELAY_TIM->CR1 |= TIM_CR1_CEN;
}

// Disable delay timer
void Delay_Disable(void) {
	DELAY_TIM_APB &= ~DELAY_TIM_PERIPH; // Disable the TIMx peripheral
}

// Enable delay timer (without full initialization)
// note: delay TIM peripheral must be already configured (by Delay_Init)
void Delay_Enable(void) {
	DELAY_TIM_APB |= DELAY_TIM_PERIPH; // Enable the TIMx peripheral
}

// Loop delay for about one millisecond
void Delay_msec(void) {
	volatile uint16_t tStart;
	volatile uint16_t tEnd;
	volatile uint16_t tDiff;

	tStart = DELAY_TIM->CNT;
	do {
		tEnd = DELAY_TIM->CNT;
		if (tEnd < tStart) tDiff = 9999 - tStart + tEnd; else tDiff = tEnd - tStart;
	} while (tDiff < 19);
}

// Milliseconds loop delay
// input:
//   mSecs - number of milliseconds
void Delay_ms(volatile uint32_t mSecs) {
	while (mSecs--) Delay_msec();
}
