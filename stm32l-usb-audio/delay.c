#include <stm32l1xx_rcc.h>
#include <misc.h>

#include <delay.h>


funcCallback_TypeDef delay_CallBack;


// TIM6 IRQ handler
void TIM6_IRQHandler(void) {
	DELAY_TIM->SR = 0xfffe; // Clear the TIMx's interrupt pending bit (TIM6 rises only UPDATE IT)
	if (delay_CallBack) delay_CallBack(); // Call callback function if it non NULL
}

// Init delay timer
void Delay_Init(funcCallback_TypeDef func_CallBack) {
	NVIC_InitTypeDef NVICInit;

	// Configure basic timer TIM6
	// Overflow every half second
	// One timer tick = 0,00005s = 0.05ms = 50us
	RCC->APB1ENR |= DELAY_TIM_PERIPH; // Enable the TIMx peripheral
	DELAY_TIM->CR1 |= TIM_CR1_ARPE; // Auto-preload enable
//	DELAY_TIM->PSC  = 1600; // TIMx prescaler [ PSC = APB1clock / (PWMfreq * OVFCounter) ]
	DELAY_TIM->PSC  = SystemCoreClock / 20000; // Delay timer prescaler, must be 1600
	DELAY_TIM->ARR  = 9999; // Delay timer auto reload value (20000 ticks pers second)
	DELAY_TIM->EGR  = TIM_EGR_UG; // Generate an update event to reload the prescaler value immediately
	// TIMx IRQ
	NVICInit.NVIC_IRQChannel = DELAY_TIM_IRQN;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 0x07; // middle priority
	NVICInit.NVIC_IRQChannelSubPriority = 0x07; // middle priority
	NVIC_Init(&NVICInit);

	delay_CallBack = func_CallBack;
	if (delay_CallBack) DELAY_TIM->DIER |= TIM_DIER_UIE; // Enable TIMx interrupt
	DELAY_TIM->CR1 |= TIM_CR1_CEN; // Counter enable
}

// Disable delay timer
void Delay_Disable(void) {
	RCC->APB1ENR &= ~DELAY_TIM_PERIPH; // Disable the TIMx peripheral
}

// Enable delay timer without full initialization
// note: Delay_Init() must be called before
void Delay_Enable(void) {
	RCC->APB1ENR |= DELAY_TIM_PERIPH; // Enable the TIMx peripheral
}

// Loop delay for 1 millisecond
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

// Loop delay for mSecs milliseconds
void Delay_ms(uint32_t mSecs) {
	while (mSecs > 0) {
		Delay_msec();
		mSecs--;
	}
}
