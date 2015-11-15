// Delay functions using SysTick timer
// Minimal delay length is 1ms


#include "delay.h"


// Delay counter
static __IO uint32_t DelayCounter;


// SysTick interrupt handler
void SysTick_Handler(void) {
	DelayCounter++;
}

// Initialize delay functions
// note: configures the SysTick counter, must be called each time when the core
//       clock has been changed
void Delay_Init(void) {
	// Set reload register to generate IRQ every millisecond
	SysTick->LOAD = (uint32_t)(SystemCoreClock / (1000UL - 1UL));

	// Set priority for SysTick IRQ
	NVIC_SetPriority(SysTick_IRQn,(1 << __NVIC_PRIO_BITS) - 1);

	// Set the SysTick counter value
	SysTick->VAL = 0UL;

	// Set SysTick source and IRQ
	SysTick->CTRL = (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk);
}

// Do a delay for a specified number of milliseconds
// input:
//   ms - number of milliseconds to wait
void Delay_ms(uint32_t ms) {
	// Enable the SysTick timer
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

	// Wait for a specified number of milliseconds
	DelayCounter = 0;
	while (DelayCounter < ms);

	// Disable the SysTick timer
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

// Do a delay for a specified number of milliseconds, MCU in sleep mode
// input:
//   ms - number of milliseconds to wait
// note: all wake-up flags must be cleared before calling this function
//       to ensure proper sleep during delay
void DelaySleep_ms(uint32_t ms) {
	// Enable the SysTick timer
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

	// Set delay counter
	DelayCounter = 0;

	// Wait for a specified number of milliseconds
	while (DelayCounter < ms) {
		__WFI();
	}

	// Disable the SysTick timer
	SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}
