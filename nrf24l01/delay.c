// Delay functions using SysTick timer
// Minimal delay length is 1ms


#include "delay.h"


// Initialize delay (configure SysTick counter)
// note: must be called each time when the system core clock has been changed
void Delay_Init(void) {
    // Set reload register to generate IRQ every millisecond
    SysTick->LOAD = (uint32_t)((SystemCoreClock / 1000UL) - 1UL);

    // Set priority for SysTick IRQ
    NVIC_SetPriority(SysTick_IRQn,(1UL << __NVIC_PRIO_BITS) - 1UL);

    // Set the SysTick counter value
    SysTick->VAL = 0UL;

    // Configure SysTick source and enable counter
    SysTick->CTRL = (SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk);
}

#if (!DELAY_INLINE)
// Do a delay for a specified number of milliseconds
// input:
//   ms - number of milliseconds to wait
void Delay_ms(uint32_t ms) {
    __IO uint32_t delay_counter = ms;

    while (delay_counter) {
        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
            delay_counter--;
        }
    }
}
#endif
