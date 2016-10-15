#ifndef __DELAY_H
#define __DELAY_H


#include <stm32l4xx.h>


// Public functions and macros

// Do a delay for a specified number of milliseconds
// input:
//   delay_counter - number of milliseconds to wait
__STATIC_INLINE void Delay_ms(__IO uint32_t delay_counter) {
	while (delay_counter) {
		if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
			delay_counter--;
		}
	}
}


// Function prototypes
void Delay_Init(void);

#endif // __DELAY_H
