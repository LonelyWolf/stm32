#ifndef __RNG_H
#define __RNG_H


#include <stm32l4xx.h>


// Note: the RNG peripheral is clocked by 48MHz clock, thus it should
//       be configured and enabled before using that library


// Definitions of RNG error flags
#define RNG_ERROR_SE               RNG_SR_SECS // RNG seed error flag
#define RNG_ERROR_CE               RNG_SR_CECS // RNG clock error flag

// The value indicating of RNG error
#define RNG_ERROR_VALUE            ((uint32_t)0xFFFFFFFF)


// Public functions and macros

// Enable random number generator
__STATIC_INLINE void RNG_Enable(void) {
	RNG->CR |= RNG_CR_RNGEN;
}

// Disable random number generator
__STATIC_INLINE void RNG_Disable(void) {
	RNG->CR &= ~RNG_CR_RNGEN;
}

// Returns status of DRDY bit
// return:
//   1: the RNG contains valid random data
//   0: no random data is available
__STATIC_INLINE uint32_t RNG_IsReady(void) {
	return (uint32_t)(RNG->SR & RNG_SR_DRDY);
}

// Check for RNG errors
// return: 0 in case of no errors, any combination of RNG_ERROR_xx values otherwise
__STATIC_INLINE uint32_t RNG_IsError(void) {
	return (uint32_t)(RNG->SR & (RNG_ERROR_SE | RNG_ERROR_CE));
}

// Get current value of random number generator
// return: random 32-bit value
__STATIC_INLINE uint32_t RNG_GetValue(void) {
	return (uint32_t)RNG->DR;
}


// Function prototypes
void RNG_Init(void);
uint32_t RNG_GetRandomNumber(void);

#endif // __RNG_H
