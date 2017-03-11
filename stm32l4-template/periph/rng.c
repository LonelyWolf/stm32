#include "rng.h"


// Startup timeouts
#define RNG_TIMEOUT                200U    // about 2ms


// Count rough delay for timeouts
// input:
//   delay - desired delay duration
// return: value for timeout counter
static uint32_t RNG_CalcDelay(uint32_t delay) {
	uint32_t cnt;

	if (SystemCoreClock > 1000000U) {
		cnt = (delay * ((SystemCoreClock / 1000000U) + 1U));
	} else {
		cnt = (((delay / 100U) + 1U) * ((SystemCoreClock / 10000U) + 1U));
	}

	return cnt;
}

// Initialize the RNG peripheral to its initial state
void RNG_Init(void) {
	// Reset the RNG peripheral
	RCC->AHB2RSTR |=  RCC_AHB2RSTR_RNGRST;
	RCC->AHB2RSTR &= ~RCC_AHB2RSTR_RNGRST;
}

// Get a random value from RNG with ready flag wait
// return: random value or RNG_ERROR_VALUE in case of timeout
uint32_t RNG_GetRandomNumber(void) {
	volatile uint32_t wait = RNG_CalcDelay(RNG_TIMEOUT);

	while (!RNG_IsReady() && --wait);

	return (wait) ? RNG_GetValue() : RNG_ERROR_VALUE;
}
