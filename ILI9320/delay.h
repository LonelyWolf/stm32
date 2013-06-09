#define DELAY_TIM_FREQUENCY_US 1000000		/* = 1MHZ -> timer runs in microseconds */
#define DELAY_TIM_FREQUENCY_MS 1000			/* = 1kHZ -> timer runs in milliseconds */


/*
 *   Declare Functions
 */
extern void Delay_ms(uint32_t nTime);
extern void Delay_us(uint32_t nTime);
