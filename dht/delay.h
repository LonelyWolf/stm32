#define DELAY_TICK_FREQUENCY_US 1000000   /* = 1MHZ -> microseconds delay */
#define DELAY_TICK_FREQUENCY_MS 1000      /* = 1kHZ -> milliseconds delay */

#define SysTick_CLKSource_HCLK         ((uint32_t)0x00000004)  /* from misc.h */

void dTimerInit(void);
void dTimerReset(void);
uint32_t dTimerGet_ms(void);
uint32_t dTimerGet_us(void);
void Delay_ms(uint32_t nTime);
void Delay_us(uint32_t nTime);
