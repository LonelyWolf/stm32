// Define to prevent recursive inclusion -------------------------------------
#ifndef __DELAY_H
#define __DELAY_H


#define DELAY_TICK_FREQUENCY_US 1000000   /* = 1MHz -> microseconds delay */
#define DELAY_TICK_FREQUENCY_MS 1000      /* = 1kHz -> milliseconds delay */


static __IO uint32_t TimingDelay; // __IO -- volatile


// Function prototypes
void Delay_ms(uint32_t nTime);
void Delay_us(uint32_t nTime);

#endif // __DELAY_H
