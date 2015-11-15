#ifndef __DELAY_H
#define __DELAY_H


#include <stm32l1xx.h>


// Function prototypes
void Delay_Init(void);
void Delay_ms(uint32_t ms);
void DelaySleep_ms(uint32_t ms);

#endif // __DELAY_H
