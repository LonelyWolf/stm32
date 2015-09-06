#ifndef __DELAY_H
#define __DELAY_H


#include <stm32l1xx.h>


// Delay timer HAL
#define DELAY_TIM               TIM6
#define DELAY_TIM_IRQN          TIM6_IRQn
#define DELAY_TIM_APB           RCC->APB1ENR
#define DELAY_TIM_PERIPH        RCC_APB1ENR_TIM6EN

// NULL declaration
#ifndef NULL
#define NULL  ((void *)0)
#endif


// Callback function prototype
typedef void (*funcCallback_TypeDef)(void);


// Function prototypes
void Delay_Init(funcCallback_TypeDef func_CallBack);
void Delay_Disable(void);
void Delay_Enable(void);

void Delay_ms(uint32_t nTime);

#endif // __DELAY_H
