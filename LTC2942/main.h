#ifndef __MAIN_H
#define __MAIN_H

#include "stm32l1xx.h"

#include "gpio.h"
#include "i2c.h"
#include "uart.h"

#include "ltc2942.h"




// Define an alias for printf
#if (USART_USE_PRINTF)
#define printf(...) USART_printf(USART1,__VA_ARGS__)
#endif




// Separator string
#define sepstr           "--------------------\r\n"




// Variables
uint32_t i;
uint32_t j;
uint32_t k;

#endif // __MAIN_H
