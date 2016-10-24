#ifndef __MAIN_H
#define __MAIN_H


// MCU header
#include "stm32l4xx.h"

// Internal peripherals
#include "rcc.h"
#include "pwr.h"
#include "gpio.h"
#include "usart.h"
#include "crc.h"
#include "delay.h"

// Standard includes
#include <string.h>


// Clock sources notation, just for nice debugging output
static char const * const _sysclk_src_str[] = {
		"UNKNOWN", "MSI", "HSI", "HSE", "MSI->PLL", "HSI->PLL", "HSE->PLL"
};


// Debug USART port
#define DBG_USART                  USART2

// Alias for printf, redirect output to debug USART port)
#if (USART_USE_PRINTF)
#define printf(...)                USART_printf(DBG_USART,__VA_ARGS__)
#endif


// Separator string
#define sepstr                     "--------------------\r\n"


#endif // __MAIN_H
