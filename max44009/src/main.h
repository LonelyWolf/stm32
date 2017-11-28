#ifndef __MAIN_H
#define __MAIN_H

// MCU header
#include "stm32l4xx.h"

// Internal peripherals
#include "rcc.h"
#include "gpio.h"
#include "delay.h"
#include "usart.h"
#include "i2c.h"

// External peripherals
#include "max44009.h"


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


// Variables

// Common variables
uint32_t i,j,k;

// Lux readings
uint32_t als_lux;

#endif // __MAIN_H
