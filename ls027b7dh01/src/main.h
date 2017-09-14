#ifndef __MAIN_H
#define __MAIN_H


// MCU header
#include "stm32l4xx.h"

// Internal peripherals
#include "rcc.h"
#include "pwr.h"
#include "gpio.h"
#include "spi.h"
#include "usart.h"
#include "delay.h"
#include "rng.h"

// External peripherals
#include "ls027b7dh01.h"

// Graphical resources
#include "font5x7.h"
#include "font7x10.h"
#include "digits5x9.h"
#include "digits8x16.h"
#include "bitmaps.h"
#include "font_digits.h"


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
