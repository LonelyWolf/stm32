#ifndef __MAIN_H
#define __MAIN_H


// Define to any non-zero value when a HSE is present
#define DEVICE_HSE_PRESENT         0


// Internal peripherals
#include "rcc.h"
#include "pwr.h"
#include "gpio.h"
#include "usart.h"
#include "exti.h"
#include "spi.h"
#include "i2c.h"
#include "rtc.h"
#include "dma.h"
#include "crc.h"
#include "rng.h"
#include "delay.h"


// Debug USART port
#define DBG_USART                  USART2

// Alias for printf, redirect output to debug USART port)
#if (USART_USE_PRINTF)
#define printf(...)                USART_printf(DBG_USART, __VA_ARGS__)
#endif


// Separator string
#define sepstr                     "--------------------\r\n"


#endif // __MAIN_H
