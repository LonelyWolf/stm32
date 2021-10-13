#ifndef __MAIN_H
#define __MAIN_H


// Define to any non-zero value when a HSE is present
#define DEVICE_HSE_PRESENT         1


// Internal peripherals
#include "rcc.h"
#include "pwr.h"
#include "gpio.h"
#include "usart.h"
#include "spi.h"
#include "i2c.h"
#include "delay.h"

#include "tmp116.h"


// Debug USART port
#define DBG_USART                  USART1

// Alias for printf, redirect output to debug USART port)
#if (USART_USE_PRINTF)
#define printf(...)                USART_printf(DBG_USART, __VA_ARGS__)
#endif

#endif // __MAIN_H
