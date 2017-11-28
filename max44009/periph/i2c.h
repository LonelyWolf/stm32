#ifndef __I2C_H
#define __I2C_H


#include <stm32l4xx.h>
#include "rcc.h"
#include "gpio.h"


// I2C HAL

// I2C1
// SCL [PB6, PB8]
#define I2C1_SCL_GPIO_PERIPH    RCC_AHB2ENR_GPIOBEN
#define I2C1_SCL_GPIO_PORT      GPIOB
#define I2C1_SCL_GPIO_PIN       GPIO_PIN_8
#define I2C1_SCL_GPIO_SRC       GPIO_PinSource8
// SDA [PB7, PB9]
#define I2C1_SDA_GPIO_PERIPH    RCC_AHB2ENR_GPIOBEN
#define I2C1_SDA_GPIO_PORT      GPIOB
#define I2C1_SDA_GPIO_PIN       GPIO_PIN_9
#define I2C1_SDA_GPIO_SRC       GPIO_PinSource9

// I2C2
// SCL [PB10, PB13]
#define I2C2_SCL_GPIO_PERIPH    RCC_AHB2ENR_GPIOBEN
#define I2C2_SCL_GPIO_PORT      GPIOB
#define I2C2_SCL_GPIO_PIN       GPIO_PIN_10
#define I2C2_SCL_GPIO_SRC       GPIO_PinSource10
// SDA [PB11, PB14]
#define I2C2_SDA_GPIO_PERIPH    RCC_AHB2ENR_GPIOBEN
#define I2C2_SDA_GPIO_PORT      GPIOB
#define I2C2_SDA_GPIO_PIN       GPIO_PIN_11
#define I2C2_SDA_GPIO_SRC       GPIO_PinSource11

// I2C3
// SCL [PC0]
#define I2C3_SCL_GPIO_PERIPH    RCC_AHB2ENR_GPIOCEN
#define I2C3_SCL_GPIO_PORT      GPIOC
#define I2C3_SCL_GPIO_PIN       GPIO_PIN_0
#define I2C3_SCL_GPIO_SRC       GPIO_PinSource0
// SDA [PC1]
#define I2C3_SDA_GPIO_PERIPH    RCC_AHB2ENR_GPIOCEN
#define I2C3_SDA_GPIO_PORT      GPIOC
#define I2C3_SDA_GPIO_PIN       GPIO_PIN_1
#define I2C3_SDA_GPIO_SRC       GPIO_PinSource1

// Definitions of I2C analog filter state
#define I2C_AF_ENABLE          ((uint32_t)0x00000000U) // Analog filter is enabled
#define I2C_AF_DISABLE         I2C_CR1_ANFOFF          // Analog filter is disabled

// Definitions for STOP generation after transfer
#define I2C_GENSTOP_YES        I2C_CR2_AUTOEND         // Generate STOP after transfer
#define I2C_GENSTOP_NO         ((uint32_t)0x00000000U) // Don't generate STOP after transfer


// Result of I2C procedures
typedef enum {
	I2C_ERROR   = 0,
	I2C_SUCCESS = !I2C_ERROR
} I2CSTATUS;


// Public functions and macros

// Enable I2C peripheral
// input:
//   I2Cx - pointer to the I2C peripheral (I2C1, etc.)
__STATIC_INLINE void I2C_Enable(I2C_TypeDef* I2Cx) {
	I2Cx->CR1 |= I2C_CR1_PE;
}

// Disable I2C peripheral
// input:
//   I2Cx - pointer to the I2C peripheral (I2C1, etc.)
__STATIC_INLINE void I2C_Disable(I2C_TypeDef* I2Cx) {
	I2Cx->CR1 &= ~I2C_CR1_PE;
}

// Clear ADDR flag
// input:
//   I2Cx - pointer to the I2C peripheral (I2C1, etc.)
__STATIC_INLINE void I2C_ClearFlag_ADDR(I2C_TypeDef* I2Cx) {
	I2Cx->ICR |= I2C_ICR_ADDRCF;
}

// Configure I2C noise filters
// input:
//   I2Cx - pointer to the I2C peripheral (I2C1, etc.)
//   af - analog filter state, I2C_AF_DISABLE or I2C_AF_ENABLE
//   df - digital filter configuration, can be a value in range from 0 to 15
//        zero value means the digital filter is disabled
//        this values means filtering capability up to (df * ti2cclk)
// note: must be called only when I2C is disabled (PE bit in I2C_CR1 register is reset)
__STATIC_INLINE void I2C_ConfigFilters(I2C_TypeDef* I2Cx, uint32_t af, uint32_t df) {
	I2Cx->CR1 &= ~(I2C_CR1_ANFOFF | I2C_CR1_DNF);
	I2Cx->CR1 |= (af & I2C_CR1_ANFOFF) | ((df << 8) & I2C_CR1_DNF);
}

// Configure the I2C timings (SDA setup/hold time and SCL high/low period)
// input:
//   I2Cx - pointer to the I2C peripheral (I2C1, etc.)
//   timing - the value for I2C_TIMINGR register
// note: must be called only when I2C is disabled (PE bit in I2C_CR1 register is reset)
__STATIC_INLINE void I2C_ConfigTiming(I2C_TypeDef* I2Cx, uint32_t timing) {
	I2Cx->TIMINGR = timing;
}

// Generate START condition
// input:
//   I2Cx - pointer to the I2C peripheral (I2C1, etc.)
//   addr - I2C device address
// note: 7-bit addressing mode
__STATIC_INLINE void I2C_GenStart(I2C_TypeDef* I2Cx, uint32_t addr) {
	I2Cx->CR2 = (addr & I2C_CR2_SADD) | I2C_CR2_START | I2C_CR2_AUTOEND;
}

// Generate STOP condition
// input:
//   I2Cx - pointer to the I2C peripheral (I2C1, etc.)
__STATIC_INLINE void I2C_GenStop(I2C_TypeDef* I2Cx) {
	I2Cx->CR2 |= I2C_CR2_STOP;
}


// Function prototypes
void I2C_Init(I2C_TypeDef* I2Cx);
I2CSTATUS I2C_IsDeviceReady(I2C_TypeDef* I2Cx, uint8_t devAddr, uint32_t Trials);
I2CSTATUS I2C_Transmit(I2C_TypeDef* I2Cx, const uint8_t *pBuf, uint32_t nbytes, uint8_t devAddr, uint32_t stop);
I2CSTATUS I2C_Receive(I2C_TypeDef* I2Cx, uint8_t *pBuf, uint32_t nbytes, uint8_t devAddr);

#endif // __I2C_H
