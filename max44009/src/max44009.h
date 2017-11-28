#ifndef __MAX44009_H
#define __MAX44009_H


#include "i2c.h"


// Accuracy of the Lux value calculation
//   0: reduced accuracy (slightly faster)
//   1: maximum accuracy (slightly slower)
#define MAX44009_ACCURACY                      1


// MAX44009 HAL
#define MAX44009_I2C_PORT                      I2C1 // I2C port where the MAX44009 connected

// Possible I2C device addresses
#define MAX44009_ADDR_G                        ((uint8_t)0x4A) // A0 pin tied to GND
#define MAX44009_ADDR_V                        ((uint8_t)0x4B) // A0 pin tied to VCC

// Current I2C device address (depends on the pin A0 connection)
#define MAX44009_ADDR                          (MAX44009_ADDR_G << 1)


// MAX44009 register definitions
#define MAX44009_REG_INTS                      ((uint8_t)0x00) // Interrupt status
#define MAX44009_REG_INTE                      ((uint8_t)0x01) // Interrupt enable
#define MAX44009_REG_CFG                       ((uint8_t)0x02) // Configuration
#define MAX44009_REG_LUXH                      ((uint8_t)0x03) // Lux reading high byte
#define MAX44009_REG_LUXL                      ((uint8_t)0x04) // Lux reading low byte
#define MAX44009_REG_THU                       ((uint8_t)0x05) // Upper threshold high byte
#define MAX44009_REG_THL                       ((uint8_t)0x06) // Lower threshold high byte
#define MAX44009_REG_THT                       ((uint8_t)0x07) // Threshold timer

// MAX44009 interrupt status end configuration definitions
#define MAX44009_INT_SET                       ((uint8_t)0x01) // Interrupt is asserted
#define MAX44009_INT_RESET                     ((uint8_t)0x00) // Interrupt is deasserted
#define MAX44009_INT_ENABLE                    ((uint8_t)0x01) // Interrupt enabled
#define MAX44009_INT_DISABLE                   ((uint8_t)0x00) // Interrupt disabled

// MAX44009 CFG register bits

#define MAX44009_CFG_CONT                      ((uint8_t)0x80) // Continuous mode
#define MAX44009_CFG_MANUAL                    ((uint8_t)0x40) // Manual configuration

// Current division ratio (CDR)
#define MAX44009_CDR_NORM                      ((uint8_t)0x00) // All photodiode current goes to the ADC
#define MAX44009_CDR_DIV8                      ((uint8_t)0x08) // Only 1/8 of the photodiode current goes to the ADC

// Integration time
#define MAX44009_IT_800ms                      ((uint8_t)0x00) // 800ms
#define MAX44009_IT_400ms                      ((uint8_t)0x01) // 400ms
#define MAX44009_IT_200ms                      ((uint8_t)0x02) // 200ms
#define MAX44009_IT_100ms                      ((uint8_t)0x03) // 100ms
#define MAX44009_IT_50ms                       ((uint8_t)0x04) // 50ms
#define MAX44009_IT_25ms                       ((uint8_t)0x05) // 25ms
#define MAX44009_IT_12d5ms                     ((uint8_t)0x06) // 12.5ms
#define MAX44009_IT_6d25ms                     ((uint8_t)0x07) // 6.25ms

// Bit mask for MODE and RANGE bits
#define MAX44009_MASK_CFG_MODE                 ((uint8_t)0xC0)

// Bit mask for CDR bit and integration time
#define MAX44009_MASK_CDR_IT                   ((uint8_t)0x0F)

// Value indicating overrange condition (used in return of GetLux() function)
#define MAX44009_OVERRANGE                     ((uint32_t)1000000000U)


// Function prototypes
uint8_t MAX44009_INTStatus(void);
void MAX44009_INTEnable(void);
void MAX44009_INTDisable(void);
void MAX44009_SetModeAutomatic(void);
void MAX44009_SetModeContinuous(void);
void MAX44009_SetModeManual(uint8_t cdr, uint8_t tim);
void MAX44009_SetThrU(uint8_t value);
void MAX44009_SetThrL(uint8_t value);
void MAX44009_SetThrT(uint8_t value);
void MAX44009_Init(void);
uint32_t MAX44009_GetLux(void);

#endif // __MAX44009_H
