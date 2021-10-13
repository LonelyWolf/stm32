#ifndef __TMP116_H
#define __TMP116_H


// NOTE: before calling any function what writes data to the sensor EEPROM,
//       user software must ensure that the EEPROM busy flag is cleared

// NOTE: before calling any function what writes data to the sensor EEPROM,
//       user software should not work with sensor while the EEPROM busy flag is set


#include "i2c.h"


// TMP116 HAL
#define TMP116_I2C_PORT                      I2C3 // I2C port where the TMP116 connected

// Possible I2C device addresses
#define TMP116_ADDR_GND                      ((uint8_t)0x48) // ADD0 pin pulled to GND (1001000)
#define TMP116_ADDR_VCC                      ((uint8_t)0x49) // ADD0 pin pulled to VCC (1001001)
#define TMP116_ADDR_SDA                      ((uint8_t)0x4A) // ADD0 pin pulled to SDA (1001010)
#define TMP116_ADDR_SCL                      ((uint8_t)0x4B) // ADD0 pin pulled to SCL (1001011)

// Current I2C device address (depends on the pin ADD0 connection, should be one of TMP116_ADDR_xx)
#define TMP116_ADDR                          ((uint8_t)(TMP116_ADDR_GND << 1))

// Define nonzero value to compile chip reset function
#define TMP116_RESET                         1
// NOTE: this feature initiates I2C general call software reset sequence by transmitting
//       special data byte 0x06 (0b00000110) to the reserved I2C address 0x00
//       before using, make sure that this will not interfere with other devices on the same bus,
//       which also can respond to a general-call command


// Temperature conversion modes
typedef enum {
	TMP116_MODE_CC = 0U, // Continuous conversion (CC)
	TMP116_MODE_OS = 3U, // One-shot conversion (OS)
	TMP116_MODE_SD = 1U  // Shutdown (SD)
} TMP116_MODE_t;

// Number of averaged samples
typedef enum {
	TMP116_AVG_1  = 0U, // 1
	TMP116_AVG_8  = 1U, // 8
	TMP116_AVG_32 = 2U, // 32
	TMP116_AVG_64 = 3U  // 64
} TMP116_AVG_t;

// Conversion cycle time in CC mode
typedef enum {
	TMP116_CONV_0 = 0U, // 15.5ms
	TMP116_CONV_1 = 1U, // 125ms
	TMP116_CONV_2 = 2U, // 250ms
	TMP116_CONV_3 = 3U, // 500ms
	TMP116_CONV_4 = 4U, // 1s
	TMP116_CONV_5 = 5U, // 4s
	TMP116_CONV_6 = 6U, // 8s
	TMP116_CONV_7 = 7U  // 16s
} TMP116_CONV_t;

// Number of the EEPROM register
typedef enum {
	TMP116_EEPROM_1 = 1U, // #1
	TMP116_EEPROM_2 = 2U, // #2
	TMP116_EEPROM_3 = 3U, // #3
	TMP116_EEPROM_4 = 4U  // #4
} TMP116_EEPROM_t;

// Set or reset bit
typedef enum {
	TMP116_BIT_SET = 1U,
	TMP116_BIT_RESET = 0U
} TMP116_BIT_t;


// Device ID
#define TMP116_DEVICE_ID                     ((uint16_t)0x1116)

// Value indicating no temperature readings is present
#define TMP116_NO_TEMPERATURE                ((uint16_t)0x8000)


// Function prototypes
#if (TMP116_RESET)
void TMP116_SoftReset(void);
#endif // TMP116_RESET

void TMP116_Init(void);

void TMP116_SetMode(TMP116_MODE_t mode);
void TMP116_SetAvg(TMP116_AVG_t avg);
void TMP116_SetConv(TMP116_CONV_t conv);
void TMP116_SetConfig(TMP116_MODE_t mode, TMP116_AVG_t avg, TMP116_CONV_t conv);
void TMP116_SetModeTA(TMP116_BIT_t bit);
void TMP116_SetAlertPin(TMP116_BIT_t state);
void TMP116_SetAlertPolarity(TMP116_BIT_t state);
void TMP116_SetLimitHigh(int16_t limit);
void TMP116_SetLimitLow(int16_t limit);

uint16_t TMP116_GetStatus(void);
uint16_t TMP116_GetID(void);
int16_t TMP116_GetTemp(void);
int16_t TMP116_GetLimitHigh(void);
int16_t TMP116_GetLimitLow(void);

uint16_t TMP116_IsDataReady(void);
uint16_t TMP116_IsAlertHigh(void);
uint16_t TMP116_IsAlertLow(void);
uint16_t TMP116_IsEEPROMBusy(void);

void TMP116_EEPROMUnlock(void);
void TMP116_EEPROMLock(void);
uint16_t TMP116_EEPROMRead(TMP116_EEPROM_t reg);
void TMP116_EEPROMWrite(TMP116_EEPROM_t reg, uint16_t value);

void TMP116_WriteConfig(void);
void TMP116_WriteLimitHigh(void);
void TMP116_WriteLimitLow(void);

#endif // __TMP116_H
