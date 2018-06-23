#ifndef __BMP280_H
#define __BMP280_H


#include "i2c.h"


//
//   0: using 32-bit values - lowest precision, less code)
//   1: using 64-bit values - more precision, more code
//   2: using float calculations - best precision, lots of code on MCU without FPU
#define BMP280_CALC_TYPE                1

#if (BMP280_CALC_TYPE < 0) || (BMP280_CALC_TYPE > 2)
#error "Please define correct value for BMP280_CALC_TYPE"
#endif

// Enable functions that operating with floats (calculations and return value)
// all the names ending with 'f'
//   0: not supported
//   1: supported
#define BMP280_FLOAT_FUNCTIONS          0


// BMP280 HAL
#define BMP280_I2C_PORT                 I2C1 // I2C port where the chip connected

// Possible I2C device address values
#define BMP280_ADDR_G                   ((uint8_t)0x76) // SDO pin tied to GND
#define BMP280_ADDR_V                   ((uint8_t)0x77) // SDO pin tied to VDDIO

// BMP280 address, select one of two values above according to the chip wiring
#define BMP280_ADDR                     (BMP280_ADDR_G << 1)


// BMP280 registers
#define BMP280_REG_CALIB00              ((uint8_t)0x88) // Calibration data calib00
#define BMP280_REG_CALIB25              ((uint8_t)0xA1) // Calibration data calib25
#define BMP280_REG_ID                   ((uint8_t)0xD0) // Chip ID
#define BMP280_REG_RESET                ((uint8_t)0xE0) // Software reset control register
#define BMP280_REG_STATUS               ((uint8_t)0xF3) // Device status register
#define BMP280_REG_CTRL_MEAS            ((uint8_t)0xF4) // Pressure and temperature measure control register
#define BMP280_REG_CONFIG               ((uint8_t)0xF5) // Configuration register
#define BMP280_REG_PRESS_MSB            ((uint8_t)0xF7) // Pressure readings MSB
#define BMP280_REG_PRESS_LSB            ((uint8_t)0xF8) // Pressure readings LSB
#define BMP280_REG_PRESS_XLSB           ((uint8_t)0xF9) // Pressure readings XLSB
#define BMP280_REG_TEMP_MSB             ((uint8_t)0xFA) // Temperature data MSB
#define BMP280_REG_TEMP_LSB             ((uint8_t)0xFB) // Temperature data LSB
#define BMP280_REG_TEMP_XLSB            ((uint8_t)0xFC) // Temperature data XLSB

// BMP280 register bits

// Software reset
#define BMP280_SOFT_RESET_KEY           ((uint8_t)0xB6) // Value to call a software chip reset routine

// Status register (0xF3)
#define BMP280_STATUS_MSK               ((uint8_t)0x09) // Mask to clear unused bits
#define BMP280_STATUS_IM_UPDATE         ((uint8_t)0x01) // Status register bit 0 (NVM data being copied to image registers)
#define BMP280_STATUS_MEASURING         ((uint8_t)0x08) // Status register bit 3 (conversion is running)

// Pressure and temperature control register (0xF4)
//   Temperature oversampling (osrs_t [7:5])
#define BMP280_OSRS_T_MSK               ((uint8_t)0xE0) // 'osrs_t' mask
#define BMP280_OSRS_T_SKIP              ((uint8_t)0x00) // Skipped
#define BMP280_OSRS_T_x1                ((uint8_t)0x20) // x1
#define BMP280_OSRS_T_x2                ((uint8_t)0x40) // x2
#define BMP280_OSRS_T_x4                ((uint8_t)0x60) // x4
#define BMP280_OSRS_T_x8                ((uint8_t)0x80) // x8
#define BMP280_OSRS_T_x16               ((uint8_t)0xA0) // x16
//   Pressure oversampling (osrs_p [4:2])
#define BMP280_OSRS_P_MSK               ((uint8_t)0x1C) // 'osrs_p' mask
#define BMP280_OSRS_P_SKIP              ((uint8_t)0x00) // Skipped
#define BMP280_OSRS_P_x1                ((uint8_t)0x04) // x1
#define BMP280_OSRS_P_x2                ((uint8_t)0x08) // x2
#define BMP280_OSRS_P_x4                ((uint8_t)0x0C) // x4
#define BMP280_OSRS_P_x8                ((uint8_t)0x10) // x8
#define BMP280_OSRS_P_x16               ((uint8_t)0x14) // x16
//   Power mode of the device (mode [1:0])
#define BMP280_MODE_MSK                 ((uint8_t)0x03) // 'mode' mask
#define BMP280_MODE_SLEEP               ((uint8_t)0x00) // Sleep mode
#define BMP280_MODE_FORCED              ((uint8_t)0x01) // Forced mode
#define BMP280_MODE_FORCED2             ((uint8_t)0x02) // Forced mode
#define BMP280_MODE_NORMAL              ((uint8_t)0x03) // Normal mode

// Configuration register: set rate, filter and interface options (0xF5)
//   Inactive duration in normal mode (t_sb [7:5])
#define BMP280_STBY_MSK                 ((uint8_t)0xE0) // 't_sb' mask
#define BMP280_STBY_0p5ms               ((uint8_t)0x00) // 0.5ms
#define BMP280_STBY_62p5ms              ((uint8_t)0x20) // 62.5ms
#define BMP280_STBY_125ms               ((uint8_t)0x40) // 125ms
#define BMP280_STBY_250ms               ((uint8_t)0x60) // 250ms
#define BMP280_STBY_500ms               ((uint8_t)0x80) // 500ms
#define BMP280_STBY_1s                  ((uint8_t)0xA0) // 1s
#define BMP280_STBY_2s                  ((uint8_t)0xC0) // 2s
#define BMP280_STBY_4s                  ((uint8_t)0xE0) // 4s
//   Time constant of the IIR filter (filter [4:2])
#define BMP280_FILTER_MSK               ((uint8_t)0x1C) // 'filter' mask
#define BMP280_FILTER_OFF               ((uint8_t)0x00) // Off
#define BMP280_FILTER_2                 ((uint8_t)0x04) // 2
#define BMP280_FILTER_4                 ((uint8_t)0x08) // 4
#define BMP280_FILTER_8                 ((uint8_t)0x0C) // 8
#define BMP280_FILTER_16                ((uint8_t)0x10) // 16

// Constant for Pascals to millimeters of mercury conversion
#define BMP280_MMHG_Q0_22               ((uint32_t)31460U) // 0.00750061683 in Q0.22 format

// Definitions of values for absent readings
#define BMP280_NO_TEMPERATURE           ((int32_t)0x80000)
#define BMP280_NO_PRESSURE              ((int32_t)0x80000)


// BMP280 function result
typedef enum {
	BMP280_ERROR   = 0,
	BMP280_SUCCESS = !BMP280_ERROR
} BMP280_RESULT;

// Structurer for compensation parameters
struct BMP280_cal_param_t {
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;
	uint16_t dig_P1;
	int16_t  dig_P2;
	int16_t  dig_P3;
	int16_t  dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;
};


// Function prototypes
BMP280_RESULT BMP280_Check(void);
void BMP280_Reset(void);

uint8_t BMP280_GetVersion(void);
uint8_t BMP280_GetStatus(void);
uint8_t BMP280_GetMode(void);

void BMP280_SetMode(uint8_t mode);
void BMP280_SetFilter(uint8_t filter);
void BMP280_SetStandby(uint8_t tsb);
void BMP280_SetOSRST(uint8_t osrs);
void BMP280_SetOSRSP(uint8_t osrs);

BMP280_RESULT BMP280_Read_Calibration(void);
BMP280_RESULT BMP280_Read_UP(int32_t *UP);
BMP280_RESULT BMP280_Read_UT(int32_t *UT);
BMP280_RESULT BMP280_Read_UTP(int32_t *UT, int32_t *UP);

int32_t  BMP280_CalcT(int32_t UT);
uint32_t BMP280_CalcP(int32_t UP);

uint32_t BMP280_Pa_to_mmHg(uint32_t p_pa);

#if (BMP280_FLOAT_FUNCTIONS)
float BMP280_CalcTf(int32_t UT);
float BMP280_CalcPf(uint32_t UP);
float BMP280_Pa_to_mmHgf(float p_pa);
#endif // BMP280_FLOAT_FUNCTIONS

#endif // __BMP280_H
