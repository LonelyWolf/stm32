#ifndef __BME280_H
#define __BME280_H


// Support high precision calculations using floats
//   0 - not supported
//   1 - supported
#define BME280_USE_FLOAT                0

// 64-bit integer calculations for pressure compensation (more precise than 32-bit)
//   0 - use 32-bit pressure compensation (less code, less precision)
//   1 - use 64-bit pressure compensation (more code, more precision)
#define BME280_USE_INT64                1


// BME280 HAL
#define BME280_I2C_PORT                 I2C2 // I2C port where the BME280 connected

// BME280 I2C related

// All possible I2C device address values
#define BME280_ADDR_G                   (uint8_t)0x76 // I2C address when SDO connected to GND
#define BME280_ADDR_V                   (uint8_t)0x77 // I2C address when SDO connected to VDDIO

// BME280 address
#define BME280_ADDR                     (BME280_ADDR_V << 1)

// BME280 registers
#define BME280_REG_CALIB00              (uint8_t)0x88 // Calibration data calib00
#define BME280_REG_CALIB25              (uint8_t)0xA1 // Calibration data calib25
#define BME280_REG_ID                   (uint8_t)0xD0 // Chip ID
#define BME280_REG_RESET                (uint8_t)0xE0 // Software reset control register
#define BME280_REG_CALIB26              (uint8_t)0xE1 // Calibration data calib26
#define BME280_REG_CTRL_HUM             (uint8_t)0xF2 // Humidity measure control register
#define BME280_REG_STATUS               (uint8_t)0xF3 // Device status register
#define BME280_REG_CTRL_MEAS            (uint8_t)0xF4 // Pressure and temperature measure control register
#define BME280_REG_CONFIG               (uint8_t)0xF5 // Configuration register
#define BME280_REG_PRESS_MSB            (uint8_t)0xF7 // Pressure readings MSB
#define BME280_REG_PRESS_LSB            (uint8_t)0xF8 // Pressure readings LSB
#define BME280_REG_PRESS_XLSB           (uint8_t)0xF9 // Pressure readings XLSB
#define BME280_REG_TEMP_MSB             (uint8_t)0xFA // Temperature data MSB
#define BME280_REG_TEMP_LSB             (uint8_t)0xFB // Temperature data LSB
#define BME280_REG_TEMP_XLSB            (uint8_t)0xFC // Temperature data XLSB
#define BME280_REG_HUM_MSB              (uint8_t)0xFD // Humidity data MSB
#define BME280_REG_HUM_LSB              (uint8_t)0xFE // Humidity data LSB

// BME280 register bits

// Software reset
#define BME280_SOFT_RESET_KEY           (uint8_t)0xB6

// Humidity oversampling control register (0xF2)
#define BME280_OSRS_H_MSK               (uint8_t)0x07 // 'osrs_h' mask
#define BME280_OSRS_H_SKIP              (uint8_t)0x00 // Skipped
#define BME280_OSRS_H_x1                (uint8_t)0x01 // x1
#define BME280_OSRS_H_x2                (uint8_t)0x02 // x2
#define BME280_OSRS_H_x4                (uint8_t)0x03 // x4
#define BME280_OSRS_H_x8                (uint8_t)0x04 // x8
#define BME280_OSRS_H_x16               (uint8_t)0x05 // x16

// Status register (0xF3)
#define BME280_STATUS_MSK               (uint8_t)0x09 // Mask to clear unused bits
#define BME280_STATUS_MEASURING         (uint8_t)0x08 // Status register bit 3 (conversion is running)
#define BME280_STATUS_IM_UPDATE         (uint8_t)0x01 // Status register bit 0 (NVM data being copied to image registers)

// Pressure and temperature control register (0xF4)
//   Temperature oversampling (osrs_t [7:5])
#define BME280_OSRS_T_MSK               (uint8_t)0xE0 // 'osrs_t' mask
#define BME280_OSRS_T_SKIP              (uint8_t)0x00 // Skipped
#define BME280_OSRS_T_x1                (uint8_t)0x20 // x1
#define BME280_OSRS_T_x2                (uint8_t)0x40 // x2
#define BME280_OSRS_T_x4                (uint8_t)0x60 // x4
#define BME280_OSRS_T_x8                (uint8_t)0x80 // x8
#define BME280_OSRS_T_x16               (uint8_t)0xA0 // x16
//   Pressure oversampling (osrs_p [4:2])
#define BME280_OSRS_P_MSK               (uint8_t)0x1C // 'osrs_p' mask
#define BME280_OSRS_P_SKIP              (uint8_t)0x00 // Skipped
#define BME280_OSRS_P_x1                (uint8_t)0x04 // x1
#define BME280_OSRS_P_x2                (uint8_t)0x08 // x2
#define BME280_OSRS_P_x4                (uint8_t)0x0C // x4
#define BME280_OSRS_P_x8                (uint8_t)0x10 // x8
#define BME280_OSRS_P_x16               (uint8_t)0x14 // x16
//   Sensor mode of the device (mode [1:0])
#define BME280_MODE_MSK                 (uint8_t)0x03 // 'mode' mask
#define BME280_MODE_SLEEP               (uint8_t)0x00 // Sleep mode
#define BME280_MODE_FORCED              (uint8_t)0x01 // Forced mode
#define BME280_MODE_NORMAL              (uint8_t)0x03 // Normal mode

// Configuration register: set rate, filter and interface options (0xF5)
//   Inactive duration in normal mode (t_sb [7:5])
#define BME280_STBY_MSK                 (uint8_t)0xE0 // 't_sb' mask
#define BME280_STBY_0p5ms               (uint8_t)0x00 // 0.5ms
#define BME280_STBY_62p5ms              (uint8_t)0x20 // 62.5ms
#define BME280_STBY_125ms               (uint8_t)0x40 // 125ms
#define BME280_STBY_250ms               (uint8_t)0x60 // 250ms
#define BME280_STBY_500ms               (uint8_t)0x80 // 500ms
#define BME280_STBY_1s                  (uint8_t)0xA0 // 1s
#define BME280_STBY_10ms                (uint8_t)0xC0 // 10ms
#define BME280_STBY_20ms                (uint8_t)0xE0 // 20ms
//   Time constant of the IIR filter (filter [4:2])
#define BME280_FILTER_MSK               (uint8_t)0x1C // 'filter' mask
#define BME280_FILTER_OFF               (uint8_t)0x00 // Off
#define BME280_FILTER_2                 (uint8_t)0x04 // 2
#define BME280_FILTER_4                 (uint8_t)0x08 // 4
#define BME280_FILTER_8                 (uint8_t)0x0C // 8
#define BME280_FILTER_16                (uint8_t)0x10 // 16

// Constant for Pascals to millimeters of mercury conversion
#define BME_MMHG_Q0_20                  (uint32_t)7865; // 0.00750061683 in Q0.20 format


// BME280 function result
typedef enum {
	BME280_ERROR   = 0,
	BME280_SUCCESS = !BME280_ERROR
} BME280_RESULT;

// Compensation parameters structure
typedef struct {
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
	uint8_t  dig_H1;
	int16_t  dig_H2;
	uint8_t  dig_H3;
	int16_t  dig_H4;
	int16_t  dig_H5;
	int8_t   dig_H6;
} BME280_Compensation_TypeDef;


// Compensation parameters storage
BME280_Compensation_TypeDef cal_param;


// Function prototypes
void BME280_WriteReg(uint8_t reg, uint8_t value);
uint8_t BME280_ReadReg(uint8_t reg);

BME280_RESULT BME280_Check(void);
inline void BME280_Reset(void);

inline uint8_t BME280_GetVersion(void);
inline uint8_t BME280_GetStatus(void);
inline uint8_t BME280_GetMode(void);

void BME280_SetMode(uint8_t mode);
void BME280_SetFilter(uint8_t filter);
void BME280_SetStandby(uint8_t tsb);
void BME280_SetOSRST(uint8_t osrs);
void BME280_SetOSRSP(uint8_t osrs);
void BME280_SetOSRSH(uint8_t osrs);

BME280_RESULT BME280_Read_Calibration(void);
BME280_RESULT BME280_Read_UP(int32_t *UP);
BME280_RESULT BME280_Read_UT(int32_t *UT);
BME280_RESULT BME280_Read_UH(int32_t *UH);
BME280_RESULT BME280_Read_UTPH(int32_t *UT, int32_t *UP, int32_t *UH);

int32_t  BME280_CalcT(int32_t UT);
uint32_t BME280_CalcP(int32_t UP);
uint32_t BME280_CalcH(int32_t UH);

uint32_t BME280_Pa_to_mmHg(uint32_t PQ24_8);
int32_t BME280_Pa_to_Alt(uint32_t P);

#if (BME280_USE_FLOAT)
float BME280_CalcTf(int32_t UT);
float BME280_CalcPf(uint32_t UP);
float BME280_CalcHf(uint32_t UH);
#endif // BME280_USE_FLOAT

#endif // __BME280_H
