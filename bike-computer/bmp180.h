// Define to prevent recursive inclusion -------------------------------------
#ifndef __BMP180_H
#define __BMP180_H


// BMP180 HAL
#define BMP180_I2C_PORT                 I2C2 // I2C port where the BMP180 connected

// BMP180 I2C related
#define BMP180_ADDR                     0xEE // BMP180 I2C address

// BMP180 registers
#define BMP180_PROM_START_ADDR          0xAA // E2PROM calibration data start register
#define BMP180_PROM_DATA_LEN            22   // E2PROM length
#define BMP180_CHIP_ID_REG              0xD0 // Chip ID
#define BMP180_VERSION_REG              0xD1 // Version
#define BMP180_CTRL_MEAS_REG            0xF4 // Measurements control (OSS[7.6], SCO[5], CTL[4.0]
#define BMP180_ADC_OUT_MSB_REG          0xF6 // ADC out MSB  [7:0]
#define BMP180_ADC_OUT_LSB_REG          0xF7 // ADC out LSB  [7:0]
#define BMP180_ADC_OUT_XLSB_REG         0xF8 // ADC out XLSB [7:3]
#define BMP180_SOFT_RESET_REG           0xE0 // Soft reset control

// BMP180 control values
#define BMP180_T_MEASURE                0x2E // temperature measurement
#define BMP180_P0_MEASURE               0x34 // pressure measurement (OSS=0, 4.5ms)
#define BMP180_P1_MEASURE               0x74 // pressure measurement (OSS=1, 7.5ms)
#define BMP180_P2_MEASURE               0xB4 // pressure measurement (OSS=2, 13.5ms)
#define BMP180_P3_MEASURE               0xF4 // pressure measurement (OSS=3, 25.5ms)

// BMP180 Pressure calculation constants
#define BMP180_PARAM_MG                 3038
#define BMP180_PARAM_MH                -7357
#define BMP180_PARAM_MI                 3791


// Calibration parameters structure
typedef struct {
	int16_t AC1;
	int16_t AC2;
	int16_t AC3;
	uint16_t AC4;
	uint16_t AC5;
	uint16_t AC6;
	int16_t B1;
	int16_t B2;
	int16_t MB;
	int16_t MC;
	int16_t MD;
	int32_t B5;
} BMP180_Calibration_TypeDef;

typedef struct {
	uint8_t OSS_delay;
	uint8_t OSS_cmd;
} BMP180_OSS_TypeDef;

typedef enum {
	BMP180_ERROR   = 0,
	BMP180_SUCCESS = !BMP180_ERROR
} BMP180_RESULT;

typedef enum {
	BMP180_LOWPOWER = 0,      // Ultra low power mode (oss = 0)
	BMP180_STANDARD = 1,      // Standard mode (oss = 1)
	BMP180_HIRES    = 2,      // High resolution (oss = 2)
	BMP180_UHIRES   = 3,      // Ultra high resolution (oss = 3)
	BMP180_ADVRES   = 4       // Advanced resolution (oss = 3, software oversampling)
} BMP180_Mode_TypeDef;


BMP180_Calibration_TypeDef BMP180_Calibration; // Calibration parameters from E2PROM of BMP180


// Delay and Commands for different BMP180 oversampling levels
static const BMP180_OSS_TypeDef BMP_OSS[] = {
		{ 6, BMP180_P0_MEASURE},
		{ 9, BMP180_P1_MEASURE},
		{15, BMP180_P2_MEASURE},
		{27, BMP180_P3_MEASURE}
};


// Function prototypes
BMP180_RESULT BMP180_Check(void);
void BMP180_Reset();
uint8_t BMP180_GetVersion(void);
void BMP180_ReadCalibration(void);

int16_t BMP180_GetTemperature(void);
int32_t BMP180_GetPressure(BMP180_Mode_TypeDef mode);
BMP180_RESULT BMP180_GetReadings(int16_t *RT, int32_t *RP, BMP180_Mode_TypeDef mode);

int32_t BMP180_hPa_to_mmHg(int32_t hPa);
int32_t BMP180_hPa_to_Altitude(int32_t hPa);

#endif // __BMP180_H
