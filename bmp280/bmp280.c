// Functions to manage the BMP280 sensor:
//   - reset the chip
//   - get chip version (chip ID)
//   - read calibration parameters
//   - chip configuration
//   - reading uncompensated values of temperature and pressure
//   - calculate compensated values of temperature and pressure
// Additional:
//   - conversion of pressure in pascals to pressure in millimeters of mercury


#include "bmp280.h"


// Structure for storing compensation parameters
static struct BMP280_cal_param_t {
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
} cal_param;

// Carries fine temperature as global value for pressure calculation
#if (BMP280_CALC_TYPE == 0) || (BMP280_CALC_TYPE == 1)
static int32_t t_fine;
#endif
#if (BMP280_CALC_TYPE == 2) || (BMP280_FLOAT_FUNCTIONS)
static float t_fine_f;
#endif


// Registers
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

// Register masks
#define BMP280_STATUS_MSK               ((uint8_t)0x09) // unused bits in 'status'
#define BMP280_OSRS_T_MSK               ((uint8_t)0xE0) // 'osrs_t' in 'control'
#define BMP280_OSRS_P_MSK               ((uint8_t)0x1C) // 'osrs_p' in 'control'
#define BMP280_MODE_MSK                 ((uint8_t)0x03) // 'mode' in 'control'
#define BMP280_STBY_MSK                 ((uint8_t)0xE0) // 't_sb' in 'config'
#define BMP280_FILTER_MSK               ((uint8_t)0x1C) // 'filter' in 'config'

// Value to call a complete power-on-reset routine
#define BMP280_SOFT_RESET_KEY           ((uint8_t)0xB6)

// Chip IDs for samples and mass production parts
#define BMP280_CHIP_ID1                 ((uint8_t)0x56)
#define BMP280_CHIP_ID2                 ((uint8_t)0x57)
#define BMP280_CHIP_ID3                 ((uint8_t)0x58)


// Write a new value to BMP280 register
// input:
//   reg - register number
//   value - new register value
static void __reg_write(uint8_t reg, uint8_t value) {
	uint8_t buf[2] = { reg, value };

	I2C_Transmit(BMP280_I2C_PORT, buf, sizeof(buf), BMP280_ADDR, I2C_GENSTOP_YES);
}

// Read a value of the BMP280 register
// input:
//   reg - register number
// return: register value (zero in case of error on I2C bus)
static uint8_t __reg_read(uint8_t reg) {
	uint8_t value = 0;

	I2C_Transmit(BMP280_I2C_PORT, &reg, 1, BMP280_ADDR, I2C_GENSTOP_NO);
	I2C_Receive(BMP280_I2C_PORT, &value, 1, BMP280_ADDR);

	return value;
}

// Bulk read a BMP280 registers
// input:
//   reg - starting register number
//   buf - pointer to the buffer to store data
//   len - number of registers to read
// return: BMP280_ERROR in case of error on I2C bus, BMP280_SUCCESS otherwise
static BMP280_RESULT __reg_read_bulk(uint8_t reg, uint8_t *buf, uint32_t count) {
	if (I2C_Transmit(BMP280_I2C_PORT, &reg, 1, BMP280_ADDR, I2C_GENSTOP_NO) == I2C_SUCCESS) {
		if (I2C_Receive(BMP280_I2C_PORT, buf, count, BMP280_ADDR) == I2C_SUCCESS) {
			return BMP280_SUCCESS;
		}
	}

	return BMP280_ERROR;
}

// Check if BMP280 present on I2C bus
// return: BMP280_SUCCESS if BMP280 present, BMP280_ERROR otherwise
BMP280_RESULT BMP280_Check(void) {
	switch (BMP280_GetVersion()) {
		case BMP280_CHIP_ID1:
		case BMP280_CHIP_ID2:
		case BMP280_CHIP_ID3:
			return BMP280_SUCCESS;
		default:
			return BMP280_ERROR;
	}
}

// Order BMP280 to do a software reset
// note: after reset the chip will be unaccessible during 3ms
inline void BMP280_Reset(void) {
	__reg_write(BMP280_REG_RESET, BMP280_SOFT_RESET_KEY);
}

// Get version of the BMP280 chip
// return: version of BMP280 chip or zero in case of chip absence or error on I2C bus
inline uint8_t BMP280_GetVersion(void) {
	return __reg_read(BMP280_REG_ID);
}

// Get current status of the BMP280 chip
// return: status of the chip or zero in case of chip absence or error on I2C bus
inline uint8_t BMP280_GetStatus(void) {
	return __reg_read(BMP280_REG_STATUS) & BMP280_STATUS_MSK;
}

// Get current sensor mode of the BMP280 chip
// return: working mode of the chip or zero in case of chip absence or error on I2C bus
inline uint8_t BMP280_GetMode(void) {
	return __reg_read(BMP280_REG_CTRL_MEAS) & BMP280_MODE_MSK;
}

// Set sensor mode of the BMP280 chip
// input:
//   mode - new mode (one of BMP280_MODE_xx values)
// note: always set the power mode after sensor configuration is done
void BMP280_SetMode(uint8_t mode) {
	// Configure 'mode' bits in 'ctrl_meas' (0xF4) register
	mode &= BMP280_MODE_MSK;
	uint8_t reg = (uint8_t)(__reg_read(BMP280_REG_CTRL_MEAS) & ~BMP280_MODE_MSK);
	__reg_write(BMP280_REG_CTRL_MEAS, reg | mode);
}

// Set coefficient of the IIR filter
// input:
//   filter - new coefficient value (one of BMP280_FILTER_x values)
void BMP280_SetFilter(uint8_t filter) {
	// Configure 'filter' bits in 'config' (0xF5) register
	filter &= BMP280_FILTER_MSK;
	uint8_t reg = (uint8_t)(__reg_read(BMP280_REG_CONFIG) & ~BMP280_FILTER_MSK);
	__reg_write(BMP280_REG_CONFIG, reg | filter);
}

// Set inactive duration in normal mode (Tstandby)
// input:
//   tsb - new inactive duration (one of BMP280_STBY_x values)
void BMP280_SetStandby(uint8_t tsb) {
	// Configure 't_sb' bits in 'config' (0xF5) register
	tsb &= BMP280_STBY_MSK;
	uint8_t reg = (uint8_t)(__reg_read(BMP280_REG_CONFIG) & ~BMP280_STBY_MSK);
	__reg_write(BMP280_REG_CONFIG, reg | tsb);
}

// Set oversampling of temperature data
// input:
//   osrs - new oversampling value (one of BMP280_OSRS_T_Xx values)
void BMP280_SetOSRST(uint8_t osrs) {
	// Configure 'osrs_t' bits in 'ctrl_meas' (0xF4) register
	osrs &= BMP280_OSRS_T_MSK;
	uint8_t reg = (uint8_t)(__reg_read(BMP280_REG_CTRL_MEAS) & ~BMP280_OSRS_T_MSK);
	__reg_write(BMP280_REG_CTRL_MEAS, reg | osrs);
}

// Set oversampling of pressure data
// input:
//   osrs - new oversampling value (one of BMP280_OSRS_P_Xx values)
void BMP280_SetOSRSP(uint8_t osrs) {
	// Configure 'osrs_p' bits in 'ctrl_meas' (0xF4) register
	osrs &= BMP280_OSRS_P_MSK;
	uint8_t reg = (uint8_t)(__reg_read(BMP280_REG_CTRL_MEAS) & ~BMP280_OSRS_P_MSK);
	__reg_write(BMP280_REG_CTRL_MEAS, reg | osrs);
}

// Read calibration data
// return: BMP280_ERROR in case of error on I2C bus, BMP280_SUCCESS otherwise
BMP280_RESULT BMP280_Read_Calibration(void) {
	// Bulk read from 'calib00' to 'calib25'
	return __reg_read_bulk(BMP280_REG_CALIB00, (uint8_t *)&cal_param, sizeof(cal_param));
}

// Read a raw (uncompensated) temperature value
// input:
//   UT - pointer to store value (signed 32-bit)
// return: BMP280_ERROR in case of error on I2C bus, BMP280_SUCCESS otherwise
// note: the '0x80000' value in UT means no temperature data is present,
//       i.e. that measurement is disabled or not ready yet
BMP280_RESULT BMP280_Read_UT(int32_t *UT) {
	uint8_t buf[3];

	// Bulk read from 'temp_msb' to 'temp_xlsb'
	if (__reg_read_bulk(BMP280_REG_TEMP_MSB, buf, sizeof(buf)) == BMP280_SUCCESS) {
		*UT = (int32_t)((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));
		return BMP280_SUCCESS;
	}

	// Default result value
	*UT = BMP280_NO_TEMPERATURE;

	return BMP280_ERROR;
}

// Read a raw (uncompensated) pressure value
// input:
//   UP - pointer to store value (signed 32-bit)
// return: BMP280_ERROR in case of error on I2C bus, BMP280_SUCCESS otherwise
// note: the '0x80000' value in UP means no pressure data is present,
//       i.e. that measurement is disabled or not ready yet
BMP280_RESULT BMP280_Read_UP(int32_t *UP) {
	uint8_t buf[3];

	// Bulk read from 'press_msb' to 'press_xlsb'
	if (__reg_read_bulk(BMP280_REG_PRESS_MSB, buf, sizeof(buf)) == BMP280_SUCCESS) {
		*UP = (int32_t)((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));
		return BMP280_SUCCESS;
	}

	// Default result value
	*UP = BMP280_NO_PRESSURE;

	return BMP280_ERROR;
}

// Read a raw (uncompensated) temperature and pressure values
// input:
//   UT - pointer to store temperature value (signed 32-bit)
//   UP - pointer to store pressure value (signed 32-bit)
// return: BMP280_ERROR in case of error on I2C bus, BMP280_SUCCESS otherwise
// note: the '0x80000' value means no data for this particular value is present,
//       i.e. that measurement is disabled or not ready yet
BMP280_RESULT BMP280_Read_UTP(int32_t *UT, int32_t *UP) {
	uint8_t buf[8];

	// Bulk read from 'press_msb' to 'temp_xlsb'
	if (__reg_read_bulk(BMP280_REG_PRESS_MSB, buf, sizeof(buf)) == BMP280_SUCCESS) {
		*UP = (int32_t)((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));
		*UT = (int32_t)((buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4));
		return BMP280_SUCCESS;
	}

	// Default result values
	*UT = BMP280_NO_TEMPERATURE;
	*UP = BMP280_NO_PRESSURE;

	return BMP280_ERROR;
}

// Calculate temperature from raw value, resolution is 0.01 degree
// input:
//   UT - raw temperature value
// return: temperature in Celsius degrees (value of '5123' represents '51.23C')
// note: code from the BMP280 datasheet (rev 1.19)
int32_t BMP280_CalcT(int32_t UT) {
#if (BMP280_CALC_TYPE != 2)
	// Integer calculations

	t_fine  = ((((UT >> 3) - ((int32_t)cal_param.dig_T1 << 1))) \
			* ((int32_t)cal_param.dig_T2)) >> 11;
	t_fine += (((((UT >> 4) - ((int32_t)cal_param.dig_T1)) \
			* ((UT >> 4) - ((int32_t)cal_param.dig_T1))) >> 12) \
			* ((int32_t)cal_param.dig_T3)) >> 14;

	return ((t_fine * 5) + 128) >> 8;
#else
	// Float calculations

	float v_x1, v_x2;

	v_x1 = (((float)UT) / 16384.0F - ((float)cal_param.dig_T1) / 1024.0F) * \
			((float)cal_param.dig_T2);
	v_x2 = ((float)UT) / 131072.0F - ((float)cal_param.dig_T1) / 8192.0F;
	v_x2 = (v_x2 * v_x2) * ((float)cal_param.dig_T3);
	t_fine_f = v_x1 + v_x2;

	return (int32_t)(((v_x1 + v_x2) / 5120.0F) * 100.0F);
#endif // BMP280_CALC_TYPE
}

// Calculate pressure from raw value, resolution is 0.001 Pa
// input:
//   UP - raw pressure value
// return: pressure in mPa (value of '100663688' represents '100663.688Pa')
// note: BMP280_CalcT() should be called before calling this function
// note: code from the BMP280 datasheet (rev 1.19)
uint32_t BMP280_CalcP(int32_t UP) {
#if (BMP280_CALC_TYPE == 0)
	// 32-bit only calculations
	int32_t v1, v2;
	uint32_t p;

	v1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
	v2 = (((v1 >> 2) * (v1 >> 2)) >> 11 ) * ((int32_t)cal_param.dig_P6);
	v2 = v2 + ((v1 * ((int32_t)cal_param.dig_P5)) << 1);
	v2 = (v2 >> 2) + (((int32_t)cal_param.dig_P4) << 16);
	v1 = (((cal_param.dig_P3 * (((v1 >> 2) * (v1 >> 2)) >> 13 )) >> 3) + \
			((((int32_t)cal_param.dig_P2) * v1) >> 1)) >> 18;
	v1 = (((32768 + v1)) * ((int32_t)cal_param.dig_P1)) >> 15;
	if (v1 == 0) {
		// avoid exception caused by division by zero
		return 0;
	}
	p = (((uint32_t)(((int32_t)1048576) - UP) - (uint32_t)(v2 >> 12))) * 3125U;
	if (p < 0x80000000U) {
		p = (p << 1) / ((uint32_t)v1);
	} else {
		p = (p / (uint32_t)v1) << 1;
	}
	v1 = (((int32_t)cal_param.dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
	v2 = (((int32_t)(p >> 2)) * ((int32_t)cal_param.dig_P8)) >> 13;
	p = (uint32_t)((int32_t)p + ((v1 + v2 + cal_param.dig_P7) >> 4));

	return p * 1000U;
#elif (BMP280_CALC_TYPE == 1)
	// 64-bit calculations
	int64_t v1, v2, p;

	v1 = (int64_t)t_fine - 128000;
	v2 = v1 * v1 * (int64_t)cal_param.dig_P6;
	v2 = v2 + ((v1 * (int64_t)cal_param.dig_P5) << 17);
	v2 = v2 + ((int64_t)cal_param.dig_P4 << 35);
	v1 = ((v1 * v1 * (int64_t)cal_param.dig_P3) >> 8) + \
			((v1 * (int64_t)cal_param.dig_P2) << 12);
	v1 = (((((int64_t)1) << 47) + v1)) * ((int64_t)cal_param.dig_P1) >> 33;
	if (v1 == 0) {
		// avoid exception caused by division by zero
		return 0;
	}
	p = 1048576 - UP;
	p = (((p << 31) - v2) * 3125) / v1;
	v1 = (((int64_t)cal_param.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	v2 = (((int64_t)cal_param.dig_P8) * p) >> 19;
	p = ((p + v1 + v2) >> 8) + ((int64_t)cal_param.dig_P7 << 4);

	return (uint32_t)((p * 1000) >> 8);
#else // BMP280_CALC_TYPE == 2
	// Float calculations
	float v_x1, v_x2, p_f;

	v_x1 = (t_fine_f / 2.0F) - 64000.0F;
	v_x2 = v_x1 * v_x1 * ((float)cal_param.dig_P6) / 32768.0F;
	v_x2 = v_x2 + v_x1 * ((float)cal_param.dig_P5) * 2.0F;
	v_x2 = (v_x2 / 4.0F) + (((float)cal_param.dig_P4) * 65536.0F);
	v_x1 = (((float)cal_param.dig_P3) * v_x1 * v_x1 / 524288.0F + \
			((float)cal_param.dig_P2) * v_x1) / 524288.0F;
	v_x1 = (1.0F + v_x1 / 32768.0F) * ((float)cal_param.dig_P1);
	p_f = 1048576.0F - (float)UP;
	if ((uint32_t)v_x1 == 0U) {
		// Avoid exception caused by division by zero
		return 0;
	}
	p_f = (p_f - (v_x2 / 4096.0F)) * 6250.0F / v_x1;
	v_x1 = ((float)cal_param.dig_P9) * p_f * p_f / 2147483648.0F;
	v_x2 = p_f * ((float)cal_param.dig_P8) / 32768.0F;
	p_f += (v_x1 + v_x2 + ((float)cal_param.dig_P7)) / 16.0F;

	return (uint32_t)(p_f * 1000.0F);
#endif // BMP280_CALC_TYPE
}

// Pa to mmHg conversion (Pascals to millimeters of mercury)
// input:
//   p_pa - pressure, milli pascals
// return: pressure, "micro-meters" of mercury (value of '739503' represents '739.503mmHg')
uint32_t BMP280_Pa_to_mmHg(uint32_t p_pa) {
#if (BMP280_CALC_TYPE == 0)
	// 32-bit fixed point calculations

	// Convert milli-Pascals to Pascals (Q32.0),
	// multiply by the magic constant (Q0.22),
	// the product is a pressure in mmHg (Q10.22)
	// note: 1Pa = ~0.00750061683mmHg
	uint32_t p = p_pa / 1000U;
	p *= 31460U; // 31460 is the ~0.00750061683 in Q0.22 format

	// (p_mmHg >> 22) -> integer part from Q10.22 value
	// (p_mmHg << 10) >> 18 -> fractional part truncated down to 14 bits
	// (XXX * 61039) / 1000000 is rough integer equivalent of float (XXX / 16383.0F) * 1000
	return ((p >> 22) * 1000U) + ((((p << 10) >> 18) * 61039U) / 1000000U);
#elif (BMP280_CALC_TYPE == 1)
	// 64-bit integer calculations

	// A bit more precision but noticeable slower on a 32-bit MCU
	uint64_t p_mmHg = ((uint64_t)p_pa * 1000000ULL) / 133322368ULL;
	return (uint32_t)p_mmHg;
#else // BMP280_CALC_TYPE == 2
	// Float calculations
	return (uint32_t)((float)p_pa / 133.322368F);
#endif // BMP280_CALC_TYPE



}

#if (BMP280_FLOAT_FUNCTIONS)

// Calculate temperature from raw value, resolution is 0.01 degree (FLOAT)
// input:
//   UT - raw temperature value
// return: temperature in Celsius degrees
// note: code from the BMP280 datasheet (rev 1.19)
float BMP280_CalcTf(int32_t UT) {
	float v_x1, v_x2;

	v_x1 = (((float)UT) / 16384.0F - ((float)cal_param.dig_T1) / 1024.0F) * \
			((float)cal_param.dig_T2);
	v_x2 = ((float)UT) / 131072.0F - ((float)cal_param.dig_T1) / 8192.0F;
	v_x2 = (v_x2 * v_x2) * ((float)cal_param.dig_T3);
	t_fine_f = v_x1 + v_x2;

	return (v_x1 + v_x2) / 5120.0F;
}

// Calculate pressure from raw value, resolution is 0.001 Pa (FLOAT)
// input:
//   UP - raw pressure value
// return: pressure in Pa
// note: BMP280_CalcT of BMP280_CalcTf must be called before calling this function
// note: code from the BMP280 datasheet (rev 1.19)
float BMP280_CalcPf(uint32_t UP) {
	float v_x1, v_x2, p;

	v_x1 = (t_fine_f / 2.0F) - 64000.0F;
	v_x2 = v_x1 * v_x1 * ((float)cal_param.dig_P6) / 32768.0F;
	v_x2 = v_x2 + v_x1 * ((float)cal_param.dig_P5) * 2.0F;
	v_x2 = (v_x2 / 4.0F) + (((float)cal_param.dig_P4) * 65536.0F);
	v_x1 = (((float)cal_param.dig_P3) * v_x1 * v_x1 / 524288.0F + \
			((float)cal_param.dig_P2) * v_x1) / 524288.0F;
	v_x1 = (1.0F + v_x1 / 32768.0F) * ((float)cal_param.dig_P1);
	p = 1048576.0F - (float)UP;
	if ((uint32_t)v_x1 == 0U) {
		// Avoid exception caused by division by zero
		return 0.0F;
	}
	p = (p - (v_x2 / 4096.0F)) * 6250.0F / v_x1;
	v_x1 = ((float)cal_param.dig_P9) * p * p / 2147483648.0F;
	v_x2 = p * ((float)cal_param.dig_P8) / 32768.0F;
	p += (v_x1 + v_x2 + ((float)cal_param.dig_P7)) / 16.0F;

	return p;
}

// Convert pressure value from Pascals to millimeters of mercury (FLOAT)
// input:
//   p_pa - pressure in Pascals
// return: pressure in mmHg
// note: float calculations with float result
float BMP280_Pa_to_mmHgf(float p_pa) {
	return p_pa / 133.322368F;
}

#endif // BMP280_FLOAT_FUNCTIONS
