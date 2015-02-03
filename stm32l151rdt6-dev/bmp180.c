#include <stm32l1xx_rcc.h>
#include <stm32l1xx_gpio.h>

#include <delay.h>
#include <i2c.h>
#include <bmp180.h>


// Write new value to BMP180 register
// input:
//   reg - register number
//   value - new register value
void BMP180_WriteReg(uint8_t reg, uint8_t value) {
	uint8_t buf[2];

	buf[0] = reg;
	buf[1] = value;
	I2Cx_Write(BMP180_I2C_PORT,&buf[0],2,BMP180_ADDR,I2C_STOP);
}

// Read BMP180 register
// input:
//   reg - register number
// return:
//   register value
uint8_t BMP180_ReadReg(uint8_t reg) {
	uint8_t value = 0; // Initialize value in case of I2C timeout

	// Send register address
	I2Cx_Write(BMP180_I2C_PORT,&reg,1,BMP180_ADDR,I2C_NOSTOP);
	// Read register value
	I2Cx_Read(BMP180_I2C_PORT,&value,1,BMP180_ADDR);

	return value;
}

// Check if BMP180 present on I2C bus
// return:
//   BMP180_SUCCESS if BMP180 present, BMP180_ERROR otherwise (not present or it was an I2C timeout)
BMP180_RESULT BMP180_Check(void) {
	uint8_t value;

	value = BMP180_ReadReg(BMP180_CHIP_ID_REG);

	return (value == 0x55) ? BMP180_SUCCESS : BMP180_ERROR;
}

// Order BMP180 to do a software reset
void BMP180_Reset() {
	BMP180_WriteReg(BMP180_SOFT_RESET_REG,0xb6);
}

// Get version of BMP180 chip
// return:
//   BMP180 chip version or zero if no BMP180 present on the I2C bus or it was an I2C timeout
uint8_t BMP180_GetVersion(void) {
	return BMP180_ReadReg(BMP180_VERSION_REG);
}

// Read calibration registers
void BMP180_ReadCalibration(void) {
	uint8_t buffer[BMP180_PROM_DATA_LEN];

	buffer[0] = BMP180_PROM_START_ADDR;
	I2Cx_Write(BMP180_I2C_PORT,&buffer[0],1,BMP180_ADDR,I2C_NOSTOP); // Send calibration first register address
	I2Cx_Read(BMP180_I2C_PORT,&buffer[0],BMP180_PROM_DATA_LEN,BMP180_ADDR);

	BMP180_Calibration.AC1 = (buffer[0]  << 8) | buffer[1];
	BMP180_Calibration.AC2 = (buffer[2]  << 8) | buffer[3];
	BMP180_Calibration.AC3 = (buffer[4]  << 8) | buffer[5];
	BMP180_Calibration.AC4 = (buffer[6]  << 8) | buffer[7];
	BMP180_Calibration.AC5 = (buffer[8]  << 8) | buffer[9];
	BMP180_Calibration.AC6 = (buffer[10] << 8) | buffer[11];
	BMP180_Calibration.B1  = (buffer[12] << 8) | buffer[13];
	BMP180_Calibration.B2  = (buffer[14] << 8) | buffer[15];
	BMP180_Calibration.MB  = (buffer[16] << 8) | buffer[17];
	BMP180_Calibration.MC  = (buffer[18] << 8) | buffer[19];
	BMP180_Calibration.MD  = (buffer[20] << 8) | buffer[21];
}

// Get uncompensated temperature value
// input:
//   UT = pointer to uncompensated temperature value
// return:
//   BMP180_ERROR if it was an I2C timeout, BMP180_SUCCESS otherwise
BMP180_RESULT BMP180_Read_UT(uint16_t *UT) {
	uint8_t buf[2];

	BMP180_WriteReg(BMP180_CTRL_MEAS_REG,BMP180_T_MEASURE);
	Delay_ms(6); // Wait for 4.5ms by datasheet

	buf[0] = BMP180_ADC_OUT_MSB_REG;
	// Send ADC MSB register address
	if (!I2Cx_Write(BMP180_I2C_PORT,&buf[0],1,BMP180_ADDR,I2C_NOSTOP)) {
		*UT = 0;
		return BMP180_ERROR;
	}
	// Read ADC MSB and LSB
	if (I2Cx_Read(BMP180_I2C_PORT,&buf[0],2,BMP180_ADDR)) {
		*UT = (buf[0] << 8) | buf[1];
		return BMP180_SUCCESS;
	} else {
		*UT = 0;
		return BMP180_ERROR;
	}
}

// Get uncompensated pressure value
// input:
//   PT = pointer to uncompensated pressure value
//   oss - oversampling level [0..3]
// return:
//   BMP180_ERROR if it was an I2C timeout, BMP180_SUCCESS otherwise
BMP180_RESULT BMP180_Read_PT(uint32_t *PT, uint8_t oss) {
	uint8_t buf[3];

	// Start pressure measurement
	BMP180_WriteReg(BMP180_CTRL_MEAS_REG,BMP_OSS[oss].OSS_cmd);
	Delay_ms(BMP_OSS[oss].OSS_delay);

	// Read pressure value
	buf[0] = BMP180_ADC_OUT_MSB_REG;
	// Send ADC MSB register address
	if (!I2Cx_Write(BMP180_I2C_PORT,&buf[0],1,BMP180_ADDR,I2C_NOSTOP)) {
		*PT = 0;
		return BMP180_ERROR;
	}
	// Read MSB, LSB, XLSB bytes
	if (I2Cx_Read(BMP180_I2C_PORT,&buf[0],3,BMP180_ADDR)) {
		*PT = ((buf[0] << 16) | (buf[1] << 8) | buf[0]) >> (8 - oss);
		return BMP180_SUCCESS;
	} else {
		*PT = 0;
		return BMP180_ERROR;
	}
}

// Get round average pressure value from three measurements with highest oversampling
// input:
//   PT = pointer to uncompensated pressure value
// return:
//   BMP180_ERROR if it was an I2C timeout, BMP180_SUCCESS otherwise
BMP180_RESULT BMP180_Read_PT_3AVG(uint32_t *UP) {
	uint32_t RAW = 0;
	uint8_t i = 0;
	uint8_t cntr = 0;

	// Measure pressure with maximum oversampling
	// Acquire 3 values, but no more than 10 measures
	*UP = 0;
	do {
		if (BMP180_Read_PT(&RAW,3)) {
			*UP += RAW;
			i++;
		}
		cntr++;
	} while (i < 3 && cntr < 10);

	if (i > 0) {
		RAW = *UP / i;
		*UP = RAW;
		return BMP180_SUCCESS;
	} else {
		*UP = 0;
		return BMP180_ERROR;
	}
}

// Calculate the real temperature from an uncompensated temperature value
// input:
//   UT - uncompensated temperature value
// return:
//   real temperature value
int16_t BMP180_Calc_RT(uint16_t UT) {
	BMP180_Calibration.B5  = (((int32_t)UT - (int32_t)BMP180_Calibration.AC6) * (int32_t)BMP180_Calibration.AC5) >> 15;
	BMP180_Calibration.B5 += ((int32_t)BMP180_Calibration.MC << 11) / (BMP180_Calibration.B5 + BMP180_Calibration.MD);

	return (BMP180_Calibration.B5 + 8) >> 4;
}

// Calculate the real pressure from an uncompensated pressure value
// input:
//   UP - uncompensated pressure value
//   oss - oversampling level of measured pressure
// return:
//   real pressure value
// note:
//   BMP180_Calc_RT() must be called before call this function
int32_t BMP180_Calc_RP(uint32_t UP, uint8_t oss) {
	int32_t B3,B6,X3,p;
	uint32_t B4,B7;

	B6 = BMP180_Calibration.B5 - 4000;
	X3 = ((BMP180_Calibration.B2 * ((B6 * B6) >> 12)) >> 11) + ((BMP180_Calibration.AC2 * B6) >> 11);
	B3 = (((((int32_t)BMP180_Calibration.AC1) * 4 + X3) << oss) + 2) >> 2;
	X3 = (((BMP180_Calibration.AC3 * B6) >> 13) + ((BMP180_Calibration.B1 * ((B6 * B6) >> 12)) >> 16) + 2) >> 2;
	B4 = (BMP180_Calibration.AC4 * (uint32_t)(X3 + 32768)) >> 15;
	B7 = ((uint32_t)UP - B3) * (50000 >> oss);
	if (B7 < 0x80000000) p = (B7 << 1) / B4; else p = (B7 / B4) << 1;
	p += ((((p >> 8) * (p >> 8) * BMP180_PARAM_MG) >> 16) + ((BMP180_PARAM_MH * p) >> 16) + BMP180_PARAM_MI) >> 4;

	return p;
}

// Get temperature value from sensor
// return:
//   temperature in tens of Celsius degree (232 -> 23.2C) if success
//   32767 if it was I2C timeout
int16_t BMP180_GetTemperature(void) {
	uint16_t UT;

	if (BMP180_Read_UT(&UT)) {
		return BMP180_Calc_RT(UT);
	} else {
		return 32767;
	}
}

// Get pressure value from sensor
// input:
//   mode - BMP180 measure accuracy mode
// return:
//   pressure value (Pa) if success
//   0 if it was I2C timeout
int32_t BMP180_GetPressure(BMP180_Mode_TypeDef mode) {
	uint32_t UP;
	uint16_t UT;

	if (!BMP180_Read_UT(&UT)) return 0;
	BMP180_Calc_RT(UT); // Temperature must calculated before calculating pressure
	if (mode == BMP180_ADVRES) {
		// Advanced resolution mode (software oversampling)
		if (BMP180_Read_PT_3AVG(&UP))
			return BMP180_Calc_RP(UP,3);
		else
			return 0;
	} else {
		if (BMP180_Read_PT(&UP,(uint8_t)mode))
			return BMP180_Calc_RP(UP,(uint8_t)mode);
		else
			return 0;
	}
}

// Get pressure and temperature values from sensor
// input:
//   RT - pointer to temperature value
//   RP - pointer to pressure value
//   mode - BMP180 measure accuracy mode
// return:
//   BMP180_ERROR if it was an I2C timeout, BMP180_SUCCESS otherwise
BMP180_RESULT BMP180_GetReadings(int16_t *RT, int32_t *RP, BMP180_Mode_TypeDef mode) {
	uint32_t UP;
	uint16_t UT;

	if (!BMP180_Read_UT(&UT)) return BMP180_ERROR;
	*RT = BMP180_Calc_RT(UT); // Temperature must calculated before calculating pressure
	if (mode == BMP180_ADVRES) {
		// Advanced resolution mode (software oversampling)
		if (BMP180_Read_PT_3AVG(&UP)) {
			*RP = BMP180_Calc_RP(UP,3);
			return BMP180_SUCCESS;
		} else return BMP180_ERROR;
	} else {
		if (BMP180_Read_PT(&UP,(uint8_t)mode)) {
			*RP = BMP180_Calc_RP(UP,(uint8_t)mode);
			return BMP180_SUCCESS;
		} else return BMP180_ERROR;
	}
}

// Fast integer Pa -> mmHg conversion (Pascals to millimeters of mercury)
// input:
//    hPa - pressure in pascals
// return:
//    pressure in millimeter of mercury
int32_t BMP180_hPa_to_mmHg(int32_t hPa) {
	// 1 hPa = 0.75006375541921 mmHg
	return (int32_t)((hPa * 0.7500637554) / 10.0);
//	return (hPa * 75) / 1000;
}

// Fast integer hPa -> Altitude conversion (Pascals to Meters)
// input:
//   hPa - pressure (pascals)
// return:
//   altitude (meters)
int32_t BMP180_hPa_to_Altitude(int32_t hPa) {
//	i = (uint32_t)(4433000 * (1 - powf(pressure_history[0]/101325.0,0.19029495)));
	return (((745 * (11390 - (hPa / 10))) / 256 + 46597) * (11390 - (hPa / 10))) / 65536 - 966;
}
