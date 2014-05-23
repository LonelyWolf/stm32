#include <stm32l1xx_rcc.h>
#include <stm32l1xx_gpio.h>
#include <stm32l1xx_i2c.h>

#include <delay.h>
#include <i2c.h>
#include <bmp180.h>


void BMP180_Reset() {
	BMP180_WriteReg(BMP180_SOFT_RESET_REG,0xb6); // Do software reset
}

void BMP180_WriteReg(uint8_t reg, uint8_t value) {
	uint8_t buf[2];

	buf[0] = reg;
	buf[1] = value;
	I2C2_Write(&buf[0],2,BMP180_ADDR,I2C_STOP); // Send register address and value
}

uint8_t BMP180_ReadReg(uint8_t reg) {
	uint8_t value;

	I2C2_Write(&reg,1,BMP180_ADDR,I2C_NOSTOP); // Send register address
	I2C2_Read(&value,1,BMP180_ADDR); // Receive register value

	return value;
}

void BMP180_ReadCalibration(void) {
	uint8_t buffer[BMP180_PROM_DATA_LEN];

	buffer[0] = BMP180_PROM_START_ADDR;
	I2C2_Write(&buffer[0],1,BMP180_ADDR,I2C_NOSTOP); // Send calibration first register address
	I2C2_Read(&buffer[0],BMP180_PROM_DATA_LEN,BMP180_ADDR);

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

uint16_t BMP180_Read_UT(void) {
	uint8_t buf[2];

	BMP180_WriteReg(BMP180_CTRL_MEAS_REG,BMP180_T_MEASURE);
	Delay_ms(6); // Wait for 4.5ms by datasheet

	buf[0] = BMP180_ADC_OUT_MSB_REG;
	I2C2_Write(&buf[0],1,BMP180_ADDR,I2C_NOSTOP); // Send ADC MSB register address
	I2C2_Read(&buf[0],2,BMP180_ADDR); // Read ADC MSB and LSB
	return (buf[0] << 8) | buf[1];
}

uint32_t BMP180_Read_PT(uint8_t oss) {
	uint8_t buf[3];

	// Start pressure measurement
	BMP180_WriteReg(BMP180_CTRL_MEAS_REG,BMP_OSS[oss].OSS_cmd);
	Delay_ms(BMP_OSS[oss].OSS_delay);

	// Read pressure value
	buf[0] = BMP180_ADC_OUT_MSB_REG;
	I2C2_Write(&buf[0],1,BMP180_ADDR,I2C_NOSTOP); // Send ADC MSB register address
	I2C2_Read(&buf[0],3,BMP180_ADDR); // Read MSB, LSB, XLSB bytes

	return ((buf[0] << 16) | (buf[1] << 8) | buf[0]) >> (8 - oss);
}

uint32_t BMP180_Read_PT_3AVG(void) {
	uint32_t UP;

	// Measure pressure 3 times with maximum oversampling and return averaging value
	UP  = BMP180_Read_PT(3);
	UP += BMP180_Read_PT(3);
	UP += BMP180_Read_PT(3);

	return UP / 3;
}

int16_t BMP180_Calc_RT(uint16_t UT) {
	BMP180_Calibration.B5  = (((int32_t)UT - (int32_t)BMP180_Calibration.AC6) * (int32_t)BMP180_Calibration.AC5) >> 15;
	BMP180_Calibration.B5 += ((int32_t)BMP180_Calibration.MC << 11) / (BMP180_Calibration.B5 + BMP180_Calibration.MD);

	return (BMP180_Calibration.B5 + 8) >> 4;
}

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

// Fast integer Pa -> mmHg conversion (Pascals to millimeters of mercury)
int32_t BMP180_hPa_to_mmHg(int32_t hPa) {
	// 1 hPa = 0.75006375541921 mmHg
	return (int32_t)((hPa * 0.7500637554) / 10.0);
//	return (hPa * 75) / 1000;
}

// Fast integer hPa -> Altitude conversion (Pascals to Meters)
int32_t BMP180_hPa_to_Altitude(int32_t hPa) {
//	i = (uint32_t)(4433000 * (1 - powf(pressure_history[0]/101325.0,0.19029495)));
	return (((745 * (11390 - (hPa / 10))) / 256 + 46597) * (11390 - (hPa / 10))) / 65536 - 966;
}
