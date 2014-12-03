#include <stm32l1xx_rcc.h>

#include "i2c.h"
#include "bmc050.h"


// Write new value to the BMC050 register
// input:
//   addr - I2C device address
//   reg - register number
//   value - new register value
void BMC050_WriteReg(uint8_t addr, uint8_t reg, uint8_t value) {
	uint8_t buf[2];

	buf[0] = reg;
	buf[1] = value;
	I2Cx_Write(BMC050_I2C_PORT,&buf[0],2,addr,I2C_STOP);
}

// Read value of the BMC050 register
// input:
//   addr - I2C device address
//   reg - register number
// return:
//   register value
uint8_t BMC050_ReadReg(uint8_t addr, uint8_t reg) {
	uint8_t value = 0; // Initialize value in case of I2C timeout

	// Send register address
	I2Cx_Write(BMC050_I2C_PORT,&reg,1,addr,I2C_NOSTOP);
	// Read register value
	I2Cx_Read(BMC050_I2C_PORT,&value,1,addr);

	return value;
}

// Initialize BMC050
void BMC050_Init(void) {
	// Set accelerometer normal power mode
	BMC050_ACC_PwrNormal();
	// Set accelerometer G-range
	BMC050_ACC_SetRange(ACC_FS_2G); // 2G range
	// Set accelerometer bandwidth
	BMC050_ACC_SetBandwidth(ACC_BW2000); // 2kHz bandwidth (unfiltered data)
	// Disable I2C WDT
	BMC050_ACC_InterfaceConfig(ACC_IF_WDT_OFF);
	// Configure INT# pins as push-pull with active level high
	BMC050_ACC_IntPinConfig(ACC_INT1_PP | ACC_INT2_PP | ACC_INT1_HIGH | ACC_INT2_HIGH);
}

// Read BMC050 accelerometer device ID
// return: device ID or zero if I2C fails
uint8_t BMC050_ACC_GetDeviceID(void) {
	return BMC050_ReadReg(BMC050_ACC_ADDR,BMC050_REG_ACC_WHO_AM_I);
}

// Read BMC050 temperature value
// return: temperature value (value 245 means 24.5C)
int16_t BMC050_ReadTemp(void) {
	float f;

	f = (((int8_t)BMC050_ReadReg(BMC050_ACC_ADDR,BMC050_REG_ACC_OUT_TEMP) * 0.5) + 24.0) * 10;

	return (uint16_t)f;
}

// Set accelerometer G-range
// input:
//   range - G-range value (ACC_FS_xxG)
void BMC050_ACC_SetRange(BMC050_ACC_FS_TypeDef range) {
	BMC050_WriteReg(BMC050_ACC_ADDR,BMC050_REG_ACC_G_RANGE,range);
}

// Set accelerometer bandwidth
// input:
//   BW - bandwidth value (ACC_BW_xxx)
void BMC050_ACC_SetBandwidth(BMC050_ACC_BW_TypeDef BW) {
	uint8_t val;

	if (BW == ACC_BW2000) {
		// Set unfiltered acceleration data (2000Hz bandwidth)
		// Set data_high_bw bit in 0x13 register
		BMC050_WriteReg(BMC050_ACC_ADDR,BMC050_REG_ACC_FILTER,ACC_BW2000);
	} else {
		// Filtered acceleration data output
		// Clear data_high_bw bit of 0x13 register
		val = BMC050_ReadReg(BMC050_ACC_ADDR,BMC050_REG_ACC_FILTER);
		BMC050_WriteReg(BMC050_ACC_ADDR,BMC050_REG_ACC_FILTER,val & ~ACC_BW2000);
		// Set bandwidth
		BMC050_WriteReg(BMC050_ACC_ADDR,BMC050_REG_ACC_BANDWIDTH,BW);
	}
}

// Do user-triggered reset of the sensor
// note: after that reset all accelerometer registers return to their default values
//       accelerometer will be accessible after 2ms delay (start-up time)
void BMC050_ACC_SoftReset(void) {
	BMC050_WriteReg(BMC050_ACC_ADDR,BMC050_REG_ACC_SOFT_RESET,BMC050_ACC_SOFT_RESET);
}

// Order accelerometer to enter normal power mode
void BMC050_ACC_PwrNormal(void) {
	// Clear suspend and lowpower_en bits of 0x11 register -> normal operation mode
	BMC050_WriteReg(BMC050_ACC_ADDR,BMC050_REG_ACC_POWER_MODE,0);
}

// Order accelerometer to enter suspend mode
void BMC050_ACC_Suspend(void) {
	// Set suspend and clear lowpower_en bits of 0x11 register -> suspend mode
	BMC050_WriteReg(BMC050_ACC_ADDR,BMC050_REG_ACC_POWER_MODE,BMC050_ACC_SUSPEND);
}

// Order accelerometer to enter suspend mode
// input:
//   sleep_duration - low power sleep phase duration time (ACC_SLEEP_xxx)
void BMC050_ACC_LowPower(BMC050_ACC_Sleep_TypeDef sleep_duration) {
	// Clear suspend and setr lowpower_en bits of 0x11 register -> low power mode
	// Also set sleep phase duration
	BMC050_WriteReg(BMC050_ACC_ADDR,BMC050_REG_ACC_POWER_MODE,(BMC050_ACC_LOWPOWER | sleep_duration) & 0x5e);
}

// Read accelerometer last value of the X-axis
int16_t BMC050_ACC_GetX(void) {
	uint8_t value[2];
	uint8_t reg = BMC050_REG_ACC_OUT_XL; // LSB part of X-axis acceleration address

	// Send register address
	I2Cx_Write(BMC050_I2C_PORT,&reg,1,BMC050_ACC_ADDR,I2C_NOSTOP);
	// Read register value
	I2Cx_Read(BMC050_I2C_PORT,value,2,BMC050_ACC_ADDR);

	return (int16_t)((int16_t)(value[1] << 8) | (int16_t)(value[0] >> 6));
}

// Read accelerometer last value of the Y-axis
int16_t BMC050_ACC_GetY(void) {
	uint8_t value[2];
	uint8_t reg = BMC050_REG_ACC_OUT_YL; // LSB part of Y-axis acceleration address

	// Send register address
	I2Cx_Write(BMC050_I2C_PORT,&reg,1,BMC050_ACC_ADDR,I2C_NOSTOP);
	// Read register value
	I2Cx_Read(BMC050_I2C_PORT,value,2,BMC050_ACC_ADDR);

	return (int16_t)((int16_t)(value[1] << 8) | (int16_t)(value[0] >> 6));
}

// Read accelerometer last value of the Z-axis
int16_t BMC050_ACC_GetZ(void) {
	uint8_t value[2];
	uint8_t reg = BMC050_REG_ACC_OUT_ZL; // LSB part of Z-axis acceleration address

	// Send register address
	I2Cx_Write(BMC050_I2C_PORT,&reg,1,BMC050_ACC_ADDR,I2C_NOSTOP);
	// Read register value
	I2Cx_Read(BMC050_I2C_PORT,value,2,BMC050_ACC_ADDR);

	return (int16_t)((int16_t)(value[1] << 8) | (int16_t)(value[0] >> 6));
}

// Read accelerometer last values of the X,Y,Z-axis
// input:
//   X - pointer to signed 16-bit variable for X-axis value
//   Y - pointer to signed 16-bit variable for Y-axis value
//   Z - pointer to signed 16-bit variable for Z-axis value
// return: new values of accelerometer axis
void BMC050_ACC_GetXYZ(int16_t *X, int16_t *Y, int16_t *Z) {
	uint8_t value[6];
	uint8_t reg = BMC050_REG_ACC_OUT_XL; // LSB part of X-axis acceleration address

	// Send register address
	I2Cx_Write(BMC050_I2C_PORT,&reg,1,BMC050_ACC_ADDR,I2C_NOSTOP);
	// Read register value
	I2Cx_Read(BMC050_I2C_PORT,value,6,BMC050_ACC_ADDR);

	*X = (int16_t)((int16_t)(value[1] << 8) | (int16_t)(value[0] >> 6));
	*Y = (int16_t)((int16_t)(value[3] << 8) | (int16_t)(value[2] >> 6));
	*Z = (int16_t)((int16_t)(value[5] << 8) | (int16_t)(value[4] >> 6));
}

// Configure accelerometer interrupts
// input:
//   irqs - bitmap of enabled interrupts (combination of ACC_IE_XXX values)
void BMC050_ACC_SetIRQ(BMC050_ACC_IE_TypeDef irqs) {
	// BMC050 supports single-byte writes only
	BMC050_WriteReg(BMC050_ACC_ADDR,BMC050_REG_ACC_IRQ1,(uint16_t)irqs >> 8);
	BMC050_WriteReg(BMC050_ACC_ADDR,BMC050_REG_ACC_IRQ2,(uint16_t)irqs & 0xff);
}

// Get accelerometer interrupts status
// return: interrupts status bitmap (combination of ACC_IRQ_XXX values)
BMC050_ACC_IRQ_TypeDef BMC050_ACC_GetIRQStatus(void) {
	uint16_t value = 0;
	uint8_t reg = BMC050_REG_ACC_STATUS_IRQL;

	// Send register address
	I2Cx_Write(BMC050_I2C_PORT,&reg,1,BMC050_ACC_ADDR,I2C_NOSTOP);
	// Read register value
	I2Cx_Read(BMC050_I2C_PORT,(uint8_t *)&value,2,BMC050_ACC_ADDR);

	return value;
}

// Set accelerometer interrupt mode or reset all latched interrupts
// input:
//   mode - accelerometer interrupt mode (ACC_IM_XXX)
// note: ACC_IM_RESET value will only reset all latched interrupts
void BMC050_ACC_SetIRQMode(BMC050_ACC_IM_TypeDef mode) {
	uint8_t val = mode;

	if (mode == ACC_IM_RESET) {
		// Get register value and send reset_int bit
		// This will reset all latched interrupts without changing current interrupt mode
		val  = BMC050_ReadReg(BMC050_ACC_ADDR,BMC050_REG_ACC_IRQ_MODE);
		val |= ACC_IM_RESET;
	}
	BMC050_WriteReg(BMC050_ACC_ADDR,BMC050_REG_ACC_IRQ_MODE,val);
}

// Configure the slope interrupt (ant-motion detection)
// input:
//   nSamples - number of samples to be evaluated for the slope interrupt (0..3)
//   threshold - threshold for the slope interrupt (LSB value of acceleration data)
void BMC050_ACC_ConfigSlopeIRQ(uint8_t nSamples, uint8_t threshold) {
	// Set number of samples
	BMC050_WriteReg(BMC050_ACC_ADDR,BMC050_REG_ACC_SLOPE_SAMPLES,nSamples & 0x03);
	// Set threshold
	BMC050_WriteReg(BMC050_ACC_ADDR,BMC050_REG_ACC_SLOPE_THRESHOLD,threshold);
}

// Get tap and slope interrupt status
// return: tap and slope interrupt status (ACC_TS_xxx values to check corresponding bits)
uint8_t BMC050_ACC_GetTSIRQ(void) {
	return BMC050_ReadReg(BMC050_ACC_ADDR,BMC050_REG_ACC_TS_IRQ);
}

// Configure accelerometer interface (enable or disable WDT)
// input:
//   mode - ACC_IF_xxx value
// note: SPI mode not supported
void BMC050_ACC_InterfaceConfig(BMC050_ACC_IF_TypeDef mode) {
	BMC050_WriteReg(BMC050_ACC_ADDR,BMC050_REG_ACC_IF_CONFIG,mode & 0x06);
}

// Configure accelerometer INT# pins
// input:
//   mode - bitmap value of ACC_INTx_XXX values
void BMC050_ACC_IntPinConfig(BMC050_ACC_IntConfig_TypeDef mode) {
	BMC050_WriteReg(BMC050_ACC_ADDR,BMC050_REG_ACC_INT_CONFIG,mode & 0x0f);
}

// Configure accelerometer interrupts mapping
// input:
//   map - bitmap value of ACC_IMx_XXX values
void BMC050_ACC_IntPinMap(BMC050_ACC_INtMapping_TypeDef map) {
	BMC050_WriteReg(BMC050_ACC_ADDR,BMC050_REG_ACC_INT_MAP1,map >> 16);
	BMC050_WriteReg(BMC050_ACC_ADDR,BMC050_REG_ACC_INT_MAP2,map >> 8);
	BMC050_WriteReg(BMC050_ACC_ADDR,BMC050_REG_ACC_INT_MAP3,map & 0xff);
}

// Read BMC050 magnetometer device ID
// return: device ID or zero if I2C fails
// note: this value can only be read if power control bit (bit#0 of register 0x4b) is enabled
uint8_t BMC050_MAG_GetDeviceID(void) {
	return BMC050_ReadReg(BMC050_MAG_ADDR,BMC050_REG_MAG_WHO_AM_I);
}
