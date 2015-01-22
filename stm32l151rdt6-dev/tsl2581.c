#include <stm32l1xx_rcc.h>

#include "i2c.h"
#include "tsl2581.h"


// Write new value to the TSL2581 register
// input:
//   addr - register address
//   value - new register value
void TSL2581_WriteReg(uint8_t addr, uint8_t value) {
	uint8_t buf[2];

	buf[0] = TSL2581_CMD_REG | (addr & 0x1f); // repeated byte protocol transaction
	buf[1] = value;
	I2Cx_Write(TSL2581_I2C_PORT,buf,2,TSL2581_ADDR,I2C_STOP);
}

// Read value of the TSL2581 register
// input:
//   addr - register address
// return:
//   register value
uint8_t TSL2581_ReadReg(uint8_t addr) {
	uint8_t value = 0; // Initialize value in case of I2C timeout
	uint8_t reg = TSL2581_CMD_REG | (addr & 0x1f); // repeated byte protocol transaction

	// Send command register
	I2Cx_Write(TSL2581_I2C_PORT,&reg,1,TSL2581_ADDR,I2C_NOSTOP);
	// Read register value
	I2Cx_Read(TSL2581_I2C_PORT,&value,1,TSL2581_ADDR);

	return value;
}

// Read value of the TSL2581 register
// input:
//   addr - register address
// return:
//   register value
uint16_t TSL2581_ReadReg16(uint8_t addr) {
	uint8_t buf[2] = { 0, 0 }; // Initialize value in case of I2C timeout
	uint8_t reg = TSL2581_CMD_REG | TSL2581_CMD_INC | (addr & 0x1f); // repeated byte protocol transaction

	// Send command register
	I2Cx_Write(TSL2581_I2C_PORT,&reg,1,TSL2581_ADDR,I2C_NOSTOP);
	// Read register value
	I2Cx_Read(TSL2581_I2C_PORT,buf,2,TSL2581_ADDR);

	return (buf[1] << 8) | buf[0];
}

// Initialize the TSL2581
void TSL2581_Init(void) {
	// Turn on chip power
	TSL2581_PowerOn();
	// Set 99.9ms integration time (37 * 2.7ms)
	TSL2581_SetTime(37);
	// Enable ADC (by datasheet a 2ms delay mandatory before enabling ADC)
	TSL2581_ADCOn();
	// Set analog gain 1x
	TSL2581_SetGain(TSL2581_GAIN1);
}

// Turn on the device
void TSL2581_PowerOn(void) {
	uint8_t ctl_val; // value of control register

	// Check if chip already enabled
	ctl_val = TSL2581_ReadReg(TSL2581_CONTROL);
	if (ctl_val & TSL2581_CTL_POWER) return; // Device is already enabled

	// Enable the chip
	TSL2581_WriteReg(TSL2581_CONTROL,TSL2581_CTL_POWER);
}

// Turn off the device
void TSL2581_PowerOff(void) {
	TSL2581_WriteReg(TSL2581_CONTROL,0x00); // Clear ADC_EN and POWER bits
}

// Enable the ADC
void TSL2581_ADCOn(void) {
	TSL2581_WriteReg(TSL2581_CONTROL,TSL2581_CTL_POWER | TSL2581_CTL_ADC);
}

// Set integration time of the ADC channels
// input:
//   time - time in 2.7-ms intervals
// note: time = 0 --> manual integration mode, 0xff = 2.7ms interval, 0x01 = 688.5ms
void TSL2581_SetTime(uint8_t time) {
	TSL2581_WriteReg(TSL2581_TIMING,time);
}

// Read ADC channel 0 value (visible and IR)
// return: 16-bit ADC value
uint16_t TSL2581_GetData0(void) {
	return TSL2581_ReadReg16(TSL2581_DATA0L);
}

// Read ADC channel 1 value (IR only)
// return: 16-bit ADC value
uint16_t TSL2581_GetData1(void) {
	return TSL2581_ReadReg16(TSL2581_DATA1L);
}

// Set analog gain of the device
// input:
//   gain - TSL2581_GAINx values
void TSL2581_SetGain(uint8_t gain) {
	if (gain > TSL2581_GAIN111) gain = TSL2581_GAIN111;
	TSL2581_WriteReg(TSL2581_ANALOG,gain);
}

// Read device ID
// return: device ID or zero if I2C fails
// note: [7..4] bits - part number (should be 1001b)
//       [3..0] bits - revision number identification
uint8_t TSL2581_GetDeviceID(void) {
	return TSL2581_ReadReg(TSL2581_ID);
}
