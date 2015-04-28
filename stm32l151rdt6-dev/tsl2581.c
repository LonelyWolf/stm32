#include <stm32l1xx_rcc.h>

#include "i2c.h"
#include "tsl2581.h"


uint8_t TSL2581_gain = TSL2581_GAIN1; // Current gain value
uint8_t TSL2581_int_cycles = TSL2581_NOM_INTEG_CYCLE; // Current integration cycles value


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
	// Set 400ms integration time (148 * 2.7ms)
	TSL2581_SetTime(TSL2581_NOM_INTEG_CYCLE);
	// Enable ADC (by datasheet a 2ms delay is mandatory before enabling ADC)
	TSL2581_ADCOn();
	// Set analog gain x1
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

// Disable the ADC
void TSL2581_ADCOff(void) {
	TSL2581_WriteReg(TSL2581_CONTROL,TSL2581_CTL_POWER & ~TSL2581_CTL_ADC);
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

// Set integration time of the ADC channels
// input:
//   time - time in 2.7-ms intervals
// note: time = 0 --> manual integration mode, 0xff = 2.7ms interval, 0x01 = 688.5ms
void TSL2581_SetTime(uint8_t time) {
	TSL2581_int_cycles = time;
	TSL2581_WriteReg(TSL2581_TIMING,TSL2581_int_cycles);
}

// Set analog gain of the device
// input:
//   gain - TSL2581_GAINx values
void TSL2581_SetGain(uint8_t gain) {
	if (gain > TSL2581_GAIN111) gain = TSL2581_GAIN111;
	TSL2581_gain = gain;
	// Disable the ADC before setting a new gain value to prevent indeterminate result
	TSL2581_ADCOff();
	TSL2581_WriteReg(TSL2581_ANALOG,TSL2581_gain);
	TSL2581_ADCOn();
}

// Read device ID
// return: device ID or zero if I2C fails
// note: [7..4] bits - part number (should be 1001b)
//       [3..0] bits - revision number identification
uint8_t TSL2581_GetDeviceID(void) {
	return TSL2581_ReadReg(TSL2581_ID);
}

// From TAOS datasheet
//////////////////////////////////////////////////////////////////////////////
// lux equation approximation without floating point calculations
// Description: Calculate the approximate illuminance (lux) given the raw
// channel values of the TSL2583. The equation if implemented
// as a piece−wise linear approximation.
// input:
//   tIntCycles − INTEG_CYCLES defined in Timing Register
//   d0 − raw channel value from channel 0 of TSL2583
//   d1 − raw channel value from channel 1 of TSL2583
// return: the approximate illuminance (lux)
// note: in case of sensor saturation (d0=d1=65535) function will return 0
uint32_t TSL2581_LuxCalc(uint16_t d0, uint16_t d1) {
	uint32_t chScale0;
	uint32_t chScale1;
	uint32_t channel1;
	uint32_t channel0;
	uint16_t b,m;
	uint32_t lux;
	uint32_t ratio = 0;

	// First, scale the channel values depending on the gain and integration time (1X, 400ms is nominal setting)

	// No scaling if nominal integration (148 cycles or 400 ms) is used
	if (TSL2581_int_cycles == TSL2581_NOM_INTEG_CYCLE) {
		chScale0 = (1 << TSL2581_CH_SCALE);
	} else {
		chScale0 = ((TSL2581_NOM_INTEG_CYCLE << TSL2581_CH_SCALE) / TSL2581_int_cycles);
	}
	switch (TSL2581_gain) {
		case 0:
			// 1x gain: no scale, nominal setting
			chScale1 = chScale0; //
			break;
		case 1:
			// 8x gain: scale/multiply value by 1/8
			chScale0 = chScale0 >> 3;
			chScale1 = chScale0;
			break;
		case 2: // 16x gain: scale/multiply value by 1/16
			chScale0 = chScale0 >> 4;
			chScale1 = chScale0;
			break;
		case 3: // 128x gain: CH0/CH1 gain correction factors applied
			chScale1 = chScale0 / TSL2581_CH1GAIN128X;
			chScale0 = chScale0 / TSL2581_CH0GAIN128X;
		break;
	}

	// Scale the channel values
	channel0 = (d0 * chScale0) >> TSL2581_CH_SCALE;
	channel1 = (d1 * chScale1) >> TSL2581_CH_SCALE;

	// Find the ratio of the channel values (Channel1/Channel0), protect against divide by zero
	if (channel0) ratio = (((channel1 << (TSL2581_RATIO_SCALE + 1)) / channel0) + 1) >> 1;
	if ((ratio >= 0) && (ratio <= TSL2581_K1C)) {
		b = TSL2581_B1C;
		m = TSL2581_M1C;
	} else if (ratio <= TSL2581_K2C) {
		b = TSL2581_B2C;
		m = TSL2581_M2C;
	} else if (ratio <= TSL2581_K3C) {
		b = TSL2581_B3C;
		m = TSL2581_M3C;
	} else if (ratio <= TSL2581_K4C) {
		b = TSL2581_B4C;
		m = TSL2581_M4C;
	} else if (ratio >  TSL2581_K5C) {
		b = TSL2581_B5C;
		m = TSL2581_M5C;
	}

	// Calculate a lux value
	lux   = (channel0 * b) - (channel1 * m);
	lux  += 1 << (TSL2581_LUX_SCALE - 1); // Round lsb (2^(LUX_SCALE − 1))
	lux >>= TSL2581_LUX_SCALE; // Strip off fractional portion

	return lux;
}
