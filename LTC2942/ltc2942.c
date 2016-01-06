// Library to interact with the LTC2942-1 chip via I2C interface.
// Compatible with LTC2941.


#include "i2c.h"
#include "ltc2942.h"


// Write new value to LTC2942 register
// input:
//   reg - register number
//   value - new register value
static void LTC2942_WriteReg(uint8_t reg, uint8_t value) {
	uint8_t buf[2] = { reg, value };

	I2Cx_Transmit(LTC2942_I2C_PORT,buf,2,LTC2942_ADDR,I2C_STOP);
}

// Read LTC2942 register
// input:
//   reg - register number
// return:
//   register value
static uint8_t LTC2942_ReadReg(uint8_t reg) {
	uint8_t value = 0; // Initialize value in case of I2C timeout

	// Send register address
	I2Cx_Transmit(LTC2942_I2C_PORT,&reg,1,LTC2942_ADDR,I2C_NOSTOP);
	// Read register value
	I2Cx_Receive(LTC2942_I2C_PORT,&value,1,LTC2942_ADDR);

	return value;
}

// Read STATUS register
// return: value of STATUS register
// note: all bits in this register will be cleared after reading
inline uint8_t LTC2942_GetStatus(void) {
	return LTC2942_ReadReg(LTC2942_REG_STATUS);
}

// Read CONTROL register
// return: value of CONTROL register
inline uint8_t LTC2942_GetControl(void) {
	return LTC2942_ReadReg(LTC2942_REG_CONTROL);
}

// Read battery voltage
// return: voltage in millivolts (value of '4263' represents 4.263V)
// note: in case of I2C error the return value will be 0x80000
uint32_t LTC2942_GetVoltage(void) {
	uint8_t buf[2];
	uint32_t value = 0x80000; // Initialize with error value in case of I2C timeout

	// Send voltage MSB register address
	buf[0] = LTC2942_REG_VOL_H;
	if (I2Cx_Transmit(LTC2942_I2C_PORT,&buf[0],1,LTC2942_ADDR,I2C_NOSTOP) == I2C_SUCCESS) {
		// Read voltage MSB and LSB
		I2Cx_Receive(LTC2942_I2C_PORT,buf,2,LTC2942_ADDR);
		// Calculate voltage
		value  = buf[1] | (buf[0] << 8);
		value *= 6000;
		value /= 65535;
	}

	return value;
}

// Read chip temperature
// return: temperature in Celsius (value of '2538' represents 25.38C)
// note: in case of I2C error the return value will be 0x80000
int32_t LTC2942_GetTemperature(void) {
	uint8_t buf[2];
	uint32_t value = 0x80000; // Initialize with error value in case of I2C timeout

	// Send temperature MSB register address
	buf[0] = LTC2942_REG_TEMP_H;
	if (I2Cx_Transmit(LTC2942_I2C_PORT,&buf[0],1,LTC2942_ADDR,I2C_NOSTOP) == I2C_SUCCESS) {
		// Read temperature MSB and LSB
		I2Cx_Receive(LTC2942_I2C_PORT,buf,2,LTC2942_ADDR);

		// Calculate temperature
		value  = (buf[1] | (buf[0] << 8)) >> 4;
		value *= 60000;
		value /= 4092;
		// By now the temperature value in Kelvins, convert it to Celsius degrees
		value -= 27315;
	}

	return value;
}

// Read accumulated charge value
// note: in case of I2C error the return value will be 0x0000
uint16_t LTC2942_GetAC(void) {
	uint8_t buf[2];
	uint32_t value = 0x0000; // Initialize with error value in case of I2C timeout

	// Send accumulated charge MSB register address
	buf[0] = LTC2942_REG_AC_H;
	if (I2Cx_Transmit(LTC2942_I2C_PORT,&buf[0],1,LTC2942_ADDR,I2C_NOSTOP) == I2C_SUCCESS) {
		// Read accumulated charge MSB and LSB
		I2Cx_Receive(LTC2942_I2C_PORT,buf,2,LTC2942_ADDR);
		value = buf[1] | (buf[0] << 8);
	}

	return value;
}

// Program accumulated charge value
// input:
//   AC - new accumulated charge value
void LTC2942_SetAC(uint16_t AC) {
	uint8_t reg;

	// Before programming new AC value the analog section must be shut down
	reg = LTC2942_ReadReg(LTC2942_REG_CONTROL);
	LTC2942_WriteReg(LTC2942_REG_CONTROL,reg | LTC2942_CTL_SHUTDOWN);

	// Program new AC value
	LTC2942_WriteReg(LTC2942_REG_AC_H,AC >> 8);
	LTC2942_WriteReg(LTC2942_REG_AC_L,(uint8_t)AC);

	// Restore old CONTROL register value
	LTC2942_WriteReg(LTC2942_REG_CONTROL,reg);
}

// Configure ADC mode
// input:
//   mode - new ADC mode (one of LTC2942_ADC_XXX values)
void LTC2942_SetADCMode(uint8_t mode) {
	uint8_t reg;

	// Read CONTROL register, clear ADC mode bits and configure new value
	reg = LTC2942_ReadReg(LTC2942_REG_CONTROL) & LTC2942_CTL_ADC_MSK;
	LTC2942_WriteReg(LTC2942_REG_CONTROL,reg | mode);
}

// Configure coulomb counter prescaling factor M between 1 and 128
// input:
//   psc - new prescaler value (one of LTC2942_PSCM_XXX values)
void LTC2942_SetPrescaler(uint8_t psc) {
	uint8_t reg;

	// Read CONTROL register, clear prescaler M bits and configure new value
	reg = LTC2942_ReadReg(LTC2942_REG_CONTROL) & LTC2942_CTL_PSCM_MSK;
	LTC2942_WriteReg(LTC2942_REG_CONTROL,reg | psc);
}

// Configure the AL/CC pin
// input:
//   mode - new pin configuration (one of LTC2942_ALCC_XXX values)
void LTC2942_SetALCCMode(uint8_t mode) {
	uint8_t reg;

	// Read CONTROL register, clear AL/CC bits and configure new value
	reg = LTC2942_ReadReg(LTC2942_REG_CONTROL) & LTC2942_CTL_ALCC_MSK;
	LTC2942_WriteReg(LTC2942_REG_CONTROL,reg | mode);
}

// Configure state of the analog section of the chip
// input:
//   state - new analog section state (one of LTC2942_AN_XXX values)
void LTC2942_SetAnalog(uint8_t state) {
	uint8_t reg;

	// Read CONTROL register value
	reg = LTC2942_ReadReg(LTC2942_REG_CONTROL);

	// Set new state of SHUTDOWN bit in CONTROL register B[0]
	if (state == LTC2942_AN_DISABLED) {
		reg |= LTC2942_CTL_SHUTDOWN;
	} else {
		reg &= ~LTC2942_CTL_SHUTDOWN;
	}

	// Write new CONTROL register value
	LTC2942_WriteReg(LTC2942_REG_CONTROL,reg);
}

// Configure charge threshold high level
// input:
//   level - new threshold level (0..65535)
inline void LTC2942_SetChargeThresholdH(uint16_t level) {
	LTC2942_WriteReg(LTC2942_REG_CTH_H,(uint8_t)(level >> 8));
	LTC2942_WriteReg(LTC2942_REG_CTH_L,(uint8_t)level);
}

// Configure charge threshold low level
// input:
//   level - new threshold level (0..65535)
inline void LTC2942_SetChargeThresholdL(uint16_t level) {
	LTC2942_WriteReg(LTC2942_REG_CTL_H,(uint8_t)(level >> 8));
	LTC2942_WriteReg(LTC2942_REG_CTL_L,(uint8_t)level);
}

// Configure voltage threshold high level
// input:
//   level - new threshold level (0..255)
inline void LTC2942_SetVoltageThresholdH(uint8_t level) {
	LTC2942_WriteReg(LTC2942_REG_VOLT_H,level);
}

// Configure voltage threshold low level
// input:
//   level - new threshold level (0..255)
inline void LTC2942_SetVoltageThresholdL(uint8_t level) {
	LTC2942_WriteReg(LTC2942_REG_VOLT_L,level);
}

// Configure temperature threshold high level
// input:
//   level - new threshold level (0..255)
inline void LTC2942_SetTemperatureThresholdH(uint8_t level) {
	LTC2942_WriteReg(LTC2942_REG_TEMPT_H,level);
}

// Configure temperature threshold low level
// input:
//   level - new threshold level (0..255)
inline void LTC2942_SetTemperatureThresholdL(uint8_t level) {
	LTC2942_WriteReg(LTC2942_REG_TEMPT_L,level);
}
