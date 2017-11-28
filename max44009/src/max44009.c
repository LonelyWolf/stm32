#include "max44009.h"


// Write new value to the MAX44009 register
// input:
//   addr - register address
//   value - new register value
static void MAX44009_WriteReg(uint8_t addr, uint8_t value) {
	uint8_t buf[2];

	buf[0] = addr;
	buf[1] = value;
	I2C_Transmit(MAX44009_I2C_PORT, buf, 2, MAX44009_ADDR, I2C_GENSTOP_YES);
}

// Read value of the MAX44009 register
// input:
//   addr - register address
// return:
//   register value
static uint8_t MAX44009_ReadReg(uint8_t addr) {
	uint8_t value = 0;

	I2C_Transmit(MAX44009_I2C_PORT, &addr, 1, MAX44009_ADDR, I2C_GENSTOP_NO);
	I2C_Receive(MAX44009_I2C_PORT, &value, 1, MAX44009_ADDR);

	return value;
}

// Read MAX44009 interrupt status
// return:
//   MAX44009_INT_RESET - no interrupt trigger event has occurred
//   MAX44009_INT_SET - interrupt has been triggered
// note: INTS bit will be cleared after calling this function
uint8_t MAX44009_INTStatus(void) {
	return (MAX44009_ReadReg(MAX44009_REG_INTS) & MAX44009_INT_SET);
}

// Enable interrupt on INT pin
void MAX44009_INTEnable(void) {
	MAX44009_WriteReg(MAX44009_REG_INTE, MAX44009_INT_ENABLE);
}

// Disable interrupt on INT pin
// note: also clears INTS bit in the STATUS register
void MAX44009_INTDisable(void) {
	MAX44009_WriteReg(MAX44009_REG_INTE, MAX44009_INT_DISABLE);
}

// Configure IC for full automatic mode
// IC will perform measures every 800ms with automatic range selection
// note: this is default mode
void MAX44009_SetModeAutomatic(void) {
	uint8_t reg;

	reg = MAX44009_ReadReg(MAX44009_REG_CFG);
	reg &= ~(MAX44009_CFG_CONT | MAX44009_CFG_MANUAL);
	MAX44009_WriteReg(MAX44009_REG_CFG, reg);
}

// Configure IC for continuous mode
// IC will perform measures in continuous mode, which means
// that as soon as one reading done, a new one begins
// Range in this mode automatic, thus the integration time
// will be chosen by internal IC circuitry
// Thus, if the IC chooses the integration time of 100ms,
// the measurements will be performed every 100ms
void MAX44009_SetModeContinuous(void) {
	uint8_t reg;

	reg = MAX44009_ReadReg(MAX44009_REG_CFG);
	reg |= MAX44009_CFG_CONT;
	reg &= ~MAX44009_CFG_MANUAL;
	MAX44009_WriteReg(MAX44009_REG_CFG, reg);
}

// Configure IC for manual mode
// IC will perform measures in continuous mode and will use
// programmed by user values of CDR and integration time
// input:
//   cdr - new state of CDR (current division ratio), one of MAX44009_CDR_xx values
//   tim - new value of integration time, one of MAX44009_IT_xx values
void MAX44009_SetModeManual(uint8_t cdr, uint8_t tim) {
	uint8_t reg;

	reg = MAX44009_ReadReg(MAX44009_REG_CFG);
	// Clear CONT, CDR and TIM bits of CFG register
	reg &= ~(MAX44009_CFG_CONT | MAX44009_CDR_DIV8 | MAX44009_IT_6d25ms);
	// Set MANUAL bit and configure new values of CDR and TIM[2:0] bits
	reg |= MAX44009_CFG_MANUAL | (cdr & MAX44009_CDR_DIV8) | (tim & MAX44009_IT_6d25ms);
	MAX44009_WriteReg(MAX44009_REG_CFG, reg);
}

// Configure MAX44009 upper threshold
// input:
//   value - new value of the upper threshold
// note: for the explanation of that value refer to the datasheet
void MAX44009_SetThrU(uint8_t value) {
	MAX44009_WriteReg(MAX44009_REG_THU, value);
}

// Configure MAX44009 lower threshold
// input:
//   value - new value of the lower threshold
// note: for the explanation of that value refer to the datasheet
void MAX44009_SetThrL(uint8_t value) {
	MAX44009_WriteReg(MAX44009_REG_THL, value);
}

// Configure MAX44009 threshold timer
// input:
//   value - new value of the threshold timer
// note: for the explanation of that value refer to the datasheet
void MAX44009_SetThrT(uint8_t value) {
	MAX44009_WriteReg(MAX44009_REG_THT, value);
}

// Configure MAX44009 to its default state
void MAX44009_Init(void) {
	// Interrupt disabled
	MAX44009_INTDisable();
	// Default mode: measurements performed every 800ms
	// Auto range: CDR and integration time are automatically determined by
	//             the internal autoranging circuitry of the IC
	MAX44009_WriteReg(MAX44009_REG_CFG, 0x03);
	// Upper threshold: maximum
	MAX44009_SetThrU(0xFF);
	// Lower threshold: minimum
	MAX44009_SetThrL(0x00);
	// Threshold timer: 25,5s
	MAX44009_SetThrT(0xFF);
}

// Get lux readings from the MAX44009
// return: lux value, millilux
//         the value of 47520 represents 47.520lux
// note: will return MAX44009_OVERRANGE value in case of overrange condition
// note: in the version with reduced accuracy, a 4-bit mantissa is used, this
//       requires one I2C transaction instead of two
//       also, the LUX value is calculated a little faster sacrificing a bit of accuracy
uint32_t MAX44009_GetLux(void) {
	uint32_t result;
	uint8_t lhb; // Lux high byte

	// Get RAW lux readings
	lhb = MAX44009_ReadReg(MAX44009_REG_LUXH);
#if (MAX44009_ACCURACY)
	uint8_t llb; // Lux low byte
	llb = MAX44009_ReadReg(MAX44009_REG_LUXL);
#endif

	// Calculate lux value using formula: 2^(exponent) * mantissa * 0.045
	result = 1 << ((lhb & 0xF0) >> 4);
	if (result == 32768) {
		// Overrange condition
		result = MAX44009_OVERRANGE;
	} else {
#if (MAX44009_ACCURACY)
		// Maximum accuracy, formula: 2^(exponent) * mantissa * 0.045
		result *= ((lhb & 0x0F) << 4) + (llb & 0x0F);
		result *= 45;
#else
		// Reduced accuracy, formula: 2^(exponent) * mantissa * 0.72
		result *= lhb & 0x0F;
		result *= 720;
#endif // MAX44009_ACCURACY
	}

	return result;
}
