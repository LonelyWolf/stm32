// Inter-integrated circuit (I2C) management:
//   - initialization and configuration
//   - data transfers in master mode
//   - check if device with specified address responds


#include <stm32l1xx_rcc.h>

#include "gpio.h"
#include "i2c.h"


// Wait until I2C flag set
// input:
//   I2Cx - I2C port
//   I2C_Flag - I2C flag (one of I2C_F_XXX values)
// return:
//   I2C_SUCCESS if flag set or I2C_ERROR in case of timeout
I2CStatus I2Cx_WaitFlagSet(I2C_TypeDef* I2Cx, uint32_t I2C_Flag) {
	uint32_t wait = I2C_WAIT_TIMEOUT;
	volatile uint16_t *preg;

	// Which I2C register will be read
	preg = (I2C_Flag & 0x80000000) ? &(I2Cx->SR1) : &(I2Cx->SR2);
	I2C_Flag &= 0xFFFF;

	// Wait for flag to be set
	while (wait--) {
		if (*preg & I2C_Flag) return I2C_SUCCESS;
	}

	return I2C_ERROR;
}

// Wait until I2C flag cleared
// input:
//   I2Cx - I2C port
//   I2C_Flag - I2C flag (one of I2C_F_XXX values)
// return:
//   I2C_SUCCESS if flag cleared or I2C_ERROR in case of timeout
I2CStatus I2Cx_WaitFlagReset(I2C_TypeDef* I2Cx, uint32_t I2C_Flag) {
	uint32_t wait = I2C_WAIT_TIMEOUT;
	volatile uint16_t *preg;

	// Determine which I2C register to be read
	preg = (I2C_Flag & 0x80000000) ? &(I2Cx->SR1) : &(I2Cx->SR2);
	I2C_Flag &= 0xFFFF;

	// Wait until flag cleared
	while (wait--) {
		if (!(*preg & I2C_Flag)) return I2C_SUCCESS;
	}

	return I2C_ERROR;
}

// Initialize specified I2C peripheral
// input:
//   I2Cx - I2C port
//   Clock - I2C speed (Hz)
// return:
//   I2C_ERROR if there was a timeout during I2C initialization, I2C_SUCCESS otherwise
// note: minimum APB1 frequency for I2C work is 2MHz
I2CStatus I2Cx_Init(I2C_TypeDef* I2Cx, uint32_t Clock) {
	RCC_ClocksTypeDef RCC_Clocks; // To compute I2C speed depending on current MCU clocking
	uint16_t reg,spd,freq;

	if (I2Cx == I2C1) {
		// Enable the I2C1 peripheral clock
		RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

		// Reset the I2C1 peripheral to initial state
		RCC->APB1RSTR |=  RCC_APB1RSTR_I2C1RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

		// Enable the I2C1 GPIO peripherals
		RCC->AHBENR |= I2C1_SCL_GPIO_PERIPH | I2C1_SDA_GPIO_PERIPH;

		// Initialize the I2C1 GPIO
		// SCL
		GPIO_set_mode(I2C1_SCL_GPIO_PORT,GPIO_Mode_AF,GPIO_PUPD_NONE,I2C1_SCL_GPIO_PIN);
		GPIO_out_cfg(I2C1_SCL_GPIO_PORT,GPIO_OT_OD,GPIO_SPD_VERYLOW,I2C1_SCL_GPIO_PIN);
		GPIO_af_cfg(I2C1_SCL_GPIO_PORT,I2C1_SCL_GPIO_SRC,GPIO_AFIO4); // AFIO4 = I2C1
		// SDA
		GPIO_set_mode(I2C1_SDA_GPIO_PORT,GPIO_Mode_AF,GPIO_PUPD_NONE,I2C1_SDA_GPIO_PIN);
		GPIO_out_cfg(I2C1_SDA_GPIO_PORT,GPIO_OT_OD,GPIO_SPD_VERYLOW,I2C1_SDA_GPIO_PIN);
		GPIO_af_cfg(I2C1_SDA_GPIO_PORT,I2C1_SDA_GPIO_SRC,GPIO_AFIO4); // AFIO4 = I2C1
	} else {
		// Enable the I2C2 peripheral clock
		RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

		// Reset the I2C2 peripheral to initial state
		RCC->APB1RSTR |=  RCC_APB1RSTR_I2C2RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;

		// Enable the I2C2 GPIO peripherals
		RCC->AHBENR |= I2C2_SCL_GPIO_PERIPH | I2C2_SDA_GPIO_PERIPH;

		// Initialize the I2C2 GPIO
		// SCL
		GPIO_set_mode(I2C2_SCL_GPIO_PORT,GPIO_Mode_AF,GPIO_PUPD_NONE,I2C2_SCL_GPIO_PIN);
		GPIO_out_cfg(I2C2_SCL_GPIO_PORT,GPIO_OT_OD,GPIO_SPD_VERYLOW,I2C2_SCL_GPIO_PIN);
		GPIO_af_cfg(I2C2_SCL_GPIO_PORT,I2C2_SCL_GPIO_SRC,GPIO_AFIO4); // AFIO4 = I2C2
		// SDA
		GPIO_set_mode(I2C2_SDA_GPIO_PORT,GPIO_Mode_AF,GPIO_PUPD_NONE,I2C2_SDA_GPIO_PIN);
		GPIO_out_cfg(I2C2_SDA_GPIO_PORT,GPIO_OT_OD,GPIO_SPD_VERYLOW,I2C2_SDA_GPIO_PIN);
		GPIO_af_cfg(I2C2_SDA_GPIO_PORT,I2C2_SDA_GPIO_SRC,GPIO_AFIO4); // AFIO4 = I2C2
	}

	// Configure the I2C peripheral

	// Get CR2 register value and clear FREQ[5:0] bits
	reg = I2Cx->CR2 & ~I2C_CR2_FREQ;

	// Get current RCC clocks
	RCC_GetClocksFreq(&RCC_Clocks);

	// Set FREQ bits depending on PCLK1 value
	freq = (uint16_t)(RCC_Clocks.PCLK1_Frequency / 1000000);
	I2Cx->CR2 |= freq;

	// TRISE can be configured only when I2C peripheral disabled
	I2Cx->CR1 &= ~I2C_CR1_PE;

	// Configure I2C speed
	if (Clock <= 100000) {
		// I2C standard speed (Clock <= 100kHz)
		spd = (uint16_t)(RCC_Clocks.PCLK1_Frequency / (Clock << 1)); // Duty cycle 50%/50%
		// I2C CCR value: Standard mode
		reg = (spd < 0x04) ? 0x04 : spd;
		// Maximum rise time for standard mode
		I2Cx->TRISE = freq + 1;
	} else {
		// I2C fast speed (100kHz > Clock <= 400kHz)
		// PCLK1 frequency must be a multiple of 10MHz
		spd = (uint16_t)(RCC_Clocks.PCLK1_Frequency / (Clock * 3)); // Duty cycle 66%/33% (Tlow/Thigh = 2)
//		spd = (uint16_t)(RCC_Clocks.PCLK1_Frequency / (Clock * 25)); // Duty cycle 64%/33% (Tlow/Thigh = 16/9)
//		reg |= I2C_CCR_DUTY; // I2C fast mode mode duty cycle = 16/9
		// I2C CCR value: Fast mode
		reg = (spd == 0) ? 1 : spd;
		reg |= I2C_CCR_FS;
		// Maximum rise time for fast mode
	    I2Cx->TRISE = (uint16_t)(((freq * 300) / 1000) + 1);
	}
	// Write to I2C CCR register
	I2Cx->CCR = reg;

	// Enable acknowledge, I2C mode, peripheral enabled
	I2Cx->CR1 = I2C_CR1_ACK | I2C_CR1_PE;

	// Set I2C own address: 0x00, 7-bit
	I2Cx->OAR1 = (1 << 14); // Bit 14 should be kept as 1

	// Wait until I2C bus is free
	if (I2Cx_WaitFlagReset(I2Cx,I2C_F_BUSY) == I2C_ERROR) return I2C_ERROR;

	return I2C_SUCCESS;
}

// Transmit an amount of data in master mode
// input:
//   I2Cx - I2C port
//   buf - pointer to the data buffer
//   nbytes - number of bytes to transmit
//   devAddr - address of target device
//   stop - generate or not STOP condition (I2C_STOP or I2C_NOSTOP values)
// return:
//   I2C_ERROR if there was a timeout during I2C operations, I2C_SUCCESS otherwise
I2CStatus I2Cx_Transmit(I2C_TypeDef* I2Cx, const uint8_t *buf, uint32_t nbytes, uint8_t devAddr, I2CStop stop) {
	// Generate START condition
	I2Cx->CR1 |= I2C_CR1_START;
	// Wait until SB flag is set
	if (I2Cx_WaitFlagSet(I2Cx,I2C_F_SB) == I2C_ERROR) return I2C_ERROR;

	// Send a slave device address (last bit reset -> transmitter mode)
	I2Cx->DR = devAddr & ~I2C_OAR1_ADD0;

	// Wait until ADDR flag is set
	if (I2Cx_WaitFlagSet(I2Cx,I2C_F_ADDR) == I2C_ERROR) return I2C_ERROR;

	// Clear the ADDR flag
	__I2C_CLEAR_ADDR(I2Cx);

	// Transmit data
	while (nbytes) {
		// Wait until TXE flag is set
		if (I2Cx_WaitFlagSet(I2Cx,I2C_F_TXE) == I2C_ERROR) return I2C_ERROR;

		// Transmit byte
		I2Cx->DR = *buf++;
		nbytes--;

		// Is byte already transmitted?
		if ((I2Cx->SR1 & I2C_SR1_BTF) && nbytes) {
			// Transmit next byte
			I2Cx->DR = *buf++;
			nbytes--;
		}
	}

	// Wait until TXE flag is set
	if (I2Cx_WaitFlagSet(I2Cx,I2C_F_TXE) == I2C_ERROR) return I2C_ERROR;

	// Generate STOP condition if specified
	if (stop == I2C_STOP) {
		// Generate STOP condition
		I2Cx->CR1 |= I2C_CR1_STOP;

		// Wait until BUSY flag is reset
		if (I2Cx_WaitFlagReset(I2Cx,I2C_F_BUSY) == I2C_ERROR) return I2C_ERROR;
	}

	return I2C_SUCCESS;
}

// Receive an amount of data in master mode
// input:
//   I2Cx - I2C port
//   buf - pointer to the data buffer
//   nbytes - number of bytes to receive
//   devAddr - address of target device
// return:
//   I2C_ERROR if there was a timeout during I2C operations, I2C_SUCCESS otherwise
I2CStatus I2Cx_Receive(I2C_TypeDef* I2Cx, uint8_t *buf, uint32_t nbytes, uint8_t devAddr) {
	// Enable acknowledge
	I2Cx->CR1 |= I2C_CR1_ACK;

	// Generate START condition
	I2Cx->CR1 |= I2C_CR1_START;
	// Wait until SB flag is set
	if (I2Cx_WaitFlagSet(I2Cx,I2C_F_SB) == I2C_ERROR) return I2C_ERROR;

	// Send a slave device address (last bit set -> receiver mode)
	I2Cx->DR = devAddr | I2C_OAR1_ADD0;
	// Wait until ADDR flag is set
	if (I2Cx_WaitFlagSet(I2Cx,I2C_F_ADDR) == I2C_ERROR) return I2C_ERROR;

	if (nbytes == 1) {
		// Receive 1 byte

		// Disable acknowledge
		I2Cx->CR1 &= ~I2C_CR1_ACK;

		// Clear the ADDR flag
		__I2C_CLEAR_ADDR(I2Cx);

		// Generate STOP condition
		I2Cx->CR1 |= I2C_CR1_STOP;
	} else if (nbytes == 2) {
		// Receive 2 bytes

		// Disable acknowledge
		I2Cx->CR1 &= ~I2C_CR1_ACK;

		// Enable POS (NACK position next)
		I2Cx->CR1 |= I2C_CR1_POS;

		// Clear the ADDR flag
		__I2C_CLEAR_ADDR(I2Cx);
	} else {
		// Receive more than 2 bytes

		// Enable acknowledge
		I2Cx->CR1 |= I2C_CR1_ACK;

		// Clear the ADDR flag
		__I2C_CLEAR_ADDR(I2Cx);
	}

	while (nbytes) {
		if (nbytes < 4) {
			if (nbytes == 1) {
				// One byte

				// Wait until RXNE flag is set (receive buffer not empty)
				if (I2Cx_WaitFlagSet(I2Cx,I2C_F_RXNE) == I2C_ERROR) return I2C_ERROR;

				// Read data
				*buf++ = I2Cx->DR;
				nbytes--;
			} else if (nbytes == 2) {
				// Two bytes

				// Wait until BTF flag set (byte transfer finished)
				if (I2Cx_WaitFlagSet(I2Cx,I2C_F_BTF) == I2C_ERROR) return I2C_ERROR;

				// Generate STOP condition
				I2Cx->CR1 |= I2C_CR1_STOP;

				// Read data
				*buf++ = I2Cx->DR;
				*buf++ = I2Cx->DR;
				nbytes -= 2;
			} else {
				// Three last bytes

				// Wait until BTF flag set (byte transfer finished)
				if (I2Cx_WaitFlagSet(I2Cx,I2C_F_BTF) == I2C_ERROR) return I2C_ERROR;
				// Disable acknowledge
				I2Cx->CR1 &= ~I2C_CR1_ACK;

				// Read data
				*buf++ = I2Cx->DR;
				nbytes--;

				// Wait until BTF flag set (byte transfer finished)
				if (I2Cx_WaitFlagSet(I2Cx,I2C_F_BTF) == I2C_ERROR) return I2C_ERROR;

				// Generate STOP condition
				I2Cx->CR1 |= I2C_CR1_STOP;

				// Read data
				*buf++ = I2Cx->DR;
				*buf++ = I2Cx->DR;
				nbytes -= 2;
			}
		} else {
			// Wait until RXNE flag is set (receive buffer not empty)
			if (I2Cx_WaitFlagSet(I2Cx,I2C_F_RXNE) == I2C_ERROR) return I2C_ERROR;

			// Read data
			*buf++ = I2Cx->DR;
			nbytes--;

			// Another byte received?
			if (I2Cx->SR1 & I2C_SR1_BTF) {
				// Read data
				*buf++ = I2Cx->DR;
				nbytes--;
			}
		}
	}

	// Disable POS (NACK position current)
	I2Cx->CR1 &= ~I2C_CR1_POS;

	// Wait until BUSY flag is reset
	if (I2Cx_WaitFlagReset(I2Cx,I2C_F_BUSY) == I2C_ERROR) return I2C_ERROR;

	return I2C_SUCCESS;
}

// Check if target device is ready for communication
// input:
//   I2Cx - I2C port
//   devAddr - target device address
//   Trials - number of trials
// return:
//   I2C_ERROR if there was a timeout during I2C operations, I2C_SUCCESS otherwise
I2CStatus I2Cx_IsDeviceReady(I2C_TypeDef* I2Cx, uint8_t devAddr, uint32_t Trials) {
	volatile uint32_t wait;
	uint16_t reg;

	do {
		// Initiate a START sequence
		I2Cx->CR1 |= I2C_CR1_START;
		// Wait for START condition generated
		if (I2Cx_WaitFlagSet(I2Cx,I2C_F_SB) == I2C_ERROR) return I2C_ERROR;

		// Send the slave address (last bit reset -> transmitter mode)
		I2Cx->DR = devAddr & ~I2C_OAR1_ADD0;

		// Wait until ADDR or AF flags set
		wait = I2C_WAIT_TIMEOUT;
		while (!((reg = I2Cx->SR1) & I2C_SR1_ADDR) && !(reg & I2C_SR1_AF) && --wait);

		// Check if device responded
		if (reg & I2C_SR1_ADDR) {
			// Generate STOP condition
			I2Cx->CR1 |= I2C_CR1_STOP;

			// Clear the ADDR flag
			__I2C_CLEAR_ADDR(I2Cx);

			// Wait until I2C bus is free
			if (I2Cx_WaitFlagReset(I2Cx,I2C_F_BUSY) == I2C_ERROR) return I2C_ERROR;

			return I2C_SUCCESS;
		} else {
			// Generate STOP condition
			I2Cx->CR1 |= I2C_CR1_STOP;

			// Clear the AF flag
			I2Cx->SR1 &= ~I2C_SR1_AF;

			// Wait until I2C bus is free
			if (I2Cx_WaitFlagReset(I2Cx,I2C_F_BUSY) == I2C_ERROR) return I2C_ERROR;
		}
	} while (--Trials);

	return I2C_ERROR;
}
