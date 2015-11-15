#include "i2c.h"


// Wait for specified I2C event (combination of I2C flags)
// input:
//   I2Cx - I2C port
//   I2C_Event - I2C event (one of I2C_EVENT_XXX values)
// return:
//   I2C_SUCCESS if event happens or I2C_ERROR in case of timeout
I2C_Status I2Cx_WaitEvent(I2C_TypeDef* I2Cx, uint32_t I2C_Event) {
	volatile uint32_t wait = I2C_WAIT_TIMEOUT;
	uint32_t reg;

	while (wait--) {
		reg  = I2Cx->SR1;
		reg |= I2Cx->SR2 << 16;
		reg &= I2C_FLAG_MASK;
		if ((reg & I2C_Event) == I2C_Event) return I2C_SUCCESS;
	}

	return I2C_ERROR;
}

// Wait until I2C flag set
// input:
//   I2Cx - I2C port
//   I2C_Flag - I2C flag (one of I2C_F_XXX values)
// return:
//   I2C_SUCCESS if flag set or I2C_ERROR in case of timeout
I2C_Status I2Cx_WaitFlagSet(I2C_TypeDef* I2Cx, uint32_t I2C_Flag) {
	volatile uint32_t wait = I2C_WAIT_TIMEOUT;
	volatile uint16_t *preg;

	// Determine which I2C register to be read
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
I2C_Status I2Cx_WaitFlagReset(I2C_TypeDef* I2Cx, uint32_t I2C_Flag) {
	volatile uint32_t wait = I2C_WAIT_TIMEOUT;
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
I2C_Status I2Cx_Init(I2C_TypeDef* I2Cx, uint32_t Clock) {
	GPIO_InitTypeDef PORT;
	RCC_ClocksTypeDef RCC_Clocks; // To compute I2C speed depending on current MCU clocking
	uint16_t reg, spd, freq;

	PORT.GPIO_Speed = GPIO_Speed_40MHz;
	PORT.GPIO_OType = GPIO_OType_OD;
	PORT.GPIO_Mode  = GPIO_Mode_AF;
	PORT.GPIO_PuPd  = GPIO_PuPd_UP;

	if (I2Cx == I2C1) {
		// Enable the I2C1 peripheral clock
		RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
		// Reset the I2C1 peripheral to initial state
		RCC->APB1RSTR |=  RCC_APB1RSTR_I2C1RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
		// Enable the I2Cx GPIO peripheral clock
		RCC->AHBENR |= I2C1_GPIO_AHB;
		// Initialize the I2C1 GPIO peripheral
		PORT.GPIO_Pin = I2C1_GPIO_SCL | I2C1_GPIO_SDA;
		GPIO_Init(I2C1_GPIO_PORT,&PORT);
		GPIO_PinAFConfig(I2C1_GPIO_PORT,I2C1_GPIO_SCL_SRC,GPIO_AF_I2C1);
		GPIO_PinAFConfig(I2C1_GPIO_PORT,I2C1_GPIO_SDA_SRC,GPIO_AF_I2C1);
	} else {
		// Enable the I2C2 peripheral clock
		RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
		// Reset the I2C2 peripheral to initial state
		RCC->APB1RSTR |=  RCC_APB1RSTR_I2C2RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C2RST;
		// Enable the I2Cx GPIO peripheral clock
		RCC->AHBENR |= I2C2_GPIO_AHB;
		// Initialize the I2C2 GPIO peripheral
		PORT.GPIO_Pin = I2C2_GPIO_SCL | I2C2_GPIO_SDA;
		GPIO_Init(I2C2_GPIO_PORT,&PORT);
		GPIO_PinAFConfig(I2C2_GPIO_PORT,I2C2_GPIO_SCL_SRC,GPIO_AF_I2C2);
		GPIO_PinAFConfig(I2C2_GPIO_PORT,I2C2_GPIO_SDA_SRC,GPIO_AF_I2C2);
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

// Send data to I2C port
// input:
//   I2Cx - I2C port
//   buf - pointer to the data buffer
//   nbytes - number of bytes to transmit
//   SlaveAddress - address of slave device
//   stop - generate or not STOP condition (I2C_STOP/I2C_NOSTOP)
// return:
//   I2C_ERROR if there was a timeout during I2C operations, I2C_SUCCESS otherwise
I2C_Status I2Cx_Write(I2C_TypeDef* I2Cx, const uint8_t* buf, uint32_t nbytes,
		uint8_t SlaveAddress, I2C_STOP_TypeDef stop) {

	// Initiate a START sequence
	I2Cx->CR1 |= I2C_CR1_START;
	// Wait for EV5
	if (I2Cx_WaitEvent(I2Cx,I2C_EVENT_EV5) == I2C_ERROR) return I2C_ERROR;

	// Send the slave address (EV5)
	I2Cx->DR = SlaveAddress & ~I2C_OAR1_ADD0; // Last bit be reset (transmitter mode)

	// Wait for EV6
	if (I2Cx_WaitEvent(I2Cx,I2C_EVENT_EV6) == I2C_ERROR) return I2C_ERROR;

	// Send first byte (EV8)
	I2Cx->DR = *buf++;
	// Send rest of data (if present)
	while (--nbytes) {
		// Wait for BTF flag set
		if (I2Cx_WaitFlagSet(I2Cx,I2C_F_BTF) == I2C_ERROR) return I2C_ERROR;
		// Transmit byte via I2C
		I2Cx->DR = *buf++;
	}
	// Wait for BTF flag set
	if (I2Cx_WaitFlagSet(I2Cx,I2C_F_BTF) == I2C_ERROR) return I2C_ERROR;

	// Transmission end
	if (stop == I2C_STOP) {
		// Generate a STOP condition
		I2Cx->CR1 |= I2C_CR1_STOP;
		// Wait for a STOP flag
		if (I2Cx_WaitFlagReset(I2Cx,I2C_F_STOPF) == I2C_ERROR) return I2C_ERROR;
	}

	return I2C_SUCCESS;
}

// Read data from I2C port
// input:
//   I2Cx - I2C port
//   buf - pointer to data buffer
//   nbytes - number of bytes to receive
//   SlaveAddress - address of slave device
// return:
//   I2C_ERROR if there was a timeout during I2C operations, I2C_SUCCESS otherwise
I2C_Status I2Cx_Read(I2C_TypeDef* I2Cx, uint8_t *buf, uint32_t nbytes,
		uint8_t SlaveAddress) {

	// Enable Acknowledgment
	I2Cx->CR1 |= I2C_CR1_ACK;
	// Clear POS flag
	I2Cx->CR1 &= ~I2C_CR1_POS; // NACK position current

	// Initiate START sequence
	I2Cx->CR1 |= I2C_CR1_START;
	// Wait for EV5
	if (I2Cx_WaitEvent(I2Cx,I2C_EVENT_EV5) == I2C_ERROR) return I2C_ERROR;

	// Send the slave address (EV5)
	I2Cx->DR = SlaveAddress | I2C_OAR1_ADD0; // Last bit set (receiver mode)

	// Wait for EV6
	if (I2Cx_WaitFlagSet(I2Cx,I2C_F_ADDR) == I2C_ERROR) return I2C_ERROR;

	// There are can be three cases:
	//   read 1 byte
	//   read 2 bytes
	//   read more than 2 bytes
	if (nbytes == 1) {
		// Receive 1 byte (AN2824 figure 2)
		I2Cx->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_ACK); // Disable I2C acknowledgment

		// EV6_1 must be atomic operation (AN2824)
		__disable_irq();
		(void)I2Cx->SR1; // Clear ADDR bit
		(void)I2Cx->SR2;
		I2Cx->CR1 |= I2C_CR1_STOP; // Generate a STOP condition
		__enable_irq();

		// Wait for RxNE flag (receive buffer not empty) EV7
		if (I2Cx_WaitFlagSet(I2Cx,I2C_F_RXNE) == I2C_ERROR) return I2C_ERROR;

		// Read received byte
		*buf = (uint8_t)I2Cx->DR;
	} else if (nbytes == 2) {
		// Receive 2 bytes (AN2824 figure 2)
		I2Cx->CR1 |= I2C_CR1_POS; // Set POS flag (NACK position next)

		// EV6_1 must be atomic operation (AN2824)
		__disable_irq();
		(void)I2Cx->SR2; // Clear ADDR bit
		I2Cx->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_ACK); // Disable I2C acknowledgment
		__enable_irq();

		// Wait for BTF flag set (byte transfer finished) EV7_3
		if (I2Cx_WaitFlagSet(I2Cx,I2C_F_BTF) == I2C_ERROR) return I2C_ERROR;

		// This should be atomic operation
		__disable_irq();
		// Generate a STOP condition
		I2Cx->CR1 |= I2C_CR1_STOP;
		// Read first received byte
		*buf++ = (uint8_t)I2Cx->DR;
		__enable_irq();

		// Read second received byte
		*buf = (uint8_t)I2Cx->DR;
	} else {
		// Receive more than 2 bytes (AN2824 figure 1)
		(void)I2Cx->SR2; // Clear ADDR bit

		// Read received bytes into buffer
		while (nbytes-- != 3) {
			// Wait for BTF (cannot guarantee 1 transfer completion time)
			if (I2Cx_WaitFlagSet(I2Cx,I2C_F_BTF) == I2C_ERROR) return I2C_ERROR;
			*buf++ = (uint8_t)I2Cx->DR;
		}

		// Wait for BTF flag set (byte transfer finished) EV7_2
		if (I2Cx_WaitFlagSet(I2Cx,I2C_F_BTF) == I2C_ERROR) return I2C_ERROR;

		// Disable the I2C acknowledgment
		I2Cx->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_ACK);

		__disable_irq();
		// Read received byte N-2
		*buf++ = (uint8_t)I2Cx->DR;
		// Generate a STOP condition
		I2Cx->CR1 |= I2C_CR1_STOP;
		__enable_irq();

		// Read received byte N-1
		*buf++ = I2Cx->DR;

		// Wait for last byte received
		if (I2Cx_WaitEvent(I2Cx,I2C_EVENT_EV7) == I2C_ERROR) return I2C_ERROR;

		// Read last received byte
		*buf = (uint8_t)I2Cx->DR;
	}

	// Wait for a STOP flag
	if (I2Cx_WaitFlagReset(I2Cx,I2C_F_STOPF) == I2C_ERROR) return I2C_ERROR;

	return I2C_SUCCESS;
}

// Check if target device is ready for communication
// input:
//   I2Cx - I2C port
//   SlaveAddress - address of slave device
//   Trials - number of trials
// return:
//   I2C_ERROR if there was a timeout during I2C operations, I2C_SUCCESS otherwise
I2C_Status I2Cx_IsDeviceReady(I2C_TypeDef* I2Cx, uint8_t SlaveAddress, uint32_t Trials) {
	volatile uint32_t wait;
	uint16_t reg;

	do {
		// Initiate a START sequence
		I2Cx->CR1 |= I2C_CR1_START;
		// Wait for EV5
		if (I2Cx_WaitFlagSet(I2Cx,I2C_F_BUSY) == I2C_ERROR) return I2C_ERROR;

		// Send the slave address (EV5)
		I2Cx->DR = SlaveAddress & ~I2C_OAR1_ADD0; // Last bit be reset (transmitter mode)

		// Wait until ADDR or AF bit set
		wait = I2C_WAIT_TIMEOUT;
		do {
			reg = I2Cx->SR1;
		} while (!(reg & I2C_SR1_ADDR) && !(reg & I2C_SR1_AF) && --wait);

		// Check if device responded
		if (reg & I2C_SR1_ADDR) {
			// Generate a STOP condition
			I2Cx->CR1 |= I2C_CR1_STOP;
			// Clear the ADDR flag
			(void)I2Cx->SR1;
			(void)I2Cx->SR2;
			// Wait for a STOP flag
			if (I2Cx_WaitFlagReset(I2Cx,I2C_F_STOPF) == I2C_ERROR) return I2C_ERROR;
			// Wait until I2C bus is free
			if (I2Cx_WaitFlagReset(I2Cx,I2C_F_BUSY) == I2C_ERROR) return I2C_ERROR;

			return I2C_SUCCESS;
		} else {
			// Generate a STOP condition
			I2Cx->CR1 |= I2C_CR1_STOP;
			// Clear the AF flag
			I2Cx->SR1 &= ~I2C_SR1_AF;
			// Wait until I2C bus is free
			if (I2Cx_WaitFlagReset(I2Cx,I2C_F_BUSY) == I2C_ERROR) return I2C_ERROR;
		}
	} while (--Trials);

	return I2C_ERROR;
}
