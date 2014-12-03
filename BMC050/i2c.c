#include <stm32l1xx_gpio.h>
#include <stm32l1xx_rcc.h>
#include <stm32l1xx_i2c.h>

#include <i2c.h>


// Init I2C2 peripheral
// input:
//   I2Cx - I2C port
//   Clock - I2C speed (Hz)
// return:
//   I2C_ERROR if there was a timeout during I2C initialization, I2C_SUCCESS otherwise
I2C_Status I2Cx_Init(I2C_TypeDef* I2Cx, uint32_t Clock) {
	GPIO_InitTypeDef PORT;
	I2C_InitTypeDef I2CInit;
	volatile uint32_t wait;

	// Initialize I2C GPIO peripherals
	if (I2Cx == I2C1) {
		// I2C1
		RCC_AHBPeriphClockCmd(I2C1_PERIPH,ENABLE);
		RCC_APB1PeriphClockCmd(I2C1_CLOCK,ENABLE);
		GPIO_PinAFConfig(I2C1_GPIO_PORT,I2C1_SCL_PIN_SRC,GPIO_AF_I2C1);
		GPIO_PinAFConfig(I2C1_GPIO_PORT,I2C1_SDA_PIN_SRC,GPIO_AF_I2C1);
		PORT.GPIO_Pin = I2C1_SCL_PIN | I2C1_SDA_PIN;
		// Reset the I2C1 peripheral to initial state
	    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1,ENABLE);
	    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1,DISABLE);
	} else {
		// I2C2
		RCC_AHBPeriphClockCmd(I2C2_PERIPH,ENABLE);
		RCC_APB1PeriphClockCmd(I2C2_CLOCK,ENABLE);
		GPIO_PinAFConfig(I2C2_GPIO_PORT,I2C2_SCL_PIN_SRC,GPIO_AF_I2C2);
		GPIO_PinAFConfig(I2C2_GPIO_PORT,I2C2_SDA_PIN_SRC,GPIO_AF_I2C2);
		PORT.GPIO_Pin = I2C2_SCL_PIN | I2C2_SDA_PIN;
		// Reset the I2C2 peripheral to initial state
	    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2,ENABLE);
	    RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C2,DISABLE);
	}
	PORT.GPIO_Speed = GPIO_Speed_40MHz;
	PORT.GPIO_OType = GPIO_OType_OD;
	PORT.GPIO_Mode  = GPIO_Mode_AF;
	PORT.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(I2C2_GPIO_PORT,&PORT);

	// Initialize the I2C peripheral
	I2CInit.I2C_Ack = I2C_Ack_Enable;  // Acknowledgment enable
	I2CInit.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // choose 7-bit address for acknowledgment
	I2CInit.I2C_ClockSpeed = Clock;
	I2CInit.I2C_DutyCycle = I2C_DutyCycle_2; // I2C fast mode duty cycle (WTF is this?)
	I2CInit.I2C_Mode = I2C_Mode_I2C; // I2C mode is I2C
	I2CInit.I2C_OwnAddress1 = 1; // This device address (7-bit or 10-bit)
	I2C_Init(I2Cx,&I2CInit); // Configure I2C
	I2Cx->CR1 |= I2C_CR1_PE; // Enable I2C

	// Wait until I2C free
	wait = I2C_WAIT_TIMEOUT;
	while ((I2Cx->SR2 & I2C_SR2_BUSY) && --wait);
	if (!wait) return I2C_ERROR;

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
	volatile uint32_t wait;

	// Initiate a START sequence
	I2Cx->CR1 |= I2C_CR1_START;
	// Wait for EV5
	wait = I2C_WAIT_TIMEOUT;
	while (!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_MODE_SELECT) && --wait);
	if (!wait) return I2C_ERROR;

	// Send the slave address (EV5)
    I2Cx->DR = SlaveAddress & 0xfe; // Last bit should be reset (transmitter mode)

	// Wait for EV6
	wait = I2C_WAIT_TIMEOUT;
	while (!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && --wait);
	if (!wait) return I2C_ERROR;

	// Send first byte (EV8)
	I2Cx->DR = *buf++;

	while (--nbytes) {
		// Wait for BTF flag set
		wait = I2C_WAIT_TIMEOUT;
		while (!I2C_GetFlagStatus(I2Cx,I2C_FLAG_BTF) && --wait);
		if (!wait) return I2C_ERROR;
		I2Cx->DR = *buf++;
	}
	// Wait for BTF flag set
	wait = I2C_WAIT_TIMEOUT;
	while (!I2C_GetFlagStatus(I2Cx,I2C_FLAG_BTF) && --wait);
	if (!wait) return I2C_ERROR;

	// Transmission end
	if (stop == I2C_STOP) {
		// Generate a STOP condition
		I2Cx->CR1 |= I2C_CR1_STOP;
		// Wait for a STOP flag
		wait = I2C_WAIT_TIMEOUT;
		while (I2C_GetFlagStatus(I2Cx,I2C_FLAG_STOPF) && --wait);
		if (!wait) return I2C_ERROR;
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
	volatile uint32_t wait;

	// Enable Acknowledgment
	I2Cx->CR1 |= I2C_CR1_ACK;
	// Clear POS flag
	I2Cx->CR1 &= ~I2C_CR1_POS; // NACK position current

	// Initiate START sequence
	I2Cx->CR1 |= I2C_CR1_START;
	// Wait for EV5
	wait = I2C_WAIT_TIMEOUT;
	while (!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_MODE_SELECT) && --wait);
	if (!wait) return I2C_ERROR;

	// Send slave address (EV5)
	I2Cx->DR = SlaveAddress | 0x01; // Last bit should set (receiver mode)

	// Wait for EV6
	wait = I2C_WAIT_TIMEOUT;
	while (!I2C_GetFlagStatus(I2Cx,I2C_FLAG_ADDR) && --wait);
	if (!wait) return I2C_ERROR;

	// There are can be three cases:
	//   read 1 byte
	//   read 2 bytes
	//   read more than 2 bytes
	if (nbytes == 1) {
		// Receive 1 byte (AN2824 figure 2)
		I2Cx->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_ACK); // Disable I2C acknowledgment
		// EV6_1 must be atomic operation (AN2824)
		__disable_irq();
		(void) I2Cx->SR1; // Clear ADDR
		(void) I2Cx->SR2;
		I2Cx->CR1 |= I2C_CR1_STOP; // Generate a STOP condition
		__enable_irq();
		// Wait for RxNE flag (receive buffer not empty) EV7
		wait = I2C_WAIT_TIMEOUT;
		while (!I2C_GetFlagStatus(I2Cx,I2C_FLAG_RXNE) && --wait);
		if (!wait) return I2C_ERROR;
		*buf++ = (uint8_t)I2Cx->DR; // Receive byte
	} else if (nbytes == 2) {
		// Receive 2 bytes (AN2824 figure 2)
		I2Cx->CR1 |= I2C_CR1_POS; // Set POS flag (NACK position next)
		// EV6_1 must be atomic operation (AN2824)
		__disable_irq();
		(void) I2Cx->SR2; // Clear ADDR
		I2Cx->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_ACK); // Disable I2C acknowledgment
		__enable_irq();
		// Wait for BTF flag set (byte transfer finished) EV7_3
		wait = I2C_WAIT_TIMEOUT;
		while (!I2C_GetFlagStatus(I2Cx,I2C_FLAG_BTF) && --wait);
		if (!wait) return I2C_ERROR;

		__disable_irq();
		I2Cx->CR1 |= I2C_CR1_STOP; // Generate a STOP condition
		*buf++ = (uint8_t)I2Cx->DR; // Faster than call I2C_ReceiveData()
		__enable_irq();

		*buf++ = (uint8_t)I2Cx->DR; // Read second received byte
	} else {
		// Receive more than 2 bytes (AN2824 figure 1)
		(void) I2Cx->SR2; // Clear ADDR flag
		while (nbytes-- != 3) {
			// Wait for BTF (cannot guarantee 1 transfer completion time)
			wait = I2C_WAIT_TIMEOUT;
			while (!I2C_GetFlagStatus(I2Cx,I2C_FLAG_BTF) && --wait);
			if (!wait) return I2C_ERROR;
			*buf++ = (uint8_t)I2Cx->DR;
		}
		// Wait for BTF flag set (byte transfer finished) EV7_2
		wait = I2C_WAIT_TIMEOUT;
		while (!I2C_GetFlagStatus(I2Cx,I2C_FLAG_BTF) && --wait);
		if (!wait) return I2C_ERROR;

		// Disable the I2C acknowledgment
		I2Cx->CR1 &= (uint16_t)~((uint16_t)I2C_CR1_ACK);

		__disable_irq();
		*buf++ = (uint8_t)I2Cx->DR; // Receive byte N-2
		I2Cx->CR1 |= I2C_CR1_STOP; // Generate a STOP condition
		__enable_irq();

		*buf++ = I2Cx->DR; // Receive byte N-1
		// Wait for last byte received
		wait = I2C_WAIT_TIMEOUT;
		while (!I2C_CheckEvent(I2Cx,I2C_EVENT_MASTER_BYTE_RECEIVED) && --wait);
		if (!wait) return I2C_ERROR;
		*buf++ = (uint8_t)I2Cx->DR; // Receive last byte

		nbytes = 0;
	}

	// Wait for a STOP flag
	wait = I2C_WAIT_TIMEOUT;
	while (I2C_GetFlagStatus(I2Cx,I2C_FLAG_STOPF) && --wait);
	if (!wait) return I2C_ERROR;

	return I2C_SUCCESS;
}
