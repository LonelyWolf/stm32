#include <stm32l1xx_gpio.h>
#include <stm32l1xx_rcc.h>
#include <stm32l1xx_i2c.h>
#include <i2c.h>


// Init I2C2 peripheral
// input:
//   Clock - I2C speed (Hz)
// return:
//   I2C_ERROR if there was a timeout during I2C initialization, I2C_SUCCESS otherwise
I2C_Status I2C2_Init(uint32_t Clock) {
	GPIO_InitTypeDef PORT;
	I2C_InitTypeDef I2CInit;
	volatile uint32_t TimeOut;

	// Init I2C2
	RCC_AHBPeriphClockCmd(I2C2_PERIPH,ENABLE);
	RCC_APB1PeriphClockCmd(I2C2_CLOCK,ENABLE);
	PORT.GPIO_Speed = GPIO_Speed_40MHz;
	PORT.GPIO_OType = GPIO_OType_PP;
	PORT.GPIO_Mode  = GPIO_Mode_AF;
	PORT.GPIO_Pin   = I2C2_SCL_PIN | I2C2_SDA_PIN;
	PORT.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_Init(I2C2_GPIO_PORT,&PORT);

	// Alternative functions of GPIO pins
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_I2C2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_I2C2);

	I2C_DeInit(I2C2_PORT); // I2C reset to initial state
	I2CInit.I2C_Ack = I2C_Ack_Enable;  // Acknowledgment enable
	I2CInit.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // choose 7-bit address for acknowledgment
	I2CInit.I2C_ClockSpeed = Clock;
	I2CInit.I2C_DutyCycle = I2C_DutyCycle_2; // I2C fast mode duty cycle (WTF is this?)
	I2CInit.I2C_Mode = I2C_Mode_I2C; // I2C mode is I2C
	I2CInit.I2C_OwnAddress1 = 1; // This device address (7-bit or 10-bit)
	I2C_Cmd(I2C2_PORT,ENABLE); // Enable I2C2
	I2C_Init(I2C2_PORT,&I2CInit); // Configure I2C2

	// Wait until I2C free
	TimeOut = 0;
	while (I2C_GetFlagStatus(I2C2_PORT,I2C_FLAG_BUSY) && TimeOut < I2C_WAIT_TIMEOUT) TimeOut++;
	if (TimeOut == I2C_WAIT_TIMEOUT) return I2C_ERROR;

	return I2C_SUCCESS;
}

// Send data to I2C port
// input:
//   buf - pointer to the data buffer
//   nbytes - number of bytes to transmit
//   SlaveAddress - address of slave device
// return:
//   I2C_ERROR if there was a timeout during I2C operations, I2C_SUCCESS otherwise
I2C_Status I2C2_Write(const uint8_t* buf, uint32_t nbytes, uint8_t SlaveAddress, I2C_STOP_TypeDef stop) {
	volatile uint32_t TimeOut;

	// Initiate start sequence
	I2C_GenerateSTART(I2C2_PORT,ENABLE);
	// Wait for EV5
	TimeOut = 0;
	while (!I2C_CheckEvent(I2C2_PORT,I2C_EVENT_MASTER_MODE_SELECT) && TimeOut < I2C_WAIT_TIMEOUT) TimeOut++;
	if (TimeOut == I2C_WAIT_TIMEOUT) return I2C_ERROR;

	// Send slave address EV5
	I2C_Send7bitAddress(I2C2_PORT,SlaveAddress,I2C_Direction_Transmitter);
	// Wait for EV6
	TimeOut = 0;
	while (!I2C_CheckEvent(I2C2_PORT,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) && TimeOut < I2C_WAIT_TIMEOUT) TimeOut++;
	if (TimeOut == I2C_WAIT_TIMEOUT) return I2C_ERROR;

	// Send first byte EV8
	I2C_SendData(I2C2_PORT,*buf++);

	while (--nbytes) {
		// Wait for BTF flag set
		TimeOut = 0;
		while (!I2C_GetFlagStatus(I2C2_PORT,I2C_FLAG_BTF) && TimeOut < I2C_WAIT_TIMEOUT) TimeOut++;
		if (TimeOut == I2C_WAIT_TIMEOUT) return I2C_ERROR;
		I2C_SendData(I2C2_PORT,*buf++);
	}
	// Wait for BTF flag set
	TimeOut = 0;
	while (!I2C_GetFlagStatus(I2C2_PORT,I2C_FLAG_BTF) && TimeOut < I2C_WAIT_TIMEOUT) TimeOut++;
	if (TimeOut == I2C_WAIT_TIMEOUT) return I2C_ERROR;

	if (stop == I2C_STOP) {
		// End transmission
		I2C_GenerateSTOP(I2C2_PORT,ENABLE);
		// Wait for STOP flag
		TimeOut = 0;
		while (I2C_GetFlagStatus(I2C2_PORT,I2C_FLAG_STOPF) && TimeOut < I2C_WAIT_TIMEOUT) TimeOut++;
		if (TimeOut == I2C_WAIT_TIMEOUT) return I2C_ERROR;
	}

	return I2C_SUCCESS;
}

// Read data from I2C port
// input:
//   buf - pointer to data buffer
//   nbytes - number of bytes to receive
//   SlaveAddress - address of slave device
// return:
//   I2C_ERROR if there was a timeout during I2C operations, I2C_SUCCESS otherwise
I2C_Status I2C2_Read(uint8_t *buf, uint32_t nbytes, uint8_t SlaveAddress) {
	volatile uint32_t TimeOut;

	I2C_AcknowledgeConfig(I2C2_PORT,ENABLE); // Enable Acknowledgment
	I2C_NACKPositionConfig(I2C2_PORT,I2C_NACKPosition_Current); // Clear POS flag

	// Initiate start sequence
	I2C_GenerateSTART(I2C2_PORT,ENABLE);
	// Wait for EV5
	TimeOut = 0;
	while (!I2C_CheckEvent(I2C2_PORT,I2C_EVENT_MASTER_MODE_SELECT) && TimeOut < I2C_WAIT_TIMEOUT) TimeOut++;
	if (TimeOut == I2C_WAIT_TIMEOUT) return I2C_ERROR;

	// Send slave address
	I2C_Send7bitAddress(I2C2_PORT,SlaveAddress,I2C_Direction_Receiver); // Send slave address
	// Wait for EV6
	TimeOut = 0;
	while (!I2C_GetFlagStatus(I2C2_PORT,I2C_FLAG_ADDR) && TimeOut < I2C_WAIT_TIMEOUT) TimeOut++;
	if (TimeOut == I2C_WAIT_TIMEOUT) return I2C_ERROR;

	// There are can be three cases:
	//   read 1 byte
	//   read 2 bytes
	//   read more than 2 bytes
	if (nbytes == 1) {
		// Receive 1 byte (AN2824 figure 2)
		I2C_AcknowledgeConfig(I2C2_PORT,DISABLE); // Disable I2C acknowledgment (clear ACK bit)
		// EV6_1 must be atomic operation (AN2824)
		__disable_irq();
		(void) I2C2_PORT->SR1; // Clear ADDR
		(void) I2C2_PORT->SR2;
		I2C_GenerateSTOP(I2C2_PORT,ENABLE); // Send STOP condition
		__enable_irq();
		// Wait for RxNE flag (receive buffer not empty) EV7
		TimeOut = 0;
		while (!I2C_GetFlagStatus(I2C2_PORT,I2C_FLAG_RXNE) && TimeOut < I2C_WAIT_TIMEOUT) TimeOut++;
		if (TimeOut == I2C_WAIT_TIMEOUT) return I2C_ERROR;
		*buf++ = I2C_ReceiveData(I2C2_PORT); // Receive byte
	} else if (nbytes == 2) {
		// Receive 2 bytes (AN2824 figure 2)
		I2C_NACKPositionConfig(I2C2_PORT,I2C_NACKPosition_Next); // Set POS flag
		// EV6_1 must be atomic operation (AN2824)
		__disable_irq();
		(void) I2C2_PORT->SR2; // Clear ADDR
		I2C_AcknowledgeConfig(I2C2_PORT,DISABLE); // Clear ACK bit
		__enable_irq();
		// Wait for BTF flag set (byte transfer finished) EV7_3
		TimeOut = 0;
		while (!I2C_GetFlagStatus(I2C2_PORT,I2C_FLAG_BTF) && TimeOut < I2C_WAIT_TIMEOUT) TimeOut++;
		if (TimeOut == I2C_WAIT_TIMEOUT) return I2C_ERROR;

		__disable_irq();
		I2C_GenerateSTOP(I2C2_PORT,ENABLE); // Send STOP condition
		*buf++ = I2C2_PORT->DR; // Faster than call I2C_ReceiveData()
		__enable_irq();

		*buf++ = I2C2_PORT->DR; // Read second received byte
	} else {
		// Receive more than 2 bytes (AN2824 figure 1)
		(void) I2C2_PORT->SR2; // Clear ADDR flag
		while (nbytes-- != 3) {
			// Wait for BTF (cannot guarantee 1 transfer completion time)
			TimeOut = 0;
			while (!I2C_GetFlagStatus(I2C2_PORT,I2C_FLAG_BTF) && TimeOut < I2C_WAIT_TIMEOUT) TimeOut++;
			if (TimeOut == I2C_WAIT_TIMEOUT) return I2C_ERROR;
			*buf++ = I2C_ReceiveData(I2C2_PORT);
		}
		// Wait for BTF flag set (byte transfer finished) EV7_2
		TimeOut = 0;
		while (!I2C_GetFlagStatus(I2C2_PORT,I2C_FLAG_BTF) && TimeOut < I2C_WAIT_TIMEOUT) TimeOut++;
		if (TimeOut == I2C_WAIT_TIMEOUT) return I2C_ERROR;

		I2C_AcknowledgeConfig(I2C2_PORT,DISABLE); // Clear ACK bit

		__disable_irq();
		*buf++ = I2C_ReceiveData(I2C2_PORT); // Receive byte N-2
		I2C_GenerateSTOP(I2C2_PORT,ENABLE); // Send STOP condition
		__enable_irq();

		*buf++ = I2C_ReceiveData(I2C2_PORT); // Receive byte N-1
		// Wait for last byte received
		TimeOut = 0;
		while (!I2C_CheckEvent(I2C2_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED) && TimeOut < I2C_WAIT_TIMEOUT) TimeOut++;
		if (TimeOut == I2C_WAIT_TIMEOUT) return I2C_ERROR;
		*buf++ = I2C_ReceiveData(I2C2_PORT); // Receive last byte

		nbytes = 0;
	}

	// Wait for STOP flag
	TimeOut = 0;
	while (I2C_GetFlagStatus(I2C2_PORT,I2C_FLAG_STOPF) && TimeOut < I2C_WAIT_TIMEOUT) TimeOut++;
	if (TimeOut == I2C_WAIT_TIMEOUT) return I2C_ERROR;

	return I2C_SUCCESS;
}
