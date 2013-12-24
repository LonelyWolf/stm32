#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_i2c.h>
#include <lm75.h>


// Init I2C with SPI_Clock_Speed (in Hz)
uint8_t LM75_Init(uint32_t SPI_Clock_Speed) {
	GPIO_InitTypeDef PORT;

	// Init I2C
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	PORT.GPIO_Pin = I2C_SDA_PIN | I2C_SCL_PIN;
	PORT.GPIO_Mode = GPIO_Mode_AF_OD;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(I2C_GPIO_PORT,&PORT);

	I2C_InitTypeDef I2CInit;
	RCC_APB1PeriphClockCmd(I2C_CLOCK,ENABLE); // Enable I2C clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

	I2C_DeInit(I2C_PORT); // I2C reset to initial state
	I2CInit.I2C_Mode = I2C_Mode_I2C; // I2C mode is I2C
	I2CInit.I2C_DutyCycle = I2C_DutyCycle_2; // I2C fast mode duty cycle (WTF is this?)
	I2CInit.I2C_OwnAddress1 = 0x00; // This device address (7-bit or 10-bit)
	I2CInit.I2C_Ack = I2C_Ack_Enable; // Acknowledgment enable
	I2CInit.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // choose 7-bit address for acknowledgment
	I2CInit.I2C_ClockSpeed = SPI_Clock_Speed;
	I2C_Cmd(I2C_PORT,ENABLE); // Enable I2C
	I2C_Init(I2C_PORT,&I2CInit); // Configure I2C

	while (I2C_GetFlagStatus(I2C_PORT,I2C_FLAG_BUSY)); // Wait until I2C free

	return 0;
}

// Read 16-bit LM75 register
uint16_t LM75_ReadReg(uint8_t reg) {
	uint16_t value;

	I2C_AcknowledgeConfig(I2C_PORT,ENABLE); // Enable I2C acknowledgment
	I2C_GenerateSTART(I2C_PORT,ENABLE);
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,LM75_ADDR,I2C_Direction_Transmitter); // Send slave address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_SendData(I2C_PORT,reg); // Send register address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_GenerateSTART(I2C_PORT,ENABLE); // Send repeated START condition (aka Re-START)
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,LM75_ADDR,I2C_Direction_Receiver); // Send slave address for READ
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); // Wait for EV6
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	value = (I2C_ReceiveData(I2C_PORT) << 8); // Receive high byte
	I2C_AcknowledgeConfig(I2C_PORT,DISABLE); // Disable I2C acknowledgment
	I2C_GenerateSTOP(I2C_PORT,ENABLE); // Send STOP condition
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	value |= I2C_ReceiveData(I2C_PORT); // Receive low byte

	return value;
}

// Write 16-bit LM75 register
void LM75_WriteReg(uint8_t reg, uint16_t value) {
	I2C_AcknowledgeConfig(I2C_PORT,ENABLE); // Enable I2C acknowledgment
	I2C_GenerateSTART(I2C_PORT,ENABLE);
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,LM75_ADDR,I2C_Direction_Transmitter); // Send slave address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_SendData(I2C_PORT,reg); // Send register address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_SendData(I2C_PORT,(uint8_t)(value >> 8)); // Send high byte
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_SendData(I2C_PORT,(uint8_t)value); // Send low byte
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_GenerateSTOP(I2C_PORT,ENABLE);
}

// Read value from LM75 configuration register (8 bit)
uint8_t LM75_ReadConf(void) {
	uint8_t value;

	I2C_AcknowledgeConfig(I2C_PORT,ENABLE); // Enable I2C acknowledgment
	I2C_GenerateSTART(I2C_PORT,ENABLE);
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,LM75_ADDR,I2C_Direction_Transmitter); // Send slave address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_SendData(I2C_PORT,LM75_REG_CONF); // Send register address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_GenerateSTART(I2C_PORT,ENABLE); // Send repeated START condition (aka Re-START)
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,LM75_ADDR,I2C_Direction_Receiver); // Send slave address for READ
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); // Wait for EV6
	I2C_AcknowledgeConfig(I2C_PORT,DISABLE); // Disable I2C acknowledgment
	I2C_GenerateSTOP(I2C_PORT,ENABLE); // Send STOP condition
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	value = I2C_ReceiveData(I2C_PORT);

	return value;
}

// Write value to LM75 configuration register  (8 bit)
void LM75_WriteConf(uint8_t value) {
	I2C_AcknowledgeConfig(I2C_PORT,ENABLE); // Enable I2C acknowledgment
	I2C_GenerateSTART(I2C_PORT,ENABLE);
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,LM75_ADDR,I2C_Direction_Transmitter); // Send slave address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_SendData(I2C_PORT,LM75_REG_CONF); // Send register address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_SendData(I2C_PORT,value);
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_GenerateSTOP(I2C_PORT,ENABLE);
}

// Set LM75 shutdown mode
// newstate:
//    ENABLE = put LM75 into powerdown mode
//    DISABLE = wake up LM75
void LM75_Shutdown(FunctionalState newstate) {
	uint8_t value;

	value = LM75_ReadConf();
	LM75_WriteConf(newstate == ENABLE ? value | 0x01 : value & 0xFE);
}

// Read temperature readings from LM75 in decimal format
// IIIF where:
//   III - integer part
//   F   - fractional part
// e.g. 355 means 35.5C
int16_t LM75_Temperature(void) {
	uint16_t raw;
	int16_t temp;

	raw = LM75_ReadReg(LM75_REG_TEMP) >> 7;
	if (raw & 0x0100) {
		// Negative temperature
		temp = -10 * (((~(uint8_t)(raw & 0xFE) + 1) & 0x7F) >> 1) - (raw & 0x01) * 5;
	} else {
		// Positive temperature
		temp = ((raw & 0xFE) >> 1) * 10 + (raw & 0x01) * 5;
	}

	return temp;
}
