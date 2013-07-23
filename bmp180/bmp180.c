#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_i2c.h>
#include <delay.h>
#include <bmp180.h>


uint8_t BMP180_Init(uint32_t SPI_Clock_Speed) {
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
	I2CInit.I2C_OwnAddress1 = 1; // This device address (7-bit or 10-bit)
	I2CInit.I2C_Ack = I2C_Ack_Enable; // Acknowledgment enable
	I2CInit.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // choose 7-bit address for acknowledgment
	I2CInit.I2C_ClockSpeed = SPI_Clock_Speed;
	I2C_Cmd(I2C_PORT,ENABLE); // Enable I2C
	I2C_Init(I2C_PORT,&I2CInit); // Configure I2C

	while (I2C_GetFlagStatus(I2C_PORT,I2C_FLAG_BUSY)); // Wait until I2C free

	return 0;
}

void BMP180_Reset() {
	BMP180_WriteReg(BMP180_SOFT_RESET_REG,0xb6); // Do software reset
}

uint8_t BMP180_WriteReg(uint8_t reg, uint8_t value) {
	I2C_GenerateSTART(I2C_PORT,ENABLE);
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,BMP180_ADDR,I2C_Direction_Transmitter); // Send slave address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_SendData(I2C_PORT,reg); // Send register address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_SendData(I2C_PORT,value); // Send register value
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_GenerateSTOP(I2C_PORT,ENABLE);

	return value;
}

uint8_t BMP180_ReadReg(uint8_t reg) {
	uint8_t value;

	I2C_GenerateSTART(I2C_PORT,ENABLE);
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,BMP180_ADDR,I2C_Direction_Transmitter); // Send slave address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_SendData(I2C_PORT,reg); // Send register address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_GenerateSTART(I2C_PORT,ENABLE); // Send repeated START condition (aka Re-START)
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,BMP180_ADDR,I2C_Direction_Receiver); // Send slave address for READ
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); // Wait for EV6
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	value = I2C_ReceiveData(I2C_PORT); // Receive ChipID
	I2C_AcknowledgeConfig(I2C_PORT,DISABLE); // Disable I2C acknowledgment
	I2C_GenerateSTOP(I2C_PORT,ENABLE); // Send STOP condition
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)

	return value;
}

void BMP180_ReadCalibration(void) {
	uint8_t i;
	uint8_t buffer[BMP180_PROM_DATA_LEN];

	I2C_AcknowledgeConfig(I2C_PORT,ENABLE); // Enable I2C acknowledge
	I2C_GenerateSTART(I2C_PORT,ENABLE); // Send START condition
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,BMP180_ADDR,I2C_Direction_Transmitter); // Send slave address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_SendData(I2C_PORT,BMP180_PROM_START_ADDR); // Send calibration first register address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_GenerateSTART(I2C_PORT,ENABLE); // Send repeated START condition (aka Re-START)
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,BMP180_ADDR,I2C_Direction_Receiver); // Send slave address for READ
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); // Wait for EV6
	for (i = 0; i < BMP180_PROM_DATA_LEN-1; i++) {
		while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
		buffer[i] = I2C_ReceiveData(I2C_PORT); // Receive byte
	}
	I2C_AcknowledgeConfig(I2C_PORT,DISABLE); // Disable I2C acknowledgment
	I2C_GenerateSTOP(I2C_PORT,ENABLE); // Send STOP condition
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	buffer[i] = I2C_ReceiveData(I2C_PORT); // Receive last byte

	BMP180_Calibration.AC1 = (buffer[0]  << 8) | buffer[1];
	BMP180_Calibration.AC2 = (buffer[2]  << 8) | buffer[3];
	BMP180_Calibration.AC3 = (buffer[4]  << 8) | buffer[5];
	BMP180_Calibration.AC4 = (buffer[6]  << 8) | buffer[7];
	BMP180_Calibration.AC5 = (buffer[8]  << 8) | buffer[9];
	BMP180_Calibration.AC6 = (buffer[10] << 8) | buffer[11];
	BMP180_Calibration.B1  = (buffer[12] << 8) | buffer[13];
	BMP180_Calibration.B2  = (buffer[14] << 8) | buffer[15];
	BMP180_Calibration.MB  = (buffer[16] << 8) | buffer[17];
	BMP180_Calibration.MC  = (buffer[18] << 8) | buffer[19];
	BMP180_Calibration.MD  = (buffer[20] << 8) | buffer[21];
}

uint16_t BMP180_Read_UT(void) {
	uint16_t UT;

	BMP180_WriteReg(BMP180_CTRL_MEAS_REG,BMP180_T_MEASURE);
	Delay_ms(6); // Wait for 4.5ms by datasheet

	I2C_AcknowledgeConfig(I2C_PORT,ENABLE); // Enable I2C acknowledge
	I2C_GenerateSTART(I2C_PORT,ENABLE); // Send START condition
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,BMP180_ADDR,I2C_Direction_Transmitter); // Send slave address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_SendData(I2C_PORT,BMP180_ADC_OUT_MSB_REG); // Send ADC MSB register address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_GenerateSTART(I2C_PORT,ENABLE); // Send repeated START condition (aka Re-START)
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,BMP180_ADDR,I2C_Direction_Receiver); // Send slave address for READ
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); // Wait for EV6
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	UT = (uint16_t)I2C_ReceiveData(I2C_PORT) << 8; // Receive MSB
	I2C_AcknowledgeConfig(I2C_PORT,DISABLE); // Disable I2C acknowledgment
	I2C_GenerateSTOP(I2C_PORT,ENABLE); // Send STOP condition
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	UT |= I2C_ReceiveData(I2C_PORT); // Receive LSB

	return UT;
}

uint32_t BMP180_Read_PT(uint8_t oss) {
	uint32_t PT;
	uint8_t cmd,delay;

	switch(oss) {
	case 0:
		cmd = BMP180_P0_MEASURE;
		delay   = 6;
		break;
	case 1:
		cmd = BMP180_P1_MEASURE;
		delay   = 9;
		break;
	case 2:
		cmd = BMP180_P2_MEASURE;
		delay   = 15;
		break;
	case 3:
		cmd = BMP180_P3_MEASURE;
		delay   = 27;
		break;
	}

	BMP180_WriteReg(BMP180_CTRL_MEAS_REG,cmd);
	Delay_ms(delay);
//	BMP180_WriteReg(0xf4,0x34);
//	Delay_ms(27);

	I2C_AcknowledgeConfig(I2C_PORT,ENABLE); // Enable I2C acknowledge
	I2C_GenerateSTART(I2C_PORT,ENABLE); // Send START condition
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,BMP180_ADDR,I2C_Direction_Transmitter); // Send slave address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_SendData(I2C_PORT,BMP180_ADC_OUT_MSB_REG); // Send ADC MSB register address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_GenerateSTART(I2C_PORT,ENABLE); // Send repeated START condition (aka Re-START)
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,BMP180_ADDR,I2C_Direction_Receiver); // Send slave address for READ
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); // Wait for EV6
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	PT = (uint32_t)I2C_ReceiveData(I2C_PORT) << 16; // Receive MSB
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	PT |= (uint32_t)I2C_ReceiveData(I2C_PORT) << 8; // Receive LSB
	I2C_AcknowledgeConfig(I2C_PORT,DISABLE); // Disable I2C acknowledgment
	I2C_GenerateSTOP(I2C_PORT,ENABLE); // Send STOP condition
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	PT |= (uint32_t)I2C_ReceiveData(I2C_PORT); // Receive XLSB

	return PT >> (8 - oss);
}

int16_t BMP180_Calc_RT(uint16_t UT) {
	BMP180_Calibration.B5  = (((int32_t)UT - (int32_t)BMP180_Calibration.AC6) * (int32_t)BMP180_Calibration.AC5) >> 15;
	BMP180_Calibration.B5 += ((int32_t)BMP180_Calibration.MC << 11) / (BMP180_Calibration.B5 + BMP180_Calibration.MD);

	return (BMP180_Calibration.B5 + 8) >> 4;
}

int32_t BMP180_Calc_RP(uint32_t UP, uint8_t oss) {
	int32_t B3,B6,X3,p;
	uint32_t B4,B7;

	B6 = BMP180_Calibration.B5 - 4000;
	X3 = ((BMP180_Calibration.B2 * ((B6 * B6) >> 12)) >> 11) + ((BMP180_Calibration.AC2 * B6) >> 11);
	B3 = (((((int32_t)BMP180_Calibration.AC1) * 4 + X3) << oss) + 2) >> 2;
	X3 = (((BMP180_Calibration.AC3 * B6) >> 13) + ((BMP180_Calibration.B1 * ((B6 * B6) >> 12)) >> 16) + 2) >> 2;
	B4 = (BMP180_Calibration.AC4 * (uint32_t)(X3 + 32768)) >> 15;
	B7 = ((uint32_t)UP - B3) * (50000 >> oss);
	if (B7 < 0x80000000) p = (B7 << 1) / B4; else p = (B7 / B4) << 1;
	p += ((((p >> 8) * (p >> 8) * BMP180_PARAM_MG) >> 16) + ((BMP180_PARAM_MH * p) >> 16) + BMP180_PARAM_MI) >> 4;

	return p;
}

// Fast integer Pa -> mmHg conversion (Pascals to millimeters of mercury)
int32_t BMP180_kpa_to_mmhg(int32_t Pa) {
	return (Pa * 75) / 10000;
}
