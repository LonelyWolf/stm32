#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_i2c.h>
#include <delay.h>
#include <uart.h>


/* BMP180 defines */
#define BMP180_ADDR                     0xEE // BMP180 address
/* BMP180 registers */
#define BMP180_PROM_START_ADDR          0xAA // E2PROM calibration data start register
#define BMP180_PROM_DATA_LEN            22   // E2PROM length
#define BMP180_CHIP_ID_REG              0xD0 // Chip ID
#define BMP180_VERSION_REG              0xD1 // Version
#define BMP180_CTRL_MEAS_REG            0xF4 // Measurements control (OSS[7.6], SCO[5], CTL[4.0]
#define BMP180_ADC_OUT_MSB_REG          0xF6 // ADC out MSB  [7:0]
#define BMP180_ADC_OUT_LSB_REG          0xF7 // ADC out LSB  [7:0]
#define BMP180_ADC_OUT_XLSB_REG         0xF8 // ADC out XLSB [7:3]
#define BMP180_SOFT_RESET_REG           0xE0 // Soft reset control
/* BMP180 control values */
#define BMP180_T_MEASURE                0x2E // temperature measurement
#define BMP180_P0_MEASURE               0x34 // pressure measurement (OSS=0, 4.5ms)
#define BMP180_P1_MEASURE               0x74 // pressure measurement (OSS=1, 7.5ms)
#define BMP180_P2_MEASURE               0xB4 // pressure measurement (OSS=2, 13.5ms)
#define BMP180_P3_MEASURE               0xF4 // pressure measurement (OSS=3, 25.5ms)


typedef struct {
	int16_t AC1;
	int16_t AC2;
	int16_t AC3;
	uint16_t AC4;
	uint16_t AC5;
	uint16_t AC6;
	int16_t B1;
	int16_t B2;
	int16_t MB;
	int16_t MC;
	int16_t MD;
} BMP180_Calibration_TypeDef;

BMP180_Calibration_TypeDef BMP180_Calibration;


uint8_t BMP180_WriteReg(uint8_t reg, uint8_t value) {
	I2C_GenerateSTART(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C1,BMP180_ADDR,I2C_Direction_Transmitter); // Send slave address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_SendData(I2C1,reg); // Send register address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_SendData(I2C1,value); // Send register value
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_GenerateSTOP(I2C1,ENABLE);

	return value;
}

uint8_t BMP180_ReadReg(uint8_t reg) {
	uint8_t value;

	I2C_GenerateSTART(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C1,BMP180_ADDR,I2C_Direction_Transmitter); // Send slave address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_SendData(I2C1,reg); // Send register address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_GenerateSTART(I2C1,ENABLE); // Send repeated START condition (aka Re-START)
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C1,BMP180_ADDR,I2C_Direction_Receiver); // Send slave address for READ
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); // Wait for EV6
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	value = I2C_ReceiveData(I2C1); // Receive ChipID
	I2C_AcknowledgeConfig(I2C1,DISABLE); // Disable I2C acknowledgement
	I2C_GenerateSTOP(I2C1,ENABLE); // Send STOP condition
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)

	return value;
}

void BMP180_ReadCalibration(void) {
	uint8_t i;
	uint8_t buffer[BMP180_PROM_DATA_LEN];

	I2C_AcknowledgeConfig(I2C1,ENABLE); // Enable I2C acknowledge
	I2C_GenerateSTART(I2C1,ENABLE); // Send START condition
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C1,BMP180_ADDR,I2C_Direction_Transmitter); // Send slave address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_SendData(I2C1,BMP180_PROM_START_ADDR); // Send calibration first register address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_GenerateSTART(I2C1,ENABLE); // Send repeated START condition (aka Re-START)
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C1,BMP180_ADDR,I2C_Direction_Receiver); // Send slave address for READ
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); // Wait for EV6
	for (i = 0; i < BMP180_PROM_DATA_LEN-1; i++) {
		while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
		buffer[i] = I2C_ReceiveData(I2C1); // Receive byte
	}
	I2C_AcknowledgeConfig(I2C1,DISABLE); // Disable I2C acknowledgement
	I2C_GenerateSTOP(I2C1,ENABLE); // Send STOP condition
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	buffer[i] = I2C_ReceiveData(I2C1); // Receive last byte

	BMP180_Calibration.AC1 =  (int16_t)(buffer[0]  << 8) | buffer[1];
	BMP180_Calibration.AC2 =  (int16_t)(buffer[2]  << 8) | buffer[3];
	BMP180_Calibration.AC3 =  (int16_t)(buffer[4]  << 8) | buffer[5];
	BMP180_Calibration.AC4 = (uint16_t)(buffer[6]  << 8) | buffer[7];
	BMP180_Calibration.AC5 = (uint16_t)(buffer[8]  << 8) | buffer[9];
	BMP180_Calibration.AC6 = (uint16_t)(buffer[10] << 8) | buffer[11];
	BMP180_Calibration.B1  =  (int16_t)(buffer[12] << 8) | buffer[13];
	BMP180_Calibration.B2  =  (int16_t)(buffer[14] << 8) | buffer[15];
	BMP180_Calibration.MB  =  (int16_t)(buffer[16] << 8) | buffer[17];
	BMP180_Calibration.MC  =  (int16_t)(buffer[18] << 8) | buffer[19];
	BMP180_Calibration.MD  =  (int16_t)(buffer[20] << 8) | buffer[21];
}

uint16_t BMP180_Read_UT(void) {
	uint16_t UT;

	BMP180_WriteReg(BMP180_CTRL_MEAS_REG,BMP180_T_MEASURE);
	Delay_ms(5); // Wait for 4.5ms by datasheet (OSS=0)

	I2C_AcknowledgeConfig(I2C1,ENABLE); // Enable I2C acknowledge
	I2C_GenerateSTART(I2C1,ENABLE); // Send START condition
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C1,BMP180_ADDR,I2C_Direction_Transmitter); // Send slave address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_SendData(I2C1,BMP180_ADC_OUT_MSB_REG); // Send calibration first register address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_GenerateSTART(I2C1,ENABLE); // Send repeated START condition (aka Re-START)
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C1,BMP180_ADDR,I2C_Direction_Receiver); // Send slave address for READ
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); // Wait for EV6
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	UT = (uint16_t)I2C_ReceiveData(I2C1) << 8; // Receive MSB
	I2C_AcknowledgeConfig(I2C1,DISABLE); // Disable I2C acknowledgement
	I2C_GenerateSTOP(I2C1,ENABLE); // Send STOP condition
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	UT |= I2C_ReceiveData(I2C1); // Receive LSB

	return UT;
}

uint32_t BMP180_Read_PT(void) {
	uint32_t PT;

	BMP180_WriteReg(BMP180_CTRL_MEAS_REG,BMP180_P0_MEASURE); // OSS=0
	Delay_ms(5); // Wait for 4.5ms by datasheet (OSS=0)

	I2C_AcknowledgeConfig(I2C1,ENABLE); // Enable I2C acknowledge
	I2C_GenerateSTART(I2C1,ENABLE); // Send START condition
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C1,BMP180_ADDR,I2C_Direction_Transmitter); // Send slave address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_SendData(I2C1,BMP180_ADC_OUT_MSB_REG); // Send calibration first register address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_GenerateSTART(I2C1,ENABLE); // Send repeated START condition (aka Re-START)
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C1,BMP180_ADDR,I2C_Direction_Receiver); // Send slave address for READ
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); // Wait for EV6
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	PT = (uint32_t)I2C_ReceiveData(I2C1) << 16; // Receive MSB
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	PT |= (uint32_t)I2C_ReceiveData(I2C1) << 8; // Receive LSB
	I2C_AcknowledgeConfig(I2C1,DISABLE); // Disable I2C acknowledgement
	I2C_GenerateSTOP(I2C1,ENABLE); // Send STOP condition
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	PT += I2C_ReceiveData(I2C1); // Receive XLSB

	return PT;
}



int main(void)
{
	UART_Init();
	UART_SendStr("\nSTM32F103RET6 is online.\n");

	GPIO_InitTypeDef PORT;

	UART_SendStr("I2C init ... ");

	// Init I2C
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	PORT.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	PORT.GPIO_Mode = GPIO_Mode_AF_OD;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&PORT);

	I2C_InitTypeDef I2CInit;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE); // Enable I2C clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

	I2C_DeInit(I2C1); // I2C reset to initial state
	I2CInit.I2C_Mode = I2C_Mode_I2C; // I2C mode is I2C
	I2CInit.I2C_DutyCycle = I2C_DutyCycle_2; // I2C fast mode duty cycle (WTF is this?)
	I2CInit.I2C_OwnAddress1 = 1; // This device address (7-bit or 10-bit)
	I2CInit.I2C_Ack = I2C_Ack_Enable; // Acknowledgement enable
	I2CInit.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // choose 7-bit address for acknowledgement
	I2CInit.I2C_ClockSpeed = 400000; // 400kHz
	I2C_Cmd(I2C1,ENABLE); // Enable I2C
	I2C_Init(I2C1,&I2CInit); // Configure I2C

	while (I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY)); // Wait until I2C free
	UART_SendStr("ready.\n");

	UART_SendStr("ChipID: ");
	uint8_t ChipID = BMP180_ReadReg(BMP180_CHIP_ID_REG);
	UART_SendHex8(ChipID); UART_SendChar('\n');

	UART_SendStr("Version: ");
	uint8_t Version = BMP180_ReadReg(BMP180_VERSION_REG);
	UART_SendHex8(Version); UART_SendChar('\n');

	BMP180_ReadCalibration();

	UART_SendStr("E2PROM Calibration values:\n");
	UART_SendStr("  AC1 = "); UART_SendHex16(BMP180_Calibration.AC1); UART_SendChar('\n');
	UART_SendStr("  AC2 = "); UART_SendHex16(BMP180_Calibration.AC2); UART_SendChar('\n');
	UART_SendStr("  AC3 = "); UART_SendHex16(BMP180_Calibration.AC3); UART_SendChar('\n');
	UART_SendStr("  AC4 = "); UART_SendHex16(BMP180_Calibration.AC4); UART_SendChar('\n');
	UART_SendStr("  AC5 = "); UART_SendHex16(BMP180_Calibration.AC5); UART_SendChar('\n');
	UART_SendStr("  AC6 = "); UART_SendHex16(BMP180_Calibration.AC6); UART_SendChar('\n');
	UART_SendStr("  B1  = "); UART_SendHex16(BMP180_Calibration.B1);  UART_SendChar('\n');
	UART_SendStr("  B2  = "); UART_SendHex16(BMP180_Calibration.B2);  UART_SendChar('\n');
	UART_SendStr("  MB  = "); UART_SendHex16(BMP180_Calibration.MB);  UART_SendChar('\n');
	UART_SendStr("  MC  = "); UART_SendHex16(BMP180_Calibration.MC);  UART_SendChar('\n');
	UART_SendStr("  MD  = "); UART_SendHex16(BMP180_Calibration.MD);  UART_SendChar('\n');

	uint16_t u_temp;
	u_temp = BMP180_Read_UT();
	UART_SendStr("Uncompensated temperature = "); UART_SendHex16(u_temp); UART_SendChar('\n');

	uint32_t u_pres;
	u_pres = BMP180_Read_PT();
	UART_SendStr("Uncompensated pressure = "); UART_SendHex32(u_pres); UART_SendChar('\n');

	UART_SendStr("---------------------------\n");

    while(1);
}
