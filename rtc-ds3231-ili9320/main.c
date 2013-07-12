#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_i2c.h>
#include <stm32f10x_exti.h>
#include <stm32f10x_usart.h>
#include <misc.h>
#include <string.h>
#include <delay.h>
#include <ili9320.h>


#define DS3231_addr     0xD0 // I2C 7-bit slave address shifted for 1 bit to the left
#define DS3231_seconds  0x00 // DS3231 seconds address
#define DS3231_control  0x0E // DS3231 control register address
#define DS3231_tmp_MSB  0x11 // DS3231 temperature MSB


#include <digits.h>


char *DOW[] = { "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday" };


volatile uint8_t tm_ready = 0;
volatile uint8_t conf_mode = 0;


// All DS3231 registers
typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t day;
	uint8_t date;
	uint8_t month;
	uint8_t year;
	uint8_t alarm1_secconds;
	uint8_t alarm1_minutes;
	uint8_t alarm1_hours;
	uint8_t alarm1_day;
	uint8_t alarm1_date;
	uint8_t alarm2_minutes;
	uint8_t alarm2_hours;
	uint8_t alarm2_day;
	uint8_t alarm2_date;
	uint8_t control;
	uint8_t status;
	uint8_t aging;
	uint8_t msb_temp;
	uint8_t lsb_temp;
} DS3231_registers_TypeDef;

// DS3231 date
typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t day_of_week;
	uint8_t date;
	uint8_t month;
	uint8_t year;
} DS3231_date_TypeDef;

// Human Readable Format date
typedef struct {
	uint8_t  Seconds;
	uint8_t  Minutes;
	uint8_t  Hours;
	uint8_t  Day;
	uint8_t  Month;
	uint16_t Year;
	uint8_t  DOW;
} HRF_date_TypeDef;


void EXTI1_IRQHandler(void) {
	if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
		EXTI_ClearITPendingBit(EXTI_Line1);

		GPIOC->ODR ^= GPIO_Pin_9; // Toggle LED
		GPIOC->ODR ^= GPIO_Pin_8; // Toggle LED

		tm_ready = 1;
	}
}

void DS3231_ReadDateRAW(DS3231_date_TypeDef* date) {
	unsigned int i;
	char buffer[7];

	I2C_AcknowledgeConfig(I2C1,ENABLE); // Enable I2C acknowledge

	I2C_GenerateSTART(I2C1,ENABLE); // Send START condition
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5

	I2C_Send7bitAddress(I2C1,DS3231_addr,I2C_Direction_Transmitter); // Send DS3231 slave address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6

	I2C_SendData(I2C1,DS3231_seconds); // Send DS3231 seconds register address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8

	I2C_GenerateSTART(I2C1,ENABLE); // Send repeated START condition (aka Re-START)
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5

	I2C_Send7bitAddress(I2C1,DS3231_addr,I2C_Direction_Receiver); // Send DS3231 slave address for READ
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); // Wait for EV6

	for (i = 0; i < 6; i++) {
		while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
		buffer[i] = I2C_ReceiveData(I2C1); // Receive byte
	}

	I2C_AcknowledgeConfig(I2C1,DISABLE); // Disable I2C acknowledgement
	I2C_GenerateSTOP(I2C1,ENABLE); // Send STOP condition

	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	buffer[i] = I2C_ReceiveData(I2C1); // Receive last byte

	memcpy(date,&buffer[0],7);
}

void DS3231_WriteDateRAW(DS3231_date_TypeDef* date) {
	unsigned int i;
	char buffer[7];

	memcpy(&buffer[0],date,7);

	I2C_AcknowledgeConfig(I2C1,ENABLE); // Enable I2C acknowledge

	I2C_GenerateSTART(I2C1,ENABLE); // Send START condition
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5

	I2C_Send7bitAddress(I2C1,DS3231_addr,I2C_Direction_Transmitter); // Send DS3231 slave address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6

	I2C_SendData(I2C1,DS3231_seconds); // Send DS3231 seconds register address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8

	for (i = 0; i < 7; i++) {
		I2C_SendData(I2C1,buffer[i]); // Send DS3231 seconds register address
		while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	}

	I2C_GenerateSTOP(I2C1,ENABLE);
}

void DS3231_ReadDate(HRF_date_TypeDef* hrf_date) {
	DS3231_date_TypeDef raw_date;

	DS3231_ReadDateRAW(&raw_date);

	hrf_date->Seconds = (raw_date.seconds >> 4) * 10 + (raw_date.seconds & 0x0f);
	hrf_date->Minutes = (raw_date.minutes >> 4) * 10 + (raw_date.minutes & 0x0f);
	hrf_date->Hours   = (raw_date.hours   >> 4) * 10 + (raw_date.hours   & 0x0f);
	hrf_date->Day     = (raw_date.date    >> 4) * 10 + (raw_date.date    & 0x0f);
	hrf_date->Month   = (raw_date.month   >> 4) * 10 + (raw_date.month   & 0x0f);
	hrf_date->Year    = (raw_date.year    >> 4) * 10 + (raw_date.year    & 0x0f) + 2000;
	hrf_date->DOW     = raw_date.day_of_week;
}

void DS3231_DateToTimeStr(DS3231_date_TypeDef* raw_date, char *str) {
	*str++ = (raw_date->hours >> 4)     + '0';
    *str++ = (raw_date->hours & 0x0f)   + '0';
    *str++ = ':';
    *str++ = (raw_date->minutes >> 4)   + '0';
    *str++ = (raw_date->minutes & 0x0f) + '0';
    *str++ = ':';
    *str++ = (raw_date->seconds >> 4)   + '0';
    *str++ = (raw_date->seconds & 0x0f) + '0';
    *str++ = 0;
}

void DS3231_DateToDateStr(DS3231_date_TypeDef* raw_date, char *str) {
	*str++ = (raw_date->date >> 4)   + '0';
    *str++ = (raw_date->date & 0x0f) + '0';
    *str++ = '.';
    *str++ = (raw_date->month >> 4)   + '0';
    *str++ = (raw_date->month & 0x0f) + '0';
    *str++ = '.';
    *str++ = '2'; *str++ = '0';
    *str++ = (raw_date->year >> 4)   + '0';
    *str++ = (raw_date->year & 0x0f) + '0';
    *str++ = 0;
}

uint8_t DS3231_ReadTemp(void) {
	I2C_AcknowledgeConfig(I2C1,ENABLE); // Enable I2C acknowledge

	I2C_GenerateSTART(I2C1,ENABLE); // Send START condition
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5

	I2C_Send7bitAddress(I2C1,DS3231_addr,I2C_Direction_Transmitter); // Send DS3231 slave address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6

	I2C_SendData(I2C1,DS3231_tmp_MSB); // Send DS3231 temperature MSB register address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8

	I2C_GenerateSTART(I2C1,ENABLE); // Send repeated START condition (aka Re-START)
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5

	I2C_Send7bitAddress(I2C1,DS3231_addr,I2C_Direction_Receiver); // Send DS3231 slave address for READ
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); // Wait for EV6

	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	uint8_t temperature = I2C_ReceiveData(I2C1); // Receive temperature MSB

	I2C_AcknowledgeConfig(I2C1,DISABLE); // Disable I2C acknowledgement

	I2C_GenerateSTOP(I2C1,ENABLE); // Send STOP condition
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)

	return temperature;
}


void USART_SendChar(char ch) {
	while (!USART_GetFlagStatus(UART4,USART_FLAG_TC)); // wait for "Transmission Complete" flag cleared
	USART_SendData(UART4,ch);
}

void USART_SendStr(char *str) {
	while (*str) USART_SendChar(*str++);
}

void USART_SendBuf(char *buf, unsigned int bufsize) {
	unsigned int i;
	for (i = 0; i < bufsize; i++) USART_SendChar(*buf++);
}

void USART_SendBufHex(char *buf, unsigned int bufsize) {
	unsigned int i;
	char ch;
	for (i = 0; i < bufsize; i++) {
		ch = *buf++;
		USART_SendChar("0123456789ABCDEF"[(ch >> 4)   % 0x10]);
		USART_SendChar("0123456789ABCDEF"[(ch & 0x0f) % 0x10]);
	}
}


int main(void)
{
	// Init LED pins
	GPIO_InitTypeDef PORT;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	PORT.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&PORT);
	GPIOC->ODR ^= GPIO_Pin_8; // Toggle LED

	// UART init
	// GPIO_InitTypeDef PORT;
	// RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE); // do not need this because PORTC already enabled
	PORT.GPIO_Pin = GPIO_Pin_10;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	PORT.GPIO_Mode = GPIO_Mode_AF_PP; // TX as AF with Push-Pull
	GPIO_Init(GPIOC,&PORT);
	PORT.GPIO_Pin = GPIO_Pin_11;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	PORT.GPIO_Mode = GPIO_Mode_IN_FLOATING; // RX as in without pull-up
	GPIO_Init(GPIOC,&PORT);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
	USART_InitTypeDef UART;
	UART.USART_BaudRate = 115200;
	UART.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // No flow control
	UART.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // RX+TX mode
	UART.USART_Parity = USART_Parity_No; // No parity check
	UART.USART_StopBits = USART_StopBits_1; // 1 stop bit
	UART.USART_WordLength = USART_WordLength_8b; // 8-bit frame
	USART_Init(UART4,&UART);
	USART_Cmd(UART4,ENABLE);

	// EXTI pin
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	PORT.GPIO_Pin = GPIO_Pin_1;
	PORT.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&PORT);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource1);

	// Configure EXTI line1
	EXTI_InitTypeDef EXTIInit;
	EXTIInit.EXTI_Line = EXTI_Line1;             // EXTI will be on line 1
	EXTIInit.EXTI_LineCmd = ENABLE;              // EXTI1 enabled
	EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;    // Generate IRQ
	EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising; // IRQ on signal rising
	EXTI_Init(&EXTIInit);

	// Configure EXTI1 interrupt
	NVIC_InitTypeDef NVICInit;
	NVICInit.NVIC_IRQChannel = EXTI1_IRQn;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVICInit.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_Init(&NVICInit);

	// Init I2C
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	PORT.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	PORT.GPIO_Mode = GPIO_Mode_AF_OD; // PP onboard?
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&PORT);

	I2C_InitTypeDef I2CInit;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,ENABLE); // Enable I2C clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

	I2C_DeInit(I2C1); // I2C reset to initial state
	I2CInit.I2C_Mode = I2C_Mode_I2C; // I2C mode is I2C ^_^
	I2CInit.I2C_DutyCycle = I2C_DutyCycle_2; // I2C fast mode duty cycle (WTF is this?)
	I2CInit.I2C_OwnAddress1 = 1; // This device address (7-bit or 10-bit)
	I2CInit.I2C_Ack = I2C_Ack_Enable; // Acknowledgement enable
	I2CInit.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // choose 7-bit address for acknowledgement
	I2CInit.I2C_ClockSpeed = 400000; // 400kHz ?
	I2C_Cmd(I2C1,ENABLE); // Enable I2C
	I2C_Init(I2C1,&I2CInit); // Configure I2C

	while (I2C_GetFlagStatus(I2C1,I2C_FLAG_BUSY)); // Wait until I2C free

	// Check connection to DS3231
	I2C_GenerateSTART(I2C1,ENABLE); // Send START condition
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C1,DS3231_addr,I2C_Direction_Transmitter); // Send DS3231 slave address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_GenerateSTOP(I2C1,ENABLE);

	// Wait for 250ms for DS3231 startup
	Delay_ms(300);

	// DS3231 init
	I2C_GenerateSTART(I2C1,ENABLE);
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C1,DS3231_addr,I2C_Direction_Transmitter); // Send DS3231 slave address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_SendData(I2C1,DS3231_control); // Send DS3231 control register address
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_SendData(I2C1,0x00); // DS3231 EOSC enabled, INTCN enabled, SQW set to 1Hz
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_SendData(I2C1,0x00); // DS3231 clear alarm flags
	while (!I2C_CheckEvent(I2C1,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_GenerateSTOP(I2C1,ENABLE);

	LCD_Init();
	LCD_Clear(RGB565(0,0,0)); // BLACK

	DS3231_date_TypeDef date;
	char st_tm[9]  = "00:00:00";
	char st_dt[11] = "00.00.0000";
	char ch;

	USART_SendStr("\nSTM32F103RET6 is online.\n");

	while (1) {
		if (USART_GetFlagStatus(UART4,USART_SR_RXNE)) {
			ch = USART_ReceiveData(UART4);
			if (conf_mode) {
				switch (ch) {
				case '?':
					USART_SendStr("Configure:\n");
					USART_SendStr(" h - Set hours\n");
					USART_SendStr(" m - Set minutes\n");
					USART_SendStr(" s - Set seconds\n");
					break;
				case 'h':
					USART_SendStr("Enter two digit HOURS value...\n");
					while (!USART_GetFlagStatus(UART4,USART_SR_RXNE)); // wait for byte from UART
					ch  = (USART_ReceiveData(UART4) - '0') << 4;
					while (!USART_GetFlagStatus(UART4,USART_SR_RXNE)); // wait for byte from UART
					ch |= (USART_ReceiveData(UART4) - '0');
					if (ch > 23) {
						USART_SendStr("Invalid hours value (acceptable 0..23)\n");
					} else {
						DS3231_ReadDateRAW(&date);
						date.hours = ch;
						DS3231_WriteDateRAW(&date);
						USART_SendStr("Value set\n");
					}
					break;
				case 'm':
					USART_SendStr("Enter two digit MINUTES value...\n");
					while (!USART_GetFlagStatus(UART4,USART_SR_RXNE)); // wait for byte from UART
					ch  = (USART_ReceiveData(UART4) - '0') << 4;
					while (!USART_GetFlagStatus(UART4,USART_SR_RXNE)); // wait for byte from UART
					ch |= (USART_ReceiveData(UART4) - '0');
					if (ch > 59) {
						USART_SendStr("Invalid minutes value (acceptable 0..59)\n");
					} else {
						DS3231_ReadDateRAW(&date);
						date.minutes = ch;
						DS3231_WriteDateRAW(&date);
						USART_SendStr("Value set\n");
					}
					break;
				case 's':
					USART_SendStr("Enter two digit SECONDS value...\n");
					while (!USART_GetFlagStatus(UART4,USART_SR_RXNE)); // wait for byte from UART
					ch  = (USART_ReceiveData(UART4) - '0') << 4;
					while (!USART_GetFlagStatus(UART4,USART_SR_RXNE)); // wait for byte from UART
					ch |= (USART_ReceiveData(UART4) - '0');
					if (ch > 59) {
						USART_SendStr("Invalid seconds value (acceptable 0..59)\n");
					} else {
						DS3231_ReadDateRAW(&date);
						date.seconds = ch;
						DS3231_WriteDateRAW(&date);
						USART_SendStr("Value set\n");
					}
					break;
				default:
					USART_SendStr("STM32F103RET6: invalid command.\n");
					break;
				}
				USART_SendStr("Configure mode out\n");
				conf_mode = 0;
			} else {
				switch (ch) {
				case '?':
					USART_SendStr("STM32F103RET6\n");
					USART_SendStr(" d - Print date\n");
					USART_SendStr(" r - Print RAW data\n");
					USART_SendStr(" c - Configure mode\n");
					break;
				case 'd':
					USART_SendStr("Now: ");
					USART_SendStr(st_dt);
					USART_SendStr(", ");
					USART_SendStr(st_tm);
					USART_SendStr(", ");
					USART_SendBuf(&DOW[date.day_of_week-1][0],strlen(&DOW[date.day_of_week-1][0]));
					USART_SendChar('\n');
					break;
				case 'r':
					USART_SendStr("RAW date time: ");
					USART_SendBufHex((char*)&date,7);
					USART_SendChar('\n');
					break;
				case 'c':
					USART_SendStr("Configure:\n");
					USART_SendStr(" h - Set hours\n");
					USART_SendStr(" m - Set minutes\n");
					USART_SendStr(" s - Set seconds\n");
					conf_mode = 1;
					break;
				default:
					USART_SendStr("STM32F103RET6: invalid command.\n");
					break;
				}
			}
		}
		if (tm_ready) {
			uint8_t c = DS3231_ReadTemp();
			LCD_BMP(100,20,40,40,&digits[40*40*(c / 10)]);
			LCD_BMP(140,20,40,40,&digits[40*40*(c % 10)]);
			LCD_BMP(180,20,40,40,&digits[40*40*10]);

			DS3231_ReadDateRAW(&date);

			DS3231_DateToTimeStr(&date,st_tm);
			DS3231_DateToDateStr(&date,st_dt);

			LCD_PutStrO(10,222,st_tm,RGB565(250,250,0),0);
			LCD_PutStrO(90,222,st_dt,RGB565(250,250,0),0);
			LCD_PutStrO(200,222,&DOW[date.day_of_week-1][0],RGB565(160,250,190),0);

			LCD_BMP(20,80,40,40,&digits[40*40*(date.hours >>   4)]);
			LCD_BMP(60,80,40,40,&digits[40*40*(date.hours & 0x0f)]);
			LCD_BMP(100,80,20,40,&digits[40*40*11]); // :
			LCD_BMP(120,80,40,40,&digits[40*40*(date.minutes >> 4)]);
			LCD_BMP(160,80,40,40,&digits[40*40*(date.minutes & 0x0f)]);
			LCD_BMP(200,80,20,40,&digits[40*40*11]); // :
			LCD_BMP(220,80,40,40,&digits[40*40*(date.seconds >> 4)]);
			LCD_BMP(260,80,40,40,&digits[40*40*(date.seconds & 0x0f)]);

			tm_ready = 0;
		}
	}
}
