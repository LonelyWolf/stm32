#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_usart.h>
#include <sdcard.h>


void USART_SendChar(char ch) {
	while (!USART_GetFlagStatus(USART3,USART_FLAG_TC)); // wait for "Transmission Complete" flag cleared
	USART_SendData(USART3,ch);
}

void USART_SendStr(char *str) {
	while (*str) USART_SendChar(*str++);
}

void USART_SendBuf(char *buf, uint16_t bufsize) {
	uint16_t i;
	char ch;
	for (i = 0; i < bufsize; i++) {
		ch = *buf++;
		USART_SendChar(ch > 32 ? ch : '.');
//		USART_SendChar(*buf++);
	}
}

void USART_SendBufHex(char *buf, uint16_t bufsize) {
	uint16_t i;
	char ch;
	for (i = 0; i < bufsize; i++) {
		ch = *buf++;
		USART_SendChar("0123456789ABCDEF"[(ch >> 4)   % 0x10]);
		USART_SendChar("0123456789ABCDEF"[(ch & 0x0f) % 0x10]);
	}
}


int main(void)
{
	GPIO_InitTypeDef PORT;

	// UART init
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	PORT.GPIO_Pin = GPIO_Pin_10;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	PORT.GPIO_Mode = GPIO_Mode_AF_PP; // TX as AF with Push-Pull
	GPIO_Init(GPIOB,&PORT);
	PORT.GPIO_Pin = GPIO_Pin_11;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	PORT.GPIO_Mode = GPIO_Mode_IN_FLOATING; // RX as in without pull-up
	GPIO_Init(GPIOB,&PORT);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	USART_InitTypeDef UART;
	UART.USART_BaudRate = 115200;
	UART.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // No flow control
	UART.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // RX+TX mode
	UART.USART_Parity = USART_Parity_No; // No parity check
	UART.USART_StopBits = USART_StopBits_1; // 1 stop bit
	UART.USART_WordLength = USART_WordLength_8b; // 8-bit frame
	USART_Init(USART3,&UART);
	USART_Cmd(USART3,ENABLE);

	USART_SendStr("\nSTM32F103RET6 is online.\n");

	SD_Init();

	uint8_t b = 0,v;
	USART_SendStr("SD_CardInit: ");
	b = SD_CardInit();
	USART_SendBufHex((char*)&b,1);
	USART_SendStr("\nCard version: ");
	v = SD_GetVersion();
	USART_SendBufHex((char*)&v,1);

	uint8_t* CSD_tab = SD_Read_CSD();
	uint8_t* CID_tab = SD_Read_CID();

	USART_SendStr("\nCSD: ");
	USART_SendBufHex((char*)CSD_tab,16);
	USART_SendStr("\nCID: ");
	USART_SendBufHex((char*)CID_tab,16);

	USART_SendStr("\nMBR: ");
	uint8_t* sector = SD_Read_Block(0x00000000);
	USART_SendBuf((char*)sector,512);
	USART_SendStr("\n");

	USART_SendChar('\n');

	while(1);
}
