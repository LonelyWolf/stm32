#include "string.h"
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_usart.h>
#include <sdcard.h>


void USART_SendChar(char ch) {
	while (!USART_GetFlagStatus(USART3,USART_FLAG_TC)); // wait for "Transmission Complete" flag cleared
	USART_SendData(USART3,ch);
}

void USART_SendHex8(uint16_t num) {
	USART_SendChar("0123456789ABCDEF"[(num >> 4)   % 0x10]);
	USART_SendChar("0123456789ABCDEF"[(num & 0x0f) % 0x10]);
}

void USART_SendHex16(uint16_t num) {
	USART_SendChar("0123456789ABCDEF"[(num >> 12)  % 0x10]);
	USART_SendChar("0123456789ABCDEF"[(num >> 8)   % 0x10]);
	USART_SendChar("0123456789ABCDEF"[(num >> 4)   % 0x10]);
	USART_SendChar("0123456789ABCDEF"[(num & 0x0f) % 0x10]);
}

void USART_SendHex32(uint32_t num) {
	// Ugly but less code and faster
	USART_SendChar("0123456789ABCDEF"[(num >> 28)  % 0x10]);
	USART_SendChar("0123456789ABCDEF"[(num >> 24)  % 0x10]);
	USART_SendChar("0123456789ABCDEF"[(num >> 20)  % 0x10]);
	USART_SendChar("0123456789ABCDEF"[(num >> 16)  % 0x10]);
	USART_SendChar("0123456789ABCDEF"[(num >> 12)  % 0x10]);
	USART_SendChar("0123456789ABCDEF"[(num >> 8)   % 0x10]);
	USART_SendChar("0123456789ABCDEF"[(num >> 4)   % 0x10]);
	USART_SendChar("0123456789ABCDEF"[(num & 0x0f) % 0x10]);
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

void USART_SendBufHexFancy(char *buf, uint16_t bufsize, uint8_t column_width) {
	uint16_t i = 0,l,pos;
	char buffer[column_width];

	while (i < bufsize) {
		// Line number
		USART_SendHex16(i);
		USART_SendChar(':'); USART_SendChar(' '); // Faster than USART_SendStr(": ");

		// Copy one line
		if (i+column_width >= bufsize) l = bufsize - i; else l = column_width;
		memcpy(buffer,&buf[i],l);

		// Hex data
		pos = 0;
		while (pos < l) USART_SendHex8(buffer[pos++]);
		USART_SendChar(' ');

		// Raw data
		pos = 0;
		while (pos < l) {
			USART_SendChar(buffer[pos] > 32 ? buffer[pos] : '.');
			pos++;
		}
		USART_SendChar('\n');

		i += l;
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

	uint8_t b = 0;
	USART_SendStr("SD_CardInit: ");
	b = SD_CardInit();
	USART_SendHex8(b);
	USART_SendStr("\nCard version: ");
	USART_SendHex8(SD_CardType);

	uint8_t response = 0;
	USART_SendStr("\nCSD: ");
	response = SD_Read_CSD();
	if (response != 0x00) {
		USART_SendStr("error = ");
		USART_SendHex8(response);
	} else USART_SendBufHex((char*)&SD_CSD[0],16);
	USART_SendStr("\nCID: ");
	response = SD_Read_CID();
	if (response != 0x00) {
		USART_SendStr("error = ");
		USART_SendHex8(response);
	} else USART_SendBufHex((char*)&SD_CID[0],16);
	USART_SendStr("\nMax bus clk.: ");
	USART_SendHex32(SD_MaxBusClkFreq);
	USART_SendStr("\nDevice size: ");
	USART_SendHex32(SD_CardCapacity);

	USART_SendStr("\nMBR:\n");
	SD_Read_Block(0x00000000);
	USART_SendBufHexFancy((char*)&SD_sector[0],512,32);
	USART_SendChar('\n');

	while(1);
}
