#include <stm32l1xx_gpio.h>
#include <stm32l1xx_rcc.h>
#include <stm32l1xx_usart.h>
#include <uart.h>
#include <string.h>


// Initialize and configure UART peripheral with specified baudrate
// input:
//   baudrate - UART speed (bits/s)
void UART2_Init(uint32_t baudrate) {
	GPIO_InitTypeDef PORT;
	USART_InitTypeDef UART;

	// UART peripheral clock enable
	RCC_AHBPeriphClockCmd(UART_PORT_PERIPH,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

	// Alternative functions of GPIO pins
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);

	// TX pin
	PORT.GPIO_Mode  = GPIO_Mode_AF;
	PORT.GPIO_Speed = GPIO_Speed_40MHz;
	PORT.GPIO_OType = GPIO_OType_PP;
	PORT.GPIO_PuPd  = GPIO_PuPd_UP;
	PORT.GPIO_Pin   = UART_TX_PIN;
	GPIO_Init(UART_GPIO_PORT,&PORT);
	// RX pin
	PORT.GPIO_Pin   = UART_RX_PIN;
	GPIO_Init(UART_GPIO_PORT,&PORT);

	// UART port
	UART.USART_BaudRate = baudrate;
	UART.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // No flow control
	UART.USART_Mode = USART_Mode_Rx | USART_Mode_Tx; // RX+TX mode
	UART.USART_WordLength = USART_WordLength_8b; // 8-bit frame
	UART.USART_Parity = USART_Parity_No; // No parity check
	UART.USART_StopBits = USART_StopBits_1; // 1 stop bit
	USART_Init(UART_PORT,&UART);
	USART_Cmd(UART_PORT,ENABLE);
}

void UART_SendChar(char ch) {
	while (!(UART_PORT->SR & USART_SR_TC)); // wait for "Transmission Complete" flag cleared
	UART_PORT->DR = ch;
}

void UART_SendInt(int32_t num) {
	char str[10]; // 10 chars max for INT32_MAX
	int i = 0;
	if (num < 0) {
		UART_SendChar('-');
		num *= -1;
	}
	do str[i++] = num % 10 + '0'; while ((num /= 10) > 0);
	for (i--; i >= 0; i--) UART_SendChar(str[i]);
}

void UART_SendInt0(int32_t num) {
	char str[10]; // 10 chars max for INT32_MAX
	int i = 0;
	if (num < 0) {
		UART_SendChar('-');
		num *= -1;
	}
	if ((num < 10) && (num >= 0)) UART_SendChar('0');
	do str[i++] = num % 10 + '0'; while ((num /= 10) > 0);
	for (i--; i >= 0; i--) UART_SendChar(str[i]);
}

void UART_SendHex8(uint16_t num) {
	UART_SendChar(HEX_CHARS[(num >> 4)   % 0x10]);
	UART_SendChar(HEX_CHARS[(num & 0x0f) % 0x10]);
}

void UART_SendHex16(uint16_t num) {
	uint8_t i;
	for (i = 12; i > 0; i -= 4) UART_SendChar(HEX_CHARS[(num >> i) % 0x10]);
	UART_SendChar(HEX_CHARS[(num & 0x0f) % 0x10]);
}

void UART_SendHex32(uint32_t num) {
	uint8_t i;
	for (i = 28; i > 0; i -= 4)	UART_SendChar(HEX_CHARS[(num >> i) % 0x10]);
	UART_SendChar(HEX_CHARS[(num & 0x0f) % 0x10]);
}

void UART_SendStr(char *str) {
	while (*str) UART_SendChar(*str++);
}

void UART_SendBuf(char *buf, uint16_t bufsize) {
	uint16_t i;
	for (i = 0; i < bufsize; i++) UART_SendChar(*buf++);
}

void UART_SendBufPrintable(char *buf, uint16_t bufsize, char subst) {
	uint16_t i;
	char ch;
	for (i = 0; i < bufsize; i++) {
		ch = *buf++;
		UART_SendChar(ch > 32 ? ch : subst);
	}
}

void UART_SendBufHex(char *buf, uint16_t bufsize) {
	uint16_t i;
	char ch;
	for (i = 0; i < bufsize; i++) {
		ch = *buf++;
		UART_SendChar(HEX_CHARS[(ch >> 4)   % 0x10]);
		UART_SendChar(HEX_CHARS[(ch & 0x0f) % 0x10]);
	}
}

void UART_SendBufHexFancy(char *buf, uint16_t bufsize, uint8_t column_width, char subst) {
	uint16_t i = 0,len,pos;
	char buffer[column_width];

	while (i < bufsize) {
		// Line number
		UART_SendHex16(i);
		UART_SendChar(':'); UART_SendChar(' '); // Faster and less code than USART_SendStr(": ");

		// Copy one line
		if (i+column_width >= bufsize) len = bufsize - i; else len = column_width;
		memcpy(buffer,&buf[i],len);

		// Hex data
		pos = 0;
		while (pos < len) UART_SendHex8(buffer[pos++]);
		UART_SendChar(' ');

		// Raw data
		pos = 0;
		do UART_SendChar(buffer[pos] > 32 ? buffer[pos] : subst); while (++pos < len);
		UART_SendChar('\n');

		i += len;
	}
}
