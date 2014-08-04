#include <misc.h> // NVIC
#include <stm32l1xx_gpio.h>
#include <stm32l1xx_rcc.h>
#include <stm32l1xx_usart.h>
#include <stm32l1xx_dma.h>
#include <uart.h>


uint8_t USART_FIFO[FIFO_BUFFER_SIZE];       // DMA FIFO receive buffer from USART


// Initialize and configure UART peripheral with specified baudrate
// input:
//   baudrate - UART speed (bits/s)
void UART2_Init(uint32_t baudrate) {
	GPIO_InitTypeDef PORT;
	NVIC_InitTypeDef NVICInit;
	DMA_InitTypeDef  DMAInit;

	// UART peripheral clock enable
	RCC_AHBPeriphClockCmd(UART_PORT_PERIPH,ENABLE);
	RCC_APB1PeriphClockCmd(UART_PORT_APB,ENABLE);

	// Alternative functions of GPIO pins
	GPIO_PinAFConfig(UART_GPIO_PORT,UART_TX_PIN_SRC,UART_GPIO_AF);
	GPIO_PinAFConfig(UART_GPIO_PORT,UART_RX_PIN_SRC,UART_GPIO_AF);

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

	// Init USART port at given speed
	UART_SetSpeed(baudrate);

	// USART2 IRQ
	USART_ITConfig(UART_PORT,USART_IT_RXNE,ENABLE); // Enable USART2 RX interrupt
	NVICInit.NVIC_IRQChannel = USART2_IRQn;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 0x02; // high priority
	NVICInit.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Init(&NVICInit);

	USART_DMACmd(UART_PORT,USART_DMAReq_Rx,ENABLE); // Enable DMA for USART2 RX

	// USART RX DMA configuration
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE); // Enable DMA1 peripheral clock
	DMAInit.DMA_BufferSize = FIFO_BUFFER_SIZE;
	DMAInit.DMA_DIR = DMA_DIR_PeripheralSRC; // Copy from peripheral
	DMAInit.DMA_M2M = DMA_M2M_Disable; // Memory-to-memory disable
	DMAInit.DMA_MemoryBaseAddr = (uint32_t)&USART_FIFO[0]; // Pointer to memory buffer
	DMAInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; // Write bytes to memory
	DMAInit.DMA_MemoryInc = DMA_MemoryInc_Enable; // Enable memory counter per write
	DMAInit.DMA_Mode = DMA_Mode_Normal; // Non-circular mode
	DMAInit.DMA_PeripheralBaseAddr = (uint32_t)(&UART_PORT->DR); // Pointer to USART_DR register
	DMAInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // Read bytes from peripheral
	DMAInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // Do not increment peripheral pointer
	DMAInit.DMA_Priority = DMA_Priority_VeryHigh; // Highest priority
	DMA_Init(DMA1_Channel6,&DMAInit); // USART2_RX connected to DMA1_Channel6 (from datasheet table 54)
	DMA_ITConfig(DMA1_Channel6,DMA_IT_TC,ENABLE); // Enable DMA transfer complete interrupt

	// USART2 DMA interrupt
	NVICInit.NVIC_IRQChannel = DMA1_Channel6_IRQn;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 0x02; // high priority
	NVICInit.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Init(&NVICInit);
}

// Init USART port at given baudrate
// input:
//   baudrate - port speed in bps (bits per second)
void UART_SetSpeed(uint32_t baudrate) {
	USART_InitTypeDef UART;

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
	UART_PORT->DR = ch;
	while (!(UART_PORT->SR & USART_SR_TC)); // wait for "Transmission Complete" flag cleared
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
