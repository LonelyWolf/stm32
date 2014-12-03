#include <misc.h> // NVIC
#include <stm32l1xx_gpio.h>
#include <stm32l1xx_rcc.h>
#include <stm32l1xx_dma.h>
#include <uart.h>


uint8_t USART_FIFO[USART_FIFO_SIZE];       // DMA FIFO receive buffer from USART


// Initialize and configure UART peripheral with specified baudrate
// input:
//   baudrate - UART speed (bits/s)
void UARTx_Init(USART_TypeDef* USARTx, uint32_t baudrate) {
	GPIO_InitTypeDef PORT;

	PORT.GPIO_Mode  = GPIO_Mode_AF;
	PORT.GPIO_Speed = GPIO_Speed_40MHz;
	PORT.GPIO_OType = GPIO_OType_PP;
	PORT.GPIO_PuPd  = GPIO_PuPd_UP;

	if (USARTx == USART1) {
		RCC_AHBPeriphClockCmd(USART1_PORT_PERIPH,ENABLE);
		RCC_APB2PeriphClockCmd(USART1_PORT_APB,ENABLE);
		PORT.GPIO_Pin = USART1_TX_PIN;
		GPIO_Init(USART1_GPIO_PORT,&PORT);
		PORT.GPIO_Pin = USART1_RX_PIN;
		GPIO_Init(USART1_GPIO_PORT,&PORT);
		GPIO_PinAFConfig(USART1_GPIO_PORT,USART1_TX_PIN_SRC,USART1_GPIO_AF);
		GPIO_PinAFConfig(USART1_GPIO_PORT,USART1_RX_PIN_SRC,USART1_GPIO_AF);
	} else if (USARTx == USART2) {
		RCC_AHBPeriphClockCmd(USART2_PORT_PERIPH,ENABLE);
		RCC_APB1PeriphClockCmd(USART2_PORT_APB,ENABLE);
		PORT.GPIO_Pin = USART2_TX_PIN;
		GPIO_Init(USART2_GPIO_PORT,&PORT);
		PORT.GPIO_Pin = USART2_RX_PIN;
		GPIO_Init(USART2_GPIO_PORT,&PORT);
		GPIO_PinAFConfig(USART2_GPIO_PORT,USART2_TX_PIN_SRC,USART2_GPIO_AF);
		GPIO_PinAFConfig(USART2_GPIO_PORT,USART2_RX_PIN_SRC,USART2_GPIO_AF);
	} else if (USARTx == USART3) {
		RCC_AHBPeriphClockCmd(USART3_PORT_PERIPH,ENABLE);
		RCC_APB1PeriphClockCmd(USART3_PORT_APB,ENABLE);
		PORT.GPIO_Pin = USART3_TX_PIN;
		GPIO_Init(USART3_GPIO_PORT,&PORT);
		PORT.GPIO_Pin = USART3_RX_PIN;
		GPIO_Init(USART3_GPIO_PORT,&PORT);
		GPIO_PinAFConfig(USART3_GPIO_PORT,USART3_TX_PIN_SRC,USART3_GPIO_AF);
		GPIO_PinAFConfig(USART3_GPIO_PORT,USART3_RX_PIN_SRC,USART3_GPIO_AF);
	}

	// Configure the USART: 1 stop bit (STOP[13:12] = 00)
	USARTx->CR2 &= ~(USART_CR2_STOP);

	// Configure the USART: 8-bit frame, no parity check, TX and RX enabled
	USARTx->CR1 &= ~(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | USART_CR1_RE);
	USARTx->CR1 |= USART_CR1_TE | USART_CR1_RE; // Transmitter and receiver enabled

	// Configure the USART: CTS and RTS hardware flow control disabled
	USARTx->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);

	// Configure USART port at given speed
	UARTx_SetSpeed(USARTx,baudrate);

	// Enable USART
	USARTx->CR1 |= USART_CR1_UE;
}

// Configure the USART port at given baudrate
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   baudrate - port speed in bps (bits per second)
void UARTx_SetSpeed(USART_TypeDef* USARTx, uint32_t baudrate) {
	uint32_t apbclock;
	uint32_t brr;
	uint32_t idiv = 0x00;
	uint32_t fdiv = 0x00;
	RCC_ClocksTypeDef RCC_ClocksStatus;

	// Configure the USART baudrate
	RCC_GetClocksFreq(&RCC_ClocksStatus);
	if (USARTx == USART1) {
		apbclock = RCC_ClocksStatus.PCLK2_Frequency;
	} else {
		apbclock = RCC_ClocksStatus.PCLK1_Frequency;
	}

	// Integer part (in case if oversampling enabled)
	if (USARTx->CR1 & USART_CR1_OVER8) {
		idiv = ((25 * apbclock) / (baudrate << 1));
	} else {
		idiv = ((25 * apbclock) / (baudrate << 2));
	}
	brr = (idiv / 100) << 4;

	// Fractional part
	fdiv = idiv - (100 * (brr >> 4));
	// If oversampling enabled
	if (USARTx->CR1 & USART_CR1_OVER8) {
		brr |= ((((fdiv << 3) + 50) / 100)) & ((uint8_t)0x07);
	} else {
		brr |= ((((fdiv << 4) + 50) / 100)) & ((uint8_t)0x0F);
	}

	USARTx->BRR = (uint16_t)brr;
}

// Initialize UART RX IRQ
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   priority - NVIC IRQ preemption priority
void UARTx_InitRxIRQ(USART_TypeDef* USARTx, uint8_t priority) {
	NVIC_InitTypeDef NVICInit;

	if (USARTx == USART1) {
		NVICInit.NVIC_IRQChannel = USART1_IRQn;
	} else if (USARTx == USART2) {
		NVICInit.NVIC_IRQChannel = USART2_IRQn;
	}
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVICInit.NVIC_IRQChannelPreemptionPriority = priority;
	NVIC_Init(&NVICInit);
}

// Initialize UART RX DMA
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   priority - NVIC DMA IRQ preemption priority
void UARTx_InitRxDMA(USART_TypeDef* USARTx, uint8_t priority) {
	NVIC_InitTypeDef NVICInit;
	DMA_InitTypeDef  DMAInit;

	// USART RX DMA configuration
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE); // Enable DMA1 peripheral clock
	DMAInit.DMA_BufferSize = USART_FIFO_SIZE;
	DMAInit.DMA_DIR = DMA_DIR_PeripheralSRC; // Copy from peripheral
	DMAInit.DMA_M2M = DMA_M2M_Disable; // Memory-to-memory disable
	DMAInit.DMA_MemoryBaseAddr = (uint32_t)&USART_FIFO[0]; // Pointer to memory buffer
	DMAInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; // Write bytes to memory
	DMAInit.DMA_MemoryInc = DMA_MemoryInc_Enable; // Enable memory counter per write
	DMAInit.DMA_Mode = DMA_Mode_Normal; // Non-circular mode
	DMAInit.DMA_PeripheralBaseAddr = (uint32_t)(&USARTx->DR); // Pointer to USART_DR register
	DMAInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // Read bytes from peripheral
	DMAInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // Do not increment peripheral pointer
	DMAInit.DMA_Priority = DMA_Priority_VeryHigh; // Highest priority
	DMA_Init(DMA1_Channel6,&DMAInit); // USART2_RX connected to DMA1_Channel6 (from datasheet table 54)
	DMA1_Channel6->CCR |= DMA_CCR6_TCIE; // Enable DMA channel6 transfer complete interrupt
	DMA1_Channel6->CCR |= DMA_CCR6_EN; // Enable DMA channel6

	// USART2 DMA interrupt
	NVICInit.NVIC_IRQChannel = DMA1_Channel6_IRQn;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVICInit.NVIC_IRQChannelPreemptionPriority = priority;
	NVIC_Init(&NVICInit);
}

// Send single character to UART
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   ch - character to send
void UART_SendChar(USART_TypeDef* USARTx, char ch) {
	USARTx->DR = ch;
	while (!(USARTx->SR & USART_SR_TC)); // wait for "Transmission Complete" flag cleared
}

// Send signed integer value as text to UART
// input
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   num - integer value to send
void UART_SendInt(USART_TypeDef* USARTx, int32_t num) {
	char str[10]; // 10 chars max for INT32_MAX
	int i = 0;

	if (num < 0) {
		UART_SendChar(USARTx,'-');
		num *= -1;
	}
	do str[i++] = num % 10 + '0'; while ((num /= 10) > 0);
	while (i--) UART_SendChar(USARTx,str[i]);
}

// Send signed integer value with leading zero as text to UART
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   num - integer value to send
void UART_SendInt0(USART_TypeDef* USARTx, int32_t num) {
	char str[10]; // 10 chars max for INT32_MAX
	int i = 0;

	if (num < 0) {
		UART_SendChar(USARTx,'-');
		num *= -1;
	}
	if ((num < 10) && (num >= 0)) UART_SendChar(USARTx,'0');
	do str[i++] = num % 10 + '0'; while ((num /= 10) > 0);
	while (i--) UART_SendChar(USARTx,str[i]);
}

// Send byte in HEX format to USART
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   num - 8-bit value to send
void UART_SendHex8(USART_TypeDef* USARTx, uint8_t num) {
	UART_SendChar(USARTx,HEX_CHARS[(num >> 4)   % 0x10]);
	UART_SendChar(USARTx,HEX_CHARS[(num & 0x0f) % 0x10]);
}

// Send 16-bit value in HEX format to USART
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   num - 16-bit value to send
void UART_SendHex16(USART_TypeDef* USARTx, uint16_t num) {
	uint8_t i;

	for (i = 12; i > 0; i -= 4) UART_SendChar(USARTx,HEX_CHARS[(num >> i) % 0x10]);
	UART_SendChar(USARTx,HEX_CHARS[(num & 0x0f) % 0x10]);
}

// Send 32-bit value in HEX format to USART
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   num - 32-bit value to send
void UART_SendHex32(USART_TypeDef* USARTx, uint32_t num) {
	uint8_t i;

	for (i = 28; i > 0; i -= 4)	UART_SendChar(USARTx,HEX_CHARS[(num >> i) % 0x10]);
	UART_SendChar(USARTx,HEX_CHARS[(num & 0x0f) % 0x10]);
}

// Send zero terminated string to USART
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   str - pointer to zero terminated string
void UART_SendStr(USART_TypeDef* USARTx, char *str) {
	while (*str) UART_SendChar(USARTx,*str++);
}

// Send buffer to USART
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   buf - pointer to the buffer
//   bufsize - size of buffer in bytes
void UART_SendBuf(USART_TypeDef* USARTx, char *buf, uint16_t bufsize) {
	while (bufsize--) UART_SendChar(USARTx,*buf++);
}

// Send buffer to USART with substitute for unprintable characters
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   buf - pointer to the buffer
//   bufsize - size of buffer in bytes
//   subst - character for substitute
void UART_SendBufPrintable(USART_TypeDef* USARTx, char *buf, uint16_t bufsize, char subst) {
	char ch;

	while (bufsize--) {
		ch = *buf++;
		UART_SendChar(USARTx,ch > 32 ? ch : subst);
	}
}

// Send buffer in HEX format to USART
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   buf - pointer to the buffer
//   bufsize - size of buffer in bytes
void UART_SendBufHex(USART_TypeDef* USARTx, char *buf, uint16_t bufsize) {
	char ch;

	while (bufsize--) {
		ch = *buf++;
		UART_SendChar(USARTx,HEX_CHARS[(ch >> 4)   % 0x10]);
		UART_SendChar(USARTx,HEX_CHARS[(ch & 0x0f) % 0x10]);
	}
}
