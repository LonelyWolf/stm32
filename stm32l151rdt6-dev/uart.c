#include <misc.h> // NVIC
#include <stm32l1xx_gpio.h>
#include <stm32l1xx_rcc.h>
#include <uart.h>


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
	USARTx->CR1 |=  USART_CR1_TE | USART_CR1_RE; // Transmitter and receiver enabled

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
	} else if (USARTx == USART3) {
		NVICInit.NVIC_IRQChannel = USART3_IRQn;
	}
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVICInit.NVIC_IRQChannelPreemptionPriority = priority;
	NVIC_Init(&NVICInit);
}

// Configure the UART TX/RX DMA channels
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   DMA_DIR - DMA direction (combination of USART_DMA_xx values)
//   pBuf - pointer to the data buffer
//   length - length of the data buffer
// note: the corresponding DMA peripheral clock must be already enabled
void UARTx_Configure_DMA(USART_TypeDef* USARTx, uint8_t DMA_DIR, uint8_t *pBuf, uint32_t length) {
	DMA_Channel_TypeDef *DMAx_Ch;

	if (DMA_DIR & USART_DMA_RX) {
		// USART DMA RX channel
		if (USARTx == USART1) {
			DMAx_Ch = USART1_DMA_CH_RX;
		} else if (USARTx == USART2) {
			DMAx_Ch = USART2_DMA_CH_RX;
		} else if (USARTx == USART3) {
			DMAx_Ch = USART3_DMA_CH_RX;
		}
		// DMA: peripheral -> memory, no circular mode, 8-bits, memory increment, medium channel priority, channel disabled
		DMAx_Ch->CCR   = DMA_CCR1_MINC | DMA_CCR1_PL_0;
		DMAx_Ch->CPAR  = (uint32_t)(&(USARTx->DR)); // Address of the peripheral data register
		DMAx_Ch->CMAR  = (uint32_t)pBuf; // Memory address
		DMAx_Ch->CNDTR = length; // Number of DMA transactions
	}
	if (DMA_DIR & USART_DMA_TX) {
		// USART DMA TX channel
		if (USARTx == USART1) {
			DMAx_Ch = USART1_DMA_CH_RX;
		} else if (USARTx == USART2) {
			DMAx_Ch = USART2_DMA_CH_RX;
		} else if (USARTx == USART3) {
			DMAx_Ch = USART3_DMA_CH_RX;
		}
		// DMA: memory -> peripheral, no circular mode, 8-bits, memory increment, medium channel priority, channel disabled
		DMAx_Ch->CCR   = DMA_CCR1_DIR | DMA_CCR1_MINC | DMA_CCR1_PL_0;
		DMAx_Ch->CPAR  = (uint32_t)(&(USARTx->DR)); // Address of the peripheral data register
		DMAx_Ch->CMAR  = (uint32_t)pBuf; // Memory address
		DMAx_Ch->CNDTR = length; // Number of DMA transactions
	}
}

// Enable/disable the USART RX/TX DMA channels
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   DMA_DIR - DMA direction (combination of USART_DMA_xx values)
//   NewState - new state of the channels (ENABLE/DISABLE)
void UARTx_SetDMA(USART_TypeDef* USARTx, uint8_t DMA_DIR, FunctionalState NewState) {
	if (NewState == ENABLE) {
		// Clear DMA interrupt flags and enable RX/TX DMA channels
		if (USARTx == USART1) {
			if (DMA_DIR & USART_DMA_TX) {
				USART1_DMA_PERIPH->IFCR = USART1_DMA_CH_TX_F;
				USART1_DMA_CH_TX->CCR |= DMA_CCR1_EN;
			}
			if (DMA_DIR & USART_DMA_RX) {
				USART1_DMA_PERIPH->IFCR = USART1_DMA_CH_RX_F;
				USART1_DMA_CH_RX->CCR |= DMA_CCR1_EN;
			}
		} else if (USARTx == USART2) {
			if (DMA_DIR & USART_DMA_TX) {
				USART2_DMA_PERIPH->IFCR = USART1_DMA_CH_TX_F;
				USART2_DMA_CH_TX->CCR |= DMA_CCR1_EN;
			}
			if (DMA_DIR & USART_DMA_RX) {
				USART2_DMA_PERIPH->IFCR = USART1_DMA_CH_RX_F;
				USART2_DMA_CH_RX->CCR |= DMA_CCR1_EN;
			}
		} else if (USARTx == USART3) {
			if (DMA_DIR & USART_DMA_TX) {
				USART3_DMA_PERIPH->IFCR = USART1_DMA_CH_TX_F;
				USART3_DMA_CH_TX->CCR |= DMA_CCR1_EN;
			}
			if (DMA_DIR & USART_DMA_RX) {
				USART3_DMA_PERIPH->IFCR = USART1_DMA_CH_RX_F;
				USART3_DMA_CH_RX->CCR |= DMA_CCR1_EN;
			}
		}
		// Enable the USART TX DMA
		if (DMA_DIR & USART_DMA_TX) USARTx->CR2 |= USART_CR3_DMAT;
		// Enable the USART RX DMA
		if (DMA_DIR & USART_DMA_RX) USARTx->CR2 |= USART_CR3_DMAR;
	} else {
		// Disable the RX/TX DMA channels
		if (USARTx == USART1) {
			if (DMA_DIR & USART_DMA_TX) USART1_DMA_CH_TX->CCR &= ~DMA_CCR1_EN;
			if (DMA_DIR & USART_DMA_RX) USART1_DMA_CH_RX->CCR &= ~DMA_CCR1_EN;
		} else if (USARTx == USART2) {
			if (DMA_DIR & USART_DMA_TX) USART2_DMA_CH_TX->CCR &= ~DMA_CCR1_EN;
			if (DMA_DIR & USART_DMA_RX) USART2_DMA_CH_RX->CCR &= ~DMA_CCR1_EN;
		} else if (USARTx == USART3) {
			if (DMA_DIR & USART_DMA_TX) USART3_DMA_CH_TX->CCR &= ~DMA_CCR1_EN;
			if (DMA_DIR & USART_DMA_RX) USART3_DMA_CH_RX->CCR &= ~DMA_CCR1_EN;
		}
		// Disable the USART RX DMA
		if (DMA_DIR & USART_DMA_RX) USARTx->CR2 &= ~USART_CR3_DMAR;
		// Disable the USART TX DMA
		if (DMA_DIR & USART_DMA_TX) USARTx->CR2 &= ~USART_CR3_DMAT;
	}
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
	while (i) UART_SendChar(USARTx,str[--i]);
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
	while (i) UART_SendChar(USARTx,str[--i]);
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
