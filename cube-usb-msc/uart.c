#include <stm32l1xx_gpio.h>
#include <stm32l1xx_rcc.h>
#include <misc.h> // NVIC

#include "uart.h"


// Initialize and configure UART peripheral with specified baudrate
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   USART_DIR - RX/TX enable (combination of USART_RX and USART_TX values)
//   baudrate - UART speed (bits/s)
void UARTx_Init(USART_TypeDef* USARTx, uint32_t USART_DIR, uint32_t baudrate) {
	GPIO_InitTypeDef PORT;

	PORT.GPIO_Mode  = GPIO_Mode_AF;
	PORT.GPIO_Speed = GPIO_Speed_40MHz;
	PORT.GPIO_OType = GPIO_OType_PP;
	PORT.GPIO_PuPd  = GPIO_PuPd_UP;

	if (USARTx == USART1) {
		// Enable the USART1 peripheral clock
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
		// Reset the USART1 peripheral to initial state
		RCC->APB2RSTR |=  RCC_APB2RSTR_USART1RST;
		RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
		// Configure the USARTx GPIO pins
		if (USART_DIR & USART_TX) {
			RCC->AHBENR  |= USART1_GPIO_AHB_TX;
			PORT.GPIO_Pin = USART1_GPIO_PIN_TX;
			GPIO_Init(USART1_GPIO_TX,&PORT);
			GPIO_PinAFConfig(USART1_GPIO_TX,USART1_GPIO_TX_SRC,USART1_GPIO_AF);
		}
		if (USART_DIR & USART_RX) {
			RCC->AHBENR  |= USART1_GPIO_AHB_RX;
			PORT.GPIO_Pin = USART1_GPIO_PIN_RX;
			GPIO_Init(USART1_GPIO_RX,&PORT);
			GPIO_PinAFConfig(USART1_GPIO_RX,USART1_GPIO_RX_SRC,USART1_GPIO_AF);
		}
	} else if (USARTx == USART2) {
		// Enable the USART2 peripheral clock
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
		// Reset the USART2 peripheral to initial state
		RCC->APB1RSTR |=  RCC_APB1RSTR_USART2RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;
		// Configure the USARTx GPIO pins
		if (USART_DIR & USART_TX) {
			RCC->AHBENR  |= USART2_GPIO_AHB_TX;
			PORT.GPIO_Pin = USART2_GPIO_PIN_TX;
			GPIO_Init(USART2_GPIO_TX,&PORT);
			GPIO_PinAFConfig(USART2_GPIO_TX,USART2_GPIO_TX_SRC,USART2_GPIO_AF);
		}
		if (USART_DIR & USART_RX) {
			RCC->AHBENR  |= USART2_GPIO_AHB_RX;
			PORT.GPIO_Pin = USART2_GPIO_PIN_RX;
			GPIO_Init(USART2_GPIO_RX,&PORT);
			GPIO_PinAFConfig(USART2_GPIO_RX,USART2_GPIO_RX_SRC,USART2_GPIO_AF);
		}
	} else if (USARTx == USART3) {
		// Enable the USART3 peripheral clock
		RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
		// Reset the USART3 peripheral to initial state
		RCC->APB1RSTR |=  RCC_APB1RSTR_USART3RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_USART3RST;
		// Configure the USARTx GPIO pins
		if (USART_DIR & USART_TX) {
			RCC->AHBENR  |= USART3_GPIO_AHB_TX;
			PORT.GPIO_Pin = USART3_GPIO_PIN_TX;
			GPIO_Init(USART3_GPIO_TX,&PORT);
			GPIO_PinAFConfig(USART3_GPIO_TX,USART3_GPIO_TX_SRC,USART3_GPIO_AF);
		}
		if (USART_DIR & USART_RX) {
			RCC->AHBENR  |= USART3_GPIO_AHB_RX;
			PORT.GPIO_Pin = USART3_GPIO_PIN_RX;
			GPIO_Init(USART3_GPIO_RX,&PORT);
			GPIO_PinAFConfig(USART3_GPIO_RX,USART3_GPIO_RX_SRC,USART3_GPIO_AF);
		}
	}

	// Configure the USART (CR1):
	//   oversampling: by 16
	//   USART prescaler and outputs: disabled
	//   word length: 8-bit
	//   wake-up method: idle line
	//   parity control: disabled
	//   parity selection: even parity
	//   interrupts (TXE,TX,RXNE,IDLE) disabled
	//   transmitter/receiver disabled/enabled according to USART_DIR variable
	//   receiver wake-up disabled
	//   no send break character
	USARTx->CR1 = USART_DIR;

	// Configure the USART (CR2):
	//   LIN mode: disabled
	//   stop bits (STOP[13:12]): 1 stop bit (00)
	//   SCLK pin: disabled
	//   clock polarity: steady low value on SCLK
	//   clock phase: the first clock transition is the first data capture edge
	//   last bit clock pulse: the clock pulse of the last data bit is not output to the SCLK pin
	//   LIN break IRQ: disabled
	//   LIN break detection: 10-bit length
	//   Address of the USART node: 0
	USARTx->CR2 = 0x0000;

	// Configure the USART (CR3):
	//   sample method: three sample bit
	//   CTS IRQ: disabled
	//   CTS: disabled
	//   RTS: disabled
	//   DMA transmitter: disabled
	//   DMA receiver: disabled
	//   Smartcard mode: disabled
	//   Smartcard NACK: disabled
	//   Half duplex: disabled
	//   IrDA: disabled
	//   Error IRQ: disabled
	//   CTS and RTS hardware flow control disabled
	USARTx->CR3 = 0x0000;

	// Set given baud rate
	UARTx_SetSpeed(USARTx,baudrate);

	// Enable the USART
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

// Initialize the USART IRQ
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   USART_IRQ - IRQ's to enable (combination of USART_IRQ_XXX values)
//   priority - NVIC IRQ preemption priority (0xFF to simple enable with standard priority)
void UARTx_InitIRQ(USART_TypeDef* USARTx, uint32_t USART_IRQ, uint8_t priority) {
	// Clear the USART flags
	USARTx->SR &= ~(USART_SR_RXNE & USART_SR_TC);

	// Enable the specified USART IRQ(s)
	USARTx->CR1 |= USART_IRQ;

	if (priority == 0xff) {
		// Simple enable the IRQ without priority
		if (USARTx == USART1) {
			NVIC_EnableIRQ(USART1_IRQn);
		} else if (USARTx == USART2) {
			NVIC_EnableIRQ(USART2_IRQn);
		} else if (USARTx == USART3) {
			NVIC_EnableIRQ(USART3_IRQn);
		}
	} else {
		// Enable the IRQ with given priority level
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
}

// Configure the UART TX/RX DMA channels
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   DMA_DIR - DMA direction (combination of USART_DMA_xx values)
//   DMA_BUF - DMA buffer mode (one of USART_DMA_BUF_XXX values: circular or normal)
//   pBuf - pointer to the data buffer
//   length - length of the data buffer
// note: the corresponding DMA peripheral clock must be already enabled
void UARTx_ConfigureDMA(USART_TypeDef* USARTx, uint8_t DMA_DIR, uint32_t DMA_BUF, uint8_t *pBuf, uint32_t length) {
	DMA_Channel_TypeDef *DMAx_Ch;

	if (DMA_DIR & USART_DMA_RX) {
		// USART DMA RX channel
		if (USARTx == USART1) {
			DMAx_Ch = USART1_DMA_RX;
		} else if (USARTx == USART2) {
			DMAx_Ch = USART2_DMA_RX;
		} else if (USARTx == USART3) {
			DMAx_Ch = USART3_DMA_RX;
		}
		// DMA channel configuration:
		//   memory to memory: disabled
		//   channel priority: medium
		//   memory size: 8-bit
		//   peripheral size: 8-bit
		//   memory increment: enabled
		//   peripheral increment: disabled
		//   circular mode: according to DMA_BUF
		//   direction: read from peripheral
		//   IRQ: disabled
		//   channel: disabled
		DMAx_Ch->CCR   = DMA_CCR1_MINC | DMA_CCR1_PL_0 | DMA_BUF;
		DMAx_Ch->CPAR  = (uint32_t)(&(USARTx->DR)); // Address of the peripheral data register
		DMAx_Ch->CMAR  = (uint32_t)pBuf; // Memory address
		DMAx_Ch->CNDTR = length; // Number of DMA transactions
	}
	if (DMA_DIR & USART_DMA_TX) {
		// USART DMA TX channel
		if (USARTx == USART1) {
			DMAx_Ch = USART1_DMA_RX;
		} else if (USARTx == USART2) {
			DMAx_Ch = USART2_DMA_RX;
		} else if (USARTx == USART3) {
			DMAx_Ch = USART3_DMA_RX;
		}
		// DMA channel configuration:
		//   memory to memory: disabled
		//   channel priority: medium
		//   memory size: 8-bit
		//   peripheral size: 8-bit
		//   memory increment: enabled
		//   peripheral increment: disabled
		//   circular mode: disabled
		//   direction: read from memory
		//   IRQ: disabled
		//   channel: disabled
		DMAx_Ch->CCR   = DMA_CCR1_DIR | DMA_CCR1_MINC | DMA_CCR1_PL_0 | DMA_BUF;
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
		// Clear the DMA interrupt flags and enable RX/TX DMA channels
		if (USARTx == USART1) {
			if (DMA_DIR & USART_DMA_TX) {
				USART1_DMA_PERIPH->IFCR = USART1_DMA_TXF;
				USART1_DMA_TX->CCR |= DMA_CCR1_EN;
			}
			if (DMA_DIR & USART_DMA_RX) {
				USART1_DMA_PERIPH->IFCR = USART1_DMA_RXF;
				USART1_DMA_RX->CCR |= DMA_CCR1_EN;
			}
		} else if (USARTx == USART2) {
			if (DMA_DIR & USART_DMA_TX) {
				USART2_DMA_PERIPH->IFCR = USART2_DMA_TXF;
				USART2_DMA_TX->CCR |= DMA_CCR1_EN;
			}
			if (DMA_DIR & USART_DMA_RX) {
				USART2_DMA_PERIPH->IFCR = USART2_DMA_RXF;
				USART2_DMA_RX->CCR |= DMA_CCR1_EN;
			}
		} else if (USARTx == USART3) {
			if (DMA_DIR & USART_DMA_TX) {
				USART3_DMA_PERIPH->IFCR = USART3_DMA_TXF;
				USART3_DMA_TX->CCR |= DMA_CCR1_EN;
			}
			if (DMA_DIR & USART_DMA_RX) {
				USART3_DMA_PERIPH->IFCR = USART3_DMA_RXF;
				USART3_DMA_RX->CCR |= DMA_CCR1_EN;
			}
		}
		// Enable the USART TX/RX DMA
		USARTx->CR3 |= DMA_DIR;
	} else {
		// Disable the RX/TX DMA channels
		if (USARTx == USART1) {
			if (DMA_DIR & USART_DMA_TX) USART1_DMA_TX->CCR &= ~DMA_CCR1_EN;
			if (DMA_DIR & USART_DMA_RX) USART1_DMA_RX->CCR &= ~DMA_CCR1_EN;
		} else if (USARTx == USART2) {
			if (DMA_DIR & USART_DMA_TX) USART2_DMA_TX->CCR &= ~DMA_CCR1_EN;
			if (DMA_DIR & USART_DMA_RX) USART2_DMA_RX->CCR &= ~DMA_CCR1_EN;
		} else if (USARTx == USART3) {
			if (DMA_DIR & USART_DMA_TX) USART3_DMA_TX->CCR &= ~DMA_CCR1_EN;
			if (DMA_DIR & USART_DMA_RX) USART3_DMA_RX->CCR &= ~DMA_CCR1_EN;
		}
		// Disable the USART TX/RX DMA
		USARTx->CR3 &= ~DMA_DIR;
	}
}

// Send single character to UART
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   ch - character to send
void UART_SendChar(USART_TypeDef* USARTx, char ch) {
	while (!(USARTx->SR & USART_SR_TXE)); // Wait while transmit data register not empty
	USARTx->DR = ch; // Transmit character (TXE flag cleared)
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
void UART_SendIntLZ(USART_TypeDef* USARTx, int32_t num) {
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

// Send unsigned integer value as text to UART
// input
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   num - unsigned integer value to send
void UART_SendIntU(USART_TypeDef* USARTx, uint32_t num) {
	char str[10]; // 10 chars max for UINT32_MAX
	int i = 0;

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

	for (i = 12; i; i -= 4) UART_SendChar(USARTx,HEX_CHARS[(num >> i) % 0x10]);
	UART_SendChar(USARTx,HEX_CHARS[(num & 0x0f) % 0x10]);
}

// Send 32-bit value in HEX format to USART
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   num - 32-bit value to send
void UART_SendHex32(USART_TypeDef* USARTx, uint32_t num) {
	uint8_t i;

	for (i = 28; i; i -= 4)	UART_SendChar(USARTx,HEX_CHARS[(num >> i) % 0x10]);
	UART_SendChar(USARTx,HEX_CHARS[(num & 0x0f) % 0x10]);
}

// Send zero terminated string to USART
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   str - pointer to zero terminated string
void UART_SendStr(USART_TypeDef* USARTx, char *str) {
	while (*str) {
		while (!(USARTx->SR & USART_SR_TXE)); // Wait while transmit data register not empty
		USARTx->DR = *str++;
	}
}

// Send buffer to USART
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   pBuf - pointer to the buffer
//   length - size of buffer in bytes
void UART_SendBuf(USART_TypeDef* USARTx, char *pBuf, uint16_t length) {
	while (length--) {
		while (!(USARTx->SR & USART_SR_TXE)); // Wait while transmit data register not empty
		USARTx->DR = *pBuf++; // Transmit character (TXE flag cleared)
	}
}

// Send buffer to USART with substitute for unprintable characters
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   pBuf - pointer to the buffer
//   length - size of buffer in bytes
//   subst - character for substitute
void UART_SendBufPrintable(USART_TypeDef* USARTx, char *pBuf, uint16_t length, char subst) {
	char ch;

	while (length--) {
		ch = *pBuf++;
		while (!(USARTx->SR & USART_SR_TXE)); // Wait while transmit data register not empty
		USARTx->DR = (ch > 32) ? ch : subst;
	}
}

// Send buffer in HEX format to USART
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   pBuf - pointer to the buffer
//   length - size of buffer in bytes
void UART_SendBufHex(USART_TypeDef* USARTx, char *pBuf, uint16_t length) {
	char ch;

	while (length--) {
		ch = *pBuf++;
		while (!(USARTx->SR & USART_SR_TXE)); // Wait while transmit data register not empty
		USARTx->DR = HEX_CHARS[(ch >> 4)   % 0x10];
		while (!(USARTx->SR & USART_SR_TXE)); // Wait while transmit data register not empty
		USARTx->DR = HEX_CHARS[(ch & 0x0f) % 0x10];
	}
}
