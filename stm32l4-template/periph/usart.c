// Universal Synchronous/Asynchronous Receiver Transmitter (USART/UART) management


#include "usart.h"


// USART port handles
#if (USART1_USE)
USART_HandleTypeDef hUSART1;
#endif
#if (USART2_USE)
USART_HandleTypeDef hUSART2;
#endif
#if (USART3_USE)
USART_HandleTypeDef hUSART3;
#endif
#if (UART4_USE)
USART_HandleTypeDef hUART4;
#endif
#if (UART5_USE)
USART_HandleTypeDef hUART5;
#endif


#if (USART1_USE)
// Populate the USART1 handle
void USART1_HandleInit(void) {
	hUSART1.Instance = USART1;
	hUSART1.AF       = GPIO_AF7;

	// TX [PA9, PB6]
	hUSART1.PIN_TX.GPIO_AHB = RCC_AHB2ENR_GPIOBEN;
	hUSART1.PIN_TX.GPIO     = GPIOB;
	hUSART1.PIN_TX.GPIO_PIN = GPIO_PIN_6;
	hUSART1.PIN_TX.GPIO_SRC = GPIO_PinSource6;

	// RX [PA10, PB7]
	hUSART1.PIN_RX.GPIO_AHB = RCC_AHB2ENR_GPIOBEN;
	hUSART1.PIN_RX.GPIO     = GPIOB;
	hUSART1.PIN_RX.GPIO_PIN = GPIO_PIN_7;
	hUSART1.PIN_RX.GPIO_SRC = GPIO_PinSource7;

#if (USART_USE_DMA)
	// DMA TX channel
#if 1
	hUSART1.DMA_TX.Channel  = DMA1_Channel4;
#else
	hUSART1.DMA_TX.Channel  = DMA2_Channel6;
#endif
	hUSART1.DMA_TX.Request  = DMA_REQUEST_2;
	hUSART1.DMA_TX.Instance = DMA_GetChannelPeripheral(hUSART1.DMA_TX.Channel);
	hUSART1.DMA_TX.ChIndex  = DMA_GetChannelIndex(hUSART1.DMA_TX.Channel);
	hUSART1.DMA_TX.State    = DMA_STATE_RESET;

	// DMA RX channel
#if 1
	hUSART1.DMA_RX.Channel  = DMA1_Channel5;
#else
	hUSART1.DMA_RX.Channel  = DMA2_Channel7;
#endif
	hUSART1.DMA_RX.Request  = DMA_REQUEST_2;
	hUSART1.DMA_RX.Instance = DMA_GetChannelPeripheral(hUSART1.DMA_RX.Channel);
	hUSART1.DMA_RX.ChIndex  = DMA_GetChannelIndex(hUSART1.DMA_RX.Channel);
	hUSART1.DMA_RX.State    = DMA_STATE_RESET;
#endif // USART_USE_DMA
}
#endif // USART1_USE

#if (USART2_USE)
// Populate the USART2 handle
void USART2_HandleInit(void) {
	hUSART2.Instance = USART2;
	hUSART2.AF       = GPIO_AF7;

	// TX [PA2]
	hUSART2.PIN_TX.GPIO_AHB = RCC_AHB2ENR_GPIOAEN;
	hUSART2.PIN_TX.GPIO     = GPIOA;
	hUSART2.PIN_TX.GPIO_PIN = GPIO_PIN_2;
	hUSART2.PIN_TX.GPIO_SRC = GPIO_PinSource2;

	// RX [PA3]
	hUSART2.PIN_RX.GPIO_AHB = RCC_AHB2ENR_GPIOAEN;
	hUSART2.PIN_RX.GPIO     = GPIOA;
	hUSART2.PIN_RX.GPIO_PIN = GPIO_PIN_3;
	hUSART2.PIN_RX.GPIO_SRC = GPIO_PinSource3;

#if (USART_USE_DMA)
	// DMA TX channel
	hUSART2.DMA_TX.Channel  = DMA1_Channel7;
	hUSART2.DMA_TX.Request  = DMA_REQUEST_2;
	hUSART2.DMA_TX.Instance = DMA_GetChannelPeripheral(hUSART2.DMA_TX.Channel);
	hUSART2.DMA_TX.ChIndex  = DMA_GetChannelIndex(hUSART2.DMA_TX.Channel);
	hUSART2.DMA_TX.State    = DMA_STATE_RESET;

	// DMA RX channel
	hUSART2.DMA_RX.Channel  = DMA1_Channel6;
	hUSART2.DMA_RX.Request  = DMA_REQUEST_2;
	hUSART2.DMA_RX.Instance = DMA_GetChannelPeripheral(hUSART2.DMA_RX.Channel);
	hUSART2.DMA_RX.ChIndex  = DMA_GetChannelIndex(hUSART2.DMA_RX.Channel);
	hUSART2.DMA_RX.State    = DMA_STATE_RESET;
#endif // USART_USE_DMA
}
#endif // USART2_USE

#if (USART3_USE)
// Populate the USART3 handle
void USART3_HandleInit(void) {
	hUSART3.Instance = USART3;
	hUSART3.AF       = GPIO_AF7;

	// TX [PB10, PC10]
	hUSART3.PIN_TX.GPIO_AHB = RCC_AHB2ENR_GPIOBEN;
	hUSART3.PIN_TX.GPIO     = GPIOB;
	hUSART3.PIN_TX.GPIO_PIN = GPIO_PIN_10;
	hUSART3.PIN_TX.GPIO_SRC = GPIO_PinSource10;

	// RX [PB11, PC11]
	hUSART3.PIN_RX.GPIO_AHB = RCC_AHB2ENR_GPIOBEN;
	hUSART3.PIN_RX.GPIO     = GPIOB;
	hUSART3.PIN_RX.GPIO_PIN = GPIO_PIN_11;
	hUSART3.PIN_RX.GPIO_SRC = GPIO_PinSource11;

#if (USART_USE_DMA)
	// DMA TX channel
	hUSART3.DMA_TX.Channel  = DMA1_Channel2;
	hUSART3.DMA_TX.Request  = DMA_REQUEST_2;
	hUSART3.DMA_TX.Instance = DMA_GetChannelPeripheral(hUSART3.DMA_TX.Channel);
	hUSART3.DMA_TX.ChIndex  = DMA_GetChannelIndex(hUSART3.DMA_TX.Channel);
	hUSART3.DMA_TX.State    = DMA_STATE_RESET;

	// DMA RX channel
	hUSART3.DMA_RX.Channel  = DMA1_Channel3;
	hUSART3.DMA_RX.Request  = DMA_REQUEST_2;
	hUSART3.DMA_RX.Instance = DMA_GetChannelPeripheral(hUSART3.DMA_RX.Channel);
	hUSART3.DMA_RX.ChIndex  = DMA_GetChannelIndex(hUSART3.DMA_RX.Channel);
	hUSART3.DMA_RX.State    = DMA_STATE_RESET;
#endif // USART_USE_DMA
}
#endif // USART3_USE

#if (UART4_USE)
// Populate the UART4 handle
void UART4_HandleInit(void) {
	hUART4.Instance = UART4;
	hUART4.AF       = GPIO_AF8;

	// TX [PC10]
	hUART4.PIN_TX.GPIO_AHB = RCC_AHB2ENR_GPIOCEN;
	hUART4.PIN_TX.GPIO     = GPIOC;
	hUART4.PIN_TX.GPIO_PIN = GPIO_PIN_10;
	hUART4.PIN_TX.GPIO_SRC = GPIO_PinSource10;

	// RX [PC11]
	hUART4.PIN_RX.GPIO_AHB = RCC_AHB2ENR_GPIOCEN;
	hUART4.PIN_RX.GPIO     = GPIOC;
	hUART4.PIN_RX.GPIO_PIN = GPIO_PIN_11;
	hUART4.PIN_RX.GPIO_SRC = GPIO_PinSource11;

#if (USART_USE_DMA)
	// DMA TX channel
	hUART4.DMA_TX.Channel  = DMA2_Channel3;
	hUART4.DMA_TX.Request  = DMA_REQUEST_2;
	hUART4.DMA_TX.Instance = DMA_GetChannelPeripheral(hUART4.DMA_TX.Channel);
	hUART4.DMA_TX.ChIndex  = DMA_GetChannelIndex(hUART4.DMA_TX.Channel);
	hUART4.DMA_TX.State    = DMA_STATE_RESET;

	// DMA RX channel
	hUART4.DMA_RX.Channel  = DMA2_Channel5;
	hUART4.DMA_RX.Request  = DMA_REQUEST_2;
	hUART4.DMA_RX.Instance = DMA_GetChannelPeripheral(hUART4.DMA_RX.Channel);
	hUART4.DMA_RX.ChIndex  = DMA_GetChannelIndex(hUART4.DMA_RX.Channel);
	hUART4.DMA_RX.State    = DMA_STATE_RESET;
#endif // USART_USE_DMA
}
#endif // UART4_USE

#if (UART5_USE)
// Populate the UART5 handle
void UART5_HandleInit(void) {
	hUART5.Instance = UART5;
	hUART5.AF       = GPIO_AF8;

	// TX [PC12]
	hUART5.PIN_TX.GPIO_AHB = RCC_AHB2ENR_GPIOCEN;
	hUART5.PIN_TX.GPIO     = GPIOC;
	hUART5.PIN_TX.GPIO_PIN = GPIO_PIN_12;
	hUART5.PIN_TX.GPIO_SRC = GPIO_PinSource12;

	// RX [PD2]
	hUART5.PIN_RX.GPIO_AHB = RCC_AHB2ENR_GPIODEN;
	hUART5.PIN_RX.GPIO     = GPIOD;
	hUART5.PIN_RX.GPIO_PIN = GPIO_PIN_2;
	hUART5.PIN_RX.GPIO_SRC = GPIO_PinSource2;

#if (USART_USE_DMA)
	// DMA TX channel
	hUART5.DMA_TX.Channel  = DMA2_Channel1;
	hUART5.DMA_TX.Request  = DMA_REQUEST_2;
	hUART5.DMA_TX.Instance = DMA_GetChannelPeripheral(hUART5.DMA_TX.Channel);
	hUART5.DMA_TX.ChIndex  = DMA_GetChannelIndex(hUART5.DMA_TX.Channel);
	hUART5.DMA_TX.State    = DMA_STATE_RESET;

	// DMA RX channel
	hUART5.DMA_RX.Channel  = DMA2_Channel2;
	hUART5.DMA_RX.Request  = DMA_REQUEST_2;
	hUART5.DMA_RX.Instance = DMA_GetChannelPeripheral(hUART5.DMA_RX.Channel);
	hUART5.DMA_RX.ChIndex  = DMA_GetChannelIndex(hUART5.DMA_RX.Channel);
	hUART5.DMA_RX.State    = DMA_STATE_RESET;
#endif
}
#endif // UART5_USE


// Initialize the USART/UART peripheral and GPIO
// input:
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
//   mode - communication direction, one of USART_MODE_xx values
void USART_Init(const USART_HandleTypeDef *USARTx, uint32_t mode) {
#if (USART1_USE)
	if (USARTx->Instance == USART1) {
		// Reset the USART1 peripheral
		RCC->APB2RSTR |=  RCC_APB2RSTR_USART1RST;
		RCC->APB2RSTR &= ~RCC_APB2RSTR_USART1RST;
		// Enable the USART1 peripheral clock
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	}
#endif // USART1_USE
#if (USART2_USE)
	if (USARTx->Instance == USART2) {
		// Reset the USART2 peripheral
		RCC->APB1RSTR1 |=  RCC_APB1RSTR1_USART2RST;
		RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_USART2RST;
		// Enable the USART2 peripheral clock
		RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
	}
#endif // USART2_USE
#if (USART3_USE)
	if (USARTx->Instance == USART3) {
		// Reset the USART3 peripheral
		RCC->APB1RSTR1 |=  RCC_APB1RSTR1_USART3RST;
		RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_USART3RST;
		// Enable the USART3 peripheral clock
		RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
	}
#endif // USART3_USE
#if (UART4_USE)
	if (USARTx->Instance == UART4) {
		// Reset the UART4 peripheral
		RCC->APB1RSTR1 |=  RCC_APB1RSTR1_UART4RST;
		RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_UART4RST;
		// Enable the UART4 peripheral clock
		RCC->APB1ENR1 |= RCC_APB1ENR1_UART4EN;
	}
#endif // UART4_USE
#if (UART5_USE)
	if (USARTx->Instance == UART5) {
		// Reset the UART5 peripheral
		RCC->APB1RSTR1 |=  RCC_APB1RSTR1_UART5RST;
		RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_UART5RST;
		// Enable the UART5 peripheral clock
		RCC->APB1ENR1 |= RCC_APB1ENR1_UART5EN;
	}
#endif // UART5_USE

	// TX pin
	if (mode & USART_MODE_TX) {
		// Configure TX pin as: AF, low speed, push-pull, pull-up
		RCC->AHB2ENR |= USARTx->PIN_TX.GPIO_AHB;
		GPIO_af_cfg(USARTx->PIN_TX.GPIO, USARTx->PIN_TX.GPIO_SRC, USARTx->AF);
		GPIO_set_mode(USARTx->PIN_TX.GPIO, GPIO_Mode_AF, GPIO_PUPD_PU, USARTx->PIN_TX.GPIO_PIN);
		GPIO_out_cfg(USARTx->PIN_TX.GPIO, GPIO_OT_PP, GPIO_SPD_LOW, USARTx->PIN_TX.GPIO_PIN);
	}

	// RX pin
	if (mode & USART_MODE_RX) {
		// Configure RX pin as: AF, low speed, push-pull, pull-up
		RCC->AHB2ENR |= USARTx->PIN_RX.GPIO_AHB;
		GPIO_af_cfg(USARTx->PIN_RX.GPIO, USARTx->PIN_RX.GPIO_SRC, USARTx->AF);
		GPIO_set_mode(USARTx->PIN_RX.GPIO, GPIO_Mode_AF, GPIO_PUPD_PU, USARTx->PIN_RX.GPIO_PIN);
		GPIO_out_cfg(USARTx->PIN_RX.GPIO, GPIO_OT_PP, GPIO_SPD_LOW, USARTx->PIN_RX.GPIO_PIN);
	}

	// Configure communication direction
	USARTx->Instance->CR1 &= ~(USART_CR1_TE | USART_CR1_RE);
	USARTx->Instance->CR1 |= mode & (USART_CR1_TE | USART_CR1_RE);
}

// Configure the USART/UART oversampling mode
// input:
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
//   ovs_mode - new oversampling mode, values USART_OVERS8 or USART_OVERS16
// note: after changing oversampling mode the baudrate must be re-configured
// note: this setting can be changed only when USART is disabled
void USART_SetOversampling(const USART_HandleTypeDef *USARTx, uint32_t ovs_mode) {
	if (ovs_mode) {
		// Oversampling by 8
		USARTx->Instance->CR1 |= USART_CR1_OVER8;
	} else {
		// Oversampling by 16
		USARTx->Instance->CR1 &= ~USART_CR1_OVER8;
	}
}

// Configure the USART/UART peripheral for achieve specified baud rate
// input:
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
//   baudrate - expected communication baud rate (bits/s)
// note: must be called only when the USART/UART is disabled
void USART_SetBaudRate(const USART_HandleTypeDef *USARTx, uint32_t baudrate) {
	uint32_t periph_freq = 0;

	// Get USART/UART peripheral clock frequency
#if (USART1_USE)
	if (USARTx->Instance == USART1) periph_freq = RCC_GetClockUSART(RCC_USART1_CLK_SRC);
#endif // USART1_USE
#if (USART2_USE)
	if (USARTx->Instance == USART2) periph_freq = RCC_GetClockUSART(RCC_USART2_CLK_SRC);
#endif // USART2_USE
#if (USART3_USE)
	if (USARTx->Instance == USART3) periph_freq = RCC_GetClockUSART(RCC_USART3_CLK_SRC);
#endif // USART3_USE
#if (UART4_USE)
	if (USARTx->Instance == UART4)  periph_freq = RCC_GetClockUSART(RCC_UART4_CLK_SRC);
#endif // UART4_USE
#if (UART5_USE)
	if (USARTx->Instance == UART5)  periph_freq = RCC_GetClockUSART(RCC_UART5_CLK_SRC);
#endif // UART5_USE

	if (periph_freq) {
		if (USARTx->Instance->CR1 & USART_CR1_OVER8) {
			// Oversampling by 8
			periph_freq = (uint16_t)(((periph_freq * 2) + (baudrate / 2)) / baudrate);
			USARTx->Instance->BRR = (uint16_t)((periph_freq & 0xFFF0) | ((periph_freq & 0x000F) >> 1));
		} else {
			// Oversampling by 16
			USARTx->Instance->BRR = (uint16_t)((periph_freq + (baudrate / 2)) / baudrate);
		}
	} else {
		// Zero value of periph_freq variable means either incorrectly specified peripheral
		// or peripheral clock source is not working (HSI/LSE is not ready)
		// Do nothing in that case
	}
}

// Get current USART/UART baud rate
// Configure the USART/UART peripheral for achieve specified baud rate
// input:
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
// return: calculated baud rate (bps)
// note: in case of non-initialized or invalid value of BRR register, zero will be returned
//       the incorrect baud rate value will be returned if BRR register contains value
//       which is invalid for current clock settings
uint32_t USART_GetBaudRate(const USART_HandleTypeDef *USARTx) {
	uint32_t periph_freq = 0;
	uint32_t result;

	// Get USART/UART peripheral clock frequency
#if (USART1_USE)
	if (USARTx->Instance == USART1) periph_freq = RCC_GetClockUSART(RCC_USART1_CLK_SRC);
#endif // USART1_USE
#if (USART2_USE)
	if (USARTx->Instance == USART2) periph_freq = RCC_GetClockUSART(RCC_USART2_CLK_SRC);
#endif // USART2_USE
#if (USART3_USE)
	if (USARTx->Instance == USART3) periph_freq = RCC_GetClockUSART(RCC_USART3_CLK_SRC);
#endif // USART3_USE
#if (UART4_USE)
	if (USARTx->Instance == UART4)  periph_freq = RCC_GetClockUSART(RCC_UART4_CLK_SRC);
#endif // UART4_USE
#if (UART5_USE)
	if (USARTx->Instance == UART5)  periph_freq = RCC_GetClockUSART(RCC_UART5_CLK_SRC);
#endif // UART5_USE

	if (periph_freq) {
		result = USARTx->Instance->BRR;
		if (USARTx->Instance->CR1 & USART_CR1_OVER8) {
			// Oversampling by 8
			if (result & 0xFFF7) {
				result = (periph_freq << 1) / ((result & 0xFFF0) | ((result & 0x0007) << 1));
			} else {
				result = 0;
			}
		} else {
			// Oversampling by 16
			result = periph_freq / USARTx->Instance->BRR;
		}
	} else {
		// Zero value of periph_freq variable means either incorrectly specified peripheral
		// or peripheral clock source is not working (HSI/LSE is not ready)
		result = 0;
	}

	return result;
}


// Configure USART/UART character frame format (data width, parity, stop bits)
// input:
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
//   databits - number of data bits, one of USART_DATAWIDTH_xx values
//   parity - parity control mode, one of USART_PARITY_xx values
//   stopbits - number of stop bits transmitted, one of USART_STOPBITS_xx values
// note: these settings can be changed only when USART is disabled
void USART_SetDataMode(const USART_HandleTypeDef *USARTx, uint32_t databits, uint32_t parity, uint32_t stopbits) {
	USARTx->Instance->CR1 &= ~(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS);
	USARTx->Instance->CR1 |= (databits | parity) & (USART_CR1_M | USART_CR1_PCE | USART_CR1_PS);
	USARTx->Instance->CR2 &= ~(USART_CR2_STOP);
	USARTx->Instance->CR2 |= stopbits & USART_CR2_STOP;
}

// Configure USART/UART hardware flow control
// input:
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
//   hwflow_mode - new hardware flow control configuration, one of USART_HWCTL_xx values
// note: this setting can be changed only when USART is disabled
void USART_SetHWFlow(const USART_HandleTypeDef *USARTx, uint32_t hwflow_mode) {
	USARTx->Instance->CR3 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);
	USARTx->Instance->CR3 |= hwflow_mode & (USART_CR3_CTSE | USART_CR3_RTSE);
}

// Enable USART interrupt
// input:
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
//   irq - IRQ to enable, one of USART_IRQ_xx values (where xx can be TXE, RXNE, TC, IDLE, PE, ERR)
void USART_EnableIRQ(USART_HandleTypeDef *USARTx, uint16_t irq) {
	if (((uint8_t)irq >> 5) == 1) {
		// USART_CR1
		USARTx->Instance->CR1 |= (1 << (irq & 0x1F));
	} else if (((uint8_t)irq >> 5) == 2) {
		// USART_CR2
		USARTx->Instance->CR2 |= (1 << (irq & 0x1F));
	} else {
		// USART_CR3
		USARTx->Instance->CR3 |= (1 << (irq & 0x1F));
	}
}

// Disable USART interrupt
// input:
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
//   irq - IRQ to enable, one of the USART_IRQ_xx values (where xx can be TXE, RXNE, TC, IDLE, PE, ERR)
void USART_DisableIRQ(USART_HandleTypeDef *USARTx, uint16_t irq) {
	if (((uint8_t)irq >> 5) == 1) {
		// USART_CR1
		USARTx->Instance->CR1 |= (1 << (irq & 0x1F));
	} else if (((uint8_t)irq >> 5) == 2) {
		// USART_CR2
		USARTx->Instance->CR2 |= (1 << (irq & 0x1F));
	} else {
		// USART_CR3
		USARTx->Instance->CR3 |= (1 << (irq & 0x1F));
	}
}

#if (USART_USE_DMA)

// Configure the UART TX/RX DMA channels
// input:
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
//   DMA_DIR - DMA direction (combination of USART_DMA_xx values)
//   DMA_MODE - DMA buffer mode, either DMA_MODE_CIRCULAR or DMA_MODE_NORMAL
//   pBuf - pointer to the data buffer
//   length - size of the data buffer
// note: the corresponding DMA peripheral clock must be already enabled
void USART_ConfigureDMA(const USART_HandleTypeDef *USARTx, uint32_t DMA_DIR, uint32_t DMA_MODE, uint8_t *pBuf, uint32_t length) {
	if (DMA_DIR & USART_DMA_RX) {
		// USART RX DMA channel configuration:
		//   memory to memory: disabled
		//   channel priority: medium
		//   memory size: 8-bit
		//   peripheral size: 8-bit
		//   memory increment: enabled
		//   peripheral increment: disabled
		//   circular mode: according to DMA_MODE
		//   direction: read from peripheral
		DMA_ConfigChannel(
				USARTx->DMA_RX.Channel,
				DMA_MODE |
				DMA_DIR_P2M |
				DMA_MSIZE_8BIT | DMA_PSIZE_8BIT |
				DMA_MINC_ENABLE | DMA_PINC_DISABLE |
				DMA_PRIORITY_MEDIUM
			);
		DMA_SetAddrM(USARTx->DMA_RX.Channel, (uint32_t)pBuf);
		DMA_SetAddrP(USARTx->DMA_RX.Channel, (uint32_t)(&(USARTx->Instance->RDR)));
		DMA_SetDataLength(USARTx->DMA_RX.Channel, length);

		// Map DMA request to DMA channel
		DMA_SetRequest(USARTx->DMA_RX.Instance, USARTx->DMA_RX.Request, USARTx->DMA_RX.ChIndex);
	}
	if (DMA_DIR & USART_DMA_TX) {
		// USART TX DMA channel configuration:
		//   memory to memory: disabled
		//   channel priority: medium
		//   memory size: 8-bit
		//   peripheral size: 8-bit
		//   memory increment: enabled
		//   peripheral increment: disabled
		//   circular mode: disabled
		//   direction: read from memory
		DMA_ConfigChannel(
				USARTx->DMA_TX.Channel,
				DMA_MODE |
				DMA_DIR_M2P |
				DMA_MSIZE_8BIT | DMA_PSIZE_8BIT |
				DMA_MINC_ENABLE | DMA_PINC_DISABLE |
				DMA_PRIORITY_MEDIUM
			);
		DMA_SetAddrM(USARTx->DMA_TX.Channel, (uint32_t)pBuf);
		DMA_SetAddrP(USARTx->DMA_TX.Channel, (uint32_t)(&(USARTx->Instance->TDR)));
		DMA_SetDataLength(USARTx->DMA_TX.Channel, length);

		// Map DMA request to DMA channel
		DMA_SetRequest(USARTx->DMA_TX.Instance, USARTx->DMA_TX.Request, USARTx->DMA_TX.ChIndex);
	}
}

// Enable/disable the USART RX/TX DMA channels
// input:
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
//   DMA_DIR - DMA direction (combination of USART_DMA_xx values)
//   NewState - new state of the channels (ENABLE/DISABLE)
void USART_SetDMA(const USART_HandleTypeDef *USARTx, uint32_t DMA_DIR, FunctionalState NewState) {
	if (NewState == ENABLE) {
		// Clear the DMA interrupt flags and enable RX/TX DMA channels
		if (DMA_DIR & USART_DMA_TX) {
			DMA_ClearFlags(USARTx->DMA_TX.Instance, USARTx->DMA_TX.ChIndex, DMA_CF_ALL);
			DMA_EnableChannel(USARTx->DMA_TX.Channel);
		}
		if (DMA_DIR & USART_DMA_RX) {
			DMA_ClearFlags(USARTx->DMA_RX.Instance, USARTx->DMA_RX.ChIndex, DMA_CF_ALL);
			DMA_EnableChannel(USARTx->DMA_RX.Channel);
		}

		// Enable the USART TX/RX DMA
		USARTx->Instance->CR3 |= DMA_DIR;
	} else {
		// Disable the RX/TX DMA channels
		if (DMA_DIR & USART_DMA_TX) { DMA_DisableChannel(USARTx->DMA_TX.Channel); }
		if (DMA_DIR & USART_DMA_RX) { DMA_DisableChannel(USARTx->DMA_RX.Channel); }

		// Disable the USART TX/RX DMA
		USARTx->Instance->CR3 &= ~DMA_DIR;
	}
}

#endif // USART_USE_DMA

// Check the USART idle state
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
//   timeout - timeout value (for simple wait loop)
// return: rest of timeout counter in case of success, zero in case of timeout
uint32_t USART_CheckIdleState(USART_HandleTypeDef *USARTx, volatile uint32_t timeout) {
	if (USARTx->Instance->CR1 & (USART_CR1_TE | USART_CR1_RE)) {
		// Both transmitter and receiver enabled
		while (!(USARTx->Instance->ISR & (USART_ISR_TEACK | USART_ISR_REACK)) && --timeout);
	} else if (USARTx->Instance->CR1 & USART_CR1_TE) {
		// Only transmitter enabled
		while (!(USARTx->Instance->ISR & USART_ISR_TEACK) && --timeout);
	} else if (USARTx->Instance->CR1 & USART_CR1_RE) {
		// Only receiver enabled
		while (!(USARTx->Instance->ISR & USART_ISR_REACK) && --timeout);
	}

	return timeout;
}

// Transmit single character via UART
// input:
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
//   ch - character to send
void USART_SendChar(USART_TypeDef *USARTx, const char ch) {
	while (!(USARTx->ISR & USART_ISR_TXE)); // Wait while transmit data register not empty
	USARTx->TDR = ch; // Transmit character (TXE flag cleared)
}

// Transmit zero terminated string via USART
// input:
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
//   str - pointer to zero terminated string
void USART_SendStr(USART_TypeDef *USARTx, const char *str) {
	while (*str) {
		while (!(USARTx->ISR & USART_ISR_TXE)); // Wait while transmit data register not empty
		USARTx->TDR = *str++;
	}
}

// Transmit signed integer value as text via UART
// input
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
//   num - integer value to send
void USART_SendInt(USART_TypeDef *USARTx, int32_t num) {
	char str[10]; // 10 chars max for INT32_MAX
	int i = 0;

	if (num < 0) {
		USART_SendChar(USARTx, '-');
		num *= -1;
	}
	do { str[i++] = (num % 10) + '0'; } while ((num /= 10) > 0);
	while (i) { USART_SendChar(USARTx, str[--i]); }
}

// Transmit signed integer value with leading zero as text via UART
// input:
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
//   num - integer value to send
void USART_SendIntLZ(USART_TypeDef *USARTx, int32_t num) {
	char str[10]; // 10 chars max for INT32_MAX
	int i = 0;

	if (num < 0) {
		USART_SendChar(USARTx, '-');
		num *= -1;
	}
	if ((num < 10) && (num >= 0)) { USART_SendChar(USARTx, '0'); }
	do { str[i++] = (num % 10) + '0'; } while ((num /= 10) > 0);
	while (i) { USART_SendChar(USARTx, str[--i]); }
}

// Transmit unsigned integer value as text via UART
// input
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
//   num - unsigned integer value to send
void USART_SendIntU(USART_TypeDef *USARTx, uint32_t num) {
	char str[10]; // 10 chars max for UINT32_MAX
	int i = 0;

	do { str[i++] = (num % 10) + '0'; } while ((num /= 10) > 0);
	while (i) { USART_SendChar(USARTx, str[--i]); }
}

// Transmit binary value in HEX format
// input:
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
//   num - value to send
void USART_SendHex(USART_TypeDef *USARTx, uint32_t num) {
	char str[9];
	char *ptr = str;

	*ptr++ = '\0';
	do { *ptr++ = HEX_CHARS[num & 0x0F]; } while (num /= 0x10);
	while (*--ptr) { USART_SendChar(USARTx, *ptr); }
}

// Transmit binary value in HEX format with leading zeroes
// input:
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
//   num - value to send
//   digits - minimal number of digits in HEX format
void USART_SendHexLZ(USART_TypeDef *USARTx, uint32_t num, uint8_t digits) {
	char str[9];
	char *ptr = str;
	uint8_t len = 0;

	*ptr++ = '\0';
	do {
		*ptr++ = HEX_CHARS[num & 0x0F];
		len++;
	} while (num /= 0x10);

	if (len < digits) {
		while (len++ < digits) { USART_SendChar(USARTx, '0'); }
	}

	while (*--ptr) { USART_SendChar(USARTx, *ptr); }
}

// Transmit data buffer
// input:
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
//   pBuf - pointer to the buffer
//   length - size of buffer in bytes
void USART_SendBuf(USART_TypeDef *USARTx, const char *pBuf, uint32_t length) {
	while (length--) {
		while (!(USARTx->ISR & USART_ISR_TXE)); // Wait while transmit data register not empty
		USARTx->TDR = *pBuf++; // Transmit character (TXE flag cleared)
	}
}

// Transmit buffer with substitute for unprintable characters
// input:
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
//   pBuf - pointer to the buffer
//   length - size of buffer in bytes
//   subst - character for substitute
void USART_SendBufPrintable(USART_TypeDef *USARTx, const char *pBuf, uint32_t length, const char subst) {
	register uint8_t ch;

	while (length--) {
		ch = *pBuf++;
		while (!(USARTx->ISR & USART_ISR_TXE)); // Wait while transmit data register not empty
		USARTx->TDR = (ch > 32) ? ch : subst;
	}
}

// Transmit buffer in HEX format
// input:
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
//   pBuf - pointer to the buffer
//   length - size of buffer in bytes
void USART_SendBufHex(USART_TypeDef *USARTx, const char *pBuf, uint32_t length) {
	register uint8_t ch;

	while (length--) {
		ch = *pBuf++;
		while (!(USARTx->ISR & USART_ISR_TXE));
		USARTx->TDR = HEX_CHARS[ch >> 4];
		while (!(USARTx->ISR & USART_ISR_TXE));
		USARTx->TDR = HEX_CHARS[ch & 0x0F];
	}
}

#if (USART_USE_PRINTF)

#include <stdarg.h>
// Transmit formatted string via USART
// input:
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
//   fmt - format string
//   ... - optional arguments
// return: number of transmitted characters
// note:
//   supported sub-set of standard format:
//     type:
//       i, d - signed integer
//       u    - unsinged integer
//       b    - binary
//       o    - octal
//       x    - hexadecimal in lowercase
//       X    - hexadecimal in uppercase
//       c    - character
//       s    - string
//     flag:
//       0    - add zero padding
//       -    - left justify field
//       .X   - add a decimal point (on 'X' place must be a single digit
//              determining number of digits after the decimal point)
// examples:
//   ("%d", 1234)     --> "1234"
//   ("%-5d", 123)    --> "123  "
//   ("%5d", 123)     --> "  123"
//   ("%05d", 123)    --> "00123"
//   ("%2s", "USART") --> "US"
//   ("%.3u", 123456) --> "123.456"
//   ("%-8.2u", 1234) --> "12.34   "
int USART_printf(USART_TypeDef *USARTx, const char *fmt, ...) {
	va_list ap;        // arguments pointer
	uint8_t flag;      // field flag
	uint8_t width;     // specified field width
	uint8_t prec;      // specified precision
	uint8_t len = 0;   // actual field length
	uint8_t radix;     // value conversion radix
	uint32_t num;      // value to convert
	char str[33];      // maximal width of 32-bit binary + string terminating symbol
	char *pstr, *tstr; // pointers for various string manipulations
	char chr;          // to hold a single character from format string

	// Initialize an arguments pointer
	va_start(ap, fmt);

	// Process format string
	while (1) {
		// Get character from the format string
		chr = *fmt++;

		// String termination?
		if (!chr) { break; }

		// Non escape character
		if (chr != '%') {
			USART_SendChar(USARTx, chr);
			continue;
		}

		// Next after an escape character
		chr = *fmt++;

		// Flag?
		flag = 0x00;
		if (chr == '0') {
			// Flag '0' --> padding
			flag = 0x01;
			chr = *fmt++;
		} else if (chr == '-') {
			// Flag '-' --> left justified
			flag = 0x02;
			chr = *fmt++;
		}

		// Width?
		width = 0;
		while ((chr > '0' - 1) && (chr < '9' + 1)) {
			width *= 10;
			width += chr - '0';
			chr = *fmt++;
		}

		// Precision?
		prec = 0;
		if (chr == '.') {
			chr = *fmt++;
			if ((chr > '0' - 1) && (chr < '9' + 1)) {
				// Flag '.' --> number with decimal point
				prec = chr - '0';
				if (prec) { flag |= 0x08; }
				chr = *fmt++;
			}
		}

		// Ensure a string is not terminated yet
		if (!chr) { break; }

		// A type is...
		// (chr | 0x20) --> convert character to lower case
		switch (chr | 0x20) {
			case 's':
				// String

				// Calculate string length
				len = 0;
				tstr = pstr = va_arg(ap, char*);
				while (*pstr++) { len++; }

				// Transmit leading spaces if string length is less than field width
				if (!(flag & 0x02) && (len < width)) {
					while (len++ < width) {
						USART_SendChar(USARTx, ' ');
					}
				}

				// Transmit string
				pstr = tstr;
				if (width) len = width;
				while (*pstr) {
					USART_SendChar(USARTx, *pstr++);
					if (!(--len)) { break; }
				}

				// Transmit trailing spaces in case of left justified field
				if (flag & 0x02) {
					while (len--) {
						USART_SendChar(USARTx, ' ');
					}
				}

				continue;
			case 'c':
				// Character
				USART_SendChar(USARTx, (char)va_arg(ap, int));
				continue;
			case 'b':
				// Binary
				num = (uint32_t)va_arg(ap, uint32_t);
				radix = 2;
				break;
			case 'o':
				// Octal
				num = (uint32_t)va_arg(ap, uint32_t);
				radix = 8;
				break;
			case 'i':
			case 'd':
				// Signed decimal
				num = (int32_t)va_arg(ap, int32_t);
				if (num & 0x80000000) {
					num = 0 - num;
					flag |= 0x04;
				}
				radix = 10;
				break;
			case 'u':
				// Unsigned decimal
				num = (uint32_t)va_arg(ap, uint32_t);
				radix = 10;
				break;
			case 'x':
				// Hexadecimal
				num = (uint32_t)va_arg(ap, uint32_t);
				radix = 16;
				break;
			default:
				// Unknown: just pass
				USART_SendChar(USARTx, chr);
				continue;
		}

		// Convert value to characters
		pstr = str;
		*pstr++ = '\0';
		len = 0;
		if (num) {
			// Value is not a zero
			do {
				*pstr = (num % radix);
				if (*pstr > 9) {
					*pstr += (chr == 'x') ? 0x27 : 0x07;
				}
				*pstr++ += '0';
				len++;
				if (flag & 0x08) {
					if (len == prec) {
						*pstr++ = '.';
						len++;
					}
				}
			} while (num /= radix);

			// In case of format with decimal point the leading zeroes must be added if
			// current length of the number is less than a specified precision
			if (flag & 0x08) {
				if (len == prec + 1) {
					*pstr++ = '0';
					len++;
				} else if (len <= prec) {
					while (len < prec) {
						*pstr++ = '0';
						len++;
					}
					*pstr++ = '.';
					*pstr++ = '0';
					len += 2;
				}
			}

			// Add negative symbol
			if (flag & 0x04) {
				*pstr++ = '-';
				len++;
			}
		} else {
			// Little optimization for a zero value
			if (flag & 0x08) {
				while (len++ < prec) { *pstr++ = '0'; }
				*pstr++ = '.';
				len++;
			} else len++;
			*pstr++ = '0';
		}

		// Add leading zeroes or spaces
		prec += (flag & 0x08) ? (width - prec) : width;
		if (!(flag & 0x02) && (len < prec)) {
			while (len++ < prec) {
				*pstr++ = (flag & 0x01) ? '0' : ' ';
			}
		}

		// Transmit value as text
		while (*--pstr) {
			USART_SendChar(USARTx, *pstr);
		}

		// Transmit trailing spaces
		if (flag & 0x02) {
			while (len++ < width) {
				USART_SendChar(USARTx, ' ');
			}
		}
	}

	// Cleanup for an arguments pointer
	va_end(ap);

	// TODO: USART: return an actual length of the transmitted data
	return 0;
}

#endif // USART_USE_PRINTF
