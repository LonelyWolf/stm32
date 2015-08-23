// Universal Synchronous/Asynchronous Receiver Transmitter (USART) management


#include <stm32l1xx_rcc.h>

#include "uart.h"


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
#if (USART4_USE)
USART_HandleTypeDef hUSART4;
#endif
#if (USART5_USE)
USART_HandleTypeDef hUSART5;
#endif


#if (USART1_USE)
// Populate the USART1 handle
void USART1_HandleInit(void) {
	hUSART1.Instance = USART1;
	hUSART1.AF       = GPIO_AFIO7;

	// TX [PA9, PB6]
	hUSART1.PIN_TX.GPIO_AHB = RCC_AHBENR_GPIOAEN;
	hUSART1.PIN_TX.GPIO     = GPIOA;
	hUSART1.PIN_TX.GPIO_PIN = GPIO_Pin_9;
	hUSART1.PIN_TX.GPIO_SRC = GPIO_PinSource9;

	// RX [PA10, PB7]
	hUSART1.PIN_RX.GPIO_AHB = RCC_AHBENR_GPIOAEN;
	hUSART1.PIN_RX.GPIO     = GPIOA;
	hUSART1.PIN_RX.GPIO_PIN = GPIO_Pin_10;
	hUSART1.PIN_RX.GPIO_SRC = GPIO_PinSource10;

#if (USART_USE_DMA)
	// DMA TX channel
	hUSART1.DMA_TX.Instance = DMA1;
	hUSART1.DMA_TX.Channel  = DMA1_Channel4;
	hUSART1.DMA_TX.CF       = DMA_IFCR_CGIF4 | DMA_IFCR_CHTIF4 | DMA_IFCR_CTCIF4 | DMA_IFCR_CTEIF4;
	hUSART1.DMA_TX.TCIF     = DMA_ISR_TCIF4;
	hUSART1.DMA_TX.HTIF     = DMA_ISR_HTIF4;
	hUSART1.DMA_TX.State    = DMA_STATE_RESET;

	// DMA RX channel
	hUSART1.DMA_RX.Instance = DMA1;
	hUSART1.DMA_RX.Channel  = DMA1_Channel5;
	hUSART1.DMA_RX.CF       = DMA_IFCR_CGIF5 | DMA_IFCR_CHTIF5 | DMA_IFCR_CTCIF5 | DMA_IFCR_CTEIF5;
	hUSART1.DMA_RX.TCIF     = DMA_ISR_TCIF5;
	hUSART1.DMA_RX.HTIF     = DMA_ISR_HTIF5;
	hUSART1.DMA_RX.State    = DMA_STATE_RESET;
#endif // USART_USE_DMA
}
#endif // USART1_USE

#if (USART2_USE)
// Populate the USART2 handle
void USART2_HandleInit(void) {
	hUSART2.Instance = USART2;
	hUSART2.AF       = GPIO_AFIO7;

	// TX [PA2]
	hUSART2.PIN_TX.GPIO_AHB = RCC_AHBENR_GPIOAEN;
	hUSART2.PIN_TX.GPIO     = GPIOA;
	hUSART2.PIN_TX.GPIO_PIN = GPIO_Pin_2;
	hUSART2.PIN_TX.GPIO_SRC = GPIO_PinSource2;

	// RX [PA3]
	hUSART2.PIN_RX.GPIO_AHB = RCC_AHBENR_GPIOAEN;
	hUSART2.PIN_RX.GPIO     = GPIOA;
	hUSART2.PIN_RX.GPIO_PIN = GPIO_Pin_3;
	hUSART2.PIN_RX.GPIO_SRC = GPIO_PinSource3;

#if (USART_USE_DMA)
	// DMA TX channel
	hUSART2.DMA_TX.Instance = DMA1;
	hUSART2.DMA_TX.Channel  = DMA1_Channel7;
	hUSART2.DMA_TX.CF       = DMA_IFCR_CGIF7 | DMA_IFCR_CHTIF7 | DMA_IFCR_CTCIF7 | DMA_IFCR_CTEIF7;
	hUSART2.DMA_TX.TCIF     = DMA_ISR_TCIF7;
	hUSART2.DMA_TX.HTIF     = DMA_ISR_HTIF7;
	hUSART2.DMA_TX.State    = DMA_STATE_RESET;

	// DMA RX channel
	hUSART2.DMA_RX.Instance = DMA1;
	hUSART2.DMA_RX.Channel  = DMA1_Channel6;
	hUSART2.DMA_RX.CF       = DMA_IFCR_CGIF6 | DMA_IFCR_CHTIF6 | DMA_IFCR_CTCIF6 | DMA_IFCR_CTEIF6;
	hUSART2.DMA_RX.TCIF     = DMA_ISR_TCIF6;
	hUSART2.DMA_RX.HTIF     = DMA_ISR_HTIF6;
	hUSART2.DMA_RX.State    = DMA_STATE_RESET;
#endif // USART_USE_DMA
}
#endif // USART2_USE

#if (USART3_USE)
// Populate the USART3 handle
void USART3_HandleInit(void) {
	hUSART3.Instance = USART3;
	hUSART3.AF       = GPIO_AFIO7;

	// TX [PB10, PC10]
	hUSART3.PIN_TX.GPIO_AHB = RCC_AHBENR_GPIOBEN;
	hUSART3.PIN_TX.GPIO     = GPIOB;
	hUSART3.PIN_TX.GPIO_PIN = GPIO_Pin_10;
	hUSART3.PIN_TX.GPIO_SRC = GPIO_PinSource10;

	// RX [PB11, PC11]
	hUSART3.PIN_RX.GPIO_AHB = RCC_AHBENR_GPIOBEN;
	hUSART3.PIN_RX.GPIO     = GPIOB;
	hUSART3.PIN_RX.GPIO_PIN = GPIO_Pin_11;
	hUSART3.PIN_RX.GPIO_SRC = GPIO_PinSource11;

#ifdef USART_USE_DMA
	// DMA TX channel
	hUSART3.DMA_TX.Instance = DMA1;
	hUSART3.DMA_TX.Channel  = DMA1_Channel2;
	hUSART3.DMA_TX.CF       = DMA_IFCR_CGIF2 | DMA_IFCR_CHTIF2 | DMA_IFCR_CTCIF2 | DMA_IFCR_CTEIF2;
	hUSART3.DMA_TX.TCIF     = DMA_ISR_TCIF2;
	hUSART3.DMA_TX.HTIF     = DMA_ISR_HTIF2;
	hUSART3.DMA_TX.State    = DMA_STATE_RESET;

	// DMA RX channel
	hUSART3.DMA_RX.Instance = DMA1;
	hUSART3.DMA_RX.Channel  = DMA1_Channel3;
	hUSART3.DMA_RX.CF       = DMA_IFCR_CGIF3 | DMA_IFCR_CHTIF3 | DMA_IFCR_CTCIF3 | DMA_IFCR_CTEIF3;
	hUSART3.DMA_RX.TCIF     = DMA_ISR_TCIF3;
	hUSART3.DMA_RX.HTIF     = DMA_ISR_HTIF3;
	hUSART3.DMA_RX.State    = DMA_STATE_RESET;
#endif // USART_USE_DMA
}
#endif // USART3_USE

#if (USART4_USE)
// Populate the USART4 handle
void USART4_HandleInit(void) {
	hUSART4.Instance = UART4;
	hUSART4.AF       = GPIO_AFIO8;

	// TX [PC10]
	hUSART4.PIN_TX.GPIO_AHB = RCC_AHBENR_GPIOCEN;
	hUSART4.PIN_TX.GPIO     = GPIOC;
	hUSART4.PIN_TX.GPIO_PIN = GPIO_Pin_10;
	hUSART4.PIN_TX.GPIO_SRC = GPIO_PinSource10;

	// RX [PC11]
	hUSART4.PIN_RX.GPIO_AHB = RCC_AHBENR_GPIOCEN;
	hUSART4.PIN_RX.GPIO     = GPIOC;
	hUSART4.PIN_RX.GPIO_PIN = GPIO_Pin_11;
	hUSART4.PIN_RX.GPIO_SRC = GPIO_PinSource11;

#ifdef USART_USE_DMA
	// DMA TX channel
	hUSART4.DMA_TX.Instance = DMA2;
	hUSART4.DMA_TX.Channel  = DMA2_Channel5;
	hUSART4.DMA_TX.CF       = DMA_IFCR_CGIF5 | DMA_IFCR_CHTIF5 | DMA_IFCR_CTCIF5 | DMA_IFCR_CTEIF5;
	hUSART4.DMA_TX.TCIF     = DMA_ISR_TCIF5;
	hUSART4.DMA_TX.HTIF     = DMA_ISR_HTIF5;
	hUSART4.DMA_TX.State    = DMA_STATE_RESET;

	// DMA RX channel
	hUSART4.DMA_RX.Instance = DMA2;
	hUSART4.DMA_RX.Channel  = DMA2_Channel3;
	hUSART4.DMA_RX.CF       = DMA_IFCR_CGIF3 | DMA_IFCR_CHTIF3 | DMA_IFCR_CTCIF3 | DMA_IFCR_CTEIF3;
	hUSART4.DMA_RX.TCIF     = DMA_ISR_TCIF3;
	hUSART4.DMA_RX.HTIF     = DMA_ISR_HTIF3;
	hUSART4.DMA_RX.State    = DMA_STATE_RESET;
#endif // USART_USE_DMA
}
#endif // USART4_USE

#if (USART5_USE)
// Populate the USART5 handle
void USART5_HandleInit(void) {
	hUSART5.Instance = UART5;
	hUSART5.AF       = GPIO_AFIO8;

	// TX [PC12]
	hUSART5.PIN_TX.GPIO_AHB = RCC_AHBENR_GPIOCEN;
	hUSART5.PIN_TX.GPIO     = GPIOC;
	hUSART5.PIN_TX.GPIO_PIN = GPIO_Pin_12;
	hUSART5.PIN_TX.GPIO_SRC = GPIO_PinSource12;

	// RX [PD2]
	hUSART5.PIN_RX.GPIO_AHB = RCC_AHBENR_GPIODEN;
	hUSART5.PIN_RX.GPIO     = GPIOD;
	hUSART5.PIN_RX.GPIO_PIN = GPIO_Pin_2;
	hUSART5.PIN_RX.GPIO_SRC = GPIO_PinSource2;

#ifdef USART_USE_DMA
	// DMA TX channel
	hUSART5.DMA_TX.Instance = DMA2;
	hUSART5.DMA_TX.Channel  = DMA2_Channel1;
	hUSART5.DMA_TX.CF       = DMA_IFCR_CGIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CTEIF1;
	hUSART5.DMA_TX.TCIF     = DMA_ISR_TCIF1;
	hUSART5.DMA_TX.HTIF     = DMA_ISR_HTIF1;
	hUSART5.DMA_TX.State    = DMA_STATE_RESET;

	// DMA RX channel
	hUSART5.DMA_RX.Instance = DMA2;
	hUSART5.DMA_RX.Channel  = DMA2_Channel2;
	hUSART5.DMA_RX.CF       = DMA_IFCR_CGIF2 | DMA_IFCR_CHTIF2 | DMA_IFCR_CTCIF2 | DMA_IFCR_CTEIF2;
	hUSART5.DMA_RX.TCIF     = DMA_ISR_TCIF2;
	hUSART5.DMA_RX.HTIF     = DMA_ISR_HTIF2;
	hUSART5.DMA_RX.State    = DMA_STATE_RESET;
#endif
}
#endif // USART5_USE


// Initialize and configure UART peripheral with specified baudrate
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   USART_DIR - RX/TX enable (combination of USART_RX and USART_TX values)
//   baudrate - UART speed (bits/s)
void USART_Init(const USART_HandleTypeDef *USARTx, uint16_t USART_DIR, uint32_t baudrate) {
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
		RCC->APB1RSTR |=  RCC_APB1RSTR_USART2RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_USART2RST;
		// Enable the USART2 peripheral clock
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	}
#endif // USART2_USE
#if (USART3_USE)
	if (USARTx->Instance == USART3) {
		// Reset the USART3 peripheral
		RCC->APB1RSTR |=  RCC_APB1RSTR_USART3RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_USART3RST;
		// Enable the USART3 peripheral clock
		RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
	}
#endif // USART3_USE
#if (USART4_USE)
	if (USARTx->Instance == UART4) {
		// Reset the UART4 peripheral
		RCC->APB1RSTR |=  RCC_APB1RSTR_UART4RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_UART4RST;
		// Enable the UART4 peripheral clock
		RCC->APB1ENR |= RCC_APB1ENR_UART4EN;
	}
#endif // USART4_USE
#if (USART5_USE)
	if (USARTx->Instance == UART5) {
		// Reset the UART5 peripheral
		RCC->APB1RSTR |=  RCC_APB1RSTR_UART5RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_UART5RST;
		// Enable the UART5 peripheral clock
		RCC->APB1ENR |= RCC_APB1ENR_UART5EN;
	}
#endif // USART5_USE

	// TX pin
	if (USART_DIR & USART_TX) {
		RCC->AHBENR |= USARTx->PIN_TX.GPIO_AHB;
		GPIO_set_mode(USARTx->PIN_TX.GPIO,GPIO_Mode_AF,GPIO_PUPD_PU,USARTx->PIN_TX.GPIO_PIN);
		GPIO_out_cfg(USARTx->PIN_TX.GPIO,GPIO_OT_PP,GPIO_SPD_HIGH,USARTx->PIN_TX.GPIO_PIN);
		GPIO_af_cfg(USARTx->PIN_TX.GPIO,USARTx->PIN_TX.GPIO_SRC,USARTx->AF);
	}

	// RX pin
	if (USART_DIR & USART_RX) {
		RCC->AHBENR |= USARTx->PIN_RX.GPIO_AHB;
		GPIO_set_mode(USARTx->PIN_RX.GPIO,GPIO_Mode_AF,GPIO_PUPD_PU,USARTx->PIN_RX.GPIO_PIN);
		GPIO_out_cfg(USARTx->PIN_RX.GPIO,GPIO_OT_PP,GPIO_SPD_HIGH,USARTx->PIN_RX.GPIO_PIN);
		GPIO_af_cfg(USARTx->PIN_RX.GPIO,USARTx->PIN_RX.GPIO_SRC,USARTx->AF);
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
	USARTx->Instance->CR1 = USART_DIR;

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
	USARTx->Instance->CR2 = 0x0000;

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
	USARTx->Instance->CR3 = 0x0000;

	// Set given baud rate
	USART_SetSpeed(USARTx,baudrate);

	// Enable the USART
	USARTx->Instance->CR1 |= USART_CR1_UE;
}

// Configure the USART port at given baudrate
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   baudrate - port speed in bps (bits per second)
void USART_SetSpeed(const USART_HandleTypeDef *USARTx, uint32_t baudrate) {
	uint32_t apbclock;
	uint32_t brr;
	uint32_t idiv = 0x00;
	uint32_t fdiv = 0x00;
	RCC_ClocksTypeDef RCC_ClocksStatus;

	// Get APB clock
	RCC_GetClocksFreq(&RCC_ClocksStatus);
	if (USARTx->Instance == USART1) {
		apbclock = RCC_ClocksStatus.PCLK2_Frequency;
	} else {
		apbclock = RCC_ClocksStatus.PCLK1_Frequency;
	}

	// Integer part
	if (USARTx->Instance->CR1 & USART_CR1_OVER8) {
		// Oversampling by 8
		idiv = ((25 * apbclock) / (baudrate << 1));
	} else {
		// Oversampling by 16
		idiv = ((25 * apbclock) / (baudrate << 2));
	}
	brr = (idiv / 100) << 4;

	// Fractional part
	fdiv = idiv - (100 * (brr >> 4));

	// Calculate and write new value of baudrate register
	if (USARTx->Instance->CR1 & USART_CR1_OVER8) {
		// Oversampling by 8
		brr |= ((((fdiv << 3) + 50) / 100)) & ((uint8_t)0x07);
	} else {
		// Oversampling by 16
		brr |= ((((fdiv << 4) + 50) / 100)) & ((uint8_t)0x0F);
	}
	USARTx->Instance->BRR = (uint16_t)brr;
}

// Initialize the USART IRQ
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   USART_IRQ - IRQ's to enable (combination of USART_IRQ_XXX values)
void USART_InitIRQ(const USART_HandleTypeDef *USARTx, uint32_t USART_IRQ) {
	// Clear the USART flags
	USARTx->Instance->SR &= ~(USART_SR_RXNE & USART_SR_TC);

	// Enable the specified USART IRQ(s)
	USARTx->Instance->CR1 |= USART_IRQ;

	// Enable the IRQ according to given USART port
	if (USARTx->Instance == USART1) {
		NVIC_EnableIRQ(USART1_IRQn);
	} else if (USARTx->Instance == USART2) {
		NVIC_EnableIRQ(USART2_IRQn);
	} else if (USARTx->Instance == USART3) {
		NVIC_EnableIRQ(USART3_IRQn);
	}
}

#if (USART_USE_DMA)
// Configure the UART TX/RX DMA channels
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   DMA_DIR - DMA direction (combination of USART_DMA_xx values)
//   DMA_BUF - DMA buffer mode (one of USART_DMA_BUF_XXX values: circular or normal)
//   pBuf - pointer to the data buffer
//   length - length of the data buffer
// note: the corresponding DMA peripheral clock must be already enabled
void USART_ConfigureDMA(const USART_HandleTypeDef *USARTx, uint8_t DMA_DIR, uint32_t DMA_BUF, uint8_t *pBuf, uint32_t length) {
	if (DMA_DIR & USART_DMA_RX) {
		// USART RX DMA channel configuration:
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
		USARTx->DMA_RX.Channel->CCR   = DMA_CCR1_MINC | DMA_CCR1_PL_0 | DMA_BUF;
		USARTx->DMA_RX.Channel->CPAR  = (uint32_t)(&(USARTx->Instance->DR)); // Address of the peripheral data register
		USARTx->DMA_RX.Channel->CMAR  = (uint32_t)pBuf; // Memory address
		USARTx->DMA_RX.Channel->CNDTR = length; // Number of DMA transactions
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
		//   IRQ: disabled
		//   channel: disabled
		USARTx->DMA_TX.Channel->CCR   = DMA_CCR1_DIR | DMA_CCR1_MINC | DMA_CCR1_PL_0 | DMA_BUF;
		USARTx->DMA_TX.Channel->CPAR  = (uint32_t)(&(USARTx->Instance->DR)); // Address of the peripheral data register
		USARTx->DMA_TX.Channel->CMAR  = (uint32_t)pBuf; // Memory address
		USARTx->DMA_TX.Channel->CNDTR = length; // Number of DMA transactions
	}
}

// Enable/disable the USART RX/TX DMA channels
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   DMA_DIR - DMA direction (combination of USART_DMA_xx values)
//   NewState - new state of the channels (ENABLE/DISABLE)
void USART_SetDMA(const USART_HandleTypeDef *USARTx, uint8_t DMA_DIR, FunctionalState NewState) {
	if (NewState == ENABLE) {
		// Clear the DMA interrupt flags and enable RX/TX DMA channels
		if (DMA_DIR & USART_DMA_TX) {
			USARTx->DMA_TX.Instance->IFCR = USARTx->DMA_TX.CF;
			USARTx->DMA_TX.Channel->CCR  |= DMA_CCR1_EN;
		}
		if (DMA_DIR & USART_DMA_RX) {
			USARTx->DMA_RX.Instance->IFCR = USARTx->DMA_RX.CF;
			USARTx->DMA_RX.Channel->CCR  |= DMA_CCR1_EN;
		}

		// Enable the USART TX/RX DMA
		USARTx->Instance->CR3 |= DMA_DIR;
	} else {
		// Disable the RX/TX DMA channels
		if (DMA_DIR & USART_DMA_TX) USARTx->DMA_TX.Channel->CCR &= ~DMA_CCR1_EN;
		if (DMA_DIR & USART_DMA_RX) USARTx->DMA_RX.Channel->CCR &= ~DMA_CCR1_EN;

		// Disable the USART TX/RX DMA
		USARTx->Instance->CR3 &= ~DMA_DIR;
	}
}
#endif // USART_USE_DMA

// Send single character to UART
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   ch - character to send
void USART_SendChar(USART_TypeDef* USARTx, const char ch) {
	while (!(USARTx->SR & USART_SR_TXE)); // Wait while transmit data register not empty
	USARTx->DR = ch; // Transmit character (TXE flag cleared)
}

// Send zero terminated string to USART
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   str - pointer to zero terminated string
void USART_SendStr(USART_TypeDef* USARTx, const char *str) {
	while (*str) {
		while (!(USARTx->SR & USART_SR_TXE)); // Wait while transmit data register not empty
		USARTx->DR = *str++;
	}
}

// Send signed integer value as text to UART
// input
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   num - integer value to send
void USART_SendInt(USART_TypeDef* USARTx, int32_t num) {
	char str[10]; // 10 chars max for INT32_MAX
	int i = 0;

	if (num < 0) {
		USART_SendChar(USARTx,'-');
		num *= -1;
	}
	do str[i++] = num % 10 + '0'; while ((num /= 10) > 0);
	while (i) USART_SendChar(USARTx,str[--i]);
}

// Send signed integer value with leading zero as text to UART
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   num - integer value to send
void USART_SendIntLZ(USART_TypeDef* USARTx, int32_t num) {
	char str[10]; // 10 chars max for INT32_MAX
	int i = 0;

	if (num < 0) {
		USART_SendChar(USARTx,'-');
		num *= -1;
	}
	if ((num < 10) && (num >= 0)) USART_SendChar(USARTx,'0');
	do str[i++] = num % 10 + '0'; while ((num /= 10) > 0);
	while (i) USART_SendChar(USARTx,str[--i]);
}

// Send unsigned integer value as text to UART
// input
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   num - unsigned integer value to send
void USART_SendIntU(USART_TypeDef* USARTx, uint32_t num) {
	char str[10]; // 10 chars max for UINT32_MAX
	int i = 0;

	do str[i++] = num % 10 + '0'; while ((num /= 10) > 0);
	while (i) USART_SendChar(USARTx,str[--i]);
}

// Send binary value in HEX format
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   num - value to send
void USART_SendHex(USART_TypeDef* USARTx, uint32_t num) {
	char str[9];
	char *ptr = str;

	*ptr++ = '\0';
	do { *ptr++ = HEX_CHARS[num % 0x10]; } while (num /= 0x10);

	while (*--ptr) USART_SendChar(USARTx,*ptr);
}

// Send binary value in HEX format with leading zeroes
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   num - value to send
//   digits - minimal number of digits in HEX format
void USART_SendHexLZ(USART_TypeDef* USARTx, uint32_t num, uint8_t digits) {
	char str[9];
	char *ptr = str;
	uint8_t len = 0;

	*ptr++ = '\0';
	do {
		*ptr++ = HEX_CHARS[num % 0x10];
		len++;
	} while (num /= 0x10);
	if (len < digits) while (len++ < digits) USART_SendChar(USARTx,'0');

	while (*--ptr) USART_SendChar(USARTx,*ptr);
}

// Send buffer to USART
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   pBuf - pointer to the buffer
//   length - size of buffer in bytes
void USART_SendBuf(USART_TypeDef* USARTx, const char *pBuf, uint16_t length) {
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
void USART_SendBufPrintable(USART_TypeDef* USARTx, const char *pBuf, uint16_t length, const char subst) {
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
void USART_SendBufHex(USART_TypeDef* USARTx, const char *pBuf, uint16_t length) {
	char ch;

	while (length--) {
		ch = *pBuf++;
		while (!(USARTx->SR & USART_SR_TXE)); // Wait while transmit data register not empty
		USARTx->DR = HEX_CHARS[(ch >> 4)   % 0x10];
		while (!(USARTx->SR & USART_SR_TXE)); // Wait while transmit data register not empty
		USARTx->DR = HEX_CHARS[(ch & 0x0f) % 0x10];
	}
}

#if (USART_USE_PRINTF)
#include <stdarg.h>
// Transmit formatted string to the USART
// input:
//   USARTx - pointer to the USART port (USART1, USART2, etc.)
//   fmt - format string
//   ... - optional arguments
// return: number of transmitted characters
// note:
//   supported sub-set of standard format:
//     type:
//       i,d - signed integer
//       u   - unsinged integer
//       b   - binary
//       o   - octal
//       x   - hexadecimal in lowercase
//       X   - hexadecimal in uppercase
//       c   - character
//       s   - string
//     flag:
//       0   - add zero padding
//       -   - left justify field
//       .X  - add a decimal point (on 'X' place must be a single digit
//             determining number of digits after the decimal point)
// examples:
//   ("%d",1234)     --> "1234"
//   ("%-5d",123)    --> "123  "
//   ("%5d",123)     --> "  123"
//   ("%05d",123)    --> "00123"
//   ("%2s","USART") --> "US"
//   ("%.3u",123456) --> "123.456"
//   ("%-8.2u",1234) --> "12.34   "
int USART_printf(USART_TypeDef* USARTx, const char *fmt, ...) {
	va_list ap;       // arguments pointer
	uint8_t flag;     // field flag
	uint8_t width;    // specified field width
	uint8_t prec;     // specified precision
	uint8_t len = 0;  // actual field length
	uint8_t radix;    // value conversion radix
	uint32_t num;     // value to convert
	char str[33];     // maximal width of 32-bit binary + string terminating symbol
	char *pstr,*tstr; // pointers for various string manipulations
	char chr;         // to hold a single character from format string

	// Initialize an arguments pointer
	va_start(ap,fmt);

	// Process format string
	while (1) {
		// Get character from the format string
		chr = *fmt++;

		// String termination?
		if (!chr) break;

		// Non escape character
		if (chr != '%') {
			USART_SendChar(USARTx,chr);
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
				if (prec) flag |= 0x08;
				chr = *fmt++;
			}
		}

		// Ensure a string is not terminated yet
		if (!chr) break;

		// A type is...
		// (chr | 0x20) --> convert character to lower case
		switch (chr | 0x20) {
			case 's':
				// String

				// Calculate string length
				len = 0;
				tstr = pstr = va_arg(ap,char*);
				while (*pstr++) len++;

				// Transmit leading spaces if string length is less than field width
				if (!(flag & 0x02) && (len < width)) while (len++ < width) USART_SendChar(USARTx,' ');

				// Transmit string
				pstr = tstr;
				if (width) len = width;
				while (*pstr) {
					USART_SendChar(USARTx,*pstr++);
					if (!(--len)) break;
				}

				// Transmit trailing spaces in case of left justified field
				if (flag & 0x02) while (len--) USART_SendChar(USARTx,' ');

				continue;
			case 'c':
				// Character
				USART_SendChar(USARTx,(char)va_arg(ap,int));
				continue;
			case 'b':
				// Binary
				num = (uint32_t)va_arg(ap,uint32_t);
				radix = 2;
				break;
			case 'o':
				// Octal
				num = (uint32_t)va_arg(ap,uint32_t);
				radix = 8;
				break;
			case 'i':
			case 'd':
				// Signed decimal
				num = (int32_t)va_arg(ap,int32_t);
				if (num & 0x80000000) {
					num = 0 - num;
					flag |= 0x04;
				}
				radix = 10;
				break;
			case 'u':
				// Unsigned decimal
				num = (uint32_t)va_arg(ap,uint32_t);
				radix = 10;
				break;
			case 'x':
				// Hexadecimal
				num = (uint32_t)va_arg(ap,uint32_t);
				radix = 16;
				break;
			default:
				// Unknown: just pass
				USART_SendChar(USARTx,chr);
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
				if (*pstr > 9) *pstr += (chr == 'x') ? 0x27 : 0x07;
				*pstr++ += '0';
				len++;
				if (flag & 0x08) {
					if (len == prec) {
						*pstr++ = '.';
						len++;
					}
				}
			} while (num /= radix);

			// In case of format with decimal point it must be added leading zeroes if
			// current length of the number is less than specified precision
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
				while (len++ < prec) *pstr++ = '0';
				*pstr++ = '.';
				len++;
			} else len++;
			*pstr++ = '0';
		}

		// Add leading zeroes or spaces
		prec += width;
		if (!(flag & 0x02) && (len < prec)) while (len++ < prec) *pstr++ = (flag & 0x01) ? '0' : ' ';

		// Transmit value as text
		while (*--pstr) USART_SendChar(USARTx,*pstr);

		// Transmit trailing spaces
		if (flag & 0x02) while (len++ < width) USART_SendChar(USARTx,' ');
	}

	// Cleanup for an arguments pointer
	va_end(ap);

	// FIXME: return an actual length of the transmitted data
	return 0;
}
#endif // USART_USE_PRINTF
