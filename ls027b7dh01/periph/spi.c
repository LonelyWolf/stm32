// Serial peripheral interface (SPI) management


#include "spi.h"


// SPI port handles
#if (SPI1_USE)
SPI_HandleTypeDef hSPI1;
#endif
#if (SPI2_USE)
SPI_HandleTypeDef hSPI2;
#endif
#if (SPI3_USE)
SPI_HandleTypeDef hSPI3;
#endif


#if (SPI1_USE)
// Populate the SPI1 handle
void SPI1_HandleInit(void) {
	hSPI1.Instance = SPI1;
	hSPI1.AF       = GPIO_AF5;

	// SCK pin [PA5, PB3]
	hSPI1.PIN_SCK.GPIO_AHB = RCC_AHB2ENR_GPIOAEN;
	hSPI1.PIN_SCK.GPIO     = GPIOA;
	hSPI1.PIN_SCK.GPIO_PIN = GPIO_PIN_5;
	hSPI1.PIN_SCK.GPIO_SRC = GPIO_PinSource5;

	// MISO pin [PA6, PB4]
	hSPI1.PIN_MISO.GPIO_AHB = RCC_AHB2ENR_GPIOAEN;
	hSPI1.PIN_MISO.GPIO     = GPIOA;
	hSPI1.PIN_MISO.GPIO_PIN = GPIO_PIN_6;
	hSPI1.PIN_MISO.GPIO_SRC = GPIO_PinSource6;

	// MOSI pin [PA7, PB5]
	hSPI1.PIN_MOSI.GPIO_AHB = RCC_AHB2ENR_GPIOAEN;
	hSPI1.PIN_MOSI.GPIO     = GPIOA;
	hSPI1.PIN_MOSI.GPIO_PIN = GPIO_PIN_7;
	hSPI1.PIN_MOSI.GPIO_SRC = GPIO_PinSource7;

#if (SPI_USE_DMA)
	// DMA TX channel
	hSPI1.DMA_TX.Instance = DMA1;
	hSPI1.DMA_TX.Channel  = DMA1_Channel3;
	hSPI1.DMA_TX.CF       = DMA_IFCR_CGIF3 | DMA_IFCR_CHTIF3 | DMA_IFCR_CTCIF3 | DMA_IFCR_CTEIF3;
	hSPI1.DMA_TX.TCIF     = DMA_ISR_TCIF3;
	hSPI1.DMA_TX.HTIF     = DMA_ISR_HTIF3;
	hSPI1.DMA_TX.State    = DMA_STATE_RESET;

	// DMA RX channel
	hSPI1.DMA_RX.Instance = DMA1;
	hSPI1.DMA_RX.Channel  = DMA1_Channel2;
	hSPI1.DMA_RX.CF       = DMA_IFCR_CGIF2 | DMA_IFCR_CHTIF2 | DMA_IFCR_CTCIF2 | DMA_IFCR_CTEIF2;
	hSPI1.DMA_RX.TCIF     = DMA_ISR_TCIF2;
	hSPI1.DMA_RX.HTIF     = DMA_ISR_HTIF2;
	hSPI1.DMA_RX.State    = DMA_STATE_RESET;
#endif // SPI_USE_DMA
}
#endif // SPI1_USE

#if (SPI2_USE)
// Populate the SPI2 handle
void SPI2_HandleInit(void) {
	hSPI2.Instance = SPI2;
	hSPI2.AF       = GPIO_AF5;

	// SCK pin [PB13, PB10]
	hSPI2.PIN_SCK.GPIO_AHB = RCC_AHB2ENR_GPIOBEN;
	hSPI2.PIN_SCK.GPIO     = GPIOB;
	hSPI2.PIN_SCK.GPIO_PIN = GPIO_PIN_13;
	hSPI2.PIN_SCK.GPIO_SRC = GPIO_PinSource13;

	// MISO pin [PC2, PB14]
	hSPI2.PIN_MISO.GPIO_AHB = RCC_AHB2ENR_GPIOBEN;
	hSPI2.PIN_MISO.GPIO     = GPIOB;
	hSPI2.PIN_MISO.GPIO_PIN = GPIO_PIN_14;
	hSPI2.PIN_MISO.GPIO_SRC = GPIO_PinSource14;

	// MOSI pin [PC3, PB15]
	hSPI2.PIN_MOSI.GPIO_AHB = RCC_AHB2ENR_GPIOBEN;
	hSPI2.PIN_MOSI.GPIO     = GPIOB;
	hSPI2.PIN_MOSI.GPIO_PIN = GPIO_PIN_15;
	hSPI2.PIN_MOSI.GPIO_SRC = GPIO_PinSource15;

#if (SPI_USE_DMA)
	// DMA TX channel
	hSPI2.DMA_TX.Instance = DMA1;
	hSPI2.DMA_TX.Channel  = DMA1_Channel5;
	hSPI2.DMA_TX.CF       = DMA_IFCR_CGIF5 | DMA_IFCR_CHTIF5 | DMA_IFCR_CTCIF5 | DMA_IFCR_CTEIF5;
	hSPI2.DMA_TX.TCIF     = DMA_ISR_TCIF5;
	hSPI2.DMA_TX.HTIF     = DMA_ISR_HTIF5;
	hSPI2.DMA_TX.State    = DMA_STATE_RESET;

	// DMA RX channel
	hSPI2.DMA_RX.Instance = DMA1;
	hSPI2.DMA_RX.Channel  = DMA1_Channel4;
	hSPI2.DMA_RX.CF       = DMA_IFCR_CGIF4 | DMA_IFCR_CHTIF4 | DMA_IFCR_CTCIF4 | DMA_IFCR_CTEIF4;
	hSPI2.DMA_RX.TCIF     = DMA_ISR_TCIF4;
	hSPI2.DMA_RX.HTIF     = DMA_ISR_HTIF4;
	hSPI2.DMA_RX.State    = DMA_STATE_RESET;
#endif // SPI_USE_DMA
}
#endif // SPI2_USE

#if (SPI3_USE)
// Populate the SPI3 handle
void SPI3_HandleInit(void) {
	hSPI3.Instance = SPI3;
	hSPI3.AF       = GPIO_AF6;

	// SCK pin [PC10, PB3]
	hSPI3.PIN_SCK.GPIO_AHB = RCC_AHB2ENR_GPIOBEN;
	hSPI3.PIN_SCK.GPIO     = GPIOB;
	hSPI3.PIN_SCK.GPIO_PIN = GPIO_PIN_3;
	hSPI3.PIN_SCK.GPIO_SRC = GPIO_PinSource3;

	// MISO pin [PC11, PB4]
	hSPI3.PIN_MISO.GPIO_AHB = RCC_AHB2ENR_GPIOBEN;
	hSPI3.PIN_MISO.GPIO     = GPIOB;
	hSPI3.PIN_MISO.GPIO_PIN = GPIO_PIN_4;
	hSPI3.PIN_MISO.GPIO_SRC = GPIO_PinSource4;

	// MOSI pin [PC12, PB5]
	hSPI3.PIN_MOSI.GPIO_AHB = RCC_AHB2ENR_GPIOBEN;
	hSPI3.PIN_MOSI.GPIO     = GPIOB;
	hSPI3.PIN_MOSI.GPIO_PIN = GPIO_PIN_5;
	hSPI3.PIN_MOSI.GPIO_SRC = GPIO_PinSource5;

#if (SPI_USE_DMA)
	// DMA TX channel
	hSPI3.DMA_TX.Instance = DMA2;
	hSPI3.DMA_TX.Channel  = DMA2_Channel2;
	hSPI3.DMA_TX.CF       = DMA_IFCR_CGIF2 | DMA_IFCR_CHTIF2 | DMA_IFCR_CTCIF2 | DMA_IFCR_CTEIF2;
	hSPI3.DMA_TX.TCIF     = DMA_ISR_TCIF2;
	hSPI3.DMA_TX.HTIF     = DMA_ISR_THIF2;
	hSPI3.DMA_TX.State    = DMA_STATE_RESET;

	// DMA RX channel
	hSPI3.DMA_RX.Instance = DMA1;
	hSPI3.DMA_RX.Channel  = DMA1_Channel1;
	hSPI3.DMA_RX.CF       = DMA_IFCR_CGIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CTEIF1;
	hSPI3.DMA_RX.TCIF     = DMA_ISR_TCIF1;
	hSPI3.DMA_RX.HTIF     = DMA_ISR_HTIF1;
	hSPI3.DMA_RX.State    = DMA_STATE_RESET;
#endif // SPI_USE_DMA
}
#endif // SPI3_USE

// SPI peripheral initialization
// input:
//   SPIx - pointer to the SPI port handle
//   clock_conf - SPI clock phase and polarity (one of SPI_CLK_XXX values)
//   SPI_DIR - SPI lines configuration (one of SPI_DIR_XXX values)
void SPI_Init(const SPI_HandleTypeDef *SPI, uint32_t clock_conf, uint16_t SPI_DIR) {
#if (SPI1_USE)
	if (SPI->Instance == SPI1) {
		// Reset the SPI1 peripheral
		RCC->APB2RSTR |=  RCC_APB2RSTR_SPI1RST;
		RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;

		// Enable the SPI1 peripheral clock
		RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	}
#endif // SPI1_USE

#if (SPI2_USE)
	if (SPI->Instance == SPI2) {
		// Reset the SPI2 peripheral
		RCC->APB1RSTR1 |=  RCC_APB1RSTR1_SPI2RST;
		RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_SPI2RST;

		// Enable the SPI2 peripheral clock
		RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN;
	}
#endif // SPI2_USE

#if (SPI3_USE)
	if (SPI->Instance == SPI3) {
		// Reset the SPI3 peripheral
		RCC->APB1RSTR1 |=  RCC_APB1RSTR1_SPI3RST;
		RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_SPI3RST;

		// Enable the SPI3 peripheral clock
		RCC->APB1ENR1 |= RCC_APB1ENR1_SPI3EN;
	}
#endif // SPI3_USE

	// SCK pin
	RCC->AHB2ENR |= SPI->PIN_SCK.GPIO_AHB;
	GPIO_set_mode(SPI->PIN_SCK.GPIO,GPIO_Mode_AF,GPIO_PUPD_NONE,SPI->PIN_SCK.GPIO_PIN);
	GPIO_out_cfg(SPI->PIN_SCK.GPIO,GPIO_OT_PP,GPIO_SPD_HIGH,SPI->PIN_SCK.GPIO_PIN);
	GPIO_af_cfg(SPI->PIN_SCK.GPIO,SPI->PIN_SCK.GPIO_SRC,SPI->AF);

	// MOSI pin
	if (SPI_DIR != SPI_DIR_RX) {
		RCC->AHB2ENR |= SPI->PIN_MOSI.GPIO_AHB;
		GPIO_set_mode(SPI->PIN_MOSI.GPIO,GPIO_Mode_AF,GPIO_PUPD_NONE,SPI->PIN_MOSI.GPIO_PIN);
		GPIO_out_cfg(SPI->PIN_MOSI.GPIO,GPIO_OT_PP,GPIO_SPD_HIGH,SPI->PIN_MOSI.GPIO_PIN);
		GPIO_af_cfg(SPI->PIN_MOSI.GPIO,SPI->PIN_MOSI.GPIO_SRC,SPI->AF);
	}

	// MISO pin
	if (SPI_DIR != SPI_DIR_TX) {
		RCC->AHB2ENR |= SPI->PIN_MISO.GPIO_AHB;
		GPIO_set_mode(SPI->PIN_MISO.GPIO,GPIO_Mode_AF,GPIO_PUPD_NONE,SPI->PIN_MISO.GPIO_PIN);
		GPIO_out_cfg(SPI->PIN_MISO.GPIO,GPIO_OT_PP,GPIO_SPD_HIGH,SPI->PIN_MISO.GPIO_PIN);
		GPIO_af_cfg(SPI->PIN_MISO.GPIO,SPI->PIN_MISO.GPIO_SRC,SPI->AF);
	}

	// SPI settings:
	//   - peripheral disabled
	//   - master mode
	//   - MSB bit first
	//   - software NSS selection
	//   - hardware CRC calculation disabled
	//   - prescaler = 256
	SPI->Instance->CR1 = SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM | clock_conf | SPI_BR_256;
	if (SPI_DIR == SPI_DIR_RX) SPI->Instance->CR1 |= SPI_CR1_RXONLY;

	//   - data frame width = 8 bit
	//   - software NSS management
	SPI->Instance->CR2 &= ~(SPI_CR2_DS | SPI_CR2_SSOE);
	SPI->Instance->CR2 |= (SPI_DW_8BIT | SPI_CR2_SSOE);
}

// Configure SPI baudrate prescaler
// input:
//   SPIx - pointer to the SPI port handle
//   prescaler - SPI prescaler, one of SPI_BR_xx values
// note: this function should not be called when communication is ongoing
void SPI_SetBaudrate(SPI_HandleTypeDef *SPI, uint32_t prescaler) {
	uint32_t reg;

	// Wait for busy flag reset because the baud rate control bits should
	// not be changed when communication is ongoing
	while (SPI1->SR & SPI_SR_BSY);

	// Clear SPI baud rate control bits and write a new value
	reg  = SPI->Instance->CR1 & ~SPI_CR1_BR;
	reg |= prescaler;
	SPI->Instance->CR1 = reg;
}

// Send data buffer to SPI
// input:
//   SPIx - pointer to the SPI port
//   pBuf - pointer to the data buffer
//   length - length of the data buffer
// note: TX only mode
// note: function waits for transfer completion of the last byte
void SPI_SendBuf(SPI_HandleTypeDef *SPI, uint8_t *pBuf, uint32_t length) {
	// TODO: SPI: implement FIFO

	// --> https://github.com/GrumpyOldPizza/arduino-STM32L4/blob/master/system/STM32L4xx/Source/stm32l4_spi.c#L1541

#if 1

	do {
		while (!(SPI->Instance->SR & SPI_SR_TXE)); // Wait until TX buffer is empty
		*((__IO uint8_t *)&SPI->Instance->DR) = *pBuf++;
	} while (--length);
	while (SPI->Instance->SR & SPI_SR_BSY); // Wait for the transmission of the last byte

#else

#if 1

	do {
		while (!(SPI->Instance->SR & SPI_SR_TXE)); // Wait until TX buffer is empty
		*((__IO uint8_t *)&SPI->Instance->DR) = *pBuf++;
		*((__IO uint8_t *)&SPI->Instance->DR) = *pBuf++;
		length -= 2;
	} while (length);
	while (SPI->Instance->SR & SPI_SR_BSY); // Wait for the transmission of the last byte

#else

	register uint16_t *pBuf16 = (uint16_t *)pBuf;
	SPI_SetDataWidth(&hSPI2, SPI_DW_16BIT);
	do {
		while (!(SPI->Instance->SR & SPI_SR_TXE)); // Wait until TX buffer is empty
		SPI->Instance->DR = __builtin_bswap16(*pBuf16++);
		length -= 2;
	} while (length);
	SPI_SetDataWidth(&hSPI2, SPI_DW_8BIT);
	while (SPI->Instance->SR & SPI_SR_BSY); // Wait for the transmission of the last byte

#endif

#endif
}

// Send data buffer to SPI (16-bit frame)
// input:
//   SPIx - pointer to the SPI port
//   pBuf - pointer to the data buffer
//   length - length of the data buffer
// note: TX only mode
// note: function waits for transfer completion of the last byte
void SPI_SendBuf16(SPI_HandleTypeDef *SPI, uint16_t *pBuf, uint32_t length) {
	do {
		while (!(SPI->Instance->SR & SPI_SR_TXE)); // Wait until TX buffer is empty
		SPI->Instance->DR = *pBuf++;
	} while (--length);
	while (SPI->Instance->SR & SPI_SR_BSY); // Wait for the transmission of the last byte
}

// Send byte to SPI and return received byte
// input:
//   SPIx - pointer to the SPI port handle
//   data - byte to send
// return: byte received via SPI
// note: full duplex mode
uint8_t SPI_SendRecv(SPI_HandleTypeDef *SPI, uint8_t data) {
	while (!(SPI->Instance->SR & SPI_SR_TXE)); // Wait until TX buffer is empty
	*((__IO uint8_t *)&SPI->Instance->DR) = data; // Send byte to SPI (TXE cleared)
	while (!(SPI->Instance->SR & SPI_SR_RXNE)); // Wait while receive buffer is empty

	return (uint8_t)SPI->Instance->DR; // Return received byte
}

// Transmit block of data from specified data buffer and receive data in same buffer
// input:
//   SPIx - pointer to the SPI port handle
//   pBuf - pointer to the data buffer
//   length - length of the data buffer
// note: receive only in full duplex mode
// note: function waits for transfer completion of the last byte
void SPI_SendRecvBuf(SPI_HandleTypeDef *SPI, uint8_t *pBuf, uint32_t length) {
	while (length--) {
		while (!(SPI->Instance->SR & SPI_SR_TXE)); // Wait until TX buffer is empty
		SPI->Instance->DR = *pBuf; // Send byte (TXE cleared)
		while (!(SPI->Instance->SR & SPI_SR_RXNE)); // Wait while RX buffer is empty
		*pBuf++ = SPI->Instance->DR; // Read received byte
	}
	while (SPI->Instance->SR & SPI_SR_BSY); // Wait for the transmission of the last byte
}

#if (SPI_USE_DMA)

// Initialize the DMA peripheral for SPI
// input:
//   SPIx - pointer to the SPI port handle
//   pBuf - pointer to the data buffer
//   length - length of the data buffer
// note: the corresponding DMA peripheral clock must be already enabled
void SPI_Configure_DMA_TX(SPI_HandleTypeDef *SPI, uint8_t *pBuf, uint32_t length) {
	// DMA: memory -> SPI, no circular mode, 8-bits, memory increment, medium channel priority, channel disabled
//	SPI->DMA_TX.Channel->CCR   = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PL_0;

	// DMA: memory -> SPI, no circular mode, 8-bits, memory increment, medium channel priority, channel disabled
	SPI->DMA_TX.Channel->CCR   = DMA_CCR_DIR | DMA_CCR_MINC | DMA_CCR_PL_0 | DMA_CCR_TCIE;    //  <------------- ENABLE TC IRQ

	SPI->DMA_TX.Channel->CPAR  = (uint32_t)(&(SPI->Instance->DR)); // Address of the peripheral data register
	SPI->DMA_TX.Channel->CMAR  = (uint32_t)pBuf; // Memory address
	SPI->DMA_TX.Channel->CNDTR = length; // Number of data
	SPI->DMA_TX.State = DMA_STATE_READY;
}

// Initialize the DMA peripheral for SPI
// input:
//   SPIx - pointer to the SPI port handle
//   pBuf - pointer to the data buffer
//   length - length of the data buffer
// note: the corresponding DMA peripheral clock must be already enabled
void SPI_Configure_DMA_RX(SPI_HandleTypeDef *SPI, uint8_t *pBuf, uint32_t length) {
	// DMA: SPI -> memory, no circular mode, 8-bits, memory increment, medium channel priority, channel disabled
	SPI->DMA_RX.Channel->CCR   = DMA_CCR_MINC | DMA_CCR_PL_0;
	SPI->DMA_RX.Channel->CPAR  = (uint32_t)(&(SPI->Instance->DR)); // Address of the peripheral data register
	SPI->DMA_RX.Channel->CMAR  = (uint32_t)pBuf; // Memory address
	SPI->DMA_RX.Channel->CNDTR = length; // Number of data
	SPI->DMA_RX.State = DMA_STATE_READY;
}

// Enable/disable SPI RX/TX DMA channels
// input:
//   SPIx - pointer to the SPI port handle
//   NewState - new state of channels (ENABLE/DISABLE)
void SPI_SetDMA(SPI_HandleTypeDef *SPI, uint8_t SPI_DMA_DIR, FunctionalState NewState) {
	if (NewState == ENABLE) {
		if (SPI_DMA_DIR & SPI_DMA_TX) {
			// Clear DMA interrupt flag and enable TX DMA channel
			SPI->DMA_TX.Instance->IFCR = SPI->DMA_TX.CF;
			SPI->DMA_TX.Channel->CCR  |= DMA_CCR_EN;
			// Enable SPI TX DMA
			SPI->Instance->CR2 |= SPI_CR2_TXDMAEN;
			// Change state of DMA channel
			SPI->DMA_TX.State = DMA_STATE_BUSY;
		}
		if (SPI_DMA_DIR & SPI_DMA_RX) {
			// Clear DMA interrupt flag and enable RX DMA channel
			SPI->DMA_RX.Instance->IFCR = SPI->DMA_RX.CF;
			SPI->DMA_RX.Channel->CCR  |= DMA_CCR_EN;
			// Enable SPI RX DMA
			SPI->Instance->CR2 |= SPI_CR2_RXDMAEN;
			// Change state of DMA channel
			SPI->DMA_RX.State = DMA_STATE_BUSY;
		}
	} else {
		if (SPI_DMA_DIR & SPI_DMA_TX) {
			// Disable TX DMA channel
			SPI->DMA_TX.Channel->CCR &= ~DMA_CCR_EN;
			// Disable SPI TX DMA
			SPI->Instance->CR2 &= ~SPI_CR2_TXDMAEN;
			// Change state of DMA channel
			SPI->DMA_TX.State = DMA_STATE_READY;
		}
		if (SPI_DMA_DIR & SPI_DMA_RX) {
			// Disable RX DMA channel
			SPI->DMA_RX.Channel->CCR &= ~DMA_CCR_EN;
			// Disable SPI RX DMA
			SPI->Instance->CR2 &= ~SPI_CR2_RXDMAEN;
			// Change state of DMA channel
			SPI->DMA_RX.State = DMA_STATE_READY;
		}
	}
}

// Handle DMA interrupt request
// input:
//   SPIx - pointer to the SPI port handle
// note: must be called from a corresponding DMA IRQ handler
void SPI_DMA_Handler(DMA_HandleTypeDef *hDMA) {
	uint16_t flags;

	// Get DMA flags status
	flags = hDMA->Instance->ISR;

	if (flags & hDMA->HTIF) {
		// Half transfer
		hDMA->State = DMA_STATE_HT;
	}

	if (flags & hDMA->TCIF) {
		// Transfer complete
		hDMA->State = DMA_STATE_TC;
	}

	// Clear the DMA channel interrupt flags
	hDMA->Instance->IFCR = hDMA->TCIF;
}

#endif // SPI_USE_DMA
