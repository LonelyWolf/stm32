// Serial peripheral interface (SPI) management


#include "gpio.h"
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
	hSPI1.AF       = GPIO_AFIO5;

	// SCK pin [PA5, PB3]
	hSPI1.PIN_SCK.GPIO_AHB = RCC_AHBENR_GPIOAEN;
	hSPI1.PIN_SCK.GPIO     = GPIOA;
	hSPI1.PIN_SCK.GPIO_PIN = GPIO_Pin_5;
	hSPI1.PIN_SCK.GPIO_SRC = GPIO_PinSource5;

	// MISO pin [PA6, PB4, PA11]
	hSPI1.PIN_MISO.GPIO_AHB = RCC_AHBENR_GPIOAEN;
	hSPI1.PIN_MISO.GPIO     = GPIOA;
	hSPI1.PIN_MISO.GPIO_PIN = GPIO_Pin_6;
	hSPI1.PIN_MISO.GPIO_SRC = GPIO_PinSource6;

	// MOSI pin [PA7, PB5, PA12]
	hSPI1.PIN_MOSI.GPIO_AHB = RCC_AHBENR_GPIOAEN;
	hSPI1.PIN_MOSI.GPIO     = GPIOA;
	hSPI1.PIN_MOSI.GPIO_PIN = GPIO_Pin_7;
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
	hSPI2.AF       = GPIO_AFIO5;

	// SCK pin [PB13]
	hSPI2.PIN_SCK.GPIO_AHB = RCC_AHBENR_GPIOBEN;
	hSPI2.PIN_SCK.GPIO     = GPIOB;
	hSPI2.PIN_SCK.GPIO_PIN = GPIO_Pin_13;
	hSPI2.PIN_SCK.GPIO_SRC = GPIO_PinSource13;

	// MISO pin [PB14]
	hSPI2.PIN_MISO.GPIO_AHB = RCC_AHBENR_GPIOBEN;
	hSPI2.PIN_MISO.GPIO     = GPIOB;
	hSPI2.PIN_MISO.GPIO_PIN = GPIO_Pin_14;
	hSPI2.PIN_MISO.GPIO_SRC = GPIO_PinSource14;

	// MOSI pin [PB15]
	hSPI2.PIN_MOSI.GPIO_AHB = RCC_AHBENR_GPIOBEN;
	hSPI2.PIN_MOSI.GPIO     = GPIOB;
	hSPI2.PIN_MOSI.GPIO_PIN = GPIO_Pin_15;
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
	hSPI3.AF       = GPIO_AFIO6;

	// SCK pin [PC10, PB3]
	hSPI3.PIN_SCK.GPIO_AHB = RCC_AHBENR_GPIOBEN;
	hSPI3.PIN_SCK.GPIO     = GPIOB;
	hSPI3.PIN_SCK.GPIO_PIN = GPIO_Pin_3;
	hSPI3.PIN_SCK.GPIO_SRC = GPIO_PinSource3;

	// MISO pin [PB4, PB11]
	hSPI3.PIN_MISO.GPIO_AHB = RCC_AHBENR_GPIOBEN;
	hSPI3.PIN_MISO.GPIO     = GPIOB;
	hSPI3.PIN_MISO.GPIO_PIN = GPIO_Pin_4;
	hSPI3.PIN_MISO.GPIO_SRC = GPIO_PinSource4;

	// MOSI pin [PB5, PB12]
	hSPI3.PIN_MOSI.GPIO_AHB = RCC_AHBENR_GPIOBEN;
	hSPI3.PIN_MOSI.GPIO     = GPIOB;
	hSPI3.PIN_MOSI.GPIO_PIN = GPIO_Pin_5;
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
//   SPI - pointer to the SPI port handle
//   SPI_direction - SPI lines configuration (one of SPI_DIR_XXX values)
//   SPI_prescaler - SPI prescaler (SPI_BR_XXX)
void SPIx_Init(const SPI_HandleTypeDef *SPI, uint16_t SPI_DIR, uint16_t prescaler) {
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
		RCC->APB1RSTR |=  RCC_APB1RSTR_SPI2RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_SPI2RST;

		// Enable the SPI2 peripheral clock
		RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
	}
#endif // SPI2_USE

#if (SPI3_USE)
	if (SPI->Instance == SPI3) {
		// Reset the SPI3 peripheral
		RCC->APB1RSTR |=  RCC_APB1RSTR_SPI3RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_SPI3RST;

		// Enable the SPI3 peripheral clock
		RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
	}
#endif // SPI3_USE

	// SCK pin
	RCC->AHBENR |= SPI->PIN_SCK.GPIO_AHB;
	GPIO_set_mode(SPI->PIN_SCK.GPIO,GPIO_Mode_AF,GPIO_PUPD_NONE,SPI->PIN_SCK.GPIO_PIN);
	GPIO_out_cfg(SPI->PIN_SCK.GPIO,GPIO_OT_PP,GPIO_SPD_HIGH,SPI->PIN_SCK.GPIO_PIN);
	GPIO_af_cfg(SPI->PIN_SCK.GPIO,SPI->PIN_SCK.GPIO_SRC,SPI->AF);

	// MOSI pin
	if (SPI_DIR != SPI_DIR_RX) {
		RCC->AHBENR |= SPI->PIN_MOSI.GPIO_AHB;
		GPIO_set_mode(SPI->PIN_MOSI.GPIO,GPIO_Mode_AF,GPIO_PUPD_NONE,SPI->PIN_MOSI.GPIO_PIN);
		GPIO_out_cfg(SPI->PIN_MOSI.GPIO,GPIO_OT_PP,GPIO_SPD_HIGH,SPI->PIN_MOSI.GPIO_PIN);
		GPIO_af_cfg(SPI->PIN_MOSI.GPIO,SPI->PIN_MOSI.GPIO_SRC,SPI->AF);
	}

	// MISO pin
	if (SPI_DIR != SPI_DIR_TX) {
		RCC->AHBENR |= SPI->PIN_MISO.GPIO_AHB;
		GPIO_set_mode(SPI->PIN_MISO.GPIO,GPIO_Mode_AF,GPIO_PUPD_NONE,SPI->PIN_MISO.GPIO_PIN);
		GPIO_out_cfg(SPI->PIN_MISO.GPIO,GPIO_OT_PP,GPIO_SPD_HIGH,SPI->PIN_MISO.GPIO_PIN);
		GPIO_af_cfg(SPI->PIN_MISO.GPIO,SPI->PIN_MISO.GPIO_SRC,SPI->AF);
	}

	// SPI settings:
	//   - master mode
	//   - CPOL low
	//   - CPHA 1 edge
	//   - data size 8 bit
	//   - MSB first bit
	//   - software NSS selection
	//   - hardware CRC calculation disabled
	//   - peripheral enabled
	SPI->Instance->CR1 = SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM | prescaler;
	if (SPI_DIR == SPI_DIR_RX) SPI->Instance->CR1 |= SPI_CR1_RXONLY;
	SPI->Instance->I2SCFGR &= ~SPI_I2SCFGR_I2SMOD; // Clear I2SMOD bit - SPI mode
	SPI->Instance->CR1 |= SPI_CR1_SPE; // Enable the specified SPI peripheral
}

// Set SPI speed
// input:
//   SPI - pointer to the SPI port handle
//   SPI_prescaler - SPI prescaler (SPI_BR_XXX)
void SPIx_SetSpeed(SPI_HandleTypeDef *SPI, uint16_t SPI_prescaler) {
	uint16_t reg;

	reg  = SPI->Instance->CR1 & ~SPI_CR1_BR; // Clear SPI baud rate bits
	reg |= SPI_prescaler; // Set SPI baud rate bits
	SPI->Instance->CR1 = reg; // Update the SPI configuration
}

// Send byte to SPI
// input:
//   SPI - pointer to the SPI port handle
//   data - byte to send
// note: TX only mode
void SPIx_Send(SPI_HandleTypeDef *SPI, uint8_t data) {
	while (!(SPI->Instance->SR & SPI_SR_TXE)); // Wait until TX buffer is empty
	SPI->Instance->DR = data; // Send byte to SPI (TXE cleared)
	while (SPI->Instance->SR & SPI_SR_BSY); // Wait until the transmission is complete
}

// Send data buffer to SPI
// input:
//   SPI - pointer to the SPI port
//   pBuf - pointer to the data buffer
//   length - length of the data buffer
// note: TX only mode
void SPIx_SendBuf(SPI_HandleTypeDef *SPI, uint8_t *pBuf, uint32_t length) {
	do {
		while (!(SPI->Instance->SR & SPI_SR_TXE)); // Wait until TX buffer is empty
		SPI->Instance->DR = *pBuf++;
	} while (--length);
	while (SPI->Instance->SR & SPI_SR_BSY); // Wait until the transmission of the last byte is complete
}

// Send data buffer to SPI (16-bit frame)
// input:
//   SPI - pointer to the SPI port
//   pBuf - pointer to the data buffer
//   length - length of the data buffer
// note: TX only mode
void SPIx_SendBuf16(SPI_HandleTypeDef *SPI, uint16_t *pBuf, uint32_t length) {
	do {
		while (!(SPI->Instance->SR & SPI_SR_TXE)); // Wait until TX buffer is empty
		SPI->Instance->DR = *pBuf++;
	} while (--length);
	while (SPI->Instance->SR & SPI_SR_BSY); // Wait until the transmission of the last byte is complete
}

// Send byte to SPI and return received byte
// input:
//   SPI - pointer to the SPI port handle
//   data - byte to send
// return: byte received by SPI
// note: full duplex mode
uint8_t SPIx_SendRecv(SPI_HandleTypeDef *SPI, uint8_t data) {
	SPI->Instance->DR = data; // Send byte to SPI (TXE cleared)
	while (!(SPI->Instance->SR & SPI_SR_RXNE)); // Wait while receive buffer is empty

	return SPI->Instance->DR; // Return received byte
}

// Receive block of data into specified data buffer
// input:
//   SPI - pointer to the SPI port handle
//   pBuf - pointer to the data buffer
//   length - length of the data buffer
//   dummy - dummy byte to send
// note: receive only in full duplex mode
void SPIx_RecvBuf(SPI_HandleTypeDef *SPI, uint8_t *pBuf, uint32_t length, uint8_t dummy) {
	while (length--) {
		while (!(SPI->Instance->SR & SPI_SR_TXE)); // Wait until TX buffer is empty
		SPI->Instance->DR = dummy; // Send dummy byte (TXE cleared)
		while (!(SPI->Instance->SR & SPI_SR_RXNE)); // Wait while RX buffer is empty
		*pBuf++ = SPI->Instance->DR; // Read received byte
	}
	while (SPI->Instance->SR & SPI_SR_BSY); // Wait until the transmission of the last byte is complete
}

// Transmit block of data from specified data buffer and receive data in same buffer
// input:
//   SPI - pointer to the SPI port handle
//   pBuf - pointer to the data buffer
//   length - length of the data buffer
// note: receive only in full duplex mode
void SPIx_SendRecvBuf(SPI_HandleTypeDef *SPI, uint8_t *pBuf, uint32_t length) {
	while (length--) {
		while (!(SPI->Instance->SR & SPI_SR_TXE)); // Wait until TX buffer is empty
		SPI->Instance->DR = *pBuf; // Send byte (TXE cleared)
		while (!(SPI->Instance->SR & SPI_SR_RXNE)); // Wait while RX buffer is empty
		*pBuf++ = SPI->Instance->DR; // Read received byte
	}
	while (SPI->Instance->SR & SPI_SR_BSY); // Wait until the transmission of the last byte is complete
}

#if (SPI_USE_DMA)
// Initialize the DMA peripheral for SPI
// input:
//   SPI - pointer to the SPI port handle
//   pBuf - pointer to the data buffer
//   length - length of the data buffer
// note: the corresponding DMA peripheral clock must be already enabled
void SPIx_Configure_DMA_TX(SPI_HandleTypeDef *SPI, uint8_t *pBuf, uint32_t length) {
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
//   SPI - pointer to the SPI port handle
//   pBuf - pointer to the data buffer
//   length - length of the data buffer
// note: the corresponding DMA peripheral clock must be already enabled
void SPIx_Configure_DMA_RX(SPI_HandleTypeDef *SPI, uint8_t *pBuf, uint32_t length) {
	// DMA: SPI -> memory, no circular mode, 8-bits, memory increment, medium channel priority, channel disabled
	SPI->DMA_RX.Channel->CCR   = DMA_CCR_MINC | DMA_CCR_PL_0;
	SPI->DMA_RX.Channel->CPAR  = (uint32_t)(&(SPI->Instance->DR)); // Address of the peripheral data register
	SPI->DMA_RX.Channel->CMAR  = (uint32_t)pBuf; // Memory address
	SPI->DMA_RX.Channel->CNDTR = length; // Number of data
	SPI->DMA_RX.State = DMA_STATE_READY;
}

// Enable/disable SPI RX/TX DMA channels
// input:
//   SPI - pointer to the SPI port handle
//   NewState - new state of channels (ENABLE/DISABLE)
void SPIx_SetDMA(SPI_HandleTypeDef *SPI, uint8_t SPI_DMA_DIR, FunctionalState NewState) {
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
//   SPI - pointer to the SPI port handle
// note: must be called from a corresponding DMA IRQ handler
void SPIx_DMA_Handler(DMA_HandleTypeDef *hDMA) {
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
