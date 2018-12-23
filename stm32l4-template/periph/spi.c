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
#if 1
	hSPI1.DMA_TX.Channel  = DMA1_Channel3;
	hSPI1.DMA_TX.Request  = DMA_REQUEST_1;
#else
	hSPI1.DMA_TX.Channel  = DMA2_Channel4;
	hSPI1.DMA_TX.Request  = DMA_REQUEST_4;
#endif
	hSPI1.DMA_TX.Instance = DMA_GetChannelPeripheral(hSPI1.DMA_TX.Channel);
	hSPI1.DMA_TX.ChIndex  = DMA_GetChannelIndex(hSPI1.DMA_TX.Channel);
	hSPI1.DMA_TX.State    = DMA_STATE_RESET;

	// DMA RX channel
#if 1
	hSPI1.DMA_RX.Channel  = DMA1_Channel2;
	hSPI1.DMA_RX.Request  = DMA_REQUEST_1;
#else
	hSPI1.DMA_RX.Channel  = DMA2_Channel3;
	hSPI1.DMA_RX.Request  = DMA_REQUEST_4;
#endif
	hSPI1.DMA_RX.Instance = DMA_GetChannelPeripheral(hSPI1.DMA_RX.Channel);
	hSPI1.DMA_RX.ChIndex  = DMA_GetChannelIndex(hSPI1.DMA_RX.Channel);
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
	hSPI2.DMA_TX.Channel  = DMA1_Channel5;
	hSPI2.DMA_TX.Request  = DMA_REQUEST_1;
	hSPI2.DMA_TX.Instance = DMA_GetChannelPeripheral(hSPI2.DMA_TX.Channel);
	hSPI2.DMA_TX.ChIndex  = DMA_GetChannelIndex(hSPI2.DMA_TX.Channel);
	hSPI2.DMA_TX.State    = DMA_STATE_RESET;

	// DMA RX channel
	hSPI2.DMA_RX.Channel  = DMA1_Channel4;
	hSPI2.DMA_RX.Request  = DMA_REQUEST_1;
	hSPI2.DMA_RX.Instance = DMA_GetChannelPeripheral(hSPI2.DMA_RX.Channel);
	hSPI2.DMA_RX.ChIndex  = DMA_GetChannelIndex(hSPI2.DMA_RX.Channel);
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
	hSPI3.DMA_TX.Channel  = DMA2_Channel2;
	hSPI3.DMA_TX.Request  = DMA_REQUEST_3;
	hSPI3.DMA_TX.Instance = DMA_GetChannelPeripheral(hSPI3.DMA_TX.Channel);
	hSPI3.DMA_TX.ChIndex  = DMA_GetChannelIndex(hSPI3.DMA_TX.Channel);
	hSPI3.DMA_TX.State    = DMA_STATE_RESET;

	// DMA RX channel
	hSPI3.DMA_RX.Channel  = DMA2_Channel1;
	hSPI3.DMA_RX.Request  = DMA_REQUEST_3;
	hSPI3.DMA_RX.Instance = DMA_GetChannelPeripheral(hSPI3.DMA_RX.Channel);
	hSPI3.DMA_RX.ChIndex  = DMA_GetChannelIndex(hSPI3.DMA_RX.Channel);
	hSPI3.DMA_RX.State    = DMA_STATE_RESET;
#endif // SPI_USE_DMA
}
#endif // SPI3_USE

// SPI peripheral initialization
// input:
//   SPIx - pointer to the SPI port handle
//   clock_conf - SPI clock phase and polarity (one of SPI_CLK_XXX or SPI_MODE_XXX values)
//   SPI_DIR - SPI lines configuration (one of SPI_DIR_XXX values)
void SPI_Init(const SPI_HandleTypeDef *SPIx, uint32_t clock_conf, uint16_t SPI_DIR) {
#if (SPI1_USE)
	if (SPIx->Instance == SPI1) {
		// Reset the SPI1 peripheral
		RCC->APB2RSTR |=  RCC_APB2RSTR_SPI1RST;
		RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;

		// Enable the SPI1 peripheral clock
		RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	}
#endif // SPI1_USE

#if (SPI2_USE)
	if (SPIx->Instance == SPI2) {
		// Reset the SPI2 peripheral
		RCC->APB1RSTR1 |=  RCC_APB1RSTR1_SPI2RST;
		RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_SPI2RST;

		// Enable the SPI2 peripheral clock
		RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN;
	}
#endif // SPI2_USE

#if (SPI3_USE)
	if (SPIx->Instance == SPI3) {
		// Reset the SPI3 peripheral
		RCC->APB1RSTR1 |=  RCC_APB1RSTR1_SPI3RST;
		RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_SPI3RST;

		// Enable the SPI3 peripheral clock
		RCC->APB1ENR1 |= RCC_APB1ENR1_SPI3EN;
	}
#endif // SPI3_USE

	// SCK pin
	RCC->AHB2ENR |= SPIx->PIN_SCK.GPIO_AHB;
	GPIO_af_cfg(SPIx->PIN_SCK.GPIO, SPIx->PIN_SCK.GPIO_SRC, SPIx->AF);
	GPIO_set_mode(SPIx->PIN_SCK.GPIO, GPIO_Mode_AF, GPIO_PUPD_NONE, SPIx->PIN_SCK.GPIO_PIN);
	GPIO_out_cfg(SPIx->PIN_SCK.GPIO, GPIO_OT_PP, GPIO_SPD_HIGH, SPIx->PIN_SCK.GPIO_PIN);

	// MOSI pin
	if (SPI_DIR != SPI_DIR_RX) {
		RCC->AHB2ENR |= SPIx->PIN_MOSI.GPIO_AHB;
		GPIO_af_cfg(SPIx->PIN_MOSI.GPIO, SPIx->PIN_MOSI.GPIO_SRC, SPIx->AF);
		GPIO_set_mode(SPIx->PIN_MOSI.GPIO, GPIO_Mode_AF, GPIO_PUPD_NONE, SPIx->PIN_MOSI.GPIO_PIN);
		GPIO_out_cfg(SPIx->PIN_MOSI.GPIO, GPIO_OT_PP, GPIO_SPD_HIGH, SPIx->PIN_MOSI.GPIO_PIN);
	}

	// MISO pin
	if (SPI_DIR != SPI_DIR_TX) {
		RCC->AHB2ENR |= SPIx->PIN_MISO.GPIO_AHB;
		GPIO_af_cfg(SPIx->PIN_MISO.GPIO, SPIx->PIN_MISO.GPIO_SRC, SPIx->AF);
		GPIO_set_mode(SPIx->PIN_MISO.GPIO, GPIO_Mode_AF, GPIO_PUPD_NONE, SPIx->PIN_MISO.GPIO_PIN);
		GPIO_out_cfg(SPIx->PIN_MISO.GPIO, GPIO_OT_PP, GPIO_SPD_HIGH, SPIx->PIN_MISO.GPIO_PIN);
	}

	// SPI settings:
	//   - peripheral disabled
	//   - master mode
	//   - MSB bit first
	//   - software NSS selection
	//   - hardware CRC calculation disabled
	//   - prescaler = 256
	SPIx->Instance->CR1 = SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM | clock_conf | SPI_BR_256;
	if (SPI_DIR == SPI_DIR_RX) {
		SPIx->Instance->CR1 |= SPI_CR1_RXONLY;
	}

	//   - data frame width = 8 bit
	//   - software NSS management
	SPIx->Instance->CR2 &= ~(SPI_CR2_DS | SPI_CR2_SSOE);
	SPIx->Instance->CR2 |= SPI_DW_8BIT | SPI_CR2_SSOE;
}

// Configure SPI baudrate prescaler
// input:
//   SPIx - pointer to the SPI port handle
//   prescaler - SPI prescaler, one of SPI_BR_xx values
// note: this function should not be called when communication is ongoing
void SPI_SetBaudrate(SPI_HandleTypeDef *SPIx, uint32_t prescaler) {
	uint32_t reg;

	// Ensure the BUSY flag is reset since the baud rate control bits
	// should not be changed when communication is ongoing
	while (SPIx->Instance->SR & SPI_SR_BSY);

	// Clear SPI baud rate control bits and write a new value
	reg  = SPIx->Instance->CR1 & ~SPI_CR1_BR;
	reg |= prescaler;
	SPIx->Instance->CR1 = reg;
}

// Send data buffer to SPI
// input:
//   SPIx - pointer to the SPI port
//   pBuf - pointer to the data buffer
//   length - length of the data buffer
// note: TX only mode
// note: function waits for transfer completion of the last byte
void SPI_SendBuf(SPI_HandleTypeDef *SPIx, uint8_t *pBuf, uint32_t length) {
	do {
		while (!(SPIx->Instance->SR & SPI_SR_TXE)); // Wait until TX buffer is empty
		*((__IO uint8_t *)&SPIx->Instance->DR) = *pBuf++;
	} while (--length);
	while (SPIx->Instance->SR & SPI_SR_BSY); // Wait for the transmission of the last byte
}

// Send data buffer to SPI (16-bit frame)
// input:
//   SPIx - pointer to the SPI port
//   pBuf - pointer to the data buffer
//   length - length of the data buffer
// note: TX only mode
// note: function waits for transfer completion of the last byte
void SPI_SendBuf16(SPI_HandleTypeDef *SPIx, uint16_t *pBuf, uint32_t length) {
	do {
		while (!(SPIx->Instance->SR & SPI_SR_TXE)); // Wait until TX buffer is empty
		SPIx->Instance->DR = *pBuf++;
	} while (--length);
	while (SPIx->Instance->SR & SPI_SR_BSY); // Wait for the transmission of the last byte
}

// Send byte to SPI and return received byte
// input:
//   SPIx - pointer to the SPI port handle
//   data - byte to send
// return: byte received via SPI
// note: full duplex mode
uint8_t SPI_SendRecv(SPI_HandleTypeDef *SPIx, uint8_t data) {
	*((__IO uint8_t *)&SPIx->Instance->DR) = data; // Send byte to SPI (TXE cleared)
	while (!(SPIx->Instance->SR & SPI_SR_RXNE)); // Wait while receive buffer is empty
	return (uint8_t)SPIx->Instance->DR; // Return received byte
}

// Transmit block of data from specified data buffer and receive data in same buffer
// input:
//   SPIx - pointer to the SPI port handle
//   pBuf - pointer to the data buffer
//   length - length of the data buffer
// note: receive only in full duplex mode
// note: function waits for transfer completion of the last byte
void SPI_SendRecvBuf(SPI_HandleTypeDef *SPIx, uint8_t *pBuf, uint32_t length) {
	while (length--) {
		while (!(SPIx->Instance->SR & SPI_SR_TXE)); // Wait until TX buffer is empty
		SPIx->Instance->DR = *pBuf; // Send byte (TXE cleared)
		while (!(SPIx->Instance->SR & SPI_SR_RXNE)); // Wait while RX buffer is empty
		*pBuf++ = SPIx->Instance->DR; // Read received byte
	}
	while (SPIx->Instance->SR & SPI_SR_BSY); // Wait for the transmission of the last byte
}

// Configure SPI CRC length and polynomial value
// input:
//   SPIx - pointer to the SPI port handle
//   crc_length - CRC length, one of SPI_CRC_xx values
//   polynomial - new value of CRC polynomial, should be odd value
// note: should be called only when SPI is disabled
void SPI_SetCRC(SPI_HandleTypeDef *SPIx, uint32_t crc_length, uint16_t polynomial) {
	SPIx->Instance->CRCPR = polynomial;
	if (crc_length) {
		// 16-bit CRC
		SPIx->Instance->CR1 |= SPI_CR1_CRCL;
	} else {
		// 8-bit CRC
		SPIx->Instance->CR1 &= ~SPI_CR1_CRCL;
	}
}

#if (SPI_USE_DMA)

// Configure the SPI TX/RX DMA channels
// input:
//   SPIx - pointer to the SPI port handle
//   DMA_DIR - DMA direction (combination of SPI_DMA_xx values)
//   DMA_MODE - DMA buffer mode, either DMA_MODE_CIRCULAR or DMA_MODE_NORMAL
//   pBuf - pointer to the data buffer
//   length - size of the data buffer
// note: the corresponding DMA peripheral clock must be already enabled
void SPI_ConfigureDMA(SPI_HandleTypeDef *SPIx, uint32_t DMA_DIR, uint32_t DMA_MODE, uint8_t *pBuf, uint32_t length) {
	// RX channel should be enabled first...
	if (DMA_DIR & SPI_DMA_RX) {
		// SPI RX DMA channel configuration:
		//   memory to memory: disabled
		//   channel priority: medium
		//   memory size: 8-bit
		//   peripheral size: 8-bit
		//   memory increment: enabled
		//   peripheral increment: disabled
		//   circular mode: according to DMA_MODE
		//   direction: read from peripheral
		DMA_ConfigChannel(
				SPIx->DMA_RX.Channel,
				DMA_MODE | DMA_DIR_P2M | \
				DMA_MSIZE_8BIT | DMA_PSIZE_8BIT | \
				DMA_MINC_ENABLE | DMA_PINC_DISABLE | \
				DMA_PRIORITY_MEDIUM
			);
		DMA_SetAddrM(SPIx->DMA_RX.Channel, (uint32_t)pBuf);
		DMA_SetAddrP(SPIx->DMA_RX.Channel, (uint32_t)(&(SPIx->Instance->DR)));
		DMA_SetDataLength(SPIx->DMA_RX.Channel, length);

		// Map DMA request to DMA channel
		DMA_SetRequest(SPIx->DMA_RX.Instance, SPIx->DMA_RX.Request, SPIx->DMA_RX.ChIndex);
	}
	if (DMA_DIR & SPI_DMA_TX) {
		// SPI TX DMA channel configuration:
		//   memory to memory: disabled
		//   channel priority: medium
		//   memory size: 8-bit
		//   peripheral size: 8-bit
		//   memory increment: enabled
		//   peripheral increment: disabled
		//   circular mode: according to DMA_MODE
		//   direction: read from memory
		DMA_ConfigChannel(
				SPIx->DMA_TX.Channel,
				DMA_MODE | DMA_DIR_M2P | \
				DMA_MSIZE_8BIT | DMA_PSIZE_8BIT | \
				DMA_MINC_ENABLE | DMA_PINC_DISABLE | \
				DMA_PRIORITY_MEDIUM
			);
		DMA_SetAddrM(SPIx->DMA_TX.Channel, (uint32_t)pBuf);
		DMA_SetAddrP(SPIx->DMA_TX.Channel, (uint32_t)(&(SPIx->Instance->DR)));
		DMA_SetDataLength(SPIx->DMA_TX.Channel, length);

		// Map DMA request to DMA channel
		DMA_SetRequest(SPIx->DMA_TX.Instance, SPIx->DMA_TX.Request, SPIx->DMA_TX.ChIndex);
	}
}

// Enable/disable the SPI RX/TX DMA channels
// input:
//   SPIx - pointer to the SPI port handle
//   DMA_DIR - DMA direction (combination of SPI_DMA_xx values)
//   NewState - new state of the channels (ENABLE/DISABLE)
void SPI_SetDMA(const SPI_HandleTypeDef *SPIx, uint32_t DMA_DIR, FunctionalState NewState) {
	if (NewState == ENABLE) {
		// Clear the DMA interrupt flags and enable RX/TX DMA channels
		if (DMA_DIR & SPI_DMA_TX) {
			DMA_ClearFlags(SPIx->DMA_TX.Instance, SPIx->DMA_TX.ChIndex, DMA_CF_ALL);
			DMA_EnableChannel(SPIx->DMA_TX.Channel);
		}
		if (DMA_DIR & SPI_DMA_RX) {
			DMA_ClearFlags(SPIx->DMA_RX.Instance, SPIx->DMA_RX.ChIndex, DMA_CF_ALL);
			DMA_EnableChannel(SPIx->DMA_RX.Channel);
		}

		// Enable the SPI TX/RX DMA
		SPIx->Instance->CR2 |= DMA_DIR;
	} else {
		// Disable the RX/TX DMA channels
		if (DMA_DIR & SPI_DMA_TX) { DMA_DisableChannel(SPIx->DMA_TX.Channel); }
		if (DMA_DIR & SPI_DMA_RX) { DMA_DisableChannel(SPIx->DMA_RX.Channel); }

		// Disable the SPI TX/RX DMA
		SPIx->Instance->CR2 &= ~DMA_DIR;
	}
}

#endif // SPI_USE_DMA
