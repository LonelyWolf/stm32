
#include <stm32l1xx_gpio.h>
#include <stm32l1xx_rcc.h>

#include <spi.h>


// SPI peripheral initialization
// input:
//   SPI - pointer to the SPI port (SPI1, SPI2, etc.)
//   SPI_direction - SPI lines configuration (one of SPI_DIR_XXX values)
//   SPI_prescaler - SPI prescaler (SPI_BR_XXX)
void SPIx_Init(SPI_TypeDef *SPI, uint16_t SPI_direction, uint16_t SPI_prescaler) {
	GPIO_InitTypeDef PORT;

	PORT.GPIO_Speed = GPIO_Speed_40MHz;
	PORT.GPIO_OType = GPIO_OType_PP;
	PORT.GPIO_Mode  = GPIO_Mode_AF;
	PORT.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	if (SPI == SPI1) {
		// Reset the SPI1 peripheral
		RCC->APB2RSTR |=  RCC_APB2RSTR_SPI1RST;
		RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI1RST;
		// Enable the SPI GPIO peripheral(s)
		RCC->AHBENR |= SPI1_PERIPH;
		// Enable the SPI1 peripheral
		RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
		// SCK pin
		PORT.GPIO_Pin = SPI1_SCK_PIN;
		GPIO_Init(SPI1_SCK_GPIO,&PORT);
		GPIO_PinAFConfig(SPI1_SCK_GPIO,SPI1_SCK_PIN_SRC,GPIO_AF_SPI1);
		if (SPI_direction != SPI_DIR_RX) {
			// MOSI pin
			PORT.GPIO_Pin = SPI1_MOSI_PIN;
			GPIO_Init(SPI1_MOSI_GPIO,&PORT);
			GPIO_PinAFConfig(SPI1_MOSI_GPIO,SPI1_MOSI_PIN_SRC,GPIO_AF_SPI1);
		}
		if (SPI_direction != SPI_DIR_TX) {
			// MISO pin
			PORT.GPIO_Pin = SPI1_MISO_PIN;
			GPIO_Init(SPI1_MISO_GPIO,&PORT);
			GPIO_PinAFConfig(SPI1_MISO_GPIO,SPI1_MISO_PIN_SRC,GPIO_AF_SPI1);
		}
	} else if (SPI == SPI2) {
		// Reset the SPI2 peripheral
		RCC->APB1RSTR |=  RCC_APB1RSTR_SPI2RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_SPI2RST;
		// Enable the SPI2 GPIO peripheral
		RCC->AHBENR |= SPI2_PERIPH;
		// Enable the SPI2 peripheral
		RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
		// SCK pin
		PORT.GPIO_Pin = SPI2_SCK_PIN;
		GPIO_Init(SPI2_SCK_GPIO,&PORT);
		GPIO_PinAFConfig(SPI2_SCK_GPIO,SPI2_SCK_PIN_SRC,GPIO_AF_SPI2);
		if (SPI_direction != SPI_DIR_RX) {
			// MOSI pin
			PORT.GPIO_Pin = SPI2_MOSI_PIN;
			GPIO_Init(SPI2_MOSI_GPIO,&PORT);
			GPIO_PinAFConfig(SPI2_MOSI_GPIO,SPI2_MOSI_PIN_SRC,GPIO_AF_SPI2);
		}
		if (SPI_direction != SPI_DIR_TX) {
			// MISO pin
			PORT.GPIO_Pin = SPI2_MISO_PIN;
			GPIO_Init(SPI2_MISO_GPIO,&PORT);
			GPIO_PinAFConfig(SPI2_MISO_GPIO,SPI2_MISO_PIN_SRC,GPIO_AF_SPI2);
		}
	} else if (SPI == SPI3) {
		// Reset the SPI3 peripheral
		RCC->APB1RSTR |=  RCC_APB1RSTR_SPI3RST;
		RCC->APB1RSTR &= ~RCC_APB1RSTR_SPI3RST;
		// Enable the SPI3 GPIO peripheral(s)
		RCC->AHBENR |= SPI3_PERIPH;
		// Enable the SPI3 peripheral
		RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
		// SCK pin
		PORT.GPIO_Pin = SPI3_SCK_PIN;
		GPIO_Init(SPI3_SCK_GPIO,&PORT);
		GPIO_PinAFConfig(SPI3_SCK_GPIO,SPI3_SCK_PIN_SRC,GPIO_AF_SPI3);
		if (SPI_direction != SPI_DIR_RX) {
			// MOSI pin
			PORT.GPIO_Pin = SPI3_MOSI_PIN;
			GPIO_Init(SPI3_MOSI_GPIO,&PORT);
			GPIO_PinAFConfig(SPI3_MOSI_GPIO,SPI3_MOSI_PIN_SRC,GPIO_AF_SPI3);
		}
		if (SPI_direction != SPI_DIR_TX) {
			// MISO pin
			PORT.GPIO_Pin = SPI3_MISO_PIN;
			GPIO_Init(SPI3_MISO_GPIO,&PORT);
			GPIO_PinAFConfig(SPI3_MISO_GPIO,SPI3_MISO_PIN_SRC,GPIO_AF_SPI3);
		}
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
	SPI->CR1 = SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM | SPI_prescaler;
	if (SPI_direction == SPI_DIR_RX) SPI->CR1 |= SPI_CR1_RXONLY;
	SPI->I2SCFGR &= ~SPI_I2SCFGR_I2SMOD; // Clear I2SMOD bit - SPI mode
	SPI->CRCPR = 7; // Polynomial for CRC calculation (must be set when the SPE bit cleared in the CR1 register)
	SPI->CR1 |= SPI_CR1_SPE; // Enable the specified SPI peripheral
}

// Set SPI speed
// input:
//   SPI - pointer to the SPI port (SPI1, SPI2, etc.)
//   SPI_prescaler - SPI prescaler (SPI_BR_XXX)
void SPIx_SetSpeed(SPI_TypeDef *SPI, uint16_t SPI_prescaler) {
	uint16_t reg;

	reg  = SPI->CR1 & ~SPI_CR1_BR; // Clear SPI baud rate bits
	reg |= SPI_prescaler; // Set SPI baud rate bits
	SPI->CR1 = reg; // Update the SPI configuration
}

// Send data via SPI
// input:
//   SPI - pointer to the SPI port (SPI1, SPI2, etc.)
//   data - byte to send
void SPIx_Send(SPI_TypeDef *SPI, uint8_t data) {
	SPI->DR = data; // Send byte to SPI (TXE cleared)
	while (!(SPI->SR & SPI_SR_RXNE)); // Wait while receive buffer is empty
	(void)SPI->DR; // Clear the RXNE bit
}

// Send/Receive data via SPI
// input:
//   SPI - pointer to the SPI port (SPI1, SPI2, etc.)
//   data - byte to send
// return: byte received by SPI
uint8_t SPIx_SendRecv(SPI_TypeDef *SPI, uint8_t data) {
	SPI->DR = data; // Send byte to SPI (TXE cleared)
	while (!(SPI->SR & SPI_SR_RXNE)); // Wait while receive buffer is empty

	return SPI->DR; // Received byte
}
