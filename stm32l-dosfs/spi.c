#include <stm32l1xx_gpio.h>
#include <stm32l1xx_rcc.h>

#include <spi.h>


// SPI peripheral initialization
// input:
//   SPI - pointer to the SPI port (SPI1, SPI2, etc.)
void SPIx_Init(SPI_TypeDef *SPI) {
	GPIO_InitTypeDef PORT;

	PORT.GPIO_Speed = GPIO_Speed_40MHz;
	PORT.GPIO_OType = GPIO_OType_PP;
	PORT.GPIO_Mode  = GPIO_Mode_AF;
	PORT.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	if (SPI == SPI1) {
		RCC_AHBPeriphClockCmd(SPI1_PERIPH,ENABLE); // Enable the SPI GPIO peripheral
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE); // Enable the SPI peripheral
		PORT.GPIO_Pin = SPI1_SCK_PIN | SPI1_MOSI_PIN | SPI1_MISO_PIN;
		GPIO_Init(SPI1_GPIO_PORT,&PORT);
		GPIO_PinAFConfig(SPI1_GPIO_PORT,SPI1_SCK_PIN_SRC, GPIO_AF_SPI1);
		GPIO_PinAFConfig(SPI1_GPIO_PORT,SPI1_MISO_PIN_SRC,GPIO_AF_SPI1);
		GPIO_PinAFConfig(SPI1_GPIO_PORT,SPI1_MOSI_PIN_SRC,GPIO_AF_SPI1);
	} else {
		RCC_AHBPeriphClockCmd(SPI2_PERIPH,ENABLE); // Enable the SPI GPIO peripheral
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE); // Enable the SPI peripheral
		PORT.GPIO_Pin = SPI2_SCK_PIN | SPI2_MOSI_PIN | SPI2_MISO_PIN;
		GPIO_Init(SPI2_GPIO_PORT,&PORT);
		GPIO_PinAFConfig(SPI2_GPIO_PORT,SPI2_SCK_PIN_SRC, GPIO_AF_SPI2);
		GPIO_PinAFConfig(SPI2_GPIO_PORT,SPI2_MISO_PIN_SRC,GPIO_AF_SPI2);
		GPIO_PinAFConfig(SPI2_GPIO_PORT,SPI2_MOSI_PIN_SRC,GPIO_AF_SPI2);
	}

	SPI->CR1 &= 0x3040; // Clear SPI settings
	// SPI settings:
	//   - master mode
	//   - 2 lines full duplex
	//   - CPOL low
	//   - CPHA 1 edge
	//   - data size 8 bit
	//   - MSB first bit
	//   - software NSS selection
	SPI->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM; // Master mode
	SPI->I2SCFGR &= ~SPI_I2SCFGR_I2SMOD; // Clear I2SMOD bit - SPI mode
	SPI->CRCPR = 7; // Polynomial for CRC calculation
	SPI->CR1 |= SPI_CR1_SPE; // Enable the specified SPI peripheral
}

// Set SPI speed
// input:
//   SPI - pointer to the SPI port (SPI1, SPI2, etc.)
//   prescaler - SPI prescaler (SPI_BR_x)
void SPIx_SetSpeed(SPI_TypeDef *SPI, uint16_t prescaler) {
	SPI->CR1 &= ~SPI_CR1_BR; // Clear SPI baud rate bits
	SPI->CR1 |= prescaler; // Set SPI baud rate bits
}

// Send/Receive data via SPI
// input:
//   SPI - pointer to the SPI port (SPI1, SPI2, etc.)
//   data - byte to send
// return: received byte from SPI
uint8_t SPIx_SendRecv(SPI_TypeDef *SPI, uint8_t data) {
	while (!(SPI->SR & SPI_SR_TXE)); // Wait while receive buffer is not empty
	SPI->DR = data; // Send byte to SPI
	while (!(SPI->SR & SPI_SR_RXNE)); // Wait while receive buffer is empty

	return SPI->DR; // Read byte from SPI
}
