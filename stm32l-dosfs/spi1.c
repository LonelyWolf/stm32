#include <stm32l1xx_gpio.h>
#include <stm32l1xx_rcc.h>
#include <stm32l1xx_spi.h>
#include <spi1.h>


// Set SPI speed
// input:
//   prescaler - SPI prescaler
void SPI1_InitSpeed(uint16_t prescaler) {
	SPI_InitTypeDef SPI;

	SPI.SPI_Mode = SPI_Mode_Master;
	SPI.SPI_BaudRatePrescaler = prescaler;
	SPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI.SPI_CPOL = SPI_CPOL_Low;
	SPI.SPI_CPHA = SPI_CPHA_1Edge;
	SPI.SPI_CRCPolynomial = 7;
	SPI.SPI_DataSize = SPI_DataSize_8b;
	SPI.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI.SPI_NSS = SPI_NSS_Soft;
	SPI_Init(SPI1_PORT,&SPI);
}

// SPI1 peripheral initialization
void SPI1_Init(void) {
	// SPI2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	RCC_AHBPeriphClockCmd(SPI1_PERIPH,ENABLE);

	// Configure SPI pins
	GPIO_InitTypeDef PORT;
	PORT.GPIO_Pin  = SPI1_SCK_PIN | SPI1_MOSI_PIN | SPI1_MISO_PIN;
	PORT.GPIO_Speed = GPIO_Speed_40MHz;
	PORT.GPIO_OType = GPIO_OType_PP;
	PORT.GPIO_Mode = GPIO_Mode_AF;
	PORT.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(SPI1_GPIO_PORT,&PORT);

	// Alternative functions of GPIO pins
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_SPI1);

	// Configure and enable SPI1
	SPI1_InitSpeed(SPI_BaudRatePrescaler_256);
	// NSS must be set to '1' due to NSS_Soft settings (otherwise it will be Multimaster mode).
	SPI_NSSInternalSoftwareConfig(SPI1_PORT,SPI_NSSInternalSoft_Set);
	SPI_Cmd(SPI1_PORT,ENABLE);
}

// Send byte via SPI1
// input:
//   data - byte to send
void SPI1_Send(uint8_t data) {
	while (SPI_I2S_GetFlagStatus(SPI1_PORT,SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1_PORT,data);
	while (SPI_I2S_GetFlagStatus(SPI1_PORT,SPI_I2S_FLAG_BSY) == SET);
}

// Send/Receive data via SPI
// input:
//   data - byte to send
// output: received byte from nRF24L01
uint8_t SPI1_SendRecv(uint8_t data) {
	while (SPI_I2S_GetFlagStatus(SPI1_PORT,SPI_I2S_FLAG_TXE) == RESET); // Wait while DR register is not empty
	SPI_I2S_SendData(SPI1_PORT,data); // Send byte to SPI
	while (SPI_I2S_GetFlagStatus(SPI1_PORT,SPI_I2S_FLAG_RXNE) == RESET); // Wait to receive byte
	return SPI_I2S_ReceiveData(SPI1_PORT); // Read byte from SPI bus
}
