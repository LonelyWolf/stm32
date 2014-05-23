#include <stm32l1xx_gpio.h>
#include <stm32l1xx_rcc.h>
#include <stm32l1xx_spi.h>
#include <spi.h>


// SPI2 peripheral initialization
void SPI2_Init(void) {
	// SPI2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
	RCC_AHBPeriphClockCmd(SPI2_PERIPH,ENABLE);

	// Configure SPI pins
	GPIO_InitTypeDef PORT;
	PORT.GPIO_Pin  = SPI_SCK_PIN | SPI_MOSI_PIN | SPI_MISO_PIN;
	PORT.GPIO_Speed = GPIO_Speed_40MHz;
	PORT.GPIO_OType = GPIO_OType_PP;
	PORT.GPIO_Mode = GPIO_Mode_AF;
	PORT.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(SPI_GPIO_PORT,&PORT);

	// Alternative functions of GPIO pins
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_SPI2);

	// Configure and enable SPI2
	SPI_InitTypeDef SPI;
	SPI.SPI_Mode = SPI_Mode_Master;
	SPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI.SPI_CPOL = SPI_CPOL_Low;
	SPI.SPI_CPHA = SPI_CPHA_1Edge;
	SPI.SPI_CRCPolynomial = 7;
	SPI.SPI_DataSize = SPI_DataSize_8b;
	SPI.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI.SPI_NSS = SPI_NSS_Soft;
	SPI_Init(SPI_PORT,&SPI);
	// NSS must be set to '1' due to NSS_Soft settings (otherwise it will be Multimaster mode).
	SPI_NSSInternalSoftwareConfig(SPI_PORT,SPI_NSSInternalSoft_Set);
	SPI_Cmd(SPI_PORT,ENABLE);
}

// Send byte via SPI2
// input:
//   data - byte to send
void SPI2_Send(uint8_t data) {
	while (SPI_I2S_GetFlagStatus(SPI_PORT,SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI_PORT,data);
	while (SPI_I2S_GetFlagStatus(SPI_PORT,SPI_I2S_FLAG_BSY) == SET);
}

// Send/Receive data via SPI
// input:
//   data - byte to send
// output: received byte from nRF24L01
uint8_t SPI2_SendRecv(uint8_t data) {
	while (SPI_I2S_GetFlagStatus(SPI_PORT,SPI_I2S_FLAG_TXE) == RESET); // Wait while DR register is not empty
	SPI_I2S_SendData(SPI_PORT,data); // Send byte to SPI
	while (SPI_I2S_GetFlagStatus(SPI_PORT,SPI_I2S_FLAG_RXNE) == RESET); // Wait to receive byte
	return SPI_I2S_ReceiveData(SPI_PORT); // Read byte from SPI bus
}
