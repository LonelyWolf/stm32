// Define to prevent recursive inclusion -------------------------------------
#ifndef __SPIx_H
#define __SPIx_H


// SPI1
#define SPI1_PORT          SPI1
#define SPI1_PERIPH        RCC_AHBPeriph_GPIOB
#define SPI1_SCK_PIN       GPIO_Pin_3    // PB3
#define SPI1_MISO_PIN      GPIO_Pin_4    // PB4
#define SPI1_MOSI_PIN      GPIO_Pin_5    // PB5
#define SPI1_SCK_PIN_SRC   GPIO_PinSource3
#define SPI1_MISO_PIN_SRC  GPIO_PinSource4
#define SPI1_MOSI_PIN_SRC  GPIO_PinSource5
#define SPI1_GPIO_PORT     GPIOB

// SPI2
#define SPI2_PORT          SPI2
#define SPI2_PERIPH        RCC_AHBPeriph_GPIOB
#define SPI2_SCK_PIN       GPIO_Pin_13    // PB13
#define SPI2_MISO_PIN      GPIO_Pin_14    // PB14
#define SPI2_MOSI_PIN      GPIO_Pin_15    // PB15
#define SPI2_SCK_PIN_SRC   GPIO_PinSource13
#define SPI2_MISO_PIN_SRC  GPIO_PinSource14
#define SPI2_MOSI_PIN_SRC  GPIO_PinSource15
#define SPI2_GPIO_PORT     GPIOB


#define SPI_BR_2           ((uint16_t)0x0000)    // SPI baud rate prescaler 2
#define SPI_BR_4           ((uint16_t)0x0008)    // SPI baud rate prescaler 4
#define SPI_BR_8           ((uint16_t)0x0010)    // SPI baud rate prescaler 8
#define SPI_BR_16          ((uint16_t)0x0018)    // SPI baud rate prescaler 16
#define SPI_BR_32          ((uint16_t)0x0020)    // SPI baud rate prescaler 32
#define SPI_BR_64          ((uint16_t)0x0028)    // SPI baud rate prescaler 64
#define SPI_BR_128         ((uint16_t)0x0030)    // SPI baud rate prescaler 128
#define SPI_BR_256         ((uint16_t)0x0038)    // SPI baud rate prescaler 256


void SPIx_Init(SPI_TypeDef *SPI);
void SPIx_SetSpeed(SPI_TypeDef *SPI, uint16_t prescaler);
uint8_t SPIx_SendRecv(SPI_TypeDef *SPI, uint8_t data);

#endif // __SPIx_H
