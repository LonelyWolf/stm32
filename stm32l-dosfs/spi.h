// Define to prevent recursive inclusion -------------------------------------
#ifndef __SPI2_H
#define __SPI2_H

// SPI2 peripheral
#define SPI2_PERIPH   RCC_AHBPeriph_GPIOB

// Use SPI2
#define SPI_PORT      SPI2
#define SPI_SCK_PIN   GPIO_Pin_13    // PB13
#define SPI_MISO_PIN  GPIO_Pin_14    // PB14
#define SPI_MOSI_PIN  GPIO_Pin_15    // PB15
#define SPI_GPIO_PORT GPIOB


void SPI2_Init(void);
void SPI2_Send(uint8_t data);
uint8_t SPI2_SendRecv(uint8_t data);

#endif // __SPI2_H
