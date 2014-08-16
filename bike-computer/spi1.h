// Define to prevent recursive inclusion -------------------------------------
#ifndef __SPI1_H
#define __SPI1_H


// SPI1 peripheral
#define SPI1_PERIPH   RCC_AHBPeriph_GPIOB

// Use SPI1
#define SPI1_PORT      SPI1
#define SPI1_SCK_PIN   GPIO_Pin_3    // PB3
#define SPI1_MISO_PIN  GPIO_Pin_4    // PB4
#define SPI1_MOSI_PIN  GPIO_Pin_5    // PB5
#define SPI1_CS_PIN    GPIO_Pin_6    // PB6
#define SPI1_GPIO_PORT GPIOB


void SPI1_InitSpeed(uint16_t prescaler);
void SPI1_Init(void);
void SPI1_Send(uint8_t data);
uint8_t SPI1_SendRecv(uint8_t data);

#endif // __SPI1_H
