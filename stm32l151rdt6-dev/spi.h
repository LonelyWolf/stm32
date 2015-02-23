// Define to prevent recursive inclusion -------------------------------------
#ifndef __SPIx_H
#define __SPIx_H

// SPI1
// SCK : PA5, PB3
// MISO: PA6, PB4, PA11
// MOSI: PA7, PB5, PA12
#define SPI1_PORT           SPI1
#define SPI1_PERIPH         RCC_AHBENR_GPIOAEN
#define SPI1_SCK_GPIO       GPIOA
#define SPI1_SCK_PIN        GPIO_Pin_5
#define SPI1_SCK_PIN_SRC    GPIO_PinSource5
#define SPI1_MISO_GPIO      GPIOA
#define SPI1_MISO_PIN       GPIO_Pin_6
#define SPI1_MISO_PIN_SRC   GPIO_PinSource6
#define SPI1_MOSI_GPIO      GPIOA
#define SPI1_MOSI_PIN       GPIO_Pin_7
#define SPI1_MOSI_PIN_SRC   GPIO_PinSource7

// SPI2
// SCK : PB13
// MISO: PB14
// MOSI: PB15
#define SPI2_PORT           SPI2
#define SPI2_PERIPH         RCC_AHBENR_GPIOBEN
#define SPI2_SCK_GPIO       GPIOB
#define SPI2_SCK_PIN        GPIO_Pin_13
#define SPI2_SCK_PIN_SRC    GPIO_PinSource13
#define SPI2_MISO_GPIO      GPIOB
#define SPI2_MISO_PIN       GPIO_Pin_14
#define SPI2_MISO_PIN_SRC   GPIO_PinSource14
#define SPI2_MOSI_GPIO      GPIOB
#define SPI2_MOSI_PIN       GPIO_Pin_15
#define SPI2_MOSI_PIN_SRC   GPIO_PinSource15

// SPI3
// SCK : PC10, PB3
// MISO: PB11, PB4
// MOSI: PB12, PB5
#define SPI3_PORT           SPI3
#define SPI3_PERIPH         RCC_AHBENR_GPIOCEN
#define SPI3_SCK_GPIO       GPIOC
#define SPI3_SCK_PIN        GPIO_Pin_10
#define SPI3_SCK_PIN_SRC    GPIO_PinSource10
#define SPI3_MISO_GPIO      GPIOC
#define SPI3_MISO_PIN       GPIO_Pin_11
#define SPI3_MISO_PIN_SRC   GPIO_PinSource11
#define SPI3_MOSI_GPIO      GPIOC
#define SPI3_MOSI_PIN       GPIO_Pin_12
#define SPI3_MOSI_PIN_SRC   GPIO_PinSource12

// SPI baud rate prescaler
#define SPI_BR_2           ((uint16_t)0x0000) // 2
#define SPI_BR_4           ((uint16_t)0x0008) // 4
#define SPI_BR_8           ((uint16_t)0x0010) // 8
#define SPI_BR_16          ((uint16_t)0x0018) // 16
#define SPI_BR_32          ((uint16_t)0x0020) // 32
#define SPI_BR_64          ((uint16_t)0x0028) // 64
#define SPI_BR_128         ((uint16_t)0x0030) // 128
#define SPI_BR_256         ((uint16_t)0x0038) // 256

// SPI line configuration
#define SPI_DIR_DUPLEX     ((uint8_t)0x00) // 2 lines full duplex
#define SPI_DIR_RX         ((uint8_t)0x01) // 1 line RX
#define SPI_DIR_TX         ((uint8_t)0x02) // 1 line TX


void SPIx_Init(SPI_TypeDef *SPI, uint16_t SPI_direction, uint16_t SPI_prescaler);
void SPIx_SetSpeed(SPI_TypeDef *SPI, uint16_t SPI_prescaler);
void SPIx_Send(SPI_TypeDef *SPI, uint8_t data);
uint8_t SPIx_SendRecv(SPI_TypeDef *SPI, uint8_t data);
void SPIx_SendBuf(SPI_TypeDef *SPI, uint8_t *pBuf, uint32_t length);

#endif // __SPIx_H
