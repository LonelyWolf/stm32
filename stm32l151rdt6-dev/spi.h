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
#define SPI1_DMA_PERIPH     DMA1
#define SPI1_DMA_CH_RX      DMA1_Channel2
#define SPI1_DMA_CH_TX      DMA1_Channel3
#define SPI1_DMA_RX_TCIF    DMA_ISR_TCIF2
#define SPI1_DMA_TX_TCIF    DMA_ISR_TCIF3
#define SPI1_DMA_CH_RX_F    DMA_IFCR_CGIF2 | DMA_IFCR_CHTIF2 | DMA_IFCR_CTCIF2 | DMA_IFCR_CTEIF2
#define SPI1_DMA_CH_TX_F    DMA_IFCR_CGIF3 | DMA_IFCR_CHTIF3 | DMA_IFCR_CTCIF3 | DMA_IFCR_CTEIF3

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
#define SPI2_DMA_PERIPH     DMA1
#define SPI2_DMA_CH_RX      DMA1_Channel4
#define SPI2_DMA_CH_TX      DMA1_Channel5
#define SPI2_DMA_RX_TCIF    DMA_ISR_TCIF4
#define SPI2_DMA_TX_TCIF    DMA_ISR_TCIF5
#define SPI2_DMA_CH_RX_F    DMA_IFCR_CGIF4 | DMA_IFCR_CHTIF4 | DMA_IFCR_CTCIF4 | DMA_IFCR_CTEIF4
#define SPI2_DMA_CH_TX_F    DMA_IFCR_CGIF5 | DMA_IFCR_CHTIF5 | DMA_IFCR_CTCIF5 | DMA_IFCR_CTEIF5

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
#define SPI3_DMA_PERIPH     DMA2
#define SPI3_DMA_CH_RX      DMA2_Channel1
#define SPI3_DMA_CH_TX      DMA2_Channel2
#define SPI3_DMA_RX_TCIF    DMA_ISR_TCIF1
#define SPI3_DMA_TX_TCIF    DMA_ISR_TCIF2
#define SPI3_DMA_CH_RX_F    DMA_IFCR_CGIF1 | DMA_IFCR_CHTIF1 | DMA_IFCR_CTCIF1 | DMA_IFCR_CTEIF1
#define SPI3_DMA_CH_TX_F    DMA_IFCR_CGIF2 | DMA_IFCR_CHTIF2 | DMA_IFCR_CTCIF2 | DMA_IFCR_CTEIF2

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

// SPI DMA TX/RX mask
#define SPI_DMA_TX         ((uint8_t)0x01)
#define SPI_DMA_RX         ((uint8_t)0x02)


// Private variables
static const uint16_t SPI_dummy_TX = 0xffff;


// Function prototypes
void SPIx_Init(SPI_TypeDef *SPI, uint16_t SPI_direction, uint16_t SPI_prescaler);
void SPIx_SetSpeed(SPI_TypeDef *SPI, uint16_t SPI_prescaler);
void SPIx_Send(SPI_TypeDef *SPI, uint8_t data);
uint8_t SPIx_SendRecv(SPI_TypeDef *SPI, uint8_t data);
void SPIx_SendBuf(SPI_TypeDef *SPI, uint8_t *pBuf, uint32_t length);
void SPIx_SendBuf16(SPI_TypeDef *SPI, uint16_t *pBuf, uint32_t length);
void SPIx_Configure_DMA_TX(SPI_TypeDef *SPI, uint8_t *pBuf, uint32_t length);
void SPIx_Configure_DMA_RX(SPI_TypeDef *SPI, uint8_t *pBuf, uint32_t length);
void SPIx_SetDMA(SPI_TypeDef *SPI, FunctionalState NewState);

#endif // __SPIx_H
