#ifndef __SPIx_H
#define __SPIx_H


#include <stm32l1xx.h>


// Define which SPI ports will be used
//   0 - port is not used
//   1 - port used
#define SPI1_USE          1
#define SPI2_USE          0
#define SPI3_USE          0

// Enable DMA functions
//   0 - DMA is not used
//   1 - DMA functions enabled
#define SPI_USE_DMA       0


#if (SPI_USE_DMA)
#include "dma.h"
#endif


// SPI handle structure
typedef struct {
	SPI_TypeDef          *Instance;  // SPI peripheral base address
	uint8_t               AF;        // SPI alternate function mapping
	GPIO_HandleTypeDef    PIN_SCK;   // SCK pin
	GPIO_HandleTypeDef    PIN_MISO;  // MISO pin
	GPIO_HandleTypeDef    PIN_MOSI;  // MOSI pin
#if (SPI_USE_DMA)
	DMA_HandleTypeDef     DMA_RX;    // DMA RX channel
	DMA_HandleTypeDef     DMA_TX;    // DMA TX channel
#endif // SPI_USE_DMA
} SPI_HandleTypeDef;


// SPI baud rate prescaler
#define SPI_BR_2           ((uint16_t)0x0000) // 2
#define SPI_BR_4           ((uint16_t)0x0008) // 4
#define SPI_BR_8           ((uint16_t)0x0010) // 8
#define SPI_BR_16          ((uint16_t)0x0018) // 16
#define SPI_BR_32          ((uint16_t)0x0020) // 32
#define SPI_BR_64          ((uint16_t)0x0028) // 64
#define SPI_BR_128         ((uint16_t)0x0030) // 128
#define SPI_BR_256         ((uint16_t)0x0038) // 256

// SPI direction configuration
#define SPI_DIR_DUPLEX     ((uint8_t)0x00) // 2 lines full duplex
#define SPI_DIR_RX         ((uint8_t)0x01) // 1 line RX
#define SPI_DIR_TX         ((uint8_t)0x02) // 1 line TX

// SPI DMA TX/RX mask
#define SPI_DMA_TX         ((uint8_t)0x01)
#define SPI_DMA_RX         ((uint8_t)0x02)


// SPI handles
#if (SPI1_USE)
extern SPI_HandleTypeDef hSPI1;
#endif
#if (SPI2_USE)
extern SPI_HandleTypeDef hSPI2;
#endif
#if (SPI3_USE)
extern SPI_HandleTypeDef hSPI3;
#endif


// Private variables
static const uint16_t SPI_dummy_TX = 0xffff;


// Function prototypes
#if (SPI1_USE)
void SPI1_HandleInit(void);
#endif
#if (SPI2_USE)
void SPI2_HandleInit(void);
#endif
#if (SPI3_USE)
void SPI3_HandleInit(void);
#endif
void SPIx_Init(const SPI_HandleTypeDef *SPI, uint16_t SPI_DIR, uint16_t prescaler);
void SPIx_SetSpeed(SPI_HandleTypeDef *SPI, uint16_t SPI_prescaler);
void SPIx_Send(SPI_HandleTypeDef *SPI, uint8_t data);
void SPIx_SendBuf(SPI_HandleTypeDef *SPI, uint8_t *pBuf, uint32_t length);
void SPIx_SendBuf16(SPI_HandleTypeDef *SPI, uint16_t *pBuf, uint32_t length);
uint8_t SPIx_SendRecv(SPI_HandleTypeDef *SPI, uint8_t data);
void SPIx_RecvBuf(SPI_HandleTypeDef *SPI, uint8_t *pBuf, uint32_t length, uint8_t dummy);
void SPIx_SendRecvBuf(SPI_HandleTypeDef *SPI, uint8_t *pBuf, uint32_t length);

#if (SPI_USE_DMA)
void SPIx_Configure_DMA_TX(SPI_HandleTypeDef *SPI, uint8_t *pBuf, uint32_t length);
void SPIx_Configure_DMA_RX(SPI_HandleTypeDef *SPI, uint8_t *pBuf, uint32_t length);
void SPIx_SetDMA(SPI_HandleTypeDef *SPI, uint8_t SPI_DMA_DIR, FunctionalState NewState);
void SPIx_DMA_Handler(DMA_HandleTypeDef *hDMA);
#endif // SPI_USE_DMA

#endif // __SPIx_H
