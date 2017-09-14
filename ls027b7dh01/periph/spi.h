#ifndef __SPIx_H
#define __SPIx_H


#include <stm32l4xx.h>
#include "gpio.h"


// Define which SPI ports will be used
//   0 - port is not used
//   1 - port used
#define SPI1_USE                   0
#define SPI2_USE                   1
#define SPI3_USE                   0

// Enable DMA functions
//   0 - DMA is not used
//   1 - DMA functions enabled
#define SPI_USE_DMA                0


#if (SPI_USE_DMA)
#include "dma.h"
#endif


// SPI handle structure
typedef struct {
	SPI_TypeDef                   *Instance;  // SPI peripheral base address
	uint8_t                        AF;        // SPI alternate function mapping
	GPIO_HandleTypeDef             PIN_SCK;   // SCK pin
	GPIO_HandleTypeDef             PIN_MISO;  // MISO pin
	GPIO_HandleTypeDef             PIN_MOSI;  // MOSI pin
#if (SPI_USE_DMA)
	DMA_HandleTypeDef              DMA_RX;    // DMA RX channel
	DMA_HandleTypeDef              DMA_TX;    // DMA TX channel
#endif // SPI_USE_DMA
} SPI_HandleTypeDef;


// SPI baud rate prescaler (PCLK/SPI_BR_x)
#define SPI_BR_2                   ((uint32_t)0x00000000U)                      // 2
#define SPI_BR_4                   (SPI_CR1_BR_0)                               // 4
#define SPI_BR_8                   (SPI_CR1_BR_1)                               // 8
#define SPI_BR_16                  (SPI_CR1_BR_1 | SPI_CR1_BR_0)                // 16
#define SPI_BR_32                  (SPI_CR1_BR_2)                               // 32
#define SPI_BR_64                  (SPI_CR1_BR_2 | SPI_CR1_BR_0)                // 64
#define SPI_BR_128                 (SPI_CR1_BR_2 | SPI_CR1_BR_1)                // 128
#define SPI_BR_256                 (SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0) // 256

// SPI direction configuration
#define SPI_DIR_DUPLEX             ((uint32_t)0x00000000U) // 2 lines full duplex
#define SPI_DIR_RX                 ((uint32_t)0x00000001U) // 1 line RX
#define SPI_DIR_TX                 ((uint32_t)0x00000002U) // 1 line TX

// SPI DMA TX/RX mask
#define SPI_DMA_TX                 ((uint8_t)0x01)
#define SPI_DMA_RX                 ((uint8_t)0x02)

// SPI clock phase and polarity configuration
#define SPI_CLK_PL_E1              ((uint32_t)0x00000000U)       // Steady LOW,  1st edge (CPOL=0 CPHA=0)
#define SPI_CLK_PL_E2              (SPI_CR1_CPHA)                // Steady LOW,  2nd edge (CPOL=0 CPHA=1)
#define SPI_CLK_PH_E1              (SPI_CR1_CPOL)                // Steady HIGH, 1st edge (CPOL=1 CPHA=0)
#define SPI_CLK_PH_E2              (SPI_CR1_CPOL | SPI_CR1_CPHA) // Steady HIGH, 2nd edge (CPOL=1 CPHA=1)

// Definition of data frame width configuration
#define SPI_DW_4BIT                (SPI_CR2_DS_0 | SPI_CR2_DS_1)                               //  4 bits
#define SPI_DW_5BIT                (SPI_CR2_DS_2)                                              //  5 bits
#define SPI_DW_6BIT                (SPI_CR2_DS_2 | SPI_CR2_DS_0)                               //  6 bits
#define SPI_DW_7BIT                (SPI_CR2_DS_2 | SPI_CR2_DS_1)                               //  7 bits
#define SPI_DW_8BIT                (SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0)                //  8 bits
#define SPI_DW_9BIT                (SPI_CR2_DS_3)                                              //  9 bits
#define SPI_DW_10BIT               (SPI_CR2_DS_3 | SPI_CR2_DS_0)                               // 10 bits
#define SPI_DW_11BIT               (SPI_CR2_DS_3 | SPI_CR2_DS_1)                               // 11 bits
#define SPI_DW_12BIT               (SPI_CR2_DS_3 | SPI_CR2_DS_1 | SPI_CR2_DS_0)                // 12 bits
#define SPI_DW_13BIT               (SPI_CR2_DS_3 | SPI_CR2_DS_2)                               // 13 bits
#define SPI_DW_14BIT               (SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_0)                // 14 bits
#define SPI_DW_15BIT               (SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1)                // 15 bits
#define SPI_DW_16BIT               (SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0) // 16 bits

// Definition of RX FIFO threshold configuration
#define SPI_RXFIFO_THH             ((uint32_t)0x00000000U) // RXNE generated on FIFO level greater or equal to 1/2 (16-bit)
#define SPI_RXFIFO_THQ             (SPI_CR2_FRXTH)         // RXNE generated on FIFO level greater or equal to 1/4 (8-bit)

// Definition of SPI CRC length
#define SPI_CRC_8BIT               ((uint32_t)0x00000000U) // 8-bit
#define SPI_CRC_16BIT              (SPI_CR1_CRCL)          // 16-bit


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


// Public macros and functions

// Enable SPI peripheral
// input:
//   SPIx - pointer to the SPI port handle
__STATIC_INLINE void SPI_Enable(SPI_HandleTypeDef *SPIx) {
	SPIx->Instance->CR1 |= SPI_CR1_SPE;
}

// Disable SPI peripheral
// input:
//   SPIx - pointer to the SPI port handle
__STATIC_INLINE void SPI_Disable(SPI_HandleTypeDef *SPIx) {
	SPIx->Instance->CR1 &= ~SPI_CR1_SPE;
}

// Configure SPI data frame width
// input:
//   SPIx - pointer to the SPI port handle
//   data_width - data frame width, one of SPI_DW_xx values
__STATIC_INLINE void SPI_SetDataWidth(SPI_HandleTypeDef *SPIx, uint32_t data_width) {
	SPIx->Instance->CR2 &= ~SPI_CR2_DS;
	SPIx->Instance->CR2 |= (data_width & SPI_CR2_DS);
}

// Configure RXFIFO threshold level that triggers an RXNE event
// input:
//   SPIx - pointer to the SPI port handle
//   threshold - threshold configuration, one of SPI_RXFIFO_THx values
__STATIC_INLINE void SPI_SetRXFIFOThreshold(SPI_HandleTypeDef *SPIx, uint32_t threshold) {
	SPIx->Instance->CR2 &= ~SPI_CR2_FRXTH;
	SPIx->Instance->CR2 |= (threshold & SPI_CR2_FRXTH);
}

// Configure SPI CRC length and polynomial value, then enable CRC calculation
// input:
//   SPIx - pointer to the SPI port handle
//   crc_length - CRC length, one of SPI_CRC_xx values
//   polynomial - new CRC polynomial, bust be odd value
// note: must be called only when SPI is disabled
__STATIC_INLINE void SPI_SetCRC(SPI_HandleTypeDef *SPIx, uint32_t crc_length, uint16_t polynomial) {
	SPIx->Instance->CR1 &= ~(SPI_CR1_CRCL);
	SPIx->Instance->CR1 |= (crc_length & SPI_CR1_CRCL);
	SPIx->Instance->CRCPR = polynomial;
	SPIx->Instance->CR1 |= SPI_CR1_CRCEN;
}

// Disable SPI CRC calculation
// input:
//   SPIx - pointer to the SPI port handle
// note: must be called only when SPI is disabled
__STATIC_INLINE void SPI_DisableCRC(SPI_HandleTypeDef *SPIx) {
	SPIx->Instance->CR1 &= ~SPI_CR1_CRCEN;
}

// Reset SPI CRC TX/RX values
// input:
//   SPIx - pointer to the SPI port handle
// note: must be called only when SPI is disabled
__STATIC_INLINE void SPI_ResetCRC(SPI_HandleTypeDef *SPIx) {
	SPIx->Instance->CR1 &= ~SPI_CR1_CRCEN;
	SPIx->Instance->CR1 |=  SPI_CR1_CRCEN;
}

// Get SPI RX CRC value
// input:
//   SPIx - pointer to the SPI port handle
// return: RX CRC value
__STATIC_INLINE uint16_t SPI_GetCRCRX(SPI_HandleTypeDef *SPIx) {
	return SPIx->Instance->RXCRCR;
}

// Get SPI TX CRC value
// input:
//   SPIx - pointer to the SPI port handle
// return: TX CRC value
__STATIC_INLINE uint16_t SPI_GetCRCTX(SPI_HandleTypeDef *SPIx) {
	return SPIx->Instance->TXCRCR;
}


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
void SPI_Init(const SPI_HandleTypeDef *SPI, uint32_t clock_conf, uint16_t SPI_DIR);
void SPI_SetBaudrate(SPI_HandleTypeDef *SPI, uint32_t prescaler);
void SPI_SendBuf(SPI_HandleTypeDef *SPI, uint8_t *pBuf, uint32_t length);
void SPI_SendBuf16(SPI_HandleTypeDef *SPI, uint16_t *pBuf, uint32_t length);
uint8_t SPI_SendRecv(SPI_HandleTypeDef *SPI, uint8_t data);
void SPI_SendRecvBuf(SPI_HandleTypeDef *SPI, uint8_t *pBuf, uint32_t length);

#if (SPI_USE_DMA)
void SPI_Configure_DMA_TX(SPI_HandleTypeDef *SPI, uint8_t *pBuf, uint32_t length);
void SPI_Configure_DMA_RX(SPI_HandleTypeDef *SPI, uint8_t *pBuf, uint32_t length);
void SPI_SetDMA(SPI_HandleTypeDef *SPI, uint8_t SPI_DMA_DIR, FunctionalState NewState);
void SPI_DMA_Handler(DMA_HandleTypeDef *hDMA);
#endif // SPI_USE_DMA

#endif // __SPIx_H
