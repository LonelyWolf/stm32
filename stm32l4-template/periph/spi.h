#ifndef __SPI_H
#define __SPI_H


#include "stm32l4xx.h"
#include "gpio.h"


// Define which SPI ports will be used
//   0 - port is not used
//   1 - port used
#define SPI1_USE                   1
#define SPI2_USE                   0
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
	SPI_TypeDef                   *Instance; // SPI peripheral base address
	uint8_t                        AF;       // SPI alternate function mapping
	GPIO_HandleTypeDef             PIN_SCK;  // SCK pin
	GPIO_HandleTypeDef             PIN_MISO; // MISO pin
	GPIO_HandleTypeDef             PIN_MOSI; // MOSI pin
#if (SPI_USE_DMA)
	DMA_HandleTypeDef              DMA_RX;   // DMA RX channel
	DMA_HandleTypeDef              DMA_TX;   // DMA TX channel
#endif // SPI_USE_DMA
} SPI_HandleTypeDef;


// SPI baud rate prescaler (PCLK/SPI_BR_x)
#define SPI_BR_2                   ((uint32_t)0x00000000U)                      // 2
#define SPI_BR_4                   SPI_CR1_BR_0                                 // 4
#define SPI_BR_8                   SPI_CR1_BR_1                                 // 8
#define SPI_BR_16                  (SPI_CR1_BR_1 | SPI_CR1_BR_0)                // 16
#define SPI_BR_32                  SPI_CR1_BR_2                                 // 32
#define SPI_BR_64                  (SPI_CR1_BR_2 | SPI_CR1_BR_0)                // 64
#define SPI_BR_128                 (SPI_CR1_BR_2 | SPI_CR1_BR_1)                // 128
#define SPI_BR_256                 (SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0) // 256

// SPI direction configuration
#define SPI_DIR_DUPLEX             ((uint32_t)0x00000000U) // 2 lines full duplex
#define SPI_DIR_RX                 ((uint32_t)0x00000001U) // 1 line RX
#define SPI_DIR_TX                 ((uint32_t)0x00000002U) // 1 line TX

// SPI clock phase and polarity configuration
#define SPI_CLK_PL_E1              ((uint32_t)0x00000000U)       // Steady LOW,  1st edge (CPOL=0 CPHA=0), mode 0
#define SPI_CLK_PL_E2              SPI_CR1_CPHA                  // Steady LOW,  2nd edge (CPOL=0 CPHA=1), mode 1
#define SPI_CLK_PH_E1              SPI_CR1_CPOL                  // Steady HIGH, 1st edge (CPOL=1 CPHA=0), mode 2
#define SPI_CLK_PH_E2              (SPI_CR1_CPOL | SPI_CR1_CPHA) // Steady HIGH, 2nd edge (CPOL=1 CPHA=1), mode 3
#define SPI_CLK_MODE_0             SPI_CLK_PL_E1 // Alias for Mode 0
#define SPI_CLK_MODE_1             SPI_CLK_PL_E2 // Alias for Mode 1
#define SPI_CLK_MODE_2             SPI_CLK_PH_E1 // Alias for Mode 2
#define SPI_CLK_MODE_3             SPI_CLK_PH_E2 // Alias for Mode 3

// Definition of data frame width configuration
#define SPI_DW_4BIT                (SPI_CR2_DS_0 | SPI_CR2_DS_1)                               //  4 bits
#define SPI_DW_5BIT                SPI_CR2_DS_2                                                //  5 bits
#define SPI_DW_6BIT                (SPI_CR2_DS_2 | SPI_CR2_DS_0)                               //  6 bits
#define SPI_DW_7BIT                (SPI_CR2_DS_2 | SPI_CR2_DS_1)                               //  7 bits
#define SPI_DW_8BIT                (SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0)                //  8 bits
#define SPI_DW_9BIT                SPI_CR2_DS_3                                                //  9 bits
#define SPI_DW_10BIT               (SPI_CR2_DS_3 | SPI_CR2_DS_0)                               // 10 bits
#define SPI_DW_11BIT               (SPI_CR2_DS_3 | SPI_CR2_DS_1)                               // 11 bits
#define SPI_DW_12BIT               (SPI_CR2_DS_3 | SPI_CR2_DS_1 | SPI_CR2_DS_0)                // 12 bits
#define SPI_DW_13BIT               (SPI_CR2_DS_3 | SPI_CR2_DS_2)                               // 13 bits
#define SPI_DW_14BIT               (SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_0)                // 14 bits
#define SPI_DW_15BIT               (SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1)                // 15 bits
#define SPI_DW_16BIT               (SPI_CR2_DS_3 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0) // 16 bits

// Definition of RX FIFO threshold configuration
#define SPI_RXFIFO_THH             ((uint32_t)0x00000000U) // RXNE generated on FIFO level greater or equal to 1/2 (16-bit)
#define SPI_RXFIFO_THQ             SPI_CR2_FRXTH           // RXNE generated on FIFO level greater or equal to 1/4 (8-bit)

// Definition of SPI CRC length
#define SPI_CRC_8BIT               ((uint32_t)0x00000000U) // 8-bit
#define SPI_CRC_16BIT              SPI_CR1_CRCL            // 16-bit

// Definition of SPI flags
#define SPI_FLAG_RXNE              SPI_SR_RXNE   // Receive buffer not empty
#define SPI_FLAG_TXE               SPI_SR_TXE    // Transmit buffer empty
#define SPI_FLAG_BSY               SPI_SR_BSY    // Busy
#define SPI_FLAG_CRCERR            SPI_SR_CRCERR // CRC error
#define SPI_FLAG_MODF              SPI_SR_MODF   // Mode fault
#define SPI_FLAG_OVR               SPI_SR_OVR    // Overrun
#define SPI_FLAG_FRE               SPI_SR_FRE    // Frame format error (TI slave mode)


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

#if (SPI_USE_DMA)
// Definitions of SPI DMA TX/RX mask
#define SPI_DMA_TX                 SPI_CR2_TXDMAEN // TX buffer DMA enable
#define SPI_DMA_RX                 SPI_CR2_RXDMAEN // RX buffer DMA enable
#endif // SPI_USE_DMA


// Public macros and functions

// Enable SPI peripheral
// input:
//   SPIx - pointer to the SPI port handle
__STATIC_FORCEINLINE void SPI_Enable(SPI_HandleTypeDef *SPIx) {
	SPIx->Instance->CR1 |= SPI_CR1_SPE;
}

// Disable SPI peripheral
// input:
//   SPIx - pointer to the SPI port handle
__STATIC_FORCEINLINE void SPI_Disable(SPI_HandleTypeDef *SPIx) {
	SPIx->Instance->CR1 &= ~SPI_CR1_SPE;
}

// Configure SPI data frame width
// input:
//   SPIx - pointer to the SPI port handle
//   data_width - data frame width, one of SPI_DW_xx values
__STATIC_INLINE void SPI_SetDataWidth(SPI_HandleTypeDef *SPIx, uint32_t data_width) {
	SPIx->Instance->CR2 &= ~SPI_CR2_DS;
	SPIx->Instance->CR2 |= data_width & SPI_CR2_DS;
}

// Configure RXFIFO threshold level that triggers an RXNE event
// input:
//   SPIx - pointer to the SPI port handle
//   threshold - threshold configuration, one of SPI_RXFIFO_THx values
__STATIC_INLINE void SPI_SetRXFIFOThreshold(SPI_HandleTypeDef *SPIx, uint32_t threshold) {
	SPIx->Instance->CR2 &= ~SPI_CR2_FRXTH;
	SPIx->Instance->CR2 |= threshold & SPI_CR2_FRXTH;
}

// Enable SPI CRC calculation
// input:
//   SPIx - pointer to the SPI port handle
// note: must be called only when SPI is disabled
__STATIC_FORCEINLINE void SPI_EnableCRC(SPI_HandleTypeDef *SPIx) {
	SPIx->Instance->CR1 |= SPI_CR1_CRCEN;
}

// Disable SPI CRC calculation
// input:
//   SPIx - pointer to the SPI port handle
// note: must be called only when SPI is disabled
__STATIC_FORCEINLINE void SPI_DisableCRC(SPI_HandleTypeDef *SPIx) {
	SPIx->Instance->CR1 &= ~SPI_CR1_CRCEN;
}

// Reset SPI CRC TX/RX values
// input:
//   SPIx - pointer to the SPI port handle
// note: must be called only when SPI is disabled
__STATIC_FORCEINLINE void SPI_ResetCRC(SPI_HandleTypeDef *SPIx) {
	SPIx->Instance->CR1 &= ~SPI_CR1_CRCEN;
	SPIx->Instance->CR1 |=  SPI_CR1_CRCEN;
}

// Set the CRCNEXT bit on the SPI peripheral
// input:
//   SPIx - pointer to the SPI port handle
// note: this bit has to be written as soon as the last data frame is written to the DR register
__STATIC_FORCEINLINE void SPI_CRCNext(SPI_HandleTypeDef *SPIx) {
	SPIx->Instance->CR1 |= SPI_CR1_CRCNEXT;
}

// Get SPI RX CRC value
// input:
//   SPIx - pointer to the SPI port handle
// return: RX CRC value
__STATIC_FORCEINLINE uint16_t SPI_GetCRCRX(SPI_HandleTypeDef *SPIx) {
	return SPIx->Instance->RXCRCR;
}

// Get SPI TX CRC value
// input:
//   SPIx - pointer to the SPI port handle
// return: TX CRC value
__STATIC_FORCEINLINE uint16_t SPI_GetCRCTX(SPI_HandleTypeDef *SPIx) {
	return SPIx->Instance->TXCRCR;
}

// Read 8-bit value from the SPI data register
// input:
//   SPIx - pointer to the SPI port handle
// return: data value
__STATIC_FORCEINLINE uint8_t SPI_DataRead8(SPI_HandleTypeDef *SPIx) {
	return (uint8_t)(SPIx->Instance->DR);
}

// Read 16-bit value from the SPI data register
// input:
//   SPIx - pointer to the SPI port handle
// return: data value
__STATIC_FORCEINLINE uint16_t SPI_DataRead16(SPI_HandleTypeDef *SPIx) {
	return (uint16_t)(SPIx->Instance->DR);
}

// Write 8-bit value to the SPI data register
// input:
//   SPIx - pointer to the SPI port handle
//   data - data to write
__STATIC_FORCEINLINE void SPI_DataWrite8(SPI_HandleTypeDef *SPIx, uint8_t data) {
	*((__IO uint8_t *)&SPIx->Instance->DR) = data;
}

// Write 16-bit value to the SPI data register
// input:
//   SPIx - pointer to the SPI port handle
//   data - data to write
__STATIC_FORCEINLINE void SPI_DataWrite16(SPI_HandleTypeDef *SPIx, uint16_t data) {
#if defined (__GNUC__)
	// For strict-aliasing
	__IO uint16_t *DR = (__IO uint16_t *)&SPIx->Instance->DR;
	*DR = data;
#else
	SPIx->Instance->DR = data;
#endif
}

// Get state of the SPI flag
// input:
//   SPIx - pointer to the SPI port handle
//   flag - flag to check, can be any of SPI_FLAG_xx or SPI_SR_xx values
// return: state of the flag, zero if it is reset
__STATIC_FORCEINLINE uint32_t SPI_GetFlag(SPI_HandleTypeDef *SPIx, uint32_t flag) {
	return (SPIx->Instance->SR & flag);
}

// Enable SPI DMA RX/TX buffer
// input:
//   SPIx - pointer to the SPI port handle
//   dir - DMA TX/RX buffer to enable, any combination of SPI_DMA_TX/SPI_DMA_RX values
__STATIC_FORCEINLINE void SPI_DMAEnable(SPI_HandleTypeDef *SPIx, uint32_t dir) {
	SPIx->Instance->CR2 |= dir;
}

// Disable SPI DMA RX/TX buffer
// input:
//   SPIx - pointer to the SPI port handle
//   dir - DMA TX/RX buffer to disable, any combination of SPI_DMA_TX/SPI_DMA_RX values
__STATIC_FORCEINLINE void SPI_DMADisable(SPI_HandleTypeDef *SPIx, uint32_t dir) {
	SPIx->Instance->CR2 &= ~dir;
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
void SPI_Init(const SPI_HandleTypeDef *SPIx, uint32_t clock_conf, uint16_t SPI_DIR);
void SPI_SetBaudrate(SPI_HandleTypeDef *SPIx, uint32_t prescaler);
void SPI_SendBuf(SPI_HandleTypeDef *SPIx, uint8_t *pBuf, uint32_t length);
void SPI_SendBuf16(SPI_HandleTypeDef *SPIx, uint16_t *pBuf, uint32_t length);
uint8_t SPI_SendRecv(SPI_HandleTypeDef *SPIx, uint8_t data);
void SPI_SendRecvBuf(SPI_HandleTypeDef *SPIx, uint8_t *pBuf, uint32_t length);
void SPI_SetCRC(SPI_HandleTypeDef *SPIx, uint32_t crc_length, uint16_t polynomial);
#if (SPI_USE_DMA)
void SPI_ConfigureDMA(SPI_HandleTypeDef *SPIx, uint32_t DMA_DIR, uint32_t DMA_MODE, uint8_t *pBuf, uint32_t length);
void SPI_SetDMA(const SPI_HandleTypeDef *SPIx, uint32_t DMA_DIR, FunctionalState NewState);
#endif // SPI_USE_DMA

#endif // __SPI_H
