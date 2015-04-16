// Define to prevent recursive inclusion -------------------------------------
#ifndef __SPIx_H
#define __SPIx_H


#define SPI_USE_1 // Define to compile SPI1 support
#define SPI_USE_2 // Define to compile SPI2 support
//#define SPI_USE_3 // Define to compile SPI3 support


// SPI pin structure
typedef struct {
	uint32_t              GPIO_AHB;  // AHB bit for GPIO port
	GPIO_TypeDef         *GPIO;      // Pointer to the pin GPIO port
	uint16_t              GPIO_PIN;  // GPIO pin
	uint8_t               GPIO_SRC;  // GPIO pin source
} Pin_HandleTypeDef;

// SPI DMA channel structure
typedef struct {
	DMA_TypeDef          *Instance;  // DMA peripheral base address
	DMA_Channel_TypeDef  *Channel;   // Pointer to the SPI DMA channel
	uint32_t              TCIF;      // Transfer complete DMA flag
	uint32_t              HTIF;      // Half transfer DMA flag
	uint32_t              CF;        // Mask to clear DMA channel flags
	uint8_t               State;     // State of the DMA channel
} DMA_HandleTypeDef;

// SPI handle structure
typedef struct {
	SPI_TypeDef          *Instance;  // SPI peripheral base address
	uint8_t               AF_SPI;    // SPI alternate function mapping
	Pin_HandleTypeDef     PIN_SCK;   // SCK pin
	Pin_HandleTypeDef     PIN_MISO;  // MISO pin
	Pin_HandleTypeDef     PIN_MOSI;  // MOSI pin
	DMA_HandleTypeDef     DMA_RX;    // DMA RX channel
	DMA_HandleTypeDef     DMA_TX;    // DMA TX channel
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

// SPI line configuration
#define SPI_DIR_DUPLEX     ((uint8_t)0x00) // 2 lines full duplex
#define SPI_DIR_RX         ((uint8_t)0x01) // 1 line RX
#define SPI_DIR_TX         ((uint8_t)0x02) // 1 line TX

// SPI DMA TX/RX mask
#define SPI_DMA_TX         ((uint8_t)0x01)
#define SPI_DMA_RX         ((uint8_t)0x02)

// DMA states
enum {
	DMA_STATE_RESET = 0x00, // DMA not initialized or disabled
	DMA_STATE_READY = 0x01, // DMA ready to use
	DMA_STATE_HT    = 0x02, // DMA half transfer flag
	DMA_STATE_TC    = 0x03, // DMA transfer complete flag
	DMA_STATE_BUSY  = 0x04, // DMA transaction is ongoing
	DMA_STATE_ERROR = 0x05  // DMA error
};


// SPI handles
#ifdef SPI_USE_1
extern SPI_HandleTypeDef hSPI1;
#endif
#ifdef SPI_USE_2
extern SPI_HandleTypeDef hSPI2;
#endif
#ifdef SPI_USE_3
extern SPI_HandleTypeDef hSPI3;
#endif


// Private variables
static const uint16_t SPI_dummy_TX = 0xffff;


// Function prototypes
#ifdef SPI_USE_1
void SPI1_HandleInit(void);
#endif
#ifdef SPI_USE_2
void SPI2_HandleInit(void);
#endif
#ifdef SPI_USE_3
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
void SPIx_Configure_DMA_TX(SPI_HandleTypeDef *SPI, uint8_t *pBuf, uint32_t length);
void SPIx_Configure_DMA_RX(SPI_HandleTypeDef *SPI, uint8_t *pBuf, uint32_t length);
void SPIx_SetDMA(SPI_HandleTypeDef *SPI, uint8_t SPI_DMA_DIR, FunctionalState NewState);
void SPIx_DMA_Handler(DMA_HandleTypeDef *hDMA);

#endif // __SPIx_H
