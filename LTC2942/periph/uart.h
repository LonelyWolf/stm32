#ifndef __UART_H
#define __UART_H


#include <stm32l1xx.h>

#include "gpio.h"


// Define which USART ports will be used
// Define which USART ports will be used
//   0 - port is not used
//   1 - port used
#define USART1_USE         1
#define USART2_USE         0
#define USART3_USE         0
#define USART4_USE         0
#define USART5_USE         0


// Support of USART DMA functions
//   0 - USART DMA is not used
//   1 - USART DMA functions availabe
#define USART_USE_DMA      0

#if (USART_USE_DMA)
#include "dma.h"
#endif


// Define to use simple printf
//   0 - USART_printf is not available
//   1 - USART_printf available
#define USART_USE_PRINTF   1

// Example of define an alias for printf to send output to USART1
//#if (USART_USE_PRINTF)
//#define printf(...) USART_printf(USART1,__VA_ARGS__)
//#endif


// USART handle structure
typedef struct {
	USART_TypeDef        *Instance;  // USART peripheral base address
	uint8_t               AF;        // USART alternate function mapping
	GPIO_HandleTypeDef    PIN_RX;    // RX pin
	GPIO_HandleTypeDef    PIN_TX;    // TX pin
#if (USART_USE_DMA)
	DMA_HandleTypeDef     DMA_RX;    // DMA RX channel
	DMA_HandleTypeDef     DMA_TX;    // DMA TX channel
#endif
} USART_HandleTypeDef;


// Chars for hexadecimal output
#define HEX_CHARS          "0123456789ABCDEF"

// USART transmit/receive configuration
#define USART_TX           USART_CR1_TE // Enable transmit
#define USART_RX           USART_CR1_RE // Enable receive
#define USART_DUPLEX       (uint16_t)(USART_CR1_TE | USART_CR1_RE) // Both transmit and receive

// USART IRQ control mask
#define USART_IRQ_TXE      USART_CR1_TXEIE
#define USART_IRQ_TC       USART_CR1_TCIE
#define USART_IRQ_RXNE     USART_CR1_RXNEIE
#define USART_IRQ_IDLE     USART_CR1_IDLEIE

// USART DMA TX/RX mask
#define USART_DMA_TX       USART_CR3_DMAT // USART transmitter DMA
#define USART_DMA_RX       USART_CR3_DMAR // USART receiver DMA

// USART DMA circular mode
#define USART_DMA_BUF_CIRC   DMA_CCR1_CIRC   // DMA circular buffer mode
#define USART_DMA_BUF_NORMAL ((uint32_t)0x0) // DMA normal buffer mode


// Public variables
// USART port handles
#if (USART1_USE)
extern USART_HandleTypeDef hUSART1;
#endif // USART1_USE
#if (USART2_USE)
extern USART_HandleTypeDef hUSART2;
#endif // USART2_USE
#if (USART3_USE)
extern USART_HandleTypeDef hUSART3;
#endif // USART3_USE
#if (USART4_USE)
extern USART_HandleTypeDef hUSART4;
#endif // USART4_USE
#if (USART5_USE)
extern USART_HandleTypeDef hUSART5;
#endif // USART5_USE


// Function prototypes
#if (USART1_USE)
void USART1_HandleInit(void);
#endif // USART1_USE
#if (USART2_USE)
void USART2_HandleInit(void);
#endif // USART2_USE
#if (USART3_USE)
void USART3_HandleInit(void);
#endif // USART3_USE
#if (USART4_USE)
void USART4_HandleInit(void);
#endif // USART4_USE
#if (USART5_USE)
void USART5_HandleInit(void);
#endif // USART5_USE

void USART_Init(const USART_HandleTypeDef *USARTx, uint16_t USART_DIR, uint32_t baudrate);
void USART_SetSpeed(const USART_HandleTypeDef *USARTx, uint32_t baudrate);
void USART_InitIRQ(const USART_HandleTypeDef *USARTx, uint32_t USART_IRQ);

#if (USART_USE_DMA)
void USART_ConfigureDMA(const USART_HandleTypeDef *USARTx, uint8_t DMA_DIR, uint32_t DMA_BUF, uint8_t *pBuf, uint32_t length);
void USART_SetDMA(const USART_HandleTypeDef *USARTx, uint8_t DMA_DIR, FunctionalState NewState);
#endif

void USART_SendChar(USART_TypeDef* USARTx, const char ch);
void USART_SendStr(USART_TypeDef* USARTx, const char *str);

void USART_SendInt(USART_TypeDef* USARTx, int32_t num);
void USART_SendIntLZ(USART_TypeDef* USARTx, int32_t num);
void USART_SendIntU(USART_TypeDef* USARTx, uint32_t num);
void USART_SendHex(USART_TypeDef* USARTx, uint32_t num);
void USART_SendHexLZ(USART_TypeDef* USARTx, uint32_t num, uint8_t digits);

void USART_SendBuf(USART_TypeDef* USARTx, const char *pBuf, uint16_t length);
void USART_SendBufPrintable(USART_TypeDef* USARTx, const char *pBuf, uint16_t length, const char subst);
void USART_SendBufHex(USART_TypeDef* USARTx, const char *pBuf, uint16_t length);

#if (USART_USE_PRINTF)
int USART_printf(USART_TypeDef* USARTx, const char *fmt, ...);
#endif // USART_USE_PRINTF

#endif // __UART_H
