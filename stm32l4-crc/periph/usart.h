#ifndef __UART_H
#define __UART_H


#include <stm32l4xx.h>

#include "gpio.h"
#include "rcc.h"


// Which USART/UART ports will be used
//   0 - port is not used
//   1 - port used
#define USART1_USE                 0
#define USART2_USE                 1
#define USART3_USE                 0
#define UART4_USE                  0
#define UART5_USE                  0

// Support of USART DMA functions
//   0 - USART DMA is not used
//   1 - USART DMA functions available
#define USART_USE_DMA              0

#if (USART_USE_DMA)
#include "dma.h"
#endif


// Define to use simple printf
//   0 - USART_printf is not available
//   1 - USART_printf available
#define USART_USE_PRINTF           1

// Example of define an alias for printf to send output to USART1
//#if (USART_USE_PRINTF)
//#define printf(...) USART_printf(USART1,__VA_ARGS__)
//#endif


// USART handle structure
typedef struct {
	USART_TypeDef                 *Instance;  // USART peripheral base address
	uint8_t                        AF;        // USART alternate function mapping
	GPIO_HandleTypeDef             PIN_RX;    // RX pin
	GPIO_HandleTypeDef             PIN_TX;    // TX pin
#if (USART_USE_DMA)
	DMA_HandleTypeDef              DMA_RX;    // DMA RX channel
	DMA_HandleTypeDef              DMA_TX;    // DMA TX channel
#endif
} USART_HandleTypeDef;


// Chars for hexadecimal output
#ifndef HEX_CHARS
#define HEX_CHARS                  "0123456789ABCDEF"
#endif

// Definitions of USART/UART transmit/receive configuration
#define USART_MODE_NONE            ((uint32_t)0x00000000U)         // Transmitter and receiver are disabled
#define USART_MODE_TX              USART_CR1_TE                    // Transmitter enabled
#define USART_MODE_RX              USART_CR1_RE                    // Receiver enabled
#define USART_MODE_DUPLEX          (USART_MODE_TX | USART_MODE_RX) // Both transmitter and receiver enabled

// Definitions of USART/UART oversampling setting
#define USART_OVERS16              ((uint32_t)0x00000000U) // oversampling by 16
#define USART_OVERS8               USART_CR1_OVER8         // oversampling by 8

// Definitions of data width configuration
#define USART_DATAWIDTH_7B         USART_CR1_M1            // 7 bits word length
#define USART_DATAWIDTH_8B         ((uint32_t)0x00000000U) // 8 bits word length
#define USART_DATAWIDTH_9B         USART_CR1_M0            // 9 bits word length

// Definitions of parity control configuration
#define USART_PARITY_NONE          ((uint32_t)0x00000000U)        // No parity control
#define USART_PARITY_EVEN          USART_CR1_PCE                  // Even parity
#define USART_PARITY_ODD           (USART_CR1_PCE | USART_CR1_PS) // Odd parity

// Definitions of stop bits configuration
#define USART_STOPBITS_0_5         USART_CR2_STOP_0                      // 0.5 stop bit
#define USART_STOPBITS_1           ((uint32_t)0x00000000U)               // 1 stop bit
#define USART_STOPBITS_1_5         (USART_CR2_STOP_0 | USART_CR2_STOP_1) // 1.5 stop bits
#define USART_STOPBITS_2           USART_CR2_STOP_1                      // 2 stop bits

// Definitions of hardware flow control configuration
#define USART_HWCTL_NONE           ((uint32_t)0x00000000U)           // hardware flow control disabled
#define USART_HWCTL_RTS            USART_CR3_RTSE                    // RTS, data is only requested when there is space in the receive buffer
#define USART_HWCTL_CTS            USART_CR3_CTSE                    // CTS, data is only transmitted when the nCTS input is asserted
#define USART_HWCTL_RTS_CTS        (USART_CR3_RTSE | USART_CR3_CTSE) // both CTS and RTS

// Definitions of USART/UART interrupts (format inspired from HAL)
// Values convention: 000ZZZZ0YY0XXXXXb
//   XXXXX - interrupt source bit position in the USART_CRx register (5 bits)
//   YY - interrupt source register (USART_CRx) (2 bits):
//          0 - Interrupt doesn't have enable bit
//          1 - USART_CR1
//          2 - USART_CR2
//          3 - USART_CR3
//   ZZZZ - position of the interrupt flag in the USART_ISR register (4 bits)
#define USART_IRQ_TXE              ((uint16_t)((7 << 8) | (1 << 5) | 7)) // Transmit data register empty
#define USART_IRQ_TC               ((uint16_t)((6 << 8) | (1 << 5) | 6)) // Transmission complete
#define USART_IRQ_RXNE             ((uint16_t)((5 << 8) | (1 << 5) | 5)) // Read data register not empty
#define USART_IRQ_IDLE             ((uint16_t)((4 << 8) | (1 << 5) | 4)) // IDLE detected
#define USART_IRQ_PE               ((uint16_t)((0 << 8) | (1 << 5) | 8)) // Parity error
#define USART_IRQ_ERR              ((uint16_t)((0 << 8) | (3 << 5)))     // Error interrupt (parity, noise and frame errors)
#define USART_IRQ_ORE              ((uint16_t)((3 << 8)))                // Overrun error (only flag position)
#define USART_IRQ_NE               ((uint16_t)((2 << 8)))                // Noise detected (only flag position)
#define USART_IRQ_FE               ((uint16_t)((1 << 8)))                // Frame error (only flag position)

// Definitions of USART status flags
#define USART_FLAG_BUSY            USART_ISR_BUSY // Busy flag
#define USART_FLAG_AUTOBAUD_CPL    USART_ISR_ABRF // Auto baud rate flag (auto baud rate operation was completed)
#define USART_FLAG_AUTOBAUD_ERR    USART_ISR_ABRE // Auto baud rate error flag (auto baud rate measurement failed)
#define USART_FLAG_TXE             USART_ISR_TXE  // Transmit data register is empty
#define USART_FLAG_RXNE            USART_ISR_RXNE // Read data register is not empty
#define USART_FLAG_TC              USART_ISR_TC   // Transmission complete
#define USART_FLAG_IDLE            USART_ISR_IDLE // Idle line detected
#define USART_FLAG_OVERRUN         USART_ISR_ORE  // Overrun error
#define USART_FLAG_NOISE           USART_ISR_NE   // START bit noise detected
#define USART_FLAG_FRAMING_ERR     USART_ISR_FE   // Framing error
#define USART_FLAG_PARITY_ERR      USART_ISR_PE   // Parity error

// Definition of combination to reset all of USART error flags
#define USART_ERROR_FLAGS          (USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | USART_ISR_PE)

// Definition of combination to reset all of USART error flags
#define USART_ERROR_CF             (USART_ICR_ORECF | USART_ICR_NCF | USART_ICR_FECF | USART_ICR_PECF)

#if (USART_USE_DMA)
// Definitions of USART/UART DMA TX/RX mask
#define USART_DMA_TX               USART_CR3_DMAT // USART transmitter DMA
#define USART_DMA_RX               USART_CR3_DMAR // USART receiver DMA
#endif // USART_USE_DMA


// Public variables

// USART/UART port handles
#if (USART1_USE)
extern USART_HandleTypeDef hUSART1;
#endif // USART1_USE
#if (USART2_USE)
extern USART_HandleTypeDef hUSART2;
#endif // USART2_USE
#if (USART3_USE)
extern USART_HandleTypeDef hUSART3;
#endif // USART3_USE
#if (UART4_USE)
extern USART_HandleTypeDef hUART4;
#endif // UART4_USE
#if (UART5_USE)
extern USART_HandleTypeDef hUART5;
#endif // UART5_USE


// Public macros/functions

// Enable USART
// input:
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
__STATIC_INLINE void USART_Enable(USART_HandleTypeDef *USARTx) {
	USARTx->Instance->CR1 |= USART_CR1_UE;
}

// Disable USART
// input:
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
__STATIC_INLINE void USART_Disable(USART_HandleTypeDef *USARTx) {
	USARTx->Instance->CR1 &= ~USART_CR1_UE;
}

// Get state of the USART flag
// input:
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
//   flag - flag to check, can be any of USART_FLAG_xx or USART_ISR_xx values
// return: state of the flag, zero if it is reset
__STATIC_INLINE uint32_t USART_GetFlags(USART_HandleTypeDef *USARTx, uint32_t flag) {
	return (USARTx->Instance->ISR & flag);
}

// Clear the USART flag
// input:
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
//   flag - any combination of USART_ICR_xx values
__STATIC_INLINE void USART_ClearFlags(USART_HandleTypeDef *USARTx, uint32_t flag) {
	USARTx->Instance->ICR = flag;
}

// Flush the received data
// input:
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
__STATIC_INLINE void USART_FlushRX(USART_HandleTypeDef *USARTx) {
	USARTx->Instance->RQR |= USART_RQR_RXFRQ;
}

// Read received data byte
// input:
//   USARTx - pointer to the USART port handler (hUSART1, hUART4, etc.)
// return: received byte
__STATIC_INLINE uint8_t USART_ReadChar(USART_TypeDef *USARTx) {
	return (uint8_t)(USARTx->RDR);
}


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
#if (UART4_USE)
void UART4_HandleInit(void);
#endif // UART4_USE
#if (UART5_USE)
void UART5_HandleInit(void);
#endif // UART5_USE

void USART_Init(const USART_HandleTypeDef *USARTx, uint32_t mode);
void USART_SetOversampling(const USART_HandleTypeDef *USARTx, uint32_t ovs_mode);
void USART_SetBaudRate(const USART_HandleTypeDef *USARTx, uint32_t baudrate);
uint32_t USART_GetBaudRate(const USART_HandleTypeDef *USARTx);
void USART_SetDataMode(const USART_HandleTypeDef *USARTx, uint32_t databits, uint32_t parity, uint32_t stopbits);
void USART_SetHWFlow(const USART_HandleTypeDef *USARTx, uint32_t hwflow_mode);
void USART_EnableIRQ(USART_HandleTypeDef *USARTx, uint16_t irq);
void USART_DisableIRQ(USART_HandleTypeDef *USARTx, uint16_t irq);

#if (USART_USE_DMA)
void USART_ConfigureDMA(const USART_HandleTypeDef *USARTx, uint32_t DMA_DIR, uint32_t DMA_MODE, uint8_t *pBuf, uint32_t length);
void USART_SetDMA(const USART_HandleTypeDef *USARTx, uint32_t DMA_DIR, FunctionalState NewState);
#endif

uint32_t USART_CheckIdleState(USART_HandleTypeDef *USARTx, volatile uint32_t timeout);

void USART_SendChar(USART_TypeDef *USARTx, const char ch);
void USART_SendStr(USART_TypeDef *USARTx, const char *str);

void USART_SendInt(USART_TypeDef *USARTx, int32_t num);
void USART_SendIntLZ(USART_TypeDef *USARTx, int32_t num);
void USART_SendIntU(USART_TypeDef *USARTx, uint32_t num);
void USART_SendHex(USART_TypeDef *USARTx, uint32_t num);
void USART_SendHexLZ(USART_TypeDef *USARTx, uint32_t num, uint8_t digits);

void USART_SendBuf(USART_TypeDef *USARTx, const char *pBuf, uint32_t length);
void USART_SendBufPrintable(USART_TypeDef *USARTx, const char *pBuf, uint32_t length, const char subst);
void USART_SendBufHex(USART_TypeDef *USARTx, const char *pBuf, uint32_t length);

#if (USART_USE_PRINTF)
int USART_printf(USART_TypeDef *USARTx, const char *fmt, ...);
#endif // USART_USE_PRINTF

#endif // __UART_H
