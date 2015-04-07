// Define to prevent recursive inclusion -------------------------------------
#ifndef __UART_H
#define __UART_H


// USART1 HAL
#define USART1_PORT         USART1
#define USART1_GPIO_TX      GPIOA
#define USART1_GPIO_RX      GPIOA
#define USART1_GPIO_AHB_TX  RCC_AHBENR_GPIOAEN
#define USART1_GPIO_AHB_RX  RCC_AHBENR_GPIOAEN
#define USART1_GPIO_PIN_TX  GPIO_Pin_9
#define USART1_GPIO_PIN_RX  GPIO_Pin_10
#define USART1_GPIO_TX_SRC  GPIO_PinSource9
#define USART1_GPIO_RX_SRC  GPIO_PinSource10
#define USART1_GPIO_AF      GPIO_AF_USART1
#define USART1_DMA_PERIPH   DMA1
#define USART1_DMA_RX       DMA1_Channel5
#define USART1_DMA_TX       DMA1_Channel4
#define USART1_DMA_RX_TCIF  DMA_ISR_TCIF5
#define USART1_DMA_TX_TCIF  DMA_ISR_TCIF4
#define USART1_DMA_RX_HTIF  DMA_ISR_HTIF5
#define USART1_DMA_TX_HTIF  DMA_ISR_HTIF4
#define USART1_DMA_RXF      DMA_IFCR_CGIF5 | DMA_IFCR_CHTIF5 | DMA_IFCR_CTCIF5 | DMA_IFCR_CTEIF5
#define USART1_DMA_TXF      DMA_IFCR_CGIF4 | DMA_IFCR_CHTIF4 | DMA_IFCR_CTCIF4 | DMA_IFCR_CTEIF4

// USART2 HAL
#define USART2_PORT         USART2
#define USART2_GPIO_TX      GPIOA
#define USART2_GPIO_RX      GPIOA
#define USART2_GPIO_AHB_TX  RCC_AHBENR_GPIOAEN
#define USART2_GPIO_AHB_RX  RCC_AHBENR_GPIOAEN
#define USART2_GPIO_PIN_TX  GPIO_Pin_2
#define USART2_GPIO_PIN_RX  GPIO_Pin_3
#define USART2_GPIO_TX_SRC  GPIO_PinSource2
#define USART2_GPIO_RX_SRC  GPIO_PinSource3
#define USART2_GPIO_AF      GPIO_AF_USART2
#define USART2_DMA_PERIPH   DMA1
#define USART2_DMA_RX       DMA1_Channel6
#define USART2_DMA_TX       DMA1_Channel7
#define USART2_DMA_RX_TCIF  DMA_ISR_TCIF6
#define USART2_DMA_TX_TCIF  DMA_ISR_TCIF7
#define USART2_DMA_RX_HTIF  DMA_ISR_HTIF6
#define USART2_DMA_TX_HTIF  DMA_ISR_HTIF7
#define USART2_DMA_RXF      DMA_IFCR_CGIF6 | DMA_IFCR_CHTIF6 | DMA_IFCR_CTCIF6 | DMA_IFCR_CTEIF6
#define USART2_DMA_TXF      DMA_IFCR_CGIF7 | DMA_IFCR_CHTIF7 | DMA_IFCR_CTCIF7 | DMA_IFCR_CTEIF7

// USART3 HAL
#define USART3_PORT         USART3
#define USART3_GPIO_TX      GPIOC
#define USART3_GPIO_RX      GPIOC
#define USART3_GPIO_AHB_TX  RCC_AHBENR_GPIOCEN
#define USART3_GPIO_AHB_RX  RCC_AHBENR_GPIOCEN
#define USART3_GPIO_PIN_TX  GPIO_Pin_10
#define USART3_GPIO_PIN_RX  GPIO_Pin_11
#define USART3_GPIO_TX_SRC  GPIO_PinSource10
#define USART3_GPIO_RX_SRC  GPIO_PinSource11
#define USART3_GPIO_AF      GPIO_AF_USART3
#define USART3_DMA_PERIPH   DMA1
#define USART3_DMA_RX       DMA1_Channel3
#define USART3_DMA_TX       DMA1_Channel2
#define USART3_DMA_RX_TCIF  DMA_ISR_TCIF3
#define USART3_DMA_TX_TCIF  DMA_ISR_TCIF2
#define USART3_DMA_RX_HTIF  DMA_ISR_HTIF3
#define USART3_DMA_TX_HTIF  DMA_ISR_HTIF2
#define USART3_DMA_RXF      DMA_IFCR_CGIF3 | DMA_IFCR_CHTIF3 | DMA_IFCR_CTCIF3 | DMA_IFCR_CTEIF3
#define USART3_DMA_TXF      DMA_IFCR_CGIF2 | DMA_IFCR_CHTIF2 | DMA_IFCR_CTCIF2 | DMA_IFCR_CTEIF2

// Chars for hexadecimal output
#define HEX_CHARS          "0123456789ABCDEF"

// USART TX/RX mask
#define USART_TX           USART_CR1_TE // Enable transmit
#define USART_RX           USART_CR1_RE // Enable receive

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


// Function prototypes
void UARTx_Init(USART_TypeDef* USARTx, uint32_t USART_DIR, uint32_t baudrate);
void UARTx_SetSpeed(USART_TypeDef* USARTx, uint32_t baudrate);
void UARTx_InitIRQ(USART_TypeDef* USARTx, uint32_t USART_IRQ, uint8_t priority);
void UARTx_ConfigureDMA(USART_TypeDef* USARTx, uint8_t DMA_DIR, uint32_t DMA_BUF, uint8_t *pBuf, uint32_t length);
void UARTx_SetDMA(USART_TypeDef* USARTx, uint8_t DMA_DIR, FunctionalState NewState);

void UART_SendChar(USART_TypeDef* USARTx, char ch);
void UART_SendInt(USART_TypeDef* USARTx, int32_t num);
void UART_SendIntLZ(USART_TypeDef* USARTx, int32_t num);
void UART_SendIntU(USART_TypeDef* USARTx, uint32_t num);
void UART_SendHex8(USART_TypeDef* USARTx, uint8_t num);
void UART_SendHex16(USART_TypeDef* USARTx, uint16_t num);
void UART_SendHex32(USART_TypeDef* USARTx, uint32_t num);
void UART_SendStr(USART_TypeDef* USARTx, char *str);
void UART_SendBuf(USART_TypeDef* USARTx, char *pBuf, uint16_t length);
void UART_SendBufPrintable(USART_TypeDef* USARTx, char *pBuf, uint16_t length, char subst);
void UART_SendBufHex(USART_TypeDef* USARTx, char *pBuf, uint16_t length);

#endif // __UART_H
