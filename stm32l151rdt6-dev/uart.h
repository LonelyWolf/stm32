// Define to prevent recursive inclusion -------------------------------------
#ifndef __UART_H
#define __UART_H


// USART1 HAL
#define USART1_PORT         USART1
#define USART1_PERIPH       RCC_AHBPeriph_GPIOA
#define USART1_TX_PIN       GPIO_Pin_9
#define USART1_RX_PIN       GPIO_Pin_10
#define USART1_TX_PIN_SRC   GPIO_PinSource9
#define USART1_RX_PIN_SRC   GPIO_PinSource10
#define USART1_GPIO_AF      GPIO_AF_USART1
#define USART1_GPIO_PORT    GPIOA
#define USART1_PORT_PERIPH  RCC_AHBPeriph_GPIOA
#define USART1_PORT_APB     RCC_APB2Periph_USART1
#define USART1_DMA_PERIPH   DMA1
#define USART1_DMA_CH_RX    DMA1_Channel5
#define USART1_DMA_CH_TX    DMA1_Channel4
#define USART1_DMA_RX_TCIF  DMA_ISR_TCIF5
#define USART1_DMA_TX_TCIF  DMA_ISR_TCIF4
#define USART1_DMA_CH_RX_F  DMA_IFCR_CGIF5 | DMA_IFCR_CHTIF5 | DMA_IFCR_CTCIF5 | DMA_IFCR_CTEIF5
#define USART1_DMA_CH_TX_F  DMA_IFCR_CGIF4 | DMA_IFCR_CHTIF4 | DMA_IFCR_CTCIF4 | DMA_IFCR_CTEIF4

// USART2 HAL
#define USART2_PORT         USART2
#define USART2_PERIPH       RCC_AHBPeriph_GPIOA
#define USART2_TX_PIN       GPIO_Pin_2
#define USART2_RX_PIN       GPIO_Pin_3
#define USART2_TX_PIN_SRC   GPIO_PinSource2
#define USART2_RX_PIN_SRC   GPIO_PinSource3
#define USART2_GPIO_AF      GPIO_AF_USART2
#define USART2_GPIO_PORT    GPIOA
#define USART2_PORT_PERIPH  RCC_AHBPeriph_GPIOA
#define USART2_PORT_APB     RCC_APB1Periph_USART2
#define USART2_DMA_PERIPH   DMA1
#define USART2_DMA_CH_RX    DMA1_Channel6
#define USART2_DMA_CH_TX    DMA1_Channel7
#define USART2_DMA_RX_TCIF  DMA_ISR_TCIF6
#define USART2_DMA_TX_TCIF  DMA_ISR_TCIF7
#define USART2_DMA_CH_RX_F  DMA_IFCR_CGIF6 | DMA_IFCR_CHTIF6 | DMA_IFCR_CTCIF6 | DMA_IFCR_CTEIF6
#define USART2_DMA_CH_TX_F  DMA_IFCR_CGIF7 | DMA_IFCR_CHTIF7 | DMA_IFCR_CTCIF7 | DMA_IFCR_CTEIF7

// USART3 HAL
#define USART3_PORT         USART3
#define USART3_PERIPH       RCC_AHBPeriph_GPIOC
#define USART3_TX_PIN       GPIO_Pin_10
#define USART3_RX_PIN       GPIO_Pin_11
#define USART3_TX_PIN_SRC   GPIO_PinSource10
#define USART3_RX_PIN_SRC   GPIO_PinSource11
#define USART3_GPIO_AF      GPIO_AF_USART3
#define USART3_GPIO_PORT    GPIOC
#define USART3_PORT_PERIPH  RCC_AHBPeriph_GPIOC
#define USART3_PORT_APB     RCC_APB1Periph_USART3
#define USART3_DMA_PERIPH   DMA1
#define USART3_DMA_CH_RX    DMA1_Channel3
#define USART3_DMA_CH_TX    DMA1_Channel2
#define USART3_DMA_RX_TCIF  DMA_ISR_TCIF3
#define USART3_DMA_TX_TCIF  DMA_ISR_TCIF2
#define USART3_DMA_CH_RX_F  DMA_IFCR_CGIF3 | DMA_IFCR_CHTIF3 | DMA_IFCR_CTCIF3 | DMA_IFCR_CTEIF3
#define USART3_DMA_CH_TX_F  DMA_IFCR_CGIF2 | DMA_IFCR_CHTIF2 | DMA_IFCR_CTCIF2 | DMA_IFCR_CTEIF2

// Chars for hexadecimal output
#define HEX_CHARS          "0123456789ABCDEF"

// USART DMA TX/RX mask
#define USART_DMA_TX       ((uint8_t)0x01)
#define USART_DMA_RX       ((uint8_t)0x02)


// Function prototypes
void UARTx_Init(USART_TypeDef* USARTx, uint32_t baudrate);
void UARTx_SetSpeed(USART_TypeDef* USARTx, uint32_t baudrate);
void UARTx_InitRxIRQ(USART_TypeDef* USARTx, uint8_t priority);
void UARTx_Configure_DMA(USART_TypeDef* USARTx, uint8_t DMA_DIR, uint8_t *pBuf, uint32_t length);
void UARTx_SetDMA(USART_TypeDef* USARTx, uint8_t DMA_DIR, FunctionalState NewState);

void UART_SendChar(USART_TypeDef* USARTx, char ch);
void UART_SendInt(USART_TypeDef* USARTx, int32_t num);
void UART_SendInt0(USART_TypeDef* USARTx, int32_t num);
void UART_SendHex8(USART_TypeDef* USARTx, uint8_t num);
void UART_SendHex16(USART_TypeDef* USARTx, uint16_t num);
void UART_SendHex32(USART_TypeDef* USARTx, uint32_t num);
void UART_SendStr(USART_TypeDef* USARTx, char *str);
void UART_SendBuf(USART_TypeDef* USARTx, char *buf, uint16_t bufsize);
void UART_SendBufPrintable(USART_TypeDef* USARTx, char *buf, uint16_t bufsize, char subst);
void UART_SendBufHex(USART_TypeDef* USARTx, char *buf, uint16_t bufsize);

#endif // __UART_H
