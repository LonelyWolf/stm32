// Define to prevent recursive inclusion -------------------------------------
#ifndef __UART_H
#define __UART_H


// USART1
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

// USART2
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

// USART3
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


#define USART_FIFO_SIZE    32                    // FIFO buffer size
#define HEX_CHARS          "0123456789ABCDEF"    // HEX characters


// Public variables
extern uint8_t USART_FIFO[];


// Function prototypes
void UARTx_Init(USART_TypeDef* USARTx, uint32_t baudrate);
void UARTx_SetSpeed(USART_TypeDef* USARTx, uint32_t baudrate);
void UARTx_InitRxIRQ(USART_TypeDef* USARTx, uint8_t priority);
void UARTx_InitRxDMA(USART_TypeDef* USARTx, uint8_t priority);

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
