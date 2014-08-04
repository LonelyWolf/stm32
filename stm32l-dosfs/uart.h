// Define to prevent recursive inclusion -------------------------------------
#ifndef __UART_H
#define __UART_H


#define UART_PORT         USART2
#define UART_TX_PIN       GPIO_Pin_2    // PA2 (USART2_TX)
#define UART_RX_PIN       GPIO_Pin_3    // PA3 (USART2_RX)
#define UART_GPIO_PORT    GPIOA
#define UART_PORT_PERIPH  RCC_AHBPeriph_GPIOA


#define HEX_CHARS         "0123456789ABCDEF"


// Function prototypes
void UART2_Init(uint32_t baudrate);

void UART_SendChar(char ch);

void UART_SendInt(int32_t num);
void UART_SendInt0(int32_t num);
void UART_SendHex8(uint16_t num);
void UART_SendHex16(uint16_t num);
void UART_SendHex32(uint32_t num);

void UART_SendStr(char *str);

void UART_SendBuf(char *buf, uint16_t bufsize);
void UART_SendBufPrintable(char *buf, uint16_t bufsize, char subst);
void UART_SendBufHex(char *buf, uint16_t bufsize);
void UART_SendBufHexFancy(char *buf, uint16_t bufsize, uint8_t column_width, char subst);

#endif // __UART_H
