#include "main.h"


USBD_HandleTypeDef USBD_Device;


GPIO_InitTypeDef PORT;


// SD card enable (PA9)
#define PWR_SD_ENABLE_PORT        GPIOA
#define PWR_SD_ENABLE_PIN         GPIO_Pin_9
#define PWR_SD_ENABLE_PERIPH      RCC_AHBENR_GPIOAEN
#define PWR_SD_ENABLE_H()         PWR_SD_ENABLE_PORT->BSRRL = PWR_SD_ENABLE_PIN
#define PWR_SD_ENABLE_L()         PWR_SD_ENABLE_PORT->BSRRH = PWR_SD_ENABLE_PIN




int main(void) {
	// Initialize the UART
	UARTx_Init(USART2,USART_TX,1382400);
	printf("--- STM32L151RDT6 ---\r\n");


	// Enable power control lines GPIO peripherals
	RCC->AHBENR |= PWR_SD_ENABLE_PERIPH;

	// Enable the DMA2 peripheral clock (for SDIO)
	RCC->AHBENR |= RCC_AHBENR_DMA2EN;

	// Configure power control lines as push-pull output without pull-up
	PORT.GPIO_Mode  = GPIO_Mode_OUT;
	PORT.GPIO_Speed = GPIO_Speed_400KHz;
	PORT.GPIO_OType = GPIO_OType_PP;
	PORT.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	PORT.GPIO_Pin = PWR_SD_ENABLE_PIN;
	GPIO_Init(PWR_SD_ENABLE_PORT,&PORT);
	PWR_SD_ENABLE_H(); // Turn it on


	// USB
	USBD_Init(&USBD_Device,&USBD_MSC_Desc,0);
	USBD_RegisterClass(&USBD_Device,&USBD_MSC);
	USBD_MSC_RegisterInterface(&USBD_Device,&USBD_MSC_fops);
	USBD_Start(&USBD_Device);

    while(1);
}
