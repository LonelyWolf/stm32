#include "main.h"


USBD_HandleTypeDef USBD_Device;


uint8_t CDC_BUF[128];


int main(void) {
	// Initialize the UART
	UARTx_Init(USART2,USART_TX,1382400);
	printf("--- STM32L151RDT6 ---\r\n");


	// Initialize the CDC Application
	USBD_Init(&USBD_Device,&USBD_CDC_Descriptor,0);
	// Add Supported Class
	USBD_RegisterClass(&USBD_Device,&USBD_CDC);
	// Add CDC Interface Class
	USBD_CDC_RegisterInterface(&USBD_Device,&USBD_CDC_fops);
	// Start Device Process
	USBD_Start(&USBD_Device);

	// Stuff the buffer
	CDC_BUF[0]  = 'H';
	CDC_BUF[1]  = 'E';
	CDC_BUF[2]  = 'L';
	CDC_BUF[3]  = 'L';
	CDC_BUF[4]  = 'O';
	CDC_BUF[5]  = ' ';
	CDC_BUF[6]  = 'C';
	CDC_BUF[7]  = 'D';
	CDC_BUF[8]  = 'C';
	CDC_BUF[9]  = '\r';
	CDC_BUF[10] = '\n';

	uint32_t i;
	while(1) {
		CDC_Itf_Transmit(CDC_BUF,11);
		for (i = 0x008FFFFF; i--; );
	}
}
