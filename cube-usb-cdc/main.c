#include "main.h"


USBD_HandleTypeDef USBD_Device;


int main(void) {
	// Initialize the CDC Application
	USBD_Init(&USBD_Device,&CDC_Desc,0);

	// Add Supported Class
	USBD_RegisterClass(&USBD_Device,USBD_CDC_CLASS);

	// Add CDC Interface Class
	USBD_CDC_RegisterInterface(&USBD_Device,&USBD_CDC_fops);

	// Start Device Process
	USBD_Start(&USBD_Device);

	while(1);
}
