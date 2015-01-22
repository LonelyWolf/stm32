#include "usb_lib.h"
#include "usb_regs.h"
#include "usb_istr.h"
#include "usb_pwr.h"
#include "hw_config.h"


// Interval between sending IN packets in frame number (1 frame = 1ms)
#define VCOM_IN_FRAME_INTERVAL             5

// Receive buffer for virtual COM port
uint8_t VCOM_RX_BUF[BULK_MAX_PACKET_SIZE];


// Private functions

// EP1 IN callback routine (device OUT, host IN)
void EP1_IN_Callback(void) {
	Handle_USBAsynchXfer();
}

// EP3 OUT callback routine (device IN, host OUT)
void EP3_OUT_Callback(void) {
	uint16_t USB_Rx_Cntr;
	uint8_t *ptr = VCOM_RX_BUF;

	// Get the received data buffer and update the counter
	USB_Rx_Cntr = USB_SIL_Read(EP3_OUT,VCOM_RX_BUF);

	// USB data will be immediately processed, this allow next USB traffic
	// being NAKed till the end of the USART Xfer

	// Send received data back (just for fun)
	while (USB_Rx_Cntr--) VCP_SendChar(*ptr++);

	// Enable the receive of data on EP3
	SetEPRxValid(ENDP3);
}

// USB SOF (Start Of Frame) callback routine
void SOF_Callback(void) {
	static uint32_t FrameCount = 0;

	if (bDeviceState == CONFIGURED) {
		if (FrameCount++ == VCOM_IN_FRAME_INTERVAL) {
			// Reset the frame counter
			FrameCount = 0;
			// Check the data to be sent through IN pipe
			Handle_USBAsynchXfer();
		}
	}
}
