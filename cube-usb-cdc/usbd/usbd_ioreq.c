#include "usbd_ioreq.h"


// Send data to the control pipe
// input:
//   pdev - pointer to the device instance
//   pBuf - pointer to the buffer with data
//   length - length of data to be sent
USBD_StatusTypeDef USBD_CtlSendData(USBD_HandleTypeDef *pdev, uint8_t *pBuf, uint16_t length) {
	// Set EP0 State
	pdev->ep0_state = USBD_EP0_DATA_IN;
	pdev->ep_in[0].total_length = length;
	pdev->ep_in[0].rem_length   = length;
	// Start the transfer
	USBD_LL_Transmit(pdev,0x00,pBuf,length);
  
	return USBD_OK;
}

// Continue sending data to the control pipe
// input:
//   pdev - pointer to the device instance
//   pBuf - pointer to the buffer with data
//   length - length of data to be sent
USBD_StatusTypeDef USBD_CtlContinueSendData(USBD_HandleTypeDef *pdev, uint8_t *pBuf, uint16_t length) {
	// Start the next transfer
	USBD_LL_Transmit(pdev,0x00,pBuf,length);

	return USBD_OK;
}

// Receive data from the control pipe
// input:
//   pdev - pointer to the device instance
//   pBuf - pointer to the buffer with data
//   length - length of data to be received
USBD_StatusTypeDef USBD_CtlPrepareRx(USBD_HandleTypeDef *pdev, uint8_t *pBuf, uint16_t length) {
	// Set EP0 State
	pdev->ep0_state = USBD_EP0_DATA_OUT;
	pdev->ep_out[0].total_length = length;
	pdev->ep_out[0].rem_length   = length;
	// Start the receiving
	USBD_LL_PrepareReceive(pdev,0,pBuf,length);

	return USBD_OK;
}

// Continue receiving data from the control pipe
// input:
//   pdev - pointer to the device instance
//   pBuf - pointer to the buffer with data
//   length - length of data to be received
USBD_StatusTypeDef USBD_CtlContinueRx(USBD_HandleTypeDef *pdev, uint8_t *pBuf, uint16_t length) {
	USBD_LL_PrepareReceive(pdev,0,pBuf,length);

	return USBD_OK;
}

// Send the ZPL packet to the control pipe
// input:
//   pdev - pointer to the device instance
USBD_StatusTypeDef USBD_CtlSendStatus(USBD_HandleTypeDef *pdev) {
	// Set EP0 State
	pdev->ep0_state = USBD_EP0_STATUS_IN;
	// Start the transfer
	USBD_LL_Transmit(pdev,0x00,NULL,0);

	return USBD_OK;
}

// Receive the ZPL packet from the control pipe
// input:
//   pdev - pointer to the device instance
USBD_StatusTypeDef USBD_CtlReceiveStatus(USBD_HandleTypeDef *pdev) {
	// Set EP0 State
	pdev->ep0_state = USBD_EP0_STATUS_OUT;
	// Start the receiving
	USBD_LL_PrepareReceive(pdev,0,NULL,0);

	return USBD_OK;
}

// Return a received data length
// input:
//   pdev - pointer to the device instance
//   epnum - endpoint number
uint16_t USBD_GetRxCount(USBD_HandleTypeDef *pdev, uint8_t epnum) {
	return USBD_LL_GetRxDataSize(pdev,epnum);
}
