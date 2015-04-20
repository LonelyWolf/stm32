#include "main.h"
#include "usb_ll.h"


// USB device handle
USB_HandleTypeDef husb;




// Low level driver callbacks




// SetupStage callback
// input:
//   husb - pointer to the USB device handle
void HAL_USB_SetupStageCallback(USB_HandleTypeDef *husb) {
	USBD_LL_SetupStage(husb->pData,(uint8_t *)husb->Setup);
}

// Data out stage callback
// input:
//   husb - pointer to the USB device handle
//   epnum - endpoint number
void HAL_USB_DataOutStageCallback(USB_HandleTypeDef *husb, uint8_t epnum) {
	USBD_LL_DataOutStage(husb->pData,epnum,husb->OUT_ep[epnum].xfer_buff);
}

// Data in stage callback
// input:
//   husb - pointer to the USB device handle
//   epnum - endpoint number
void HAL_USB_DataInStageCallback(USB_HandleTypeDef *husb, uint8_t epnum) {
	USBD_LL_DataInStage(husb->pData,epnum,husb->IN_ep[epnum].xfer_buff);
}

// USB start of frame callback
// input:
//   husb - pointer to the USB device handle
void HAL_USB_SOFCallback(USB_HandleTypeDef *husb) {
	USBD_LL_SOF(husb->pData);
}

// USB reset callback
// input:
//   husb - pointer to the USB device handle
void HAL_USB_ResetCallback(USB_HandleTypeDef *husb) {
	// Set default speed of the USB device
	USBD_LL_SetSpeed(husb->pData,USBD_SPEED_FULL);
	// Reset the USB device
	USBD_LL_Reset(husb->pData);
}

// USB suspend callback
// input:
//   husb - pointer to the USB device handle
void HAL_USB_SuspendCallback(USB_HandleTypeDef *husb) {
	USBD_LL_Suspend(husb->pData);
}

// USB resume callback
// input:
//   husb - pointer to the USB device handle
void HAL_USB_ResumeCallback(USB_HandleTypeDef *husb) {
	USBD_LL_Resume(husb->pData);
}

// Isochronous OUT transfer complete callback
// input:
//   husb - pointer to the USB device handle
//   epnum - endpoint number
void HAL_USB_ISOOUTIncompleteCallback(USB_HandleTypeDef *husb, uint8_t epnum) {
	USBD_LL_IsoOUTIncomplete(husb->pData,epnum);
}

// Isochronous IN transfer complete callback
// input:
//   husb - pointer to the USB device handle
//   epnum - endpoint number
void HAL_USB_ISOINIncompleteCallback(USB_HandleTypeDef *husb, uint8_t epnum) {
	USBD_LL_IsoINIncomplete(husb->pData,epnum);
}

// USB connect callback
// input:
//   husb - pointer to the USB device handle
void HAL_USB_ConnectCallback(USB_HandleTypeDef *husb) {
	USBD_LL_DevConnected(husb->pData);
}

// USB disconnect callback
// input:
//   husb - pointer to the USB device handle
void HAL_USB_DisconnectCallback(USB_HandleTypeDef *husb) {
	USBD_LL_DevDisconnected(husb->pData);
}




// Low level driver interface




// Initializes the Low Level portion of the Device driver
// input:
//   pdev - pointer to USB device handle
// return: USBD status
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef *pdev) {
	uint32_t i = 0;
	uint32_t wInterrupt_Mask = 0;

	// Set low level Driver parameters
	husb.Instance = USB;
	husb.Init.dev_endpoints = 8;
	husb.Init.EP0_MPS = USB_EP0MPS_64;
	husb.Init.PHY_itface = USB_PHY_EMBEDDED;
	husb.Init.speed = USB_SPEED_FULL;
	husb.Init.low_power_enable = 0;

	// Link the driver to the stack
	husb.pData  = pdev;
	pdev->pData = &husb;

	// Initialize low level driver
	HAL_USB_Init(pdev->pData);

	// Initialize endpoints structures
	for (i = 0; i < husb.Init.dev_endpoints; i++) {
		// Initialize ep structure
		husb.IN_ep[i].is_in = 1;
		husb.IN_ep[i].num   = i;
		// Control until ep is activated
		husb.IN_ep[i].type      = USB_EP_TYPE_CTRL;
		husb.IN_ep[i].maxpacket = 0;
		husb.IN_ep[i].xfer_buff = 0;
		husb.IN_ep[i].xfer_len  = 0;
	}

	for (i = 0; i < husb.Init.dev_endpoints; i++) {
		// Initialize ep structure
		husb.OUT_ep[i].is_in = 0;
		husb.OUT_ep[i].num   = i;
		// Control until ep is activated
		husb.OUT_ep[i].type      = USB_EP_TYPE_CTRL;
		husb.OUT_ep[i].maxpacket = 0;
		husb.OUT_ep[i].xfer_buff = 0;
		husb.OUT_ep[i].xfer_len  = 0;
	}

	// Initialize USB device

	// CNTR_FRES = 1
	husb.Instance->CNTR = USB_CNTR_FRES;

	// CNTR_FRES = 0
	husb.Instance->CNTR = 0;

	// Clear pending interrupts
	husb.Instance->ISTR = 0;

	// Set buffer table address
	husb.Instance->BTABLE = BTABLE_ADDRESS;

	// Set wInterrupt_Mask global variable
	wInterrupt_Mask = USB_CNTR_CTRM | USB_CNTR_WKUPM | USB_CNTR_SUSPM | USB_CNTR_ERRM | USB_CNTR_ESOFM | USB_CNTR_RESETM;

	// Set interrupt mask
	husb.Instance->CNTR = wInterrupt_Mask;
	husb.USB_Address = 0;
	husb.State = USB_READY;

	// Configure endpoints
	HAL_USB_PMAConfig(pdev->pData,      0x00,USB_SNG_BUF,0x040);
	HAL_USB_PMAConfig(pdev->pData,      0x80,USB_SNG_BUF,0x080);
	HAL_USB_PMAConfig(pdev->pData, CDC_IN_EP,USB_SNG_BUF,0x0C0);
	HAL_USB_PMAConfig(pdev->pData,CDC_CMD_EP,USB_SNG_BUF,0x100);
	HAL_USB_PMAConfig(pdev->pData,CDC_OUT_EP,USB_SNG_BUF,0x110);

	return USBD_OK;
}

// Deinitialize the low level portion of the device driver
// input:
//   pdev - pointer to the USB device handle
USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef *pdev) {
	HAL_USB_DeInit(pdev->pData);

	return USBD_OK;
}

// Start the low level portion of the device driver
// input:
//   pdev - pointer to the USB device handle
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef *pdev) {
	// Software connect the USB device
	HAL_USB_SetConnectionState(pdev->pData,1);

	return USBD_OK;
}

// Stop the low level portion of the device driver
// input:
//   pdev - pointer to the USB device handle
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef *pdev) {
	HAL_USB_Stop(pdev->pData);

	return USBD_OK;
}

// Open and configure an endpoint of the low level driver
// input:
//   pdev - pointer to the USB device handle
//   ep_addr - endpoint address
//   ep_mps - endpoint maximum packet size
//   ep_type - endpoint type
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t ep_type, uint16_t ep_mps) {
	HAL_USB_EP_Open(pdev->pData,ep_addr,ep_mps,ep_type);

	return USBD_OK;
}

// Close an endpoint of the low level driver
// input:
//   pdev - pointer to the USB device handle
//   ep_addr - endpoint address
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
	HAL_USB_EP_Close(pdev->pData,ep_addr);

	return USBD_OK;
}

// Flush an endpoint of the low level driver
// input:
//   pdev - pointer to the USB device handle
//   ep_addr - endpoint address
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
	HAL_USB_EP_Flush(pdev->pData,ep_addr);

	return USBD_OK;
}

// Set a STALL condition on an endpoint of the low level driver
// input:
//   pdev - pointer to the USB device handle
//   ep_addr - endpoint address
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
	HAL_USB_EP_SetStall(pdev->pData,ep_addr);

	return USBD_OK;
}

// Clear a STALL condition for an endpoint of the low level driver
// input:
//   pdev - pointer to the USB device handle
//   ep_addr - endpoint address
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
	HAL_USB_EP_ClrStall(pdev->pData, ep_addr);

	return USBD_OK;
}

// Return a STALL condition for an endpoint of the low level driver
// input:
//   pdev - pointer to the USB device handle
//   ep_addr - endpoint address
// return: zero value in case of non-STALL condition, non-zero otherwise
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
	USB_HandleTypeDef *husb = pdev->pData;

	return ((ep_addr & 0x80) == 0x80) ? husb->IN_ep[ep_addr & 0x7F].is_stall : husb->OUT_ep[ep_addr & 0x7F].is_stall;
}

// Assign an address to the USB device
// input:
//   pdev - pointer to the USB device handle
//   dev_addr - new device address
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef *pdev, uint8_t dev_addr) {
	HAL_USB_SetAddress(pdev->pData,dev_addr);

	return USBD_OK;
}

// Transmit data over an endpoint
// input:
//   pdev - pointer to the USB device handle
//   ep_addr - endpoint address
//   pbuf - pointer to the data buffer
//   size - data buffer size
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint16_t size) {
	HAL_USB_EP_Transmit(pdev->pData,ep_addr,pbuf,size);

	return USBD_OK;
}

// Prepare an endpoint for data reception
// input:
//   pdev - pointer to the USB device handle
//   ep_addr - endpoint address
//   pbuf - pointer to the data buffer
//   size - data buffer size
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef *pdev, uint8_t ep_addr, uint8_t *pbuf, uint16_t size) {
	HAL_USB_EP_Receive(pdev->pData,ep_addr,pbuf,size);

	return USBD_OK;
}

// Return the last transfered packet size
// input:
//   pdev - pointer to the USB device handle
//   ep_addr - endpoint address
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef *pdev, uint8_t ep_addr) {
	return HAL_USB_EP_GetRxCount(pdev->pData,ep_addr);
}

// Static single allocation
// input:
//   size - size of allocated memory
void *USBD_static_malloc(uint32_t size) {
	static uint32_t mem[MAX_STATIC_ALLOC_SIZE];

	return mem;
}

// Dummy memory free
// input:
//   p - pointer to allocated memory
void USBD_static_free(void *p) {
}
