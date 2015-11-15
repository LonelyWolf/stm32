#include "usbd_msc.h"


// USB MSC class bulk-only-transfer state machine
USBD_MSC_BOT_HandleTypeDef USBD_MSC_BOT;


// USB MSC class private function prototypes
uint8_t  USBD_MSC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
uint8_t  USBD_MSC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
uint8_t  USBD_MSC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
uint8_t  USBD_MSC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
uint8_t  USBD_MSC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
uint8_t *USBD_MSC_GetHSCfgDesc(uint16_t *length);
uint8_t *USBD_MSC_GetFSCfgDesc(uint16_t *length);
uint8_t *USBD_MSC_GetOtherSpeedCfgDesc(uint16_t *length);
uint8_t *USBD_MSC_GetDeviceQualifierDescriptor(uint16_t *length);


// USB mass storage class
USBD_ClassTypeDef USBD_MSC = {
		USBD_MSC_Init,
		USBD_MSC_DeInit,
		USBD_MSC_Setup,
		NULL,              // EP0_TxSent
		NULL,              // EP0_RxReady
		USBD_MSC_DataIn,
		USBD_MSC_DataOut,
		NULL,              // SOF
		NULL,
		NULL,
		USBD_MSC_GetHSCfgDesc,
		USBD_MSC_GetFSCfgDesc,
		USBD_MSC_GetOtherSpeedCfgDesc,
		USBD_MSC_GetDeviceQualifierDescriptor,
};


// USB Mass storage device configuration descriptor (HighSpeed USB)
uint8_t USBD_MSC_CfgHSDesc[USB_MSC_CONFIG_DESC_SIZ] = {
		0x09,                          // bLength: configuration descriptor size
		USB_DESC_TYPE_CONFIGURATION,   // bDescriptorType: configuration
		USB_MSC_CONFIG_DESC_SIZ,
		0x00,
		0x01,   // bNumInterfaces: 1 interface
		0x01,   // bConfigurationValue:
		0x04,   // iConfiguration:
		0xC0,   // bmAttributes:
		0x32,   // MaxPower 100 mA
		// Mass Storage interface
		0x09,   // bLength: Interface Descriptor size */
		0x04,   // bDescriptorType: */
		0x00,   // bInterfaceNumber: Number of Interface */
		0x00,   // bAlternateSetting: Alternate setting */
		0x02,   // bNumEndpoints: 2
		0x08,   // bInterfaceClass: MSC Class
		0x06,   // bInterfaceSubClass : SCSI transparent
		0x50,   // nInterfaceProtocol
		0x05,   // iInterface:
		// IN endpoint
		0x07,   // Endpoint descriptor length
		0x05,   // Endpoint descriptor type
		MSC_EPIN_ADDR,   // Endpoint address (IN, address 1)
		0x02,   // Bulk endpoint type
		LOBYTE(MSC_MAX_HS_PACKET),
		HIBYTE(MSC_MAX_HS_PACKET),
		0x00,   // Polling interval (ignored for bulk endpoints)
		// OUT endpoint
		0x07,   // Endpoint descriptor length
		0x05,   // Endpoint descriptor type
		MSC_EPOUT_ADDR,   // Endpoint address (OUT, address 1)
		0x02,   // Bulk endpoint type
		LOBYTE(MSC_MAX_HS_PACKET),
		HIBYTE(MSC_MAX_HS_PACKET),
		0x00     // Polling interval (ignored for bulk endpoints)
};

// USB Mass storage device configuration descriptor (FullSpeed USB)
uint8_t USBD_MSC_CfgFSDesc[USB_MSC_CONFIG_DESC_SIZ] = {
		0x09,                          // bLength: configuration descriptor size
		USB_DESC_TYPE_CONFIGURATION,   // bDescriptorType: configuration
		USB_MSC_CONFIG_DESC_SIZ,
		0x00,
		0x01,   // bNumInterfaces: 1 interface
		0x01,   // bConfigurationValue:
		0x04,   // iConfiguration:
		0xC0,   // bmAttributes:
		0x32,   // MaxPower 100 mA
		// Mass Storage interface
		0x09,   // bLength: Interface Descriptor size */
		0x04,   // bDescriptorType: */
		0x00,   // bInterfaceNumber: Number of Interface */
		0x00,   // bAlternateSetting: Alternate setting */
		0x02,   // bNumEndpoints: 2
		0x08,   // bInterfaceClass: MSC Class
		0x06,   // bInterfaceSubClass : SCSI transparent
		0x50,   // nInterfaceProtocol
		0x05,   // iInterface:
		// IN endpoint
		0x07,   // Endpoint descriptor length
		0x05,   // Endpoint descriptor type
		MSC_EPIN_ADDR,   // Endpoint address (IN, address 1)
		0x02,   // Bulk endpoint type
		LOBYTE(MSC_MAX_FS_PACKET),
		HIBYTE(MSC_MAX_FS_PACKET),
		0x00,   // Polling interval (ignored for bulk endpoints)
		// OUT endpoint
		0x07,   // Endpoint descriptor length
		0x05,   // Endpoint descriptor type
		MSC_EPOUT_ADDR,   // Endpoint address (OUT, address 1)
		0x02,   // Bulk endpoint type
		LOBYTE(MSC_MAX_FS_PACKET),
		HIBYTE(MSC_MAX_FS_PACKET),
		0x00     // Polling interval (ignored for bulk endpoints)
};

// USB Mass storage device configuration descriptor (????Speed USB)
uint8_t USBD_MSC_OtherSpeedCfgDesc[USB_MSC_CONFIG_DESC_SIZ] = {
		0x09,                          // bLength: configuration descriptor size
		USB_DESC_TYPE_CONFIGURATION,   // bDescriptorType: configuration
		USB_MSC_CONFIG_DESC_SIZ,
		0x00,
		0x01,   // bNumInterfaces: 1 interface
		0x01,   // bConfigurationValue:
		0x04,   // iConfiguration:
		0xC0,   // bmAttributes:
		0x32,   // MaxPower 100 mA
		// Mass Storage interface
		0x09,   // bLength: Interface Descriptor size */
		0x04,   // bDescriptorType: */
		0x00,   // bInterfaceNumber: Number of Interface */
		0x00,   // bAlternateSetting: Alternate setting */
		0x02,   // bNumEndpoints: 2
		0x08,   // bInterfaceClass: MSC Class
		0x06,   // bInterfaceSubClass : SCSI transparent
		0x50,   // nInterfaceProtocol
		0x05,   // iInterface:
		// IN endpoint
		0x07,   // Endpoint descriptor length
		0x05,   // Endpoint descriptor type
		MSC_EPIN_ADDR,   // Endpoint address (IN, address 1)
		0x02,   // Bulk endpoint type
		0x40,   // Packet size: 64 bytes
		0x00,
		0x00,   // Polling interval (ignored for bulk endpoints)
		// OUT endpoint
		0x07,   // Endpoint descriptor length
		0x05,   // Endpoint descriptor type
		MSC_EPOUT_ADDR,   // Endpoint address (OUT, address 1)
		0x02,   // Bulk endpoint type
		0x40,   // Packet size: 64 bytes
		0x00,
		0x00     // Polling interval (ignored for bulk endpoints)
};

// USB device qualifier descriptor
uint8_t USBD_MSC_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] = {
		USB_LEN_DEV_QUALIFIER_DESC,
		USB_DESC_TYPE_DEVICE_QUALIFIER,
		0x00,
		0x02,
		0x00,
		0x00,
		0x00,
		MSC_MAX_FS_PACKET,
		0x01,
		0x00
};


// Initialize the mass storage configuration
// input:
//   pdev - pointer to the device handle
//   cfgidx - USB configuration index
uint8_t USBD_MSC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
	int16_t ret = 0;

	if (pdev->dev_speed == USBD_SPEED_HIGH) {
		// Open OUT endpoint
		USBD_LL_OpenEP(pdev,MSC_EPOUT_ADDR,USBD_EP_TYPE_BULK,MSC_MAX_HS_PACKET);
		// Open IN endpoint
		USBD_LL_OpenEP(pdev,MSC_EPIN_ADDR,USBD_EP_TYPE_BULK,MSC_MAX_HS_PACKET);
	} else {
		// Open OUT endpoint
		USBD_LL_OpenEP(pdev,MSC_EPOUT_ADDR,USBD_EP_TYPE_BULK,MSC_MAX_FS_PACKET);
		// Open IN endpoint
		USBD_LL_OpenEP(pdev,MSC_EPIN_ADDR,USBD_EP_TYPE_BULK,MSC_MAX_FS_PACKET);
	}
	// Directly assign pClassData pointer to the MSC BOT state machine instead of dummy malloc()
	pdev->pClassData = &USBD_MSC_BOT;
//	pdev->pClassData = USBD_malloc(sizeof(USBD_MSC_BOT_HandleTypeDef));
	if (pdev->pClassData == NULL) {
		ret = 1;
	} else {
		// Initialize the BOT layer
		MSC_BOT_Init(pdev);
		ret = 0;
	}

	return ret;
}

// Deinitialize the mass storage configuration
// input:
//   pdev - pointer to the device handle
//   cfgidx - USB configuration index
uint8_t USBD_MSC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
	// Close MSC endpoints
	USBD_LL_CloseEP(pdev,MSC_EPOUT_ADDR);
	USBD_LL_CloseEP(pdev,MSC_EPIN_ADDR);
  
	// Deinitialize the BOT layer
	MSC_BOT_DeInit(pdev);

	// Free the MSC class resources
	if (pdev->pClassData != NULL) {
		USBD_free(pdev->pClassData);
		pdev->pClassData = NULL;
	}

	return 0;
}

// Handle the mass storage specific requests
// input:
//   pdev - pointer to the device handle
//   req - pointer to the structure with USB request
uint8_t USBD_MSC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
	USBD_MSC_BOT_HandleTypeDef *hmsc = (USBD_MSC_BOT_HandleTypeDef*)pdev->pClassData;

	switch (req->bmRequest & USB_REQ_TYPE_MASK) {
		case USB_REQ_TYPE_CLASS :
			// Class request
			switch (req->bRequest) {
				case BOT_GET_MAX_LUN :
					// Get maximal logical unit number
					if ((req->wValue == 0) && (req->wLength == 1) && (req->bmRequest & 0x80)) {
						hmsc->max_lun = ((USBD_StorageTypeDef *)pdev->pUserData)->GetMaxLun();
						USBD_CtlSendData(pdev,(uint8_t *)&hmsc->max_lun,1);
					} else {
						USBD_CtlError(pdev,req);

						return USBD_FAIL;
					}
					break;
				case BOT_RESET:
					// Reset the BOT
					if ((req->wValue == 0) && (req->wLength == 0) && !(req->bmRequest & 0x80)) {
						MSC_BOT_Reset(pdev);
					} else {
						USBD_CtlError(pdev,req);

						return USBD_FAIL;
					}
					break;
				default:
					// Unknown class request
					USBD_CtlError(pdev,req);

					return USBD_FAIL;
			}
			break;
		case USB_REQ_TYPE_STANDARD:
			// Interface and endpoint requests
			switch (req->bRequest) {
				case USB_REQ_GET_INTERFACE:
					USBD_CtlSendData(pdev,(uint8_t *)&hmsc->interface,1);
					break;
				case USB_REQ_SET_INTERFACE:
					hmsc->interface = (uint8_t)(req->wValue);
					break;
				case USB_REQ_CLEAR_FEATURE:
					// Flush the FIFO and clear the stall status
					USBD_LL_FlushEP(pdev,(uint8_t)req->wIndex);
					// Reactivate the endpoint
					USBD_LL_CloseEP(pdev,(uint8_t)req->wIndex);
					if ((uint8_t)req->wIndex & 0x80) {
						// Open IN endpoint
						if (pdev->dev_speed == USBD_SPEED_HIGH) {
							USBD_LL_OpenEP(pdev,MSC_EPIN_ADDR,USBD_EP_TYPE_BULK,MSC_MAX_HS_PACKET);
						} else {
							USBD_LL_OpenEP(pdev,MSC_EPIN_ADDR,USBD_EP_TYPE_BULK,MSC_MAX_FS_PACKET);
						}
					} else {
						if (pdev->dev_speed == USBD_SPEED_HIGH) {
							// Open OUT endpoint
							USBD_LL_OpenEP(pdev,MSC_EPOUT_ADDR,USBD_EP_TYPE_BULK,MSC_MAX_HS_PACKET);
						} else {
							USBD_LL_OpenEP(pdev,MSC_EPOUT_ADDR,USBD_EP_TYPE_BULK,MSC_MAX_FS_PACKET);
						}
					}
					// Handle BOT error
					MSC_BOT_CplClrFeature(pdev,(uint8_t)req->wIndex);
					break;
			}
			break;
		default:
			// Unknown request, ignore it?
			break;
	}
  return 0;
}

// Handle data IN stage
// input:
//   pdev - pointer to the device handle
//   epnum - endpoint number
uint8_t USBD_MSC_DataIn(USBD_HandleTypeDef *pdev,uint8_t epnum) {
	MSC_BOT_DataIn(pdev,epnum);

	return 0;
}

// Handle data OUT stage
// input:
//   pdev - pointer to the device handle
//   epnum - endpoint number
uint8_t USBD_MSC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum) {
	MSC_BOT_DataOut(pdev,epnum);

	return 0;
}

// Return configuration descriptor for HighSpeed USB
// input:
//   length - pointer to the variable with descriptor length
uint8_t *USBD_MSC_GetHSCfgDesc(uint16_t *length) {
	*length = sizeof(USBD_MSC_CfgHSDesc);

	return USBD_MSC_CfgHSDesc;
}

// Return configuration descriptor for FullSpeed USB
// input:
//   length - pointer to the variable with descriptor length
uint8_t *USBD_MSC_GetFSCfgDesc(uint16_t *length) {
	*length = sizeof(USBD_MSC_CfgFSDesc);

	return USBD_MSC_CfgFSDesc;
}

// Return configuration descriptor for ???Speed USB
// input:
//   length - pointer to the variable with descriptor length
uint8_t *USBD_MSC_GetOtherSpeedCfgDesc(uint16_t *length) {
	*length = sizeof(USBD_MSC_OtherSpeedCfgDesc);

	return USBD_MSC_OtherSpeedCfgDesc;
}
/**
* @brief  DeviceQualifierDescriptor 
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
// Return device qualifier descriptor
// input:
//   length - pointer to the variable with descriptor length
uint8_t *USBD_MSC_GetDeviceQualifierDescriptor(uint16_t *length) {
	*length = sizeof(USBD_MSC_DeviceQualifierDesc);

	return USBD_MSC_DeviceQualifierDesc;
}

// Register the MSC interface
// Return configuration descriptor for HighSpeed USB
// input:
//   pdev - pointer to the device handle
//   fops - pointer to the MSC interface callback structure
uint8_t USBD_MSC_RegisterInterface(USBD_HandleTypeDef *pdev, USBD_StorageTypeDef *fops) {
	if (fops != NULL) pdev->pUserData= fops;

	return 0;
}
