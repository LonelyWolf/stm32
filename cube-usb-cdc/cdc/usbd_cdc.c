#include "usbd_cdc.h"
#include "usbd_desc.h"
#include "usbd_ctlreq.h"


uint8_t  USBD_CDC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
uint8_t  USBD_CDC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx);
uint8_t  USBD_CDC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req);
uint8_t  USBD_CDC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum);
uint8_t  USBD_CDC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum);
uint8_t  USBD_CDC_EP0_RxReady(USBD_HandleTypeDef *pdev);
uint8_t *USBD_CDC_GetFSCfgDesc(uint16_t *length);
uint8_t *USBD_CDC_GetHSCfgDesc(uint16_t *length);
uint8_t *USBD_CDC_GetOtherSpeedCfgDesc(uint16_t *length);
uint8_t *USBD_CDC_GetDeviceQualifierDescriptor(uint16_t *length);


// USB Standard Device Descriptor
uint8_t USBD_CDC_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] = {
		USB_LEN_DEV_QUALIFIER_DESC,
		USB_DESC_TYPE_DEVICE_QUALIFIER,
		0x00,
		0x02,
		0x00,
		0x00,
		0x00,
		0x40,
		0x01,
		0x00,
};

// CDC interface class callbacks structure
USBD_ClassTypeDef USBD_CDC = {
		USBD_CDC_Init,
		USBD_CDC_DeInit,
		USBD_CDC_Setup,
		NULL,                 // EP0_TxSent
		USBD_CDC_EP0_RxReady,
		USBD_CDC_DataIn,
		USBD_CDC_DataOut,
		NULL,                 // SOF
		NULL,                 // IsoINIncomplete
		NULL,                 // IsoOutIncomplete
		USBD_CDC_GetHSCfgDesc,
		USBD_CDC_GetFSCfgDesc,
		USBD_CDC_GetOtherSpeedCfgDesc,
		USBD_CDC_GetDeviceQualifierDescriptor,
};

// USB CDC device Configuration Descriptor
uint8_t USBD_CDC_CfgHSDesc[USB_CDC_CONFIG_DESC_SIZ] = {
		// Configuration Descriptor
		0x09,   /* bLength: Configuration Descriptor size */
		USB_DESC_TYPE_CONFIGURATION,      /* bDescriptorType: Configuration */
		USB_CDC_CONFIG_DESC_SIZ,                /* wTotalLength:no of returned bytes */
		0x00,
		0x02,   /* bNumInterfaces: 2 interface */
		0x01,   /* bConfigurationValue: Configuration value */
		0x00,   /* iConfiguration: Index of string descriptor describing the configuration */
		0xC0,   /* bmAttributes: self powered */
		0x32,   /* MaxPower 0 mA */

		// ---------------------------------------------------------------------------

		// Interface Descriptor
		0x09,   /* bLength: Interface Descriptor size */
		USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
		// Interface descriptor type
		0x00,   /* bInterfaceNumber: Number of Interface */
		0x00,   /* bAlternateSetting: Alternate setting */
		0x01,   /* bNumEndpoints: One endpoints used */
		0x02,   /* bInterfaceClass: Communication Interface Class */
		0x02,   /* bInterfaceSubClass: Abstract Control Model */
		0x01,   /* bInterfaceProtocol: Common AT commands */
		0x00,   /* iInterface: */

		// Header Functional Descriptor
		0x05,   /* bLength: Endpoint Descriptor size */
		0x24,   /* bDescriptorType: CS_INTERFACE */
		0x00,   /* bDescriptorSubtype: Header Func Desc */
		0x10,   /* bcdCDC: spec release number */
		0x01,

		// Call Management Functional Descriptor
		0x05,   /* bFunctionLength */
		0x24,   /* bDescriptorType: CS_INTERFACE */
		0x01,   /* bDescriptorSubtype: Call Management Func Desc */
		0x00,   /* bmCapabilities: D0+D1 */
		0x01,   /* bDataInterface: 1 */

		// ACM Functional Descriptor
		0x04,   /* bFunctionLength */
		0x24,   /* bDescriptorType: CS_INTERFACE */
		0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
		0x02,   /* bmCapabilities */

		// Union Functional Descriptor
		0x05,   /* bFunctionLength */
		0x24,   /* bDescriptorType: CS_INTERFACE */
		0x06,   /* bDescriptorSubtype: Union func desc */
		0x00,   /* bMasterInterface: Communication class interface */
		0x01,   /* bSlaveInterface0: Data Class Interface */

		// Endpoint 2 Descriptor
		0x07,                           /* bLength: Endpoint Descriptor size */
		USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
		CDC_CMD_EP,                     /* bEndpointAddress */
		0x03,                           /* bmAttributes: Interrupt */
		LOBYTE(CDC_CMD_PACKET_SIZE),     /* wMaxPacketSize: */
		HIBYTE(CDC_CMD_PACKET_SIZE),
		0x10,                           /* bInterval: */

		// ---------------------------------------------------------------------------

		// Data class interface descriptor
		0x09,   /* bLength: Endpoint Descriptor size */
		USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
		0x01,   /* bInterfaceNumber: Number of Interface */
		0x00,   /* bAlternateSetting: Alternate setting */
		0x02,   /* bNumEndpoints: Two endpoints used */
		0x0A,   /* bInterfaceClass: CDC */
		0x00,   /* bInterfaceSubClass: */
		0x00,   /* bInterfaceProtocol: */
		0x00,   /* iInterface: */

		// Endpoint OUT Descriptor
		0x07,   /* bLength: Endpoint Descriptor size */
		USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
		CDC_OUT_EP,                        /* bEndpointAddress */
		0x02,                              /* bmAttributes: Bulk */
		LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
		HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),
		0x00,                              /* bInterval: ignore for Bulk transfer */

		// Endpoint IN Descriptor
		0x07,   /* bLength: Endpoint Descriptor size */
		USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
		CDC_IN_EP,                         /* bEndpointAddress */
		0x02,                              /* bmAttributes: Bulk */
		LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
		HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),
		0x00                               /* bInterval: ignore for Bulk transfer */
};


// USB CDC device Configuration Descriptor
uint8_t USBD_CDC_CfgFSDesc[USB_CDC_CONFIG_DESC_SIZ] = {
		// Configuration Descriptor
		0x09,   /* bLength: Configuration Descriptor size */
		USB_DESC_TYPE_CONFIGURATION,      /* bDescriptorType: Configuration */
		USB_CDC_CONFIG_DESC_SIZ,                /* wTotalLength:no of returned bytes */
		0x00,
		0x02,   /* bNumInterfaces: 2 interface */
		0x01,   /* bConfigurationValue: Configuration value */
		0x00,   /* iConfiguration: Index of string descriptor describing the configuration */
		0xC0,   /* bmAttributes: self powered */
		0x32,   /* MaxPower 0 mA */

		// ---------------------------------------------------------------------------

		// Interface Descriptor
		0x09,   /* bLength: Interface Descriptor size */
		USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
		// Interface descriptor type
		0x00,   /* bInterfaceNumber: Number of Interface */
		0x00,   /* bAlternateSetting: Alternate setting */
		0x01,   /* bNumEndpoints: One endpoints used */
		0x02,   /* bInterfaceClass: Communication Interface Class */
		0x02,   /* bInterfaceSubClass: Abstract Control Model */
		0x01,   /* bInterfaceProtocol: Common AT commands */
		0x00,   /* iInterface: */

		// Header Functional Descriptor
		0x05,   /* bLength: Endpoint Descriptor size */
		0x24,   /* bDescriptorType: CS_INTERFACE */
		0x00,   /* bDescriptorSubtype: Header Func Desc */
		0x10,   /* bcdCDC: spec release number */
		0x01,

		// Call Management Functional Descriptor
		0x05,   /* bFunctionLength */
		0x24,   /* bDescriptorType: CS_INTERFACE */
		0x01,   /* bDescriptorSubtype: Call Management Func Desc */
		0x00,   /* bmCapabilities: D0+D1 */
		0x01,   /* bDataInterface: 1 */

		// ACM Functional Descriptor
		0x04,   /* bFunctionLength */
		0x24,   /* bDescriptorType: CS_INTERFACE */
		0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
		0x02,   /* bmCapabilities */

		// Union Functional Descriptor
		0x05,   /* bFunctionLength */
		0x24,   /* bDescriptorType: CS_INTERFACE */
		0x06,   /* bDescriptorSubtype: Union func desc */
		0x00,   /* bMasterInterface: Communication class interface */
		0x01,   /* bSlaveInterface0: Data Class Interface */

		// Endpoint 2 Descriptor
		0x07,                           /* bLength: Endpoint Descriptor size */
		USB_DESC_TYPE_ENDPOINT,   /* bDescriptorType: Endpoint */
		CDC_CMD_EP,                     /* bEndpointAddress */
		0x03,                           /* bmAttributes: Interrupt */
		LOBYTE(CDC_CMD_PACKET_SIZE),     /* wMaxPacketSize: */
		HIBYTE(CDC_CMD_PACKET_SIZE),
		0x10,                           /* bInterval: */

		// ---------------------------------------------------------------------------

		// Data class interface descriptor
		0x09,   /* bLength: Endpoint Descriptor size */
		USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
		0x01,   /* bInterfaceNumber: Number of Interface */
		0x00,   /* bAlternateSetting: Alternate setting */
		0x02,   /* bNumEndpoints: Two endpoints used */
		0x0A,   /* bInterfaceClass: CDC */
		0x00,   /* bInterfaceSubClass: */
		0x00,   /* bInterfaceProtocol: */
		0x00,   /* iInterface: */

		// Endpoint OUT Descriptor
		0x07,   /* bLength: Endpoint Descriptor size */
		USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
		CDC_OUT_EP,                        /* bEndpointAddress */
		0x02,                              /* bmAttributes: Bulk */
		LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
		HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
		0x00,                              /* bInterval: ignore for Bulk transfer */

		// Endpoint IN Descriptor
		0x07,   /* bLength: Endpoint Descriptor size */
		USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
		CDC_IN_EP,                         /* bEndpointAddress */
		0x02,                              /* bmAttributes: Bulk */
		LOBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
		HIBYTE(CDC_DATA_FS_MAX_PACKET_SIZE),
		0x00                               /* bInterval: ignore for Bulk transfer */
};

uint8_t USBD_CDC_OtherSpeedCfgDesc[USB_CDC_CONFIG_DESC_SIZ] = {
		0x09,   /* bLength: Configuation Descriptor size */
		USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION,
		USB_CDC_CONFIG_DESC_SIZ,
		0x00,
		0x02,   /* bNumInterfaces: 2 interfaces */
		0x01,   /* bConfigurationValue: */
		0x04,   /* iConfiguration: */
		0xC0,   /* bmAttributes: */
		0x32,   /* MaxPower 100 mA */

		// Interface Descriptor
		0x09,   /* bLength: Interface Descriptor size */
		USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: Interface */
		// Interface descriptor type
		0x00,   /* bInterfaceNumber: Number of Interface */
		0x00,   /* bAlternateSetting: Alternate setting */
		0x01,   /* bNumEndpoints: One endpoints used */
		0x02,   /* bInterfaceClass: Communication Interface Class */
		0x02,   /* bInterfaceSubClass: Abstract Control Model */
		0x01,   /* bInterfaceProtocol: Common AT commands */
		0x00,   /* iInterface: */

		// Header Functional Descriptor
		0x05,   /* bLength: Endpoint Descriptor size */
		0x24,   /* bDescriptorType: CS_INTERFACE */
		0x00,   /* bDescriptorSubtype: Header Func Desc */
		0x10,   /* bcdCDC: spec release number */
		0x01,

		// Call Management Functional Descriptor
		0x05,   /* bFunctionLength */
		0x24,   /* bDescriptorType: CS_INTERFACE */
		0x01,   /* bDescriptorSubtype: Call Management Func Desc */
		0x00,   /* bmCapabilities: D0+D1 */
		0x01,   /* bDataInterface: 1 */

		// ACM Functional Descriptor
		0x04,   /* bFunctionLength */
		0x24,   /* bDescriptorType: CS_INTERFACE */
		0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
		0x02,   /* bmCapabilities */

		// Union Functional Descriptor
		0x05,   /* bFunctionLength */
		0x24,   /* bDescriptorType: CS_INTERFACE */
		0x06,   /* bDescriptorSubtype: Union func desc */
		0x00,   /* bMasterInterface: Communication class interface */
		0x01,   /* bSlaveInterface0: Data Class Interface */

		// Endpoint 2 Descriptor
		0x07,                           /* bLength: Endpoint Descriptor size */
		USB_DESC_TYPE_ENDPOINT      ,   /* bDescriptorType: Endpoint */
		CDC_CMD_EP,                     /* bEndpointAddress */
		0x03,                           /* bmAttributes: Interrupt */
		LOBYTE(CDC_CMD_PACKET_SIZE),     /* wMaxPacketSize: */
		HIBYTE(CDC_CMD_PACKET_SIZE),
		0xFF,                           /* bInterval: */

		// ---------------------------------------------------------------------------

		// Data class interface descriptor
		0x09,   /* bLength: Endpoint Descriptor size */
		USB_DESC_TYPE_INTERFACE,  /* bDescriptorType: */
		0x01,   /* bInterfaceNumber: Number of Interface */
		0x00,   /* bAlternateSetting: Alternate setting */
		0x02,   /* bNumEndpoints: Two endpoints used */
		0x0A,   /* bInterfaceClass: CDC */
		0x00,   /* bInterfaceSubClass: */
		0x00,   /* bInterfaceProtocol: */
		0x00,   /* iInterface: */

		// Endpoint OUT Descriptor
		0x07,   /* bLength: Endpoint Descriptor size */
		USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
		CDC_OUT_EP,                        /* bEndpointAddress */
		0x02,                              /* bmAttributes: Bulk */
		0x40,                              /* wMaxPacketSize: */
		0x00,
		0x00,                              /* bInterval: ignore for Bulk transfer */

		// Endpoint IN Descriptor
		0x07,   /* bLength: Endpoint Descriptor size */
		USB_DESC_TYPE_ENDPOINT,     /* bDescriptorType: Endpoint */
		CDC_IN_EP,                        /* bEndpointAddress */
		0x02,                             /* bmAttributes: Bulk */
		0x40,                             /* wMaxPacketSize: */
		0x00,
		0x00                              /* bInterval */
};

// Initialize the CDC layer
// input:
//   pdev - pointer to the device instance
//   cfgidx - configuration index
uint8_t USBD_CDC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
	uint8_t ret = 0;
	USBD_CDC_HandleTypeDef *hcdc;
  
	if (pdev->dev_speed == USBD_SPEED_HIGH ) {
		// Open EP IN
		USBD_LL_OpenEP(pdev,CDC_IN_EP,USBD_EP_TYPE_BULK,CDC_DATA_HS_IN_PACKET_SIZE);
		// Open EP OUT
		USBD_LL_OpenEP(pdev,CDC_OUT_EP,USBD_EP_TYPE_BULK,CDC_DATA_HS_OUT_PACKET_SIZE);
	} else {
		// Open EP IN
		USBD_LL_OpenEP(pdev,CDC_IN_EP,USBD_EP_TYPE_BULK,CDC_DATA_FS_IN_PACKET_SIZE);
		// Open EP OUT
		USBD_LL_OpenEP(pdev,CDC_OUT_EP,USBD_EP_TYPE_BULK,CDC_DATA_FS_OUT_PACKET_SIZE);
	}

	// Open Command IN EP
	USBD_LL_OpenEP(pdev,CDC_CMD_EP,USBD_EP_TYPE_INTR,CDC_CMD_PACKET_SIZE);
	pdev->pClassData = (void *)USBD_malloc(sizeof(USBD_CDC_HandleTypeDef));
	if (pdev->pClassData == NULL) {
		ret = 1;
	} else {
		hcdc = (USBD_CDC_HandleTypeDef*)pdev->pClassData;

		// Initialize physical interface components
		((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Init();
    
		// Initialize TX/RX states
		hcdc->TxState = 0;
		hcdc->RxState = 0;

		// Prepare OUT endpoint to receive next packet
		if (pdev->dev_speed == USBD_SPEED_HIGH) {
			USBD_LL_PrepareReceive(pdev,CDC_OUT_EP,hcdc->RxBuffer,CDC_DATA_HS_OUT_PACKET_SIZE);
		} else {
			USBD_LL_PrepareReceive(pdev,CDC_OUT_EP,hcdc->RxBuffer,CDC_DATA_FS_OUT_PACKET_SIZE);
		}
	}

	return ret;
}

// Deinitialize the CDC layer
// input:
//   pdev - pointer to the device instance
//   cfgidx - configuration index
uint8_t USBD_CDC_DeInit(USBD_HandleTypeDef *pdev, uint8_t cfgidx) {
	uint8_t ret = 0;
  
	// Close EP IN
	USBD_LL_CloseEP(pdev,CDC_IN_EP);

	// Close EP OUT
	USBD_LL_CloseEP(pdev,CDC_OUT_EP);
  
	// Close Command IN EP
	USBD_LL_CloseEP(pdev,CDC_CMD_EP);

	// DeInit physical interface components
	if (pdev->pClassData != NULL) {
		((USBD_CDC_ItfTypeDef *)pdev->pUserData)->DeInit();
		USBD_free(pdev->pClassData);
		pdev->pClassData = NULL;
	}

	return ret;
}

// Handle a CDC specific requests
// input:
//   pdev - pointer to the device instance
//   req - pointer to the structure with USB request
uint8_t USBD_CDC_Setup(USBD_HandleTypeDef *pdev, USBD_SetupReqTypedef *req) {
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassData;
	static uint8_t ifalt = 0;
    
	switch (req->bmRequest & USB_REQ_TYPE_MASK) {
		case USB_REQ_TYPE_CLASS :
			if (req->wLength) {
				if (req->bmRequest & 0x80) {
					((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Control(req->bRequest,(uint8_t *)hcdc->data,req->wLength);
					USBD_CtlSendData(pdev,(uint8_t *)hcdc->data,req->wLength);
				} else {
					hcdc->CmdOpCode = req->bRequest;
					hcdc->CmdLength = req->wLength;
        
					USBD_CtlPrepareRx(pdev,(uint8_t *)hcdc->data,req->wLength);
				}
			} else {
				((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Control(req->bRequest,(uint8_t*)req,0);
			}
		break;
		case USB_REQ_TYPE_STANDARD:
			switch (req->bRequest) {
				case USB_REQ_GET_INTERFACE:
					USBD_CtlSendData(pdev,&ifalt,1);
				break;
				case USB_REQ_SET_INTERFACE:
				break;
			}
		default:
			break;
	}

	return USBD_OK;
}

// Process a transfered data from CDC
// input:
//   pdev - pointer to the device instance
//   epnum - endpoint number
uint8_t USBD_CDC_DataIn(USBD_HandleTypeDef *pdev, uint8_t epnum) {
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassData;
  
	if (pdev->pClassData != NULL) {
		hcdc->TxState = 0;

		return USBD_OK;
	}

	return USBD_FAIL;
}

// Process a received data from CDC
// input:
//   pdev - pointer to the device instance
//   epnum - endpoint number
uint8_t  USBD_CDC_DataOut(USBD_HandleTypeDef *pdev, uint8_t epnum) {
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)pdev->pClassData;

	// Get the received data length
	hcdc->RxLength = USBD_LL_GetRxDataSize(pdev,epnum);
  
	// USB data will be immediately processed, this allow next USB traffic being
	// NAKed till the end of the application Xfer
	if (pdev->pClassData != NULL) {
		((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Receive(hcdc->RxBuffer,&hcdc->RxLength);

		return USBD_OK;
	}

	return USBD_FAIL;
}

// ???????
// input:
//   pdev - pointer to the device instance
uint8_t USBD_CDC_EP0_RxReady(USBD_HandleTypeDef *pdev) {
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)pdev->pClassData;

	if ((pdev->pUserData != NULL) && (hcdc->CmdOpCode != 0xFF)) {
		((USBD_CDC_ItfTypeDef *)pdev->pUserData)->Control(hcdc->CmdOpCode,(uint8_t *)hcdc->data,hcdc->CmdLength);
		hcdc->CmdOpCode = 0xFF;
	}

	return USBD_OK;
}

// Return configuration descriptor for FullSpeed USB interface
// input:
//   length - pointer to the variable with data length value
// return: pointer to a buffer with descriptor
uint8_t *USBD_CDC_GetFSCfgDesc(uint16_t *length) {
	*length = sizeof(USBD_CDC_CfgFSDesc);

	return USBD_CDC_CfgFSDesc;
}

// Return configuration descriptor for HighSpeed USB interface
// input:
//   length - pointer to the variable with data length value
// return: pointer to a buffer with descriptor
uint8_t *USBD_CDC_GetHSCfgDesc(uint16_t *length) {
	*length = sizeof (USBD_CDC_CfgHSDesc);

	return USBD_CDC_CfgHSDesc;
}

// Return configuration descriptor for unknown USB speed interface
// input:
//   length - pointer to the variable with data length value
// return: pointer to a buffer with descriptor
uint8_t *USBD_CDC_GetOtherSpeedCfgDesc(uint16_t *length) {
	*length = sizeof(USBD_CDC_OtherSpeedCfgDesc);

	return USBD_CDC_OtherSpeedCfgDesc;
}

// Return device qualifier descriptor
// input:
//   length - pointer to the variable with data length value
// return: pointer to a buffer with descriptor
uint8_t *USBD_CDC_GetDeviceQualifierDescriptor(uint16_t *length) {
	*length = sizeof(USBD_CDC_DeviceQualifierDesc);

	return USBD_CDC_DeviceQualifierDesc;
}

// Register the CDC interface
// input:
//   pdev - pointer to the device instance
//   fops - pointer to the CDC interface callback structure
uint8_t USBD_CDC_RegisterInterface(USBD_HandleTypeDef *pdev, USBD_CDC_ItfTypeDef *fops) {
	uint8_t ret = USBD_FAIL;

	if (fops != NULL) {
		pdev->pUserData = fops;
		ret = USBD_OK;
	}

	return ret;
}

// Configure the CDC transmit buffer
// input:
//   pdev - pointer to the device instance
//   pBuf - pointer to the transmit buffer
//   length - length of the buffer
uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pBuf, uint16_t length) {
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)pdev->pClassData;
	hcdc->TxBuffer = pBuf;
	hcdc->TxLength = length;

	return USBD_OK;
}

// Configure the CDC receive buffer
// input:
//   pdev - pointer to the device instance
//   pBuf - pointer to the receive buffer
uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *pdev,uint8_t *pBuf) {
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)pdev->pClassData;
	hcdc->RxBuffer = pBuf;

	return USBD_OK;
}

// Transmit packet to CDC
// input:
//   pdev - pointer to the device instance
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *pdev) {
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)pdev->pClassData;

	if (pdev->pClassData != NULL) {
		if (hcdc->TxState == 0) {
			// TX transfer IN progress
			hcdc->TxState = 1;

			// Transmit next packet
			USBD_LL_Transmit(pdev,CDC_IN_EP,hcdc->TxBuffer,hcdc->TxLength);

			return USBD_OK;
		}

		return USBD_BUSY;
	}

	return USBD_FAIL;
}

// Prepare OUT endpoint for reception
// input:
//   pdev - pointer to the device instance
uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *pdev) {
	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)pdev->pClassData;
  
	// Suspend or resume USB OUT process
	if (pdev->pClassData != NULL) {
		if (pdev->dev_speed == USBD_SPEED_HIGH) {
			// Prepare OUT endpoint to receive next packet
			USBD_LL_PrepareReceive(pdev,CDC_OUT_EP,hcdc->RxBuffer,CDC_DATA_HS_OUT_PACKET_SIZE);
		} else {
			// Prepare OUT endpoint to receive next packet
			USBD_LL_PrepareReceive(pdev,CDC_OUT_EP,hcdc->RxBuffer,CDC_DATA_FS_OUT_PACKET_SIZE);
		}

		return USBD_OK;
	}

	return USBD_FAIL;
}
