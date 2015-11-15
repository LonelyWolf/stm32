#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"


#define USBD_VID                     0x0483
#define USBD_PID                     0x572A
#define USBD_LANGID_STRING           0x409
#define USBD_MANUFACTURER_STRING     "STMicroelectronics"
#define USBD_PRODUCT_STRING_FS       "STM32 Mass Storage"
#define USBD_CONFIGURATION_STRING_FS "MSC Config"
#define USBD_INTERFACE_STRING_FS     "MSC Interface"
#define USBD_SERIALNUMBER_STRING_FS  "MSC00000001A"
#define USB_SIZ_BOS_DESC             0x0C


uint8_t *USBD_MSC_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_MSC_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_MSC_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_MSC_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_MSC_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_MSC_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_MSC_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
#ifdef USB_SUPPORT_USER_STRING_DESC
uint8_t *USBD_MSC_USRStringDesc(USBD_SpeedTypeDef speed, uint8_t idx, uint16_t *length);
#endif

#if (USBD_LPM_ENABLED == 1)
uint8_t *USBD_USR_BOSDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
#endif


// MSC descriptor
USBD_DescriptorsTypeDef USBD_MSC_Desc = {
		USBD_MSC_DeviceDescriptor,
		USBD_MSC_LangIDStrDescriptor,
		USBD_MSC_ManufacturerStrDescriptor,
		USBD_MSC_ProductStrDescriptor,
		USBD_MSC_SerialStrDescriptor,
		USBD_MSC_ConfigStrDescriptor,
		USBD_MSC_InterfaceStrDescriptor,
#if (USBD_LPM_ENABLED == 1)  
		USBD_USR_BOSDescriptor,
#endif  
};

// USB standard device descriptor
uint8_t USBD_FS_DeviceDesc[USB_LEN_DEV_DESC] = {
		0x12,                       // bLength
		USB_DESC_TYPE_DEVICE,       // bDescriptorType
#if (USBD_LPM_ENABLED == 1)
		0x01,                       // bcdUSB   changed to USB version 2.01
                                    //          in order to support LPM L1 suspend resume test of USBCV3.0
#else  
		0x00,                       // bcdUSB
#endif  
		0x02,
		0x00,                       // bDeviceClass (use class information in the interface descriptors)
		0x00,                       // bDeviceSubClass
		0x00,                       // bDeviceProtocol
		USB_MAX_EP0_SIZE,           // bMaxPacketSize
		LOBYTE(USBD_VID),           // idVendor
		HIBYTE(USBD_VID),
		LOBYTE(USBD_PID),           // idProduct
		HIBYTE(USBD_PID),
		LOBYTE(USB_DEV_RELEASE),    // bcdDevice - USB device release number
		HIBYTE(USB_DEV_RELEASE),
		USBD_IDX_MFC_STR,           // Index of manufacturer string
		USBD_IDX_PRODUCT_STR,       // Index of product string
		USBD_IDX_SERIAL_STR,        // Index of serial number string
		USBD_MAX_NUM_CONFIGURATION  // bNumConfigurations
};

#if (USBD_LPM_ENABLED == 1)
// BOS descriptor
uint8_t USBD_FS_BOSDesc[USB_SIZ_BOS_DESC] = {
		0x5,
		USB_DESC_TYPE_BOS,
		0xC,
		0x0,
		0x1,  // 1 device capability
		0x7,
		USB_DEVICE_CAPABITY_TYPE,
		0x2,
		0x2,  // LPM capability bit set
		0x0,
		0x0,
		0x0
};
#endif

// Buffer for the language ID descriptor
uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] = {
		USB_LEN_LANGID_STR_DESC,
		USB_DESC_TYPE_STRING,
		LOBYTE(USBD_LANGID_STRING),
		HIBYTE(USBD_LANGID_STRING),
};

// Buffer for the USB string descriptor
uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ];

// Return the device descriptor
// input:
//   speed - current device speed
//   length - pointer to the descriptor buffer
uint8_t *USBD_MSC_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
	*length = sizeof(USBD_FS_DeviceDesc);

	return USBD_FS_DeviceDesc;
}

// Return the LangID string descriptor
// input:
//   speed - current device speed
//   length - pointer to the descriptor buffer
uint8_t *USBD_MSC_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
	*length = sizeof(USBD_LangIDDesc);

	return USBD_LangIDDesc;
}

// Return the product string descriptor
// input:
//   speed - current device speed
//   length - pointer to the descriptor buffer
uint8_t *USBD_MSC_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
	if (speed == USBD_SPEED_HIGH) {
		USBD_GetString((uint8_t *)USBD_PRODUCT_STRING_FS,USBD_StrDesc,length);
	} else {
		USBD_GetString((uint8_t *)USBD_PRODUCT_STRING_FS,USBD_StrDesc,length);
	}

	return USBD_StrDesc;
}

// Return the manufacturer string descriptor
// input:
//   speed - current device speed
//   length - pointer to the descriptor buffer
uint8_t *USBD_MSC_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
	USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING,USBD_StrDesc,length);

	return USBD_StrDesc;
}

// Return the serial number string descriptor
// input:
//   speed - current device speed
//   length - pointer to the descriptor buffer
uint8_t *USBD_MSC_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
	if (speed == USBD_SPEED_HIGH) {
		USBD_GetString((uint8_t *)USBD_SERIALNUMBER_STRING_FS,USBD_StrDesc,length);
	} else {
		USBD_GetString((uint8_t *)USBD_SERIALNUMBER_STRING_FS,USBD_StrDesc,length);
	}

	return USBD_StrDesc;
}

// Return the configuration string descriptor
// input:
//   speed - current device speed
//   length - pointer to the descriptor buffer
uint8_t *USBD_MSC_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
	if (speed == USBD_SPEED_HIGH) {
		USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING_FS,USBD_StrDesc,length);
	} else {
		USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING_FS,USBD_StrDesc,length);
	}

	return USBD_StrDesc;
}

// Return the interface string descriptor
// input:
//   speed - current device speed
//   length - pointer to the descriptor buffer
uint8_t *USBD_MSC_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
	if (speed == USBD_SPEED_HIGH) {
		USBD_GetString((uint8_t *)USBD_INTERFACE_STRING_FS,USBD_StrDesc,length);
	} else {
		USBD_GetString((uint8_t *)USBD_INTERFACE_STRING_FS,USBD_StrDesc,length);
	}

	return USBD_StrDesc;
}

#if (USBD_LPM_ENABLED == 1)
// Return the BOS descriptor
// input:
//   speed - current device speed
//   length - pointer to the descriptor buffer
uint8_t *USBD_USR_BOSDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
	*length = sizeof(USBD_FS_BOSDesc);

	return (uint8_t*)USBD_FS_BOSDesc;
}
#endif
