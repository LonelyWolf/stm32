#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"


#define USBD_VID                     0x0483
#define USBD_PID                     0x5740
#define USBD_LANGID_STRING           0x409
#define USBD_MANUFACTURER_STRING     "STMicroelectronics"
#define USBD_PRODUCT_STRING_FS       "STM32 Virtual ComPort"
#define USBD_CONFIGURATION_STRING_FS "CDC Config"
#define USBD_INTERFACE_STRING_FS     "CDC Interface"
#define USB_SIZ_BOS_DESC             0x0C


uint8_t *USBD_CDC_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_CDC_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_CDC_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_CDC_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_CDC_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_CDC_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
uint8_t *USBD_CDC_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
#ifdef USB_SUPPORT_USER_STRING_DESC
uint8_t *USBD_CDC_USRStringDesc(USBD_SpeedTypeDef speed, uint8_t idx, uint16_t *length);
#endif

#if (USBD_LPM_ENABLED == 1)
uint8_t *USBD_USR_BOSDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
#endif


// CDC descriptor
USBD_DescriptorsTypeDef USBD_CDC_Descriptor = {
		USBD_CDC_DeviceDescriptor,
		USBD_CDC_LangIDStrDescriptor,
		USBD_CDC_ManufacturerStrDescriptor,
		USBD_CDC_ProductStrDescriptor,
		USBD_CDC_SerialStrDescriptor,
		USBD_CDC_ConfigStrDescriptor,
		USBD_CDC_InterfaceStrDescriptor,
};

// USB standard device descriptor
uint8_t USBD_DeviceDesc[USB_LEN_DEV_DESC] = {
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

// Buffer for the USB serial descriptor
uint8_t USBD_StringSerial[USB_SIZ_STRING_SERIAL] = {
		USB_SIZ_STRING_SERIAL,
		USB_DESC_TYPE_STRING,
};

// Buffer for the USB string descriptor
uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ];


// Private function prototypes
void IntToUnicode(uint32_t value, uint8_t *pBuf, uint8_t length);
void Get_SerialNum(void);


// Return the device descriptor
// input:
//   speed - current device speed
//   length - pointer to the descriptor buffer
uint8_t *USBD_CDC_DeviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
	*length = sizeof(USBD_DeviceDesc);

	return (uint8_t*)USBD_DeviceDesc;
}

// Return the LangID string descriptor
// input:
//   speed - current device speed
//   length - pointer to the descriptor buffer
uint8_t *USBD_CDC_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
	*length = sizeof(USBD_LangIDDesc);

	return (uint8_t*)USBD_LangIDDesc;
}

// Return the product string descriptor
// input:
//   speed - current device speed
//   length - pointer to the descriptor buffer
uint8_t *USBD_CDC_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
	USBD_GetString((uint8_t *)USBD_PRODUCT_STRING_FS,USBD_StrDesc,length);

	return USBD_StrDesc;
}

// Return the manufacturer string descriptor
// input:
//   speed - current device speed
//   length - pointer to the descriptor buffer
uint8_t *USBD_CDC_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
	USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING,USBD_StrDesc,length);

	return USBD_StrDesc;
}

// Return the serial number string descriptor
// input:
//   speed - current device speed
//   length - pointer to the descriptor buffer
uint8_t *USBD_CDC_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
	*length = USB_SIZ_STRING_SERIAL;

	// Update the serial number string descriptor with the data from the unique ID
	Get_SerialNum();

	return USBD_StringSerial;
}

// Return the configuration string descriptor
// input:
//   speed - current device speed
//   length - pointer to the descriptor buffer
uint8_t *USBD_CDC_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
	USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING_FS,USBD_StrDesc,length);

	return USBD_StrDesc;
}

// Return the interface string descriptor
// input:
//   speed - current device speed
//   length - pointer to the descriptor buffer
uint8_t *USBD_CDC_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length) {
	USBD_GetString((uint8_t *)USBD_INTERFACE_STRING_FS,USBD_StrDesc,length);

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

// Generate the serial number string descriptor
void Get_SerialNum(void) {
	uint32_t deviceserial0, deviceserial1, deviceserial2;

	// Get unique device ID (96-bits)
	deviceserial0 = *(uint32_t*)DEVICE_ID1;
	deviceserial1 = *(uint32_t*)DEVICE_ID2;
	deviceserial2 = *(uint32_t*)DEVICE_ID3;

	if (deviceserial0) {
		// Generate a USB serial number from device ID
		IntToUnicode(deviceserial0,&USBD_StringSerial[2 ],8);
		IntToUnicode(deviceserial1,&USBD_StringSerial[18],8);
		IntToUnicode(deviceserial2,&USBD_StringSerial[34],8);
	} else {
		// A device ID value zero, generate some trash as serial
		for (deviceserial0 = 2; deviceserial0 < USB_SIZ_STRING_SERIAL; deviceserial0 += 2) {
			USBD_StringSerial[deviceserial0]     = (deviceserial0 % 10) + '0';
			USBD_StringSerial[deviceserial0 + 1] = 0;
		}
	}
}

// Convert 32-bit HEX value into char
// input:
//   value - 32-bit value to convert
//   pBuf - pointer to the buffer
//   length - buffer length
void IntToUnicode(uint32_t value, uint8_t *pBuf, uint8_t length) {
	uint8_t idx;
  
	for (idx = 0; idx < length; idx++) {
		pBuf[idx << 1] = (((value >> 28)) < 0xA) ? (value >> 28) + '0' : (value >> 28) + 'A' - 10;
		value <<= 4;
		pBuf[(idx << 1) + 1] = 0;
	}
}
