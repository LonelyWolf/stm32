#include "usb_lib.h"
#include "usb_desc.h"
#include "hw_config.h"


// Private constants

// USB Standard Device Descriptor
const uint8_t USB_DeviceDescriptor[] = {
		USB_DESC_SIZE_DEVICE, // bLength
		USB_DESC_TYPE_DEVICE, // bDescriptorType: device descriptor (0x01)
		0x00, // bcdUSB: USB version 2.00 (0x0200)
		0x02,
		0x02, // bDeviceClass: Communication Device Class
		0x00, // bDeviceSubclass: unused
		0x00, // bDeviceProtocaol: unused
		0x40, // bMaxPacketSize: control pipe max size
		LOBYTE(USB_VID), // idVendor: USB VID
		HIBYTE(USB_VID),
		LOBYTE(USB_PID), // idProduct: USB PID
		HIBYTE(USB_PID),
		LOBYTE(USB_REV), // bcdDevice: device release
		HIBYTE(USB_REV),
		0x01, // iManufacturer: index of string descriptor describing manufacturer
		0x02, // iProduct: index of string descriptor describing product
		0x03, // iSerialNumber: index of string descriptor describing the device serial number
		0x01  // bNumConfigurations: one configuration
};

// USB string descriptor zero
const uint8_t USB_StringLangID[] = {
		USB_STRING_SIZE_LANGID, // Descriptor length
		USB_DESC_TYPE_STRING, // String descriptor
		0x09, // U.S. English (0x0409)
		0x04
};

// USB string descriptor: Vendor
const uint8_t USB_StringVendor[] = {
		USB_STRING_SIZE_VENDOR, // Descriptor length
		USB_DESC_TYPE_STRING, // String descriptor
		// "Wolk"
		'W', 0, 'o', 0, 'l', 0, 'k', 0
};

// USB string descriptor: Product
const uint8_t USB_StringProduct[] = {
		USB_STRING_SIZE_PRODUCT, // Descriptor length
		USB_DESC_TYPE_STRING, // String descriptor
		// "Wolk USB Mass Storage"
		'W', 0, 'o', 0, 'l', 0, 'k', 0, ' ', 0,
		'V', 0, 'C', 0, 'O', 0, 'M', 0, ' ', 0, 'p', 0, 'o', 0, 'r', 0, 't', 0
};

// USB string descriptor: Serial number
uint8_t USB_StringSerial[] = {
		USB_STRING_SIZE_SERIAL, // Descriptor length
		USB_DESC_TYPE_STRING, // String descriptor
		// Dummy data for serial number
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

// Virtual COM port configuration descriptor
const uint8_t VCOM_ConfigDescriptor[] = {
		// Configuration Descriptor
		USB_DESC_SIZE_CONFIG,           // bLength
		USB_DESC_TYPE_CONFIGURATION,    // bDescriptorType: CONFIGURATION
		LOBYTE(VCOM_DESC_SIZE_CONFIG),  // wTotalLength: length of the total configuration block,
		HIBYTE(VCOM_DESC_SIZE_CONFIG),  //               including this descriptor
		0x02, // bNumInterfaces: 2 interfaces
		0x01, // bConfigurationValue: configuration ID
		0x00, // iConfiguration: (unused)
		0xC0, // bmAttributes: self powered
		0x7D, // MaxPower: 250mA maximum power consumption

		// Interface Descriptor
		USB_DESC_SIZE_INTERFACE, // bLength
		USB_DESC_TYPE_INTERFACE, // bDescriptorType: INTERFACE
		// Interface descriptor type
		0x00, // bInterfaceNumber: Number of Interface
		0x00, // bAlternateSetting: Alternate setting
		0x01, // bNumEndpoints: One endpoint used
		0x02, // bInterfaceClass: Communication Interface Class
		0x02, // bInterfaceSubClass: Abstract Control Model
		0x01, // bInterfaceProtocol: Common AT commands: V.250 etc
		0x00, // iInterface:

		// Header Functional Descriptor
		0x05, // bLength: Endpoint Descriptor size
		USB_DESC_TYPE_CS_INTERFACE, // bDescriptorType: class-specific interface
		0x00, // bDescriptorSubtype: Header functional descriptor
		0x10, // bcdCDC: USB class definitions for CDC release number (1.10)
		0x01,

		// Call Management Functional Descriptor
		0x05, // bFunctionLength
		USB_DESC_TYPE_CS_INTERFACE, // bDescriptorType: class-specific interface
		0x01, // bDescriptorSubtype: Call Management Func Desc
		0x00, // bmCapabilities: D0+D1
		0x01, // bDataInterface: 1

		// ACM Functional Descriptor
		0x04, // bFunctionLength
		USB_DESC_TYPE_CS_INTERFACE, // bDescriptorType: class-specific interface
		0x02, // bDescriptorSubtype: Abstract Control Management desc
		0x02, // bmCapabilities

		// Union Functional Descriptor
		0x05, // bFunctionLength
		USB_DESC_TYPE_CS_INTERFACE, // bDescriptorType: class-specific interface
		0x06, // bDescriptorSubtype: Union func desc
		0x00, // bMasterInterface: Communication class interface
		0x01, // bSlaveInterface0: Data Class Interface

		// Endpoint 2 Descriptor
		USB_DESC_SIZE_ENDPOINT, // bLength
		USB_DESC_TYPE_ENDPOINT, // bDescriptorType: endpoint
		0x82, // bEndpointAddress: IN endpoint 2
		0x03, // bmAttributes: interrupt
		LOBYTE(VCOM_INT_SIZE), // wMaxPacketSize: maximum packet size
		HIBYTE(VCOM_INT_SIZE),
		0xFF, // bInterval: ???

		// Data class interface descriptor
		USB_DESC_SIZE_INTERFACE, // bLength
		USB_DESC_TYPE_INTERFACE, // bDescriptorType: INTERFACE
		0x01, // bInterfaceNumber: interface 0 (index of this interface)
		0x00, // bAlternateSetting: index of this alternate setting
		0x02, // bNumEndpoints: 2 endpoints
		0x0A, // bInterfaceClass: CDC
		0x00, // bInterfaceSubClass:
		0x00, // bInterfaceProtocol:
		0x00, // iInterface:

		// Endpoint 1 Descriptor
		USB_DESC_SIZE_ENDPOINT, // bLength
		USB_DESC_TYPE_ENDPOINT, // bDescriptorType: endpoint
		0x81, // bEndpointAddress: IN endpoint 1
		0x02, // bmAttributes: bulk endpoint
		LOBYTE(BULK_MAX_PACKET_SIZE), // wMaxPacketSize: maximum packet size
		HIBYTE(BULK_MAX_PACKET_SIZE),
		0x00,  // bInterval: unused

		// Endpoint 3 Descriptor
		USB_DESC_SIZE_ENDPOINT, // bLength
		USB_DESC_TYPE_ENDPOINT, // bDescriptorType: endpoint
		0x03, // bEndpointAddress: OUT endpoint 3
		0x02, // bmAttributes: bulk endpoint
		LOBYTE(BULK_MAX_PACKET_SIZE), // wMaxPacketSize: maximum packet size
		HIBYTE(BULK_MAX_PACKET_SIZE),
		0x00 // bInterval: unused
};
