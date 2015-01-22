// Define to prevent recursive inclusion
#ifndef __USB_DESC_H
#define __USB_DESC_H

// Exported constants

// USB devID
#define USB_VID                                       0x0483 // STMicroelectronics
#define USB_PID                                       0x5740
#define USB_REV                                       0x0001 // Device release v0.01

// Structure sizes
#define USB_DESC_SIZE_DEVICE                          18
#define USB_DESC_SIZE_CONFIG                          9
#define USB_DESC_SIZE_INTERFACE                       9
#define USB_DESC_SIZE_ENDPOINT                        7
#define USB_STRING_SIZE_LANGID                        4
#define USB_STRING_SIZE_VENDOR                        10
#define USB_STRING_SIZE_PRODUCT                       30
#define USB_STRING_SIZE_SERIAL                        26
#define VCOM_DESC_SIZE_CONFIG                         67

// USB descriptor types
#define USB_DESC_TYPE_DEVICE                          0x01
#define USB_DESC_TYPE_CONFIGURATION                   0x02
#define USB_DESC_TYPE_STRING                          0x03
#define USB_DESC_TYPE_INTERFACE                       0x04
#define USB_DESC_TYPE_ENDPOINT                        0x05
#define USB_DESC_TYPE_CS_INTERFACE                    0x24
#define USB_DESC_TYPE_CS_ENDPOINT                     0x25

// VCOM
#define VCOM_INT_SIZE                                 0x08

// External variables
extern const uint8_t USB_DeviceDescriptor[USB_DESC_SIZE_DEVICE];
extern const uint8_t USB_StringLangID[USB_STRING_SIZE_LANGID];
extern const uint8_t USB_StringVendor[USB_STRING_SIZE_VENDOR];
extern const uint8_t USB_StringProduct[USB_STRING_SIZE_PRODUCT];
extern       uint8_t USB_StringSerial[USB_STRING_SIZE_SERIAL];
extern const uint8_t VCOM_ConfigDescriptor[VCOM_DESC_SIZE_CONFIG];

#endif // __USB_DESC_H
