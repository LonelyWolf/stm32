/**
  ******************************************************************************
  * @file    usb_desc.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Descriptors for Audio Speaker Demo
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

#include "usb_lib.h"
#include "usb_desc.h"


// Private constants

// USB Standard Device Descriptor
const uint8_t Speaker_DeviceDescriptor[] = {
		SPEAKER_DEVICE_DESC_SIZE,   // bLength
		USB_DEVICE_DESCRIPTOR_TYPE, // bDescriptorType
		0x00,                       // bcdUSB: USB version 2.00 (0x0002)
		0x02,
		0x00,                       // bDeviceClass
		0x00,                       // bDeviceSubClass
		0x00,                       // bDeviceProtocol
		0x40,                       // bMaxPacketSize 40
		0x83,                       // idVendor   = 0x0483 (STMicroelectronics)
		0x04,
		0x30,                       // idProduct  = 0x5730
		0x57,
		0x00,                       // bcdDevice: rel 2.00 (0x0002)
		0x02,
		0x01,                       // iManufacturer (index of string descriptor describing manufacturer)
		0x02,                       // iProduct (index of string descriptor describing product)
		0x03,                       // iSerialNumber (index of string descriptor describing the device serial number)
		0x01	                    // bNumConfigurations (one configuration)
};

// USB Configuration Descriptor
// All Descriptors (Configuration, Interface, Endpoint, Class, Vendor)
const uint8_t Speaker_ConfigDescriptor[] = {
		// Configuration 1
		0x09,	                           // bLength
		USB_CONFIGURATION_DESCRIPTOR_TYPE, // bDescriptorType
		0x6D,                              // wTotalLength: 110 bytes (0x006d)
		0x00,
		0x02, 	                           // bNumInterfaces (2 interfaces)
		0x01,	                           // bConfigurationValue
		0x00, 	                           // iConfiguration
		0x80,                              // bmAttributes: bus powered
		0x7D,                              // bMaxPower = 250mA (in 2mA units)

		// USB Speaker Standard interface descriptor
		SPEAKER_SIZ_INTERFACE_DESC_SIZE, // bLength
		USB_INTERFACE_DESCRIPTOR_TYPE,   // bDescriptorType
		0x00, 	                         // bInterfaceNumber
		0x00,	                         // bAlternateSetting
		0x00,	                         // bNumEndpoints
		USB_DEVICE_CLASS_AUDIO,          // bInterfaceClass
		AUDIO_SUBCLASS_AUDIOCONTROL,     // bInterfaceSubClass
		AUDIO_PROTOCOL_UNDEFINED,        // bInterfaceProtocol
		0x00,	                         // iInterface

		// USB Speaker Class-specific AC Interface Descriptor
		SPEAKER_SIZ_INTERFACE_DESC_SIZE, // bLength
		AUDIO_INTERFACE_DESCRIPTOR_TYPE, // bDescriptorType
		AUDIO_CONTROL_HEADER, 	         // bDescriptorSubtype
		0x00,	                         // bcdADC: v1.00
		0x01,
		0x27,	                         // wTotalLength = 39 (0x0027)
		0x00,
		0x01,                            // bInCollection (one streaming interface)
		0x01,                            // baInterfaceNr

		// USB Speaker Input Terminal Descriptor
		AUDIO_INPUT_TERMINAL_DESC_SIZE,  // bLength
		AUDIO_INTERFACE_DESCRIPTOR_TYPE, // bDescriptorType
		AUDIO_CONTROL_INPUT_TERMINAL,    // bDescriptorSubType
		0x01,                            // bTerminalID
		0x01,                            // wTerminalType AUDIO_TERMINAL_USB_STREAMING (0x0101)
		0x01,
		0x00,                            // bAssocTerminal
		0x01,                            // bNrChannels (one channel)
		0x00,                            // wChannelConfig 0x0000 (mono)
		0x00,
		0x00,                            // iChannelNames
		0x00,                            // iTerminal

		// USB Speaker Audio Feature Unit Descriptor
		0x09,                            // bLength
		AUDIO_INTERFACE_DESCRIPTOR_TYPE, // bDescriptorType
		AUDIO_CONTROL_FEATURE_UNIT,      // bDescriptorSubType
		0x02,                            // bUnitID
		0x01,                            // bSourceID
		0x01,                            // bControlSize
		AUDIO_CONTROL_MUTE,              // bmaControls(0)
		0x00,                            // bmaControls(1)
		0x00,                            // iTerminal

		// USB Speaker Output Terminal Descriptor
		0x09,                            // bLength
		AUDIO_INTERFACE_DESCRIPTOR_TYPE, // bDescriptorType
		AUDIO_CONTROL_OUTPUT_TERMINAL,   // bDescriptorSubType
		0x03,                            // bTerminalID
		0x01,                            // wTerminalType  0x0301
		0x03,
		0x00,                            // bAssocTerminal
		0x02,                            // bSourceID
		0x00,                            // iTerminal

		// USB Speaker Standard AS Interface Descriptor - Audio Streaming Zero Bandwidth
		// Interface 1, Alternate Setting 0
		SPEAKER_SIZ_INTERFACE_DESC_SIZE, // bLength
		USB_INTERFACE_DESCRIPTOR_TYPE,   // bDescriptorType
		0x01,                            // bInterfaceNumber
		0x00,                            // bAlternateSetting
		0x00,                            // bNumEndpoints
		USB_DEVICE_CLASS_AUDIO,          // bInterfaceClass
		AUDIO_SUBCLASS_AUDIOSTREAMING,   // bInterfaceSubClass
		AUDIO_PROTOCOL_UNDEFINED,        // bInterfaceProtocol
		0x00,                            // iInterface

		// USB Speaker Standard AS Interface Descriptor - Audio Streaming Operational
		// Interface 1, Alternate Setting 1
		SPEAKER_SIZ_INTERFACE_DESC_SIZE, // bLength
		USB_INTERFACE_DESCRIPTOR_TYPE,   // bDescriptorType
		0x01,                            // bInterfaceNumber
		0x01,                            // bAlternateSetting
		0x01,                            // bNumEndpoints
		USB_DEVICE_CLASS_AUDIO,          // bInterfaceClass
		AUDIO_SUBCLASS_AUDIOSTREAMING,   // bInterfaceSubClass
		AUDIO_PROTOCOL_UNDEFINED,        // bInterfaceProtocol
		0x00,                            // iInterface

		// USB Speaker Audio Streaming Interface Descriptor
		AUDIO_STREAMING_INTERFACE_DESC_SIZE, // bLength
		AUDIO_INTERFACE_DESCRIPTOR_TYPE,     // bDescriptorType
		AUDIO_STREAMING_GENERAL,             // bDescriptorSybType
		0x01,                                // bTerminalLink (terminal ID)
		0x01,                                // bDelay
		0x02,                                // wFormatTag AUDIO_FORMAT_PCM8 (0x0002)
		0x00,

		// USB Speaker Audio Type I Format Interface Descriptor
		11,                              // bLength
		AUDIO_INTERFACE_DESCRIPTOR_TYPE, // bDescriptorType
		AUDIO_STREAMING_FORMAT_TYPE,     // bDescriptorSubType
		AUDIO_FORMAT_TYPE_I,             // bFormatType
		0x01,                            // bNrChannels (one channel)
		0x01,                            // bSubFrameSize (one byte per audio subframe)
		0x08,                            // bBitResolution (8-bit)
		0x01,                            // bSamFreqType (one frequency supported)
		0x44,                            // tSamFreq: 44100 = 0x00AC44
		0xAC,
		0x00,

		// Endpoint 1 - Standard Descriptor
		AUDIO_STANDARD_ENDPOINT_DESC_SIZE, // bLength
		USB_ENDPOINT_DESCRIPTOR_TYPE,      // bDescriptorType
		0x01,                              // bEndpointAddress: 1 OUT endpoint
		USB_ENDPOINT_TYPE_ISOCHRONOUS,     // bmAttributes: isochronous, not shared
		0x64,                              // wMaxPacketSize: 100 bytes per packet
		0x00,
		0x01,                              // bInterval (one packet per frame)
		0x00,                              // bRefresh
		0x00,                              // bSynchAddress

		// Endpoint - Audio Streaming Descriptor
		AUDIO_STREAMING_ENDPOINT_DESC_SIZE, // bLength
		AUDIO_ENDPOINT_DESCRIPTOR_TYPE,     // bDescriptorType
		AUDIO_ENDPOINT_GENERAL,             // bDescriptorSubType
		0x00,                               // bmAttributes
		0x00,                               // bLockDelayUnits (0x00)
		0x00,                               // wLockDelay (0x0000)
		0x00,
};

// USB String Descriptor (optional)
const uint8_t Speaker_StringLangID[SPEAKER_SIZ_STRING_LANGID] = {
		0x04,
		0x03,
		0x09,
		0x04
};

// LangID = 0x0409: U.S. English
const uint8_t Speaker_StringVendor[SPEAKER_SIZ_STRING_VENDOR] = {
		SPEAKER_SIZ_STRING_VENDOR,  // Size of manufacturer string
		USB_STRING_DESCRIPTOR_TYPE, // bDescriptorType
		// Manufacturer: "STMicroelectronics"
		'S', 0, 'T', 0, 'M', 0, 'i', 0, 'c', 0, 'r', 0, 'o', 0, 'e', 0,
		'l', 0, 'e', 0, 'c', 0, 't', 0, 'r', 0, 'o', 0, 'n', 0, 'i', 0,
		'c', 0, 's', 0
};

const uint8_t Speaker_StringProduct[SPEAKER_SIZ_STRING_PRODUCT] = {
		SPEAKER_SIZ_STRING_PRODUCT, // bLength
		USB_STRING_DESCRIPTOR_TYPE, // bDescriptorType
		// Descriptor "Wolk USB Speaker"
		'W', 0, 'o', 0, 'l', 0, 'k', 0, ' ', 0,
		'U', 0, 'S', 0, 'B', 0, ' ', 0,
		'S', 0, 'p', 0, 'e', 0, 'a', 0, 'k', 0, 'e', 0, 'r', 0
};

uint8_t Speaker_StringSerial[SPEAKER_SIZ_STRING_SERIAL] = {
		SPEAKER_SIZ_STRING_SERIAL,  // bLength
		USB_STRING_DESCRIPTOR_TYPE, // bDescriptorType
		'S', 0, 'T', 0, 'M', 0, '3', 0, '2', 0
};
