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
const uint8_t USB_DeviceDescriptor[] = {
		USB_DESC_SIZE_DEVICE, // bLength
		USB_DESC_TYPE_DEVICE, // bDescriptorType: device descriptor (0x01)
		0x00, // bcdUSB: USB version 2.00 (0x0200)
		0x02,
		0x00, // bDeviceClass: interface level (0x0000)
		0x00, // bDeviceSubclass: unused
		0x00, // bDeviceProtocaol: unused
		0x40, // bMaxPacketSize: control pipe max size
		USB_VID & 0xFF, // idVendor: USB VID
		USB_VID >> 8,
		USB_PID & 0xFF, // idProduct: USB PID
		USB_PID >> 8,
		USB_REV & 0xFF, // bcdDevice: device release
		USB_REV >> 8,
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
		// "Wolk USB Microphone"
		'W', 0, 'o', 0, 'l', 0, 'k', 0, ' ', 0,
		'U', 0, 'S', 0, 'B', 0, ' ', 0,
		'M', 0, 'i', 0, 'c', 0, 'r', 0, 'o', 0, 'p', 0, 'h', 0, 'o', 0, 'n', 0, 'e', 0
};

// USB string descriptor: Serial number
uint8_t USB_StringSerial[] = {
		USB_STRING_SIZE_SERIAL, // Descriptor length
		USB_DESC_TYPE_STRING, // String descriptor
		// Dummy data for serial number
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

// USB microphone configuration descriptor
const uint8_t Mic_ConfigDescriptor[] = {
		// Configuration 1
		USB_DESC_SIZE_CONFIG, // bLength
		USB_DESC_TYPE_CONFIGURATION, // bDescriptorType
		MIC_DESC_SIZE_CONFIG & 0xFF, // wTotalLength: length of the total configuration block,
		MIC_DESC_SIZE_CONFIG >> 8,   //               including this descriptor
		0x02, // 2 interfaces
		0x01, // configuration ID
		0x00, // iConfiguration (unused)
		0x80, // bmAttributes: bus powered device
		0x7D, // 250mA maximum power consumption

		// AC (AudioControl) interface descriptor

		// Standard AC interface descriptor
		USB_DESC_SIZE_INTERFACE, // bLength
		USB_DESC_TYPE_INTERFACE, // bDescriptorType: interface
		0x00, // bInterfaceNumber: interface 0 (index of this interface)
		0x00, // bAlternateSetting: index of this alternate setting
		0x00, // bNumEndpoints: 0 endpoints
		USB_IF_CLASS_AUDIO, // bInterfaceClass: AUDIO
		USB_IF_SUBCLASS_AUDIO_CONTROL, // bInterfaceSubclass: AUDIO_CONTROL
		0x00, // bInterfaceProtocol: unused
		0x00, // iInterface: unused

		// Class-specific AC interface descriptor
		0x09, // bLength
		USB_DESC_TYPE_CS_INTERFACE, // bDescriptorType: class-specific interface
		0x01, // bDescriptorSubtype: HEADER subtype
		0x00, // bcdADC: revision of class specification 1.00 (0x0100)
		0x01,
		0x27, // wTotalLength: total size of class specific descriptors (0x0027)
		0x00,
		0x01, // bInCollection: 1 streaming interface
		0x01, // baInterfaceNr(1): AudioStreaming interface 1 belongs to this AC interface

		// Microphone input terminal descriptor
		0x0C, // bLength
		USB_DESC_TYPE_CS_INTERFACE, // bDescriptorType: class-specific interface
		0x02, // bDescriptorSubtype: INPUT_TERMINAL subtype
		0x01, // bTerminalID: ID of this terminal
		0x01, // wTerminalType: microphone (0x0201)
		0x02,
		0x00, // bAssocTerminal: no association
		0x01, // bNrChannels: single channel
		0x00, // wChannelConfig: mono (0x0000)
		0x00,
		0x00, // iChannelNames: unused
		0x00, // iTerminal: unused

	    // Microphone output terminal descriptor
		0x09, // bLength
		USB_DESC_TYPE_CS_INTERFACE, // bDescriptorType: class-specific interface
		0x03, // bDescriptorSubtype: OUTPUT_TERMINAL subtype
		0x02, // bTerminalID: ID of this terminal
		0x01, // wTerminalType: USB streaming (0x0101)
		0x01,
		0x00, // bAssocTerminal: unused
		0x01, // bSourceID: from input terminal
		0x00, // iTerminal: unused

		// Microphone audio FU (feature unit) descriptor
		0x09, // bLength
		USB_DESC_TYPE_CS_INTERFACE, // bDescriptorType: class-specific interface
		USB_IF_SUBCLASS_AUDIO_FEATURE, // bDescriptorSubtype: FEATURE_UNIT
		0x03, // bUnitID: unique ID of this unit within the audio function
		0x01, // bSourceID: ID of the terminal to which this FU connected
		0x01, // bControlSize: bmaControls are one byte size
		0x03, // bmaControls(0): controls for master channel (mute, volume)
		0x00, // bmaControls(1): controls for channel 1 (no controls)
		0x00, // iFeature: string descriptor of this FU, unused

		// AS (AudioStreaming) interface descriptor

		// Microphone standard AS interface descriptor (Alt. Set. 0)
		USB_DESC_SIZE_INTERFACE, // bLength
		USB_DESC_TYPE_INTERFACE, // bDescriptorType: interface
		0x01, // bInterfaceNumber: interface 1 (index of this interface)
		0x00, // bAlternateSetting: index of this alternate setting
		0x00, // bNumEndpoints: 0 endpoints
		USB_IF_CLASS_AUDIO, // bInterfaceClass: AUDIO
		USB_IF_SUBCLASS_AUDIO_STREAMING, // bInterfaceSubclass: AUDIO_STREAMING
		0x00, // bInterfaceProtocol: unused
		0x00, // iInterface: unused

		// Microphone standard AS interface descriptor
		USB_DESC_SIZE_INTERFACE, // bLength
		USB_DESC_TYPE_INTERFACE, // bDescriptorType: interface
		0x01, // bInterfaceNumber: index of this interface
		0x01, // bAlternateSetting: index of this alternate setting
		0x01, // bNumEndpoints: one endpoint
		USB_IF_CLASS_AUDIO, // bInterfaceClass: AUDIO
		USB_IF_SUBCLASS_AUDIO_STREAMING, // bInterfaceSubclass: AUDIO_STREAMING
		0x00, // bInterfaceProtocol: unused
		0x00, // iInterface: unused

		// Microphone class-specific AS general interface descriptor
		0x07, // bLength
		USB_DESC_TYPE_CS_INTERFACE, // bDescriptorType: class-specific interface
		0x01, // bDescriptorSubtype: GENERAL subtype
		0x02, // bTerminalLink: uint ID of the output terminal
		0x01, // bDelay: interface delay
		0x01, // wFormatTag: PCM format (0x0001)
		0x00,

		// Microphone Type-I format type descriptor
		0x11, // bLength
		USB_DESC_TYPE_CS_INTERFACE, // bDescriptorType: class-specific interface
		0x02, // bDescriptorSubtype: FORMAT_TYPE subtype
		0x01, // bFormatType: FORMAT_TYPE_I
		0x01, // bNrChannels: single channel
		0x02, // bSubFrameSize: two bytes per audio subframe
		0x10, // bBitResolution: 16 bit per sample
		0x03, // bSamFreqType: three frequencies supported
		0x40, // tSamFreq: 8000Hz (0x001F40)
		0x1F,
		0x00,
		0x22, // tSamFreq: 22050Hz (0x005622)
		0x56,
		0x00,
		0x44, // tSamFreq: 44100Hz (0x00AC44)
		0xAC,
		0x00,

		// Microphone standard endpoint descriptor
		USB_DESC_SIZE_ENDPOINT, // bLength
		USB_DESC_TYPE_ENDPOINT, // bDescriptorType: endpoint
		0x81, // bEndpointAddress: IN endpoint 1
		0x01, // bmAttributes: isochronous, not shared
		0x10, // wMaxPacketSize: 16 bytes per packet (0x0010)
		0x00,
		0x01, // bInterval: one packet per frame
		0x00, // bRefresh: unused
		0x00, // bSynchAddress: unused

		// Microphone class-specific isochronous audio data endpoint descriptor
		0x07, // bLength
		USB_DESC_TYPE_CS_ENDPOINT, // bDescriptorType: class-specific endpoint
		0x01, // bDescriptorSubtype: GENERAL subtype
		0x01, // bmAttributes: sampling control, no pitch control, no packet padding
		0x00, // bLockDelayUnits: unused
		0x00, // wLockDelay: unused (0x0000)
		0x00
};
