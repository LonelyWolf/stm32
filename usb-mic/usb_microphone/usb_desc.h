/**
  ******************************************************************************
  * @file    usb_desc.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Descriptor Header for Audio Speaker Demo
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


// Define to prevent recursive inclusion
#ifndef __USB_DESC_H
#define __USB_DESC_H


// Exported constants

// USB devID
#define USB_VID                                       0x0483 // STMicroelectronics
#define USB_PID                                       0x5730
#define USB_REV                                       0x0001 // Device release v0.01

// Structure sizes
#define USB_DESC_SIZE_DEVICE                          18
#define USB_DESC_SIZE_CONFIG                          9
#define USB_DESC_SIZE_INTERFACE                       9
#define USB_DESC_SIZE_ENDPOINT                        9
#define USB_STRING_SIZE_LANGID                        4
#define USB_STRING_SIZE_VENDOR                        10
#define USB_STRING_SIZE_PRODUCT                       40
#define USB_STRING_SIZE_SERIAL                        26
#define MIC_DESC_SIZE_CONFIG                          115

// USB Descriptor Types
#define USB_DESC_TYPE_DEVICE                          0x01
#define USB_DESC_TYPE_CONFIGURATION                   0x02
#define USB_DESC_TYPE_STRING                          0x03
#define USB_DESC_TYPE_INTERFACE                       0x04
#define USB_DESC_TYPE_ENDPOINT                        0x05
#define USB_DESC_TYPE_CS_INTERFACE                    0x24
#define USB_DESC_TYPE_CS_ENDPOINT                     0x25

// USB Interface classes
#define USB_IF_CLASS_AUDIO                     0x01
#define USB_IF_SUBCLASS_AUDIO_CONTROL          0x01
#define USB_IF_SUBCLASS_AUDIO_STREAMING        0x02
#define USB_IF_SUBCLASS_AUDIO_FEATURE          0x06


// Exported variables
extern const uint8_t USB_DeviceDescriptor[USB_DESC_SIZE_DEVICE];
extern const uint8_t USB_StringLangID[USB_STRING_SIZE_LANGID];
extern const uint8_t USB_StringVendor[USB_STRING_SIZE_VENDOR];
extern const uint8_t USB_StringProduct[USB_STRING_SIZE_PRODUCT];
extern       uint8_t USB_StringSerial[USB_STRING_SIZE_SERIAL];
extern const uint8_t Mic_ConfigDescriptor[MIC_DESC_SIZE_CONFIG];

#endif // __USB_DESC_H
