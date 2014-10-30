/**
  ******************************************************************************
  * @file    usb_prop.h
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   All processing related to Virtual COM Port Demo (Endpoint 0)
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
#ifndef __USB_PROP_H
#define __USB_PROP_H


// Exported defines
#define USBdev_GetConfiguration          NOP_Process
//#define USBdev_SetConfiguration          NOP_Process
#define USBdev_GetInterface              NOP_Process
#define USBdev_SetInterface              NOP_Process
#define USBdev_GetStatus                 NOP_Process
#define USBdev_ClearFeature              NOP_Process
#define USBdev_SetEndPointFeature        NOP_Process
#define USBdev_SetDeviceFeature          NOP_Process
//#define USBdev_SetDeviceAddress          NOP_Process

// USB audio class-specific request codes
#define SET_CUR                       0x01
#define SET_MIN                       0x02
#define SET_MAX                       0x03
#define SET_RES                       0x04
#define SET_MEM                       0x05
#define GET_CUR                       0x81
#define GET_MIN                       0x82
#define GET_MAX                       0x83
#define GET_RES                       0x84
#define GET_MEM                       0x85
#define GET_STAT                      0xFF


// Public variables
extern uint8_t MIC_MUTE;
extern int16_t MIC_VOLUME;
extern int16_t MIC_VOLUME_MIN;
extern int16_t MIC_VOLUME_MAX;
extern int16_t MIC_VOLUME_RES;


// Function prototypes
void USBdev_Init(void);
void USBdev_Reset(void);
void USBdev_SetConfiguration(void);
void USBdev_SetDeviceAddress(void);
void USBdev_Status_In(void);
void USBdev_Status_Out(void);
RESULT USBdev_Data_Setup(uint8_t);
RESULT USBdev_NoData_Setup(uint8_t);
RESULT USBdev_Get_Interface_Setting(uint8_t Interface, uint8_t AlternateSetting);
uint8_t *USBdev_GetDeviceDescriptor(uint16_t);
uint8_t *USBdev_GetConfigDescriptor(uint16_t);
uint8_t *USBdev_GetStringDescriptor(uint16_t);

uint8_t *Mic_FU_Get(uint16_t Length);
uint8_t *Mic_FU_Set(uint16_t Length);
uint8_t *Mic_EP_Get(uint16_t Length);
uint8_t *Mic_EP_Set(uint16_t Length);


#endif // __USB_PROP_H
