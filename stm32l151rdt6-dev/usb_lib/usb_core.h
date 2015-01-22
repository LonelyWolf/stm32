// Define to prevent recursive inclusion
#ifndef __USB_CORE_H
#define __USB_CORE_H


// Exported constants
#define Type_Recipient (pInformation->USBbmRequestType & (REQUEST_TYPE | RECIPIENT))
#define Usb_rLength Usb_wLength
#define Usb_rOffset Usb_wOffset
#define USBwValue   USBwValues.w
#define USBwValue0  USBwValues.bw.bb0
#define USBwValue1  USBwValues.bw.bb1
#define USBwIndex   USBwIndexs.w
#define USBwIndex0  USBwIndexs.bw.bb0
#define USBwIndex1  USBwIndexs.bw.bb1
#define USBwLength  USBwLengths.w
#define USBwLength0 USBwLengths.bw.bb0
#define USBwLength1 USBwLengths.bw.bb1


// Exported functions
uint8_t Setup0_Process(void);
uint8_t Post0_Process(void);
uint8_t Out0_Process(void);
uint8_t In0_Process(void);

RESULT Standard_SetEndPointFeature(void);
RESULT Standard_SetDeviceFeature(void);

uint8_t *Standard_GetConfiguration(uint16_t Length);
RESULT   Standard_SetConfiguration(void);
uint8_t *Standard_GetInterface(uint16_t Length);
RESULT   Standard_SetInterface(void);
uint8_t *Standard_GetDescriptorData(uint16_t Length, PONE_DESCRIPTOR pDesc);

uint8_t *Standard_GetStatus(uint16_t Length);
RESULT   Standard_ClearFeature(void);
void SetDeviceAddress(uint8_t Val);
void NOP_Process(void);


// Exported variables
extern DEVICE_PROP            Device_Property;
extern USER_STANDARD_REQUESTS User_Standard_Requests;
extern DEVICE                 Device_Table;
extern DEVICE_INFO            Device_Info;
extern __IO uint16_t          SaveRState;
extern __IO uint16_t          SaveTState;

#endif // __USB_CORE_H
