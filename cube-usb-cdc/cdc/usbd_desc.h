#ifndef __USBD_DESC_H
#define __USBD_DESC_H


#include "usbd_def.h"


// Exported constants

#ifndef STM32L1XX_HD
//	STM32 DevID (for STM32L Cat.1,2 devices)
#define         DEVICE_ID1          (0x1FF80050)
#define         DEVICE_ID2          (0x1FF80054)
#define         DEVICE_ID3          (0x1FF80058)
#else
// STM32 DevID (for STM32L Cat.3,4,5 devices)
#define         DEVICE_ID1          (0x1FF800D0)
#define         DEVICE_ID2          (0x1FF800D4)
#define         DEVICE_ID3          (0x1FF800D8)
#endif

#define  USB_SIZ_STRING_SERIAL       0x32
#define  USB_DEV_RELEASE             0x0001 // USB device release number (0.01)


extern USBD_DescriptorsTypeDef USBD_CDC_Descriptor;

#endif // __USBD_DESC_H
