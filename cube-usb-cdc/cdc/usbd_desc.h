// Define to prevent recursive inclusion
#ifndef __USBD_DESC_H
#define __USBD_DESC_H


#include "usbd_def.h"


// Exported constants
#define         DEVICE_ID1          (0x1FF80050)
#define         DEVICE_ID2          (0x1FF80054)
#define         DEVICE_ID3          (0x1FF80064)

#define  USB_SIZ_STRING_SERIAL       0x1A


// Exported functions
extern USBD_DescriptorsTypeDef CDC_Desc;


#endif // __USBD_DESC_H
