// Define to prevent recursive inclusion
#ifndef __USBD_CDC_IF_H
#define __USBD_CDC_IF_H


#include "usbd_cdc.h"


// Public variables
extern USBD_CDC_ItfTypeDef USBD_CDC_fops;


// Function prototypes
uint8_t CDC_Itf_Transmit(uint8_t* pBuf, uint16_t length);

#endif // __USBD_CDC_IF_H
