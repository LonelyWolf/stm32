// Define to prevent recursive inclusion
#ifndef __USBD_CDC_IF_H
#define __USBD_CDC_IF_H


#include "usbd_cdc.h"


// Periodically, the state of the buffer "UserTxBuffer" is checked
// The period depends on CDC_POLLING_INTERVAL
#define CDC_POLLING_INTERVAL             5 // in ms. The max is 65 and the min is 1


// Public variables
extern USBD_CDC_ItfTypeDef  USBD_CDC_fops;

#endif // __USBD_CDC_IF_H
