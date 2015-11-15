// Define to prevent recursive inclusion
#ifndef __USBD_CONF_H
#define __USBD_CONF_H


#include "usb_ll.h"


// USB device handle
extern USB_HandleTypeDef husb;


// USBD parameters
#define USBD_MAX_NUM_INTERFACES      1
#define USBD_MAX_NUM_CONFIGURATION   1
#define USBD_MAX_STR_DESC_SIZ        0x100
#define USBD_SUPPORT_USER_STRING     0
#define USBD_SELF_POWERED            1

// MSC packet size (bigger size -> better performance)
#define MSC_MEDIA_PACKET             2048


// Memory management macros

// For footprint reasons and since only one allocation is handled in the MSC class
// driver, the malloc/free is changed into a static allocation method
void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *p);

// MSC Class Driver Structure size
#define MAX_STATIC_ALLOC_SIZE        155

// Memory management macros
#define USBD_malloc                  (uint32_t *)USBD_static_malloc
#define USBD_free                    USBD_static_free
#define USBD_memset                  // Not used
#define USBD_memcpy                  // Not used

#endif // __USBD_CONF_H
