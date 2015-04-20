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

// MSC Class Config
#define MSC_MEDIA_PACKET             8192

// CDC Class Config
#define USBD_CDC_INTERVAL            1000

// DFU Class Config
#define USBD_DFU_MAX_ITF_NUM         1
#define USBD_DFU_XFERS_IZE           1024

// AUDIO Class Config
#define USBD_AUDIO_FREQ              22100

// Memory management macros

// For footprint reasons and since only one allocation is handled in the CDC class
// driver, the malloc/free is changed into a static allocation method
void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *p);

// CDC Class Driver Structure size
#define MAX_STATIC_ALLOC_SIZE        140

// Memory management macros
#define USBD_malloc                  (uint32_t *)USBD_static_malloc
#define USBD_free                    USBD_static_free
#define USBD_memset                  // Not used
#define USBD_memcpy                  // Not used

#endif // __USBD_CONF_H
