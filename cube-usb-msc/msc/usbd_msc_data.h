#ifndef __USBD_MSC_DATA_H
#define __USBD_MSC_DATA_H


#include "usbd_conf.h"


#define MODE_SENSE6_LEN           8
#define MODE_SENSE10_LEN          8
#define LENGTH_INQUIRY_PAGE00     7
#define LENGTH_FORMAT_CAPACITIES  20


extern const uint8_t MSC_Page00_Inquiry_Data[];  
extern const uint8_t MSC_Mode_Sense6_data[];
extern const uint8_t MSC_Mode_Sense10_data[] ;

#endif // __USBD_MSC_DATA_H
