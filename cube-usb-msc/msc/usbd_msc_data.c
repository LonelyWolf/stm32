#include "usbd_msc_data.h"


// USB mass storage page 0 inquiry data
const uint8_t MSC_Page00_Inquiry_Data[] = {
		0x00,
		0x00,
		0x00,
		(LENGTH_INQUIRY_PAGE00 - 4),
		0x00,
		0x80,
		0x83
};

// USB mass storage sense 6 data
const uint8_t MSC_Mode_Sense6_data[] = {
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00
};

// USB mass storage sense 10 data
const uint8_t MSC_Mode_Sense10_data[] = {
		0x00,
		0x06,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00,
		0x00
};
