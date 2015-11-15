#include "usbd_msc_storage.h"


#define STORAGE_LUN_NBR                  1  
#define STORAGE_BLK_NBR                  0x10000  
#define STORAGE_BLK_SIZ                  0x200


int8_t STORAGE_Init(uint8_t lun);
int8_t STORAGE_GetCapacity(uint8_t lun, uint32_t *block_num, uint16_t *block_size);
int8_t STORAGE_IsReady(uint8_t lun);
int8_t STORAGE_IsWriteProtected(uint8_t lun);
int8_t STORAGE_Read(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
int8_t STORAGE_Write(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
int8_t STORAGE_GetMaxLun(void);


// USB Mass storage standard inquiry data
int8_t STORAGE_Inquirydata[] = {
		// LUN 0
		0x00,
		0x80,
		0x02,
		0x02,
		(STANDARD_INQUIRY_DATA_LEN - 5),
		0x00,
		0x00,
		0x00,
		'S', 'T', 'M', ' ', ' ', ' ', ' ', ' ', // Manufacturer : 8 bytes
		'P', 'r', 'o', 'd', 'u', 'c', 't', ' ', // Product      : 16 Bytes
		' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
		'0', '.', '0', '1',                     // Version      : 4 Bytes
}; 

USBD_StorageTypeDef USBD_MSC_fops = {
		STORAGE_Init,
		STORAGE_GetCapacity,
		STORAGE_IsReady,
		STORAGE_IsWriteProtected,
		STORAGE_Read,
		STORAGE_Write,
		STORAGE_GetMaxLun,
		STORAGE_Inquirydata,
};


// Initialize the storage unit (medium)
// input:
//   lun - logical unit number
// return: 0 = OK | -1 = ERROR
int8_t STORAGE_Init(uint8_t lun) {
	int8_t result = -1;
	SDResult SDRes;

	// Initialize the SDIO peripheral GPIO pins
	SD_SDIO_GPIO_Init();

	// Initialize the SD card
	SDRes = SD_Init();
	if (SDRes == SDR_Success) {
		// SD card initialized, set 4-bit bus
		if (SDCard.Type != SDCT_MMC) {
			// MMC doesn't support 4-bit bus
			if (SDCard.SCR[1] & 0x05) {
				// Set 4-bit bus width
				SD_SetBusWidth(SD_BUS_4BIT);
			}
		}
		result = 0;
	}

	return result;
}

// Return the medium capacity
// input:
//   lun - logical unit number
//   block_num - pointer to the variable with number of blocks
//   block_size - pointer to the variable with size of block
// return: 0 = OK | -1 = ERROR
int8_t STORAGE_GetCapacity(uint8_t lun, uint32_t *block_num, uint16_t *block_size) {
	*block_num  = SDCard.BlockCount;
	*block_size = SDCard.BlockSize;

	// FIXME: or every time read the card ID here?

	return 0;
}

// Check whether the medium is ready
// input:
//   lun - logical unit number
// return: 0 = OK | -1 = ERROR
int8_t STORAGE_IsReady(uint8_t lun) {
	// FIXME: how to quickly check for SD card presence?
	return 0;
}

// Check whether the medium is write protected
// input:
//   lun - logical unit number
// return: 0 = OK | -1 = ERROR
int8_t STORAGE_IsWriteProtected(uint8_t lun) {
	// FIXME: check for SD write protection or always return 0?
	return 0;
}

// Read data from the medium
// input:
//   lun - logical unit number
//   pBuf - pointer to the data buffer
//   blk_addr - logical block address
//   blk_len - blocks number
// return: 0 = OK | -1 = ERROR
int8_t STORAGE_Read(uint8_t lun, uint8_t *pBuf, uint32_t blk_addr, uint16_t blk_len) {
	uint32_t read_length = blk_len * SDCard.BlockSize;
	SDResult SDRes;
	int8_t result = -1;

	// Start block reading from the SD card
	SDRes = SD_ReadBlock_DMA(blk_addr * SDCard.BlockSize,(uint32_t *)pBuf,read_length);
	if (SDRes == SDR_Success) {
		// Wait while data transfered by DMA
		SDRes = SD_CheckRead(read_length);
		if (SDRes == SDR_Success) result = 0;
	}

	return result;
}

// Write data to the medium
// input:
//   lun - logical unit number
//   pBuf - pointer to the data buffer
//   blk_addr - logical block address
//   blk_len - blocks number
// return: 0 = OK | -1 = ERROR
int8_t STORAGE_Write(uint8_t lun, uint8_t *pBuf, uint32_t blk_addr, uint16_t blk_len) {
	uint32_t read_length = blk_len * SDCard.BlockSize;
	SDResult SDRes;
	int8_t result = -1;

	// Start block write to the SD card
	SDRes = SD_WriteBlock_DMA(blk_addr * SDCard.BlockSize,(uint32_t *)pBuf,read_length);
	if (SDRes == SDR_Success) {
		// Wait while data transfered by DMA
		SDRes = SD_CheckWrite(read_length);
		if (SDRes == SDR_Success) result = 0;
	}

	return result;
}

// Return the maximum supported LUNs
int8_t STORAGE_GetMaxLun(void) {
	return STORAGE_LUN_NBR - 1;
}
