#ifndef __USBD_MSC_H
#define __USBD_MSC_H


#include  "usbd_msc_bot.h"
#include  "usbd_msc_scsi.h"
#include  "usbd_ioreq.h"


#define MSC_MAX_FS_PACKET            0x40  // Maximal packet size for FullSpeed USB
#define MSC_MAX_HS_PACKET            0x200 // Maximal packet size for HighSpeed USB

#define BOT_GET_MAX_LUN              0xFE  // Get maximal logical unit number
#define BOT_RESET                    0xFF  // Reset the BOT

#define USB_MSC_CONFIG_DESC_SIZ      32    // MSC configuration descriptor size

#define MSC_EPIN_ADDR                0x81  // MSC IN endpoint address
#define MSC_EPOUT_ADDR               0x02  // MSC OUT endpoint address


// USBD storage structure
typedef struct _USBD_STORAGE {
	int8_t (*Init)(uint8_t lun);
	int8_t (*GetCapacity)(uint8_t lun, uint32_t *block_num, uint16_t *block_size);
	int8_t (*IsReady)(uint8_t lun);
	int8_t (*IsWriteProtected)(uint8_t lun);
	int8_t (*Read)(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
	int8_t (*Write)(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
	int8_t (*GetMaxLun)(void);
	int8_t *pInquiry;
} USBD_StorageTypeDef;

// BOT state machine structure
typedef struct {
	uint32_t                 max_lun;
	uint32_t                 interface;
	uint8_t                  bot_state;
	uint8_t                  bot_status;
	uint16_t                 bot_data_length;
	uint8_t                  bot_data[MSC_MEDIA_PACKET];
	USBD_MSC_BOT_CBWTypeDef  cbw;
	USBD_MSC_BOT_CSWTypeDef  csw;
	USBD_SCSI_SenseTypeDef   scsi_sense[SENSE_LIST_DEEPTH];
	uint8_t                  scsi_sense_head;
	uint8_t                  scsi_sense_tail;
	uint16_t                 scsi_blk_size;
	uint32_t                 scsi_blk_nbr;
	uint32_t                 scsi_blk_addr;
	uint32_t                 scsi_blk_len;
} USBD_MSC_BOT_HandleTypeDef;


// USB mass storage class
extern USBD_ClassTypeDef     USBD_MSC;


uint8_t USBD_MSC_RegisterInterface(USBD_HandleTypeDef *pdev, USBD_StorageTypeDef *fops);

#endif // __USBD_MSC_H
