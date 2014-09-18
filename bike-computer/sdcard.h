// Define to prevent recursive inclusion -------------------------------------
#ifndef __SDCARD_H
#define __SDCARD_H


// SDCARD SPI peripheral
#define SDCARD_SPI_PORT      SPI1

// SDCARD GPIO peripherals
#define SDCARD_PORT_PERIPH   RCC_AHBPeriph_GPIOB

// SDCARD CS (Chip Select) pin
#define SDCARD_CS_PORT       GPIOB
#define SDCARD_CS_PIN        GPIO_Pin_6    // PB6

// SDCARD CS pin macros
#define SDCARD_CS_L()        SDCARD_CS_PORT->BSRRH = SDCARD_CS_PIN
#define SDCARD_CS_H()        SDCARD_CS_PORT->BSRRL = SDCARD_CS_PIN

// SD commands  index
#define SD_CMD_GO_IDLE_STATE                       ((uint8_t)0)
#define SD_CMD_SEND_OP_COND                        ((uint8_t)1)
#define SD_CMD_ALL_SEND_CID                        ((uint8_t)2)  // Not supported in SPI mode
#define SD_CMD_SEND_REL_ADDR                       ((uint8_t)3)  // Not supported in SPI mode
#define SD_CMD_SET_DSR                             ((uint8_t)4)  // Not supported in SPI mode
#define SD_CMD_SD_CMD_SEN_OP_COND                  ((uint8_t)5)
#define SD_CMD_HS_SWITCH                           ((uint8_t)6)
#define SD_CMD_SEL_DESEL_CARD                      ((uint8_t)7)  // Not supported in SPI mode
#define SD_CMD_HS_SEND_EXT_CSD                     ((uint8_t)8)
#define SD_CMD_SEND_CSD                            ((uint8_t)9)
#define SD_CMD_SEND_CID                            ((uint8_t)10)
#define SD_CMD_READ_DAT_UNTIL_STOP                 ((uint8_t)11) // Not supported in SPI mode
#define SD_CMD_STOP_TRANSMISSION                   ((uint8_t)12)
#define SD_CMD_SEND_STATUS                         ((uint8_t)13)
#define SD_CMD_HS_BUSTEST_READ                     ((uint8_t)14)
#define SD_CMD_GO_INACTIVE_STATE                   ((uint8_t)15) // Not supported in SPI mode
#define SD_CMD_SET_BLOCKLEN                        ((uint8_t)16)
#define SD_CMD_READ_SINGLE_BLOCK                   ((uint8_t)17)
#define SD_CMD_READ_MULT_BLOCK                     ((uint8_t)18)
#define SD_CMD_HS_BUSTEST_WRITE                    ((uint8_t)19)
#define SD_CMD_WRITE_DAT_UNTIL_STOP                ((uint8_t)20) // Not supported in SPI mode
#define SD_CMD_WRITE_SINGLE_BLOCK                  ((uint8_t)24)
#define SD_CMD_WRITE_MULT_BLOCK                    ((uint8_t)25)
#define SD_CMD_PROG_CSD                            ((uint8_t)27)
#define SD_CMD_SET_WRITE_PROT                      ((uint8_t)28) // Not supported in SPI mode
#define SD_CMD_CLR_WRITE_PROT                      ((uint8_t)29) // Not supported in SPI mode
#define SD_CMD_SEND_WRITE_PROT                     ((uint8_t)30) // Not supported in SPI mode
#define SD_CMD_SD_ERASE_GRP_START                  ((uint8_t)32) // To set the address of the first write
                                                                 // block to be erased. (For SD card only)
#define SD_CMD_SD_ERASE_GRP_END                    ((uint8_t)33) // To set the address of the last write block of the
                                                                 // continuous range to be erased. (For SD card only)
#define SD_CMD_ERASE_GRP_START                     ((uint8_t)35) // To set the address of the first write block to be erased.
                                                                 // (For MMC card only spec 3.31)
#define SD_CMD_ERASE_GRP_END                       ((uint8_t)36) // To set the address of the last write block of the
                                                                 // continuous range to be erased. (For MMC card only spec 3.31)
#define SD_CMD_ERASE                               ((uint8_t)38)
#define SD_CMD_FAST_IO                             ((uint8_t)39) // Not supported in SPI mode
#define SD_CMD_GO_IRQ_STATE                        ((uint8_t)40) // Not supported in SPI mode
#define SD_CMD_LOCK_UNLOCK                         ((uint8_t)42)
#define SD_CMD_APP_CMD                             ((uint8_t)55)
#define SD_CMD_GEN_CMD                             ((uint8_t)56)
#define SD_CMD_READ_OCR                            ((uint8_t)58) // Read OCR register
#define SD_CMD_CRC_ON_OFF                          ((uint8_t)59) // On/Off CRC check by SD Card (in SPI mode)

// Following commands are SD Card Specific commands.
// SD_CMD_APP_CMD should be sent before sending these commands.
#define SD_CMD_SD_APP_OP_COND                      ((uint8_t)41) // For SD Card only

// Mask for R6 response
#define SD_CHECK_PATTERN                           ((uint32_t)0x000001AA)

// Control tokens
#define SD_TOKEN_START_BLOCK                       ((uint8_t)0xfe) // Start block
#define SD_TOKEN_DATA_ACCEPTED                     ((uint8_t)0x05) // Data accepted
#define SD_TOKEN_WRITE_CRC_ERROR                   ((uint8_t)0x0b) // Data rejected due to a CRC error
#define SD_TOKEN_WRITE_ERROR                       ((uint8_t)0x0d) // Data rejected due to a write error
#define SD_TOKEN_READ_ERROR                        ((uint8_t)0x01) // Data read error
#define SD_TOKEN_READ_CC_ERROR                     ((uint8_t)0x02) // Internal card controller error
#define SD_TOKEN_READ_ECC_ERROR                    ((uint8_t)0x04) // Card ECC failed
#define SD_TOKEN_READ_RANGE_ERROR                  ((uint8_t)0x08) // Read address out of range

// Masks for R1 response
#define SD_R1_IDLE                                 ((uint8_t)0x01) // The card is in idle state
#define SD_R1_ILLEGAL_CMD                          ((uint8_t)0x04) // Illegal command
#define SD_R1_CRC_ERROR                            ((uint8_t)0x08) // The CRC check of the last command failed
#define SD_R1_ADDR_ERROR                           ((uint8_t)0x20) // Incorrect address specified
#define SD_R1_PARAM_ERROR                          ((uint8_t)0x40) // Parameter error


// SD card response type
typedef enum {
	SD_R1                   = 0x01, // R1
	SD_R1b                  = 0x02, // R1b
	SD_R2                   = 0x03, // R2
	SD_R3                   = 0x04, // R3
	SD_R7                   = 0x05  // R7
} SDCmdResp_TypeDef;

typedef enum {
	SDCT_UNKNOWN            = 0x00,
	SDCT_SDSC_V1            = 0x01,  // Standard capacity SD card v1.0
	SDCT_SDSC_V2            = 0x02,  // Standard capacity SD card v2.0
	SDCT_MMC                = 0x03,  // MMC
	SDCT_SDHC               = 0x04   // High capacity SD card (SDHC or SDXC)
} SDCardType_TypeDef;

typedef enum {
	SDR_Success             = 0x00,
	SDR_Timeout             = 0x01,
	SDR_CRCError            = 0x02,  // Computed CRC not equal to received from SD card
	SDR_ReadError           = 0x03,  // Read block error (response for CMD17)
	SDR_WriteError          = 0x04,  // Write block error (response for CMD24)
	SDR_WriteErrorInternal  = 0x05,  // Write block error due to internal card error
	SDR_Unsupported         = 0x06,  // Unsupported card found
	SDR_BadResponse         = 0x07,
	SDR_SetBlockSizeFailed  = 0x08,  // Set block size command failed (response for CMD16)
	SDR_UnknownCard         = 0x09,
	SDR_NoResponse          = 0x0a,
	SDR_AddrError           = 0x0b,  // Address error (misaligned or out of bounds)
	SDR_WriteCRCError       = 0x0c   // Data write rejected due to a CRC error
} SDResult_TypeDef;

typedef struct {
	SDCardType_TypeDef   CardType;            // Card type (detected by SD_Init())
	uint32_t             CardCapacity;        // Card capacity (MBytes for SDHC/SDXC, bytes otherwise)
	uint32_t             CardBlockSize;       // SD card block size (bytes), determined in SD_ReadCSD()
	uint32_t             CardMaxBusClkFreq;   // Maximum card bus frequency (MHz)
	uint8_t              CSD[16];             // SD card CSD register (card structure data)
	uint8_t              CID[16];             // SD card CID register (card indentification number)
	uint8_t              CardCSDVer;          // SD card CSD register version
} SDCard_TypeDef;


extern SDCard_TypeDef SDCard;                 // SD card parameters


uint16_t CRC16_buf(const uint8_t * pBuf, uint16_t len);

SDResult_TypeDef SD_Init(void);

SDResult_TypeDef SD_ReadCSD(void);
SDResult_TypeDef SD_ReadCID(void);
SDResult_TypeDef SD_ReadBlock(uint32_t addr, uint8_t *pBuf, uint32_t len);
SDResult_TypeDef SD_WriteBlock(uint32_t addr, uint8_t *pBuf, uint32_t len);

#endif // __SDCARD_H
