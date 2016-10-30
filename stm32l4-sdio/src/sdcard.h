// Define to prevent recursive inclusion
#ifndef __SDCARD_SDIO_H
#define __SDCARD_SDIO_H


#include <stm32l4xx.h>
#include "gpio.h"


// Enable usage of SDIO 4-bit bus
//   0 - use 1-bit bus
//   1 - use 4-bit bus
#define SDIO_USE_4BIT                 1

// Enable usage of DMA transfers
//   0 - no DMA-related code
//   1 - use DMA-related code
#define SDIO_USE_DMA                  1


#if (SDIO_USE_DMA)
#include "dma.h"

// Handle for SDIO DMA channel
DMA_HandleTypeDef                     SDIO_DMA_CH;

// SDIO DMA channel parameters
#define SDIO_DMA_CHANNEL              DMA2_Channel4 // 4 or 5
#define SDIO_DMA_REQUEST              DMA_REQUEST_7

// SDIO DMA channel direction
#define SDIO_DMA_DIR_TX               DMA_DIR_M2P // SDIO write
#define SDIO_DMA_DIR_RX               DMA_DIR_P2M // SDIO read
#endif // SDIO_USE_DMA

// SDIO HAL
#define SDIO_GPIO_PERIPH              (RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_GPIODEN)
#define SDIO_GPIO_AF                  GPIO_AF12

// SDIO CMD pin (PD2)
#define SDIO_GPIO_CMD_PORT            GPIOD
#define SDIO_GPIO_CMD_PIN             GPIO_PIN_2
#define SDIO_GPIO_CMD_SRC             GPIO_PinSource2

// SDIO CK pin (PC12)
#define SDIO_GPIO_CK_PORT             GPIOC
#define SDIO_GPIO_CK_PIN              GPIO_PIN_12
#define SDIO_GPIO_CK_SRC              GPIO_PinSource12

// SDIO D0 pin (PC8)
#define SDIO_GPIO_D0_PORT             GPIOC
#define SDIO_GPIO_D0_PIN              GPIO_PIN_8
#define SDIO_GPIO_D0_SRC              GPIO_PinSource8

#if (SDIO_USE_4BIT)
// SDIO D1 pin (PC9)
#define SDIO_GPIO_D1_PORT             GPIOC
#define SDIO_GPIO_D1_PIN              GPIO_PIN_9
#define SDIO_GPIO_D1_SRC              GPIO_PinSource9

// SDIO D2 pin (PC10)
#define SDIO_GPIO_D2_PORT             GPIOC
#define SDIO_GPIO_D2_PIN              GPIO_PIN_10
#define SDIO_GPIO_D2_SRC              GPIO_PinSource10

// SDIO D3 pin (PC11)
#define SDIO_GPIO_D3_PORT             GPIOC
#define SDIO_GPIO_D3_PIN              GPIO_PIN_11
#define SDIO_GPIO_D3_SRC              GPIO_PinSource11

#endif // SDIO_USE_4BIT


#ifdef STM32L1XX_HD
// Alias word address of bits of the SDIO DCTRL register
#define SDIO_DCTRL_OFFSET             ((uint32_t)(&(SDMMC1->DCTRL)) - PERIPH_BASE)
#define SDIO_DCTRL_DTEN_BN            0 // DTEN
#define SDIO_DCTRL_DTEN_BB            *(__IO uint32_t *)(PERIPH_BB_BASE + (SDMMC_DCTRL_OFFSET << 5) + (SDMMC_DCTRL_DTEN_BN << 2))

// Alias word address of bits of the SDIO CLKCR register
#define SDIO_CLKCR_OFFSET             ((uint32_t)(&(SDMMC1->CLKCR)) - PERIPH_BASE)
#define SDIO_CLKCR_CLKEN_BN           8 // CLKEN
#define SDIO_CLKCR_CLKEN_BB           *(__IO uint32_t *)(PERIPH_BB_BASE + (SDMMC_CLKCR_OFFSET << 5) + (SDMMC_CLKCR_CLKEN_BN << 2))
#endif // STM32L1XX_HD


// SDIO power supply control bits
#define SD_PWR_OFF                    ((uint32_t)0x00000000U) // Power off: the clock to card is stopped
#define SD_PWR_ON                     (SDMMC_POWER_PWRCTRL)   // Power on: the card is clocked

// SDIO clock divider
#define SD_CLK_DIV_400K               ((uint32_t)0x00000076U) // SDIO clock 400kHz  (48MHz / (0x76 + 2) = 400kHz)
#define SD_CLK_DIV_1M                 ((uint32_t)0x0000002EU) // SDIO clock 1MHz    (48MHz / (0x2e + 2) = 1MHz)
#define SD_CLK_DIV_2M                 ((uint32_t)0x00000016U) // SDIO clock 2MHz    (48MHz / (0x16 + 2) = 2MHz)
#define SD_CLK_DIV_4M                 ((uint32_t)0x0000000AU) // SDIO clock 4MHz    (48MHz / (0x0a + 2) = 4MHz)
#define SD_CLK_DIV_6M85               ((uint32_t)0x00000005U) // SDIO clock 6.85MHz (48MHz / (0x05 + 2) = 6.85MHz)
#define SD_CLK_DIV_8M                 ((uint32_t)0x00000004U) // SDIO clock 8MHz    (48MHz / (0x04 + 2) = 8MHz)
#define SD_CLK_DIV_9M6                ((uint32_t)0x00000003U) // SDIO clock 9.6MHz  (48MHz / (0x03 + 2) = 9.6MHz)
#define SD_CLK_DIV_12M                ((uint32_t)0x00000002U) // SDIO clock 12MHz   (48MHz / (0x02 + 2) = 12MHz)
#define SD_CLK_DIV_16M                ((uint32_t)0x00000001U) // SDIO clock 16MHz   (48MHz / (0x01 + 2) = 16MHz)
#define SD_CLK_DIV_24M                ((uint32_t)0x00000000U) // SDIO clock 24MHz   (48MHz / (0x00 + 2) = 24MHz)

// SDIO clocks for initialization and data transfer
#define SD_CLK_DIV_INIT               (SD_CLK_DIV_400K) // SDIO initialization frequency (400kHz)
//#define SD_CLK_DIV_TRAN               (SD_CLK_DIV_400K) // SDIO data transfer 400kHz
//#define SD_CLK_DIV_TRAN               (SD_CLK_DIV_1M)   // SDIO data transfer 1MHz
//#define SD_CLK_DIV_TRAN               (SD_CLK_DIV_8M)   // SDIO data transfer 8MHz
//#define SD_CLK_DIV_TRAN               (SD_CLK_DIV_9M6)  // SDIO data transfer 9.6MHz
//#define SD_CLK_DIV_TRAN               (SD_CLK_DIV_12M)  // SDIO data transfer 12MHz
//#define SD_CLK_DIV_TRAN               (SD_CLK_DIV_16M)  // SDIO data transfer 16MHz
#define SD_CLK_DIV_TRAN               (SD_CLK_DIV_24M) // SDIO data transfer frequency

// SDIO CMD response type
#define SD_RESP_NONE                  ((uint32_t)0x00000000U) // No response
#define SD_RESP_SHORT                 (SDMMC_CMD_WAITRESP_0)  // Short response
#define SD_RESP_LONG                  (SDMMC_CMD_WAITRESP)    // Long response

// SD commands  index
#define SD_CMD_GO_IDLE_STATE          ((uint8_t)0)
#define SD_CMD_SEND_OP_COND           ((uint8_t)1)  // MMC only
#define SD_CMD_ALL_SEND_CID           ((uint8_t)2)  // Not supported in SPI mode
#define SD_CMD_SEND_REL_ADDR          ((uint8_t)3)  // Not supported in SPI mode
#define SD_CMD_SET_BUS_WIDTH          ((uint8_t)6)
#define SD_CMD_SEL_DESEL_CARD         ((uint8_t)7)  // Not supported in SPI mode
#define SD_CMD_HS_SEND_EXT_CSD        ((uint8_t)8)
#define SD_CMD_SEND_CSD               ((uint8_t)9)
#define SD_CMD_SEND_CID               ((uint8_t)10)
#define SD_CMD_READ_DAT_UNTIL_STOP    ((uint8_t)11) // Not supported in SPI mode
#define SD_CMD_STOP_TRANSMISSION      ((uint8_t)12)
#define SD_CMD_SEND_STATUS            ((uint8_t)13)
#define SD_CMD_GO_INACTIVE_STATE      ((uint8_t)15) // Not supported in SPI mode
#define SD_CMD_SET_BLOCKLEN           ((uint8_t)16)
#define SD_CMD_READ_SINGLE_BLOCK      ((uint8_t)17)
#define SD_CMD_READ_MULT_BLOCK        ((uint8_t)18)
#define SD_CMD_WRITE_DAT_UNTIL_STOP   ((uint8_t)20) // Not supported in SPI mode
#define SD_CMD_WRITE_BLOCK            ((uint8_t)24)
#define SD_CMD_WRITE_MULTIPLE_BLOCK   ((uint8_t)25)
#define SD_CMD_PROG_CSD               ((uint8_t)27)
#define SD_CMD_SET_WRITE_PROT         ((uint8_t)28) // Not supported in SPI mode
#define SD_CMD_CLR_WRITE_PROT         ((uint8_t)29) // Not supported in SPI mode
#define SD_CMD_SEND_WRITE_PROT        ((uint8_t)30) // Not supported in SPI mode
#define SD_CMD_ERASE                  ((uint8_t)38)
#define SD_CMD_LOCK_UNLOCK            ((uint8_t)42)
#define SD_CMD_APP_CMD                ((uint8_t)55)
#define SD_CMD_READ_OCR               ((uint8_t)58) // Read OCR register
#define SD_CMD_CRC_ON_OFF             ((uint8_t)59) // On/Off CRC check by SD Card (in SPI mode)

// Following commands are SD Card Specific commands.
// SD_CMD_APP_CMD should be sent before sending these commands.
#define SD_CMD_SET_BUS_WIDTH          ((uint8_t)6)  // ACMD6
#define SD_CMD_SD_SEND_OP_COND        ((uint8_t)41) // ACMD41
#define SD_CMD_SET_CLR_CARD_DETECT    ((uint8_t)42) // ACMD42
#define SD_CMD_SEND_SCR               ((uint8_t)51) // ACMD51

// Pattern for R6 response
#define SD_CHECK_PATTERN              ((uint32_t)0x000001AAU)

// R6 response error bits
#define SD_R6_GENERAL_UNKNOWN_ERROR   ((uint32_t)0x00002000U)
#define SD_R6_ILLEGAL_CMD             ((uint32_t)0x00004000U)
#define SD_R6_COM_CRC_FAILED          ((uint32_t)0x00008000U)

// Argument for ACMD41 to select voltage window
#define SD_OCR_VOLTAGE                ((uint32_t)0x80100000U)

// Mask for errors in card status value
#define SD_OCR_ALL_ERRORS             ((uint32_t)0xFDFFE008U) // All possible error bits
#define SD_OCR_OUT_OF_RANGE           ((uint32_t)0x80000000U) // The command's argument was out of allowed range
#define SD_OCR_ADDRESS_ERROR          ((uint32_t)0x40000000U) // A misaligned address used in the command
#define SD_OCR_BLOCK_LEN_ERROR        ((uint32_t)0x20000000U) // The transfer block length is not allowed for this card
#define SD_OCR_ERASE_SEQ_ERROR        ((uint32_t)0x10000000U) // An error in the sequence of erase commands occurred
#define SD_OCR_ERASE_PARAM            ((uint32_t)0x08000000U) // An invalid selection of write-blocks for erase occurred
#define SD_OCR_WP_VIOLATION           ((uint32_t)0x04000000U) // Attempt to write to a protected block or to the write protected card
#define SD_OCR_LOCK_UNLOCK_FAILED     ((uint32_t)0x01000000U) // Sequence or password error in lock/unlock card command
#define SD_OCR_COM_CRC_ERROR          ((uint32_t)0x00800000U) // The CRC check of the previous command failed
#define SD_OCR_ILLEGAL_COMMAND        ((uint32_t)0x00400000U) // Command not legal for the card state
#define SD_OCR_CARD_ECC_FAILED        ((uint32_t)0x00200000U) // Card internal ECC was applied but failed to correct the data
#define SD_OCR_CC_ERROR               ((uint32_t)0x00100000U) // Internal card controller error
#define SD_OCR_ERROR                  ((uint32_t)0x00080000U) // A general or an unknown error occurred during the operation
#define SD_OCR_STREAM_R_UNDERRUN      ((uint32_t)0x00040000U) // The card could not sustain data transfer in stream read operation
#define SD_OCR_STREAM_W_OVERRUN       ((uint32_t)0x00020000U) // The card could not sustain data programming in stream mode
#define SD_OCR_CSD_OVERWRITE          ((uint32_t)0x00010000U) // CSD overwrite error
#define SD_OCR_WP_ERASE_SKIP          ((uint32_t)0x00008000U) // Only partial address space was erased
#define SD_OCR_CARD_ECC_DISABLED      ((uint32_t)0x00004000U) // The command has been executed without using the internal ECC
#define SD_OCR_ERASE_RESET            ((uint32_t)0x00002000U) // An erase sequence was cleared before executing
#define SD_OCR_AKE_SEQ_ERROR          ((uint32_t)0x00000008U) // Error in the sequence of the authentication process

// Card state (OCR[12:9] bits CURRENT_STATE)
#define SD_STATE_IDLE                 ((uint8_t)0x00) // Idle
#define SD_STATE_READY                ((uint8_t)0x01) // Ready
#define SD_STATE_IDENT                ((uint8_t)0x02) // Identification
#define SD_STATE_STBY                 ((uint8_t)0x03) // Stand-by
#define SD_STATE_TRAN                 ((uint8_t)0x04) // Transfer
#define SD_STATE_DATA                 ((uint8_t)0x05) // Sending data
#define SD_STATE_RCV                  ((uint8_t)0x06) // Receive data
#define SD_STATE_PRG                  ((uint8_t)0x07) // Programming
#define SD_STATE_DIS                  ((uint8_t)0x08) // Disconnect
#define SD_STATE_ERROR                ((uint8_t)0xFF) // Error or unknown state

// Mask for ACMD41
#define SD_STD_CAPACITY               ((uint32_t)0x00000000U)
#define SD_HIGH_CAPACITY              ((uint32_t)0x40000000U)

// Timeout for CMD0 or CMD8
#define SD_CMD_TIMEOUT                ((uint32_t)0x00010000U)

// SDIO timeout for data transfer ((48MHz / CLKDIV / 1000) * timeout_ms)
#define SD_DATA_R_TIMEOUT             ((uint32_t)((48000000U / (SD_CLK_DIV_TRAN + 2U) / 1000U) * 100U)) // Data read timeout is 100ms
#define SD_DATA_W_TIMEOUT             ((uint32_t)((48000000U / (SD_CLK_DIV_TRAN + 2U) / 1000U) * 250U)) // Date write timeout is 250ms

// Trials count for ACMD41
#define SD_ACMD41_TRIALS              ((uint32_t)0x0000FFFF)

// Bitmap to clear the SDIO command flags
#define SDIO_ICR_CMD                  ((uint32_t)(SDMMC_ICR_CCRCFAILC | SDMMC_ICR_CTIMEOUTC | SDMMC_ICR_CMDRENDC | SDMMC_ICR_CMDSENTC))

// Bitmap to clear the SDIO data flags
#define SDIO_ICR_DATA                 ((uint32_t)(SDMMC_ICR_RXOVERRC | SDMMC_ICR_DCRCFAILC | SDMMC_ICR_DTIMEOUTC | SDMMC_ICR_DBCKENDC | SDMMC_ICR_STBITERRC))

// Bitmap to clear the SDIO static flags (command and data)
#define SDIO_ICR_STATIC               ((uint32_t)(SDMMC_ICR_CCRCFAILC | SDMMC_ICR_DCRCFAILC | SDMMC_ICR_CTIMEOUTC | \
												SDMMC_ICR_DTIMEOUTC | SDMMC_ICR_TXUNDERRC | SDMMC_ICR_RXOVERRC  | \
												SDMMC_ICR_CMDRENDC  | SDMMC_ICR_CMDSENTC  | SDMMC_ICR_DATAENDC  | \
												SDMMC_ICR_DBCKENDC))

// SDIO bus width
#define SD_BUS_1BIT                   ((uint32_t)0x00000000U) // 1-bit wide bus (SDIO_D0 used)
#define SD_BUS_4BIT                   (SDMMC_CLKCR_WIDBUS_0)  // 4-bit wide bus (SDIO_D[3:0] used)
#define SD_BUS_8BIT                   (SDMMC_CLKCR_WIDBUS_1)  // 8-bit wide bus (SDIO_D[7:0] used)

// SDIO transfer flags
#define SDIO_XFER_COMMON_FLAGS        (SDMMC_STA_DTIMEOUT | SDMMC_STA_DCRCFAIL | SDMMC_STA_STBITERR)

// SDIO flags for single block receive
#define SDIO_RX_SB_FLAGS              (SDIO_XFER_COMMON_FLAGS | SDMMC_STA_DBCKEND | SDMMC_STA_RXOVERR)

// SDIO flags for multiple block receive
#define SDIO_RX_MB_FLAGS              (SDIO_XFER_COMMON_FLAGS | SDMMC_STA_DATAEND | SDMMC_STA_RXOVERR)

// SDIO flags for single block transmit
#define SDIO_TX_SB_FLAGS              (SDIO_XFER_COMMON_FLAGS | SDMMC_STA_DBCKEND | SDMMC_STA_TXUNDERR)

// SDIO flags for multiple block transmit
#define SDIO_TX_MB_FLAGS              (SDIO_XFER_COMMON_FLAGS | SDMMC_STA_DATAEND | SDMMC_STA_TXUNDERR)

// SDIO transfer error flags
#define SDIO_XFER_ERROR_FLAGS         (SDIO_XFER_COMMON_FLAGS | SDMMC_STA_TXUNDERR | SDMMC_STA_RXOVERR)

// RCA for the MMC card
#define SDIO_MMC_RCA                  ((uint16_t)0x0001)


// SD card response type
enum {
	SD_R1  = 0x01, // R1
	SD_R1b = 0x02, // R1b
	SD_R2  = 0x03, // R2
	SD_R3  = 0x04, // R3
	SD_R6  = 0x05, // R6 (SDIO only)
	SD_R7  = 0x06  // R7
};

// Card type
enum {
	SDCT_UNKNOWN = 0x00,
	SDCT_SDSC_V1 = 0x01,  // Standard capacity SD card v1.0
	SDCT_SDSC_V2 = 0x02,  // Standard capacity SD card v2.0
	SDCT_MMC     = 0x03,  // MMC
	SDCT_SDHC    = 0x04   // High capacity SD card (SDHC or SDXC)
};

// SD functions result
typedef enum {
	SDR_Success             = 0x00,
	SDR_Timeout             = 0x01,  // Timeout
	SDR_CRCError            = 0x02,  // Response for command received but CRC check failed
	SDR_ReadError           = 0x03,  // Read block error (response for CMD17)
	SDR_WriteError          = 0x04,  // Write block error (response for CMD24)
	SDR_WriteErrorInternal  = 0x05,  // Write block error due to internal card error
	SDR_Unsupported         = 0x06,  // Unsupported card found
	SDR_BadResponse         = 0x07,
	SDR_SetBlockSizeFailed  = 0x08,  // Set block size command failed (response for CMD16)
	SDR_UnknownCard         = 0x09,
	SDR_NoResponse          = 0x0a,
	SDR_AddrOutOfRange      = 0x0b,  // Address out of range
	SDR_WriteCRCError       = 0x0c,  // Data write rejected due to a CRC error
	SDR_InvalidVoltage      = 0x0d,  // Unsupported voltage range
	SDR_DataTimeout         = 0x0e,  // Data block transfer timeout
	SDR_DataCRCFail         = 0x0f,  // Data block transfer CRC failed
	SDR_RXOverrun           = 0x10,  // Receive FIFO overrun
	SDR_TXUnderrun          = 0x11,  // Transmit FIFO underrun
	SDR_StartBitError       = 0x12,  // Start bit not detected on all data signals
	SDR_AddrMisaligned      = 0x13,  // A misaligned address which did not match the block length was used in the command
	SDR_BlockLenError       = 0x14,  // The transfer block length is not allowed for this card
	SDR_EraseSeqError       = 0x15,  // An error in the sequence of erase commands occurred
	SDR_EraseParam          = 0x16,  // An invalid selection of write-blocks for erase occurred
	SDR_WPViolation         = 0x17,  // Attempt to write to a protected block or to the write protected card
	SDR_LockUnlockFailed    = 0x18,  // Error in lock/unlock command
	SDR_ComCRCError         = 0x19,  // The CRC check of the previous command failed
	SDR_IllegalCommand      = 0x1a,  // Command is not legal for the the current card state
	SDR_CardECCFailed       = 0x1b,  // Card internal ECC was applied but failed to correct the data
	SDR_CCError             = 0x1c,  // Internal card controller error
	SDR_GeneralError        = 0x1d,  // A general or an unknown error occurred during the operation
	SDR_StreamUnderrun      = 0x1e,  // The card could not sustain data transfer in stream read operation
	SDR_StreamOverrun       = 0x1f,  // The card could not sustain data programming in stream mode
	SDR_CSDOverwrite        = 0x20,  // CSD overwrite error
	SDR_WPEraseSkip         = 0x21,  // Only partial address space was erased
	SDR_ECCDisabled         = 0x22,  // The command has been executed without using the internal ECC
	SDR_EraseReset          = 0x23,  // An erase sequence was cleared before executing
	SDR_AKESeqError         = 0x24,  // Error in the sequence of the authentication process
	SDR_UnknownError        = 0xFF   // Unknown error
} SDResult;

// SD card description
typedef struct {
	uint8_t     Type;            // Card type (detected by SD_Init())
	uint32_t    Capacity;        // Card capacity (MBytes for SDHC/SDXC, bytes otherwise)
	uint32_t    BlockCount;      // SD card blocks count
	uint32_t    BlockSize;       // SD card block size (bytes), determined in SD_ReadCSD()
	uint32_t    MaxBusClkFreq;   // Maximum card bus frequency (MHz)
	uint8_t     CSDVer;          // SD card CSD register version
	uint16_t    RCA;             // SD card RCA address (only for SDIO)
	uint8_t     MID;             // SD card manufacturer ID
	uint16_t    OID;             // SD card OEM/Application ID
	uint8_t     PNM[5];          // SD card product name (5-character ASCII string)
	uint8_t     PRV;             // SD card product revision (two BCD digits: '6.2' will be 01100010b)
	uint32_t    PSN;             // SD card serial number
	uint16_t    MDT;             // SD card manufacturing date
	uint8_t     CSD[16];         // SD card CSD register (card structure data)
	uint8_t     CID[16];         // SD card CID register (card identification number)
	uint8_t     SCR[8];          // SD card SCR register (SD card configuration)
} SDCard_TypeDef;


// Exported variables

// SD card parameters
extern SDCard_TypeDef SDCard;


// Public functions and macros

#ifndef __GNUC__
// Change an endianess of unsigned long integer (uint32_t)
#define SWAP_UINT32(x) (((x) >> 24) | (((x) & 0x00FF0000) >> 8) | (((x) & 0x0000FF00) << 8) | ((x) << 24))
#endif


// Function prototypes
void SD_GPIO_Init(void);
void SD_GPIO_DeInit(void);
void SD_SDIO_Init(void);
void SD_SDIO_DeInit(void);

SDResult SD_SetBlockSize(uint32_t block_size);
SDResult SD_Init(void);
SDResult SD_SetBusWidth(uint32_t BW);
void SD_SetBusClock(uint32_t clk_div);
void SD_GetCardInfo(void);
SDResult SD_HighSpeed(void);

SDResult SD_StopTransfer(void);
SDResult SD_GetCardStatus(uint8_t *pStatus);

SDResult SD_ReadBlock(uint32_t addr, uint32_t *pBuf, uint32_t len);
SDResult SD_WriteBlock(uint32_t addr, uint32_t *pBuf, uint32_t length);

#if (SDIO_USE_DMA)
void SD_Configure_DMA(uint32_t *pBuf, uint32_t length, uint8_t direction);
SDResult SD_ReadBlock_DMA(uint32_t addr, uint32_t *pBuf, uint32_t length);
SDResult SD_WriteBlock_DMA(uint32_t addr, uint32_t *pBuf, uint32_t length);
SDResult SD_CheckRead(uint32_t length);
SDResult SD_CheckWrite(uint32_t length);
#endif // SDIO_USE_DMA

#endif // __SDCARD_SDIO_H
