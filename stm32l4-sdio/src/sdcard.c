#include "sdcard.h"
#include "delay.h"


// SD card parameters
SDCard_TypeDef SDCard;


// Initialize the SDIO GPIO lines
void SD_GPIO_Init(void) {
	// Enable the SDIO corresponding GPIO peripherals
	RCC->AHB2ENR |= SDIO_GPIO_PERIPH;

	// Configure SDIO GPIO pins

	// SDIO_CMD
	GPIO_set_mode(SDIO_GPIO_CMD_PORT,GPIO_Mode_AF,GPIO_PUPD_PU,SDIO_GPIO_CMD_PIN);
	GPIO_out_cfg(SDIO_GPIO_CMD_PORT,GPIO_OT_PP,GPIO_SPD_HIGH,SDIO_GPIO_CMD_PIN);
	GPIO_af_cfg(SDIO_GPIO_CMD_PORT,SDIO_GPIO_CMD_SRC,SDIO_GPIO_AF);

	// SDIO_CK
	GPIO_set_mode(SDIO_GPIO_CK_PORT,GPIO_Mode_AF,GPIO_PUPD_PU,SDIO_GPIO_CK_PIN);
	GPIO_out_cfg(SDIO_GPIO_CK_PORT,GPIO_OT_PP,GPIO_SPD_HIGH,SDIO_GPIO_CK_PIN);
	GPIO_af_cfg(SDIO_GPIO_CK_PORT,SDIO_GPIO_CK_SRC,SDIO_GPIO_AF);

	// SDIO_D0
	GPIO_set_mode(SDIO_GPIO_D0_PORT,GPIO_Mode_AF,GPIO_PUPD_PU,SDIO_GPIO_D0_PIN);
	GPIO_out_cfg(SDIO_GPIO_D0_PORT,GPIO_OT_PP,GPIO_SPD_HIGH,SDIO_GPIO_D0_PIN);
	GPIO_af_cfg(SDIO_GPIO_D0_PORT,SDIO_GPIO_D0_SRC,SDIO_GPIO_AF);

#if (SDIO_USE_4BIT)
	// SDIO_D1
	GPIO_set_mode(SDIO_GPIO_D1_PORT,GPIO_Mode_AF,GPIO_PUPD_PU,SDIO_GPIO_D1_PIN);
	GPIO_out_cfg(SDIO_GPIO_D1_PORT,GPIO_OT_PP,GPIO_SPD_HIGH,SDIO_GPIO_D1_PIN);
	GPIO_af_cfg(SDIO_GPIO_D1_PORT,SDIO_GPIO_D1_SRC,SDIO_GPIO_AF);

	// SDIO_D2
	GPIO_set_mode(SDIO_GPIO_D2_PORT,GPIO_Mode_AF,GPIO_PUPD_PU,SDIO_GPIO_D2_PIN);
	GPIO_out_cfg(SDIO_GPIO_D2_PORT,GPIO_OT_PP,GPIO_SPD_HIGH,SDIO_GPIO_D2_PIN);
	GPIO_af_cfg(SDIO_GPIO_D2_PORT,SDIO_GPIO_D2_SRC,SDIO_GPIO_AF);

	// SDIO_D3
	GPIO_set_mode(SDIO_GPIO_D3_PORT,GPIO_Mode_AF,GPIO_PUPD_PU,SDIO_GPIO_D3_PIN);
	GPIO_out_cfg(SDIO_GPIO_D3_PORT,GPIO_OT_PP,GPIO_SPD_HIGH,SDIO_GPIO_D3_PIN);
	GPIO_af_cfg(SDIO_GPIO_D3_PORT,SDIO_GPIO_D3_SRC,SDIO_GPIO_AF);
#endif // SDIO_USE_4BIT
}

// De-initialize the SDIO GPIO lines to its default state
void SD_GPIO_DeInit(void) {
	// SDIO_CMD
	GPIO_set_mode(SDIO_GPIO_CMD_PORT,GPIO_Mode_AN,GPIO_PUPD_PU,SDIO_GPIO_CMD_PIN);
	GPIO_out_cfg(SDIO_GPIO_CMD_PORT,GPIO_OT_PP,GPIO_SPD_LOW,SDIO_GPIO_CMD_PIN);
	GPIO_af_cfg(SDIO_GPIO_CMD_PORT,SDIO_GPIO_CMD_SRC,GPIO_AF0);

	// SDIO_CK
	GPIO_set_mode(SDIO_GPIO_CK_PORT,GPIO_Mode_AN,GPIO_PUPD_PU,SDIO_GPIO_CK_PIN);
	GPIO_out_cfg(SDIO_GPIO_CK_PORT,GPIO_OT_PP,GPIO_SPD_LOW,SDIO_GPIO_CK_PIN);
	GPIO_af_cfg(SDIO_GPIO_CK_PORT,SDIO_GPIO_CK_SRC,GPIO_AF0);

	// SDIO_D0
	GPIO_set_mode(SDIO_GPIO_D0_PORT,GPIO_Mode_AN,GPIO_PUPD_PU,SDIO_GPIO_D0_PIN);
	GPIO_out_cfg(SDIO_GPIO_D0_PORT,GPIO_OT_PP,GPIO_SPD_LOW,SDIO_GPIO_D0_PIN);
	GPIO_af_cfg(SDIO_GPIO_D0_PORT,SDIO_GPIO_D0_SRC,GPIO_AF0);

#if (SDIO_USE_4BIT)
	// SDIO_D1
	GPIO_set_mode(SDIO_GPIO_D1_PORT,GPIO_Mode_AN,GPIO_PUPD_PU,SDIO_GPIO_D1_PIN);
	GPIO_out_cfg(SDIO_GPIO_D1_PORT,GPIO_OT_PP,GPIO_SPD_LOW,SDIO_GPIO_D1_PIN);
	GPIO_af_cfg(SDIO_GPIO_D1_PORT,SDIO_GPIO_D1_SRC,GPIO_AF0);

	// SDIO_D2
	GPIO_set_mode(SDIO_GPIO_D2_PORT,GPIO_Mode_AN,GPIO_PUPD_PU,SDIO_GPIO_D2_PIN);
	GPIO_out_cfg(SDIO_GPIO_D2_PORT,GPIO_OT_PP,GPIO_SPD_LOW,SDIO_GPIO_D2_PIN);
	GPIO_af_cfg(SDIO_GPIO_D2_PORT,SDIO_GPIO_D2_SRC,GPIO_AF0);

	// SDIO_D3
	GPIO_set_mode(SDIO_GPIO_D3_PORT,GPIO_Mode_AN,GPIO_PUPD_PU,SDIO_GPIO_D3_PIN);
	GPIO_out_cfg(SDIO_GPIO_D3_PORT,GPIO_OT_PP,GPIO_SPD_LOW,SDIO_GPIO_D3_PIN);
	GPIO_af_cfg(SDIO_GPIO_D3_PORT,SDIO_GPIO_D3_SRC,GPIO_AF0);
#endif // SDIO_USE_4BIT
}

// Configure the SDIO peripheral
void SD_SDIO_Init(void) {
	// Enable the SDIO peripheral
	RCC->APB2ENR |= RCC_APB2ENR_SDMMC1EN;

	// Reset the SDIO peripheral
	RCC->APB2RSTR |=  RCC_APB2RSTR_SDMMC1RST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_SDMMC1RST;

	// Configure SDIO peripheral clock:
	//   - rising edge of SDIOCLK
	//   - bus: 1-bit
	//   - power saving: enabled
	//   - SDIOCLK bypass: disabled
	//   - clock speed: 400kHz (SDMMC_CLK_DIV_INIT)
	//   - HW flow control: enabled
	//   - clock: enabled
	SDMMC1->CLKCR = SD_BUS_1BIT | SD_CLK_DIV_INIT | SDMMC_CLKCR_CLKEN | SDMMC_CLKCR_PWRSAV | SDMMC_CLKCR_HWFC_EN;
//	SDMMC1->CLKCR = SD_BUS_1BIT | SD_CLK_DIV_INIT | SDMMC_CLKCR_CLKEN | SDMMC_CLKCR_PWRSAV;
}

// De-initialize the SDIO peripheral
void SD_SDIO_DeInit(void) {
	// Disable SDIO clock
	SDMMC1->POWER = SD_PWR_OFF;

	// Disable the SDIO peripheral
	RCC->APB2ENR &= ~RCC_APB2ENR_SDMMC1EN;
}

// Send command to SD card
// input:
//   cmd - SD card command
//   arg - argument for command (32-bit)
//   resp_type - response type, one of SD_RESP_xx values
static void SD_Cmd(uint8_t cmd, uint32_t arg, uint32_t resp_type) {
	// Clear command flags
	SDMMC1->ICR = SDIO_ICR_CMD;

	// Program an argument for command
	SDMMC1->ARG = arg;

	// Program command value and response type, enable CPSM
	SDMMC1->CMD = cmd | resp_type | SDMMC_CMD_CPSMEN;
}

// Check R1 response
// input:
//   cmd - the sent command
// return: SDResult
static SDResult SD_GetR1Resp(uint8_t cmd) {
	volatile uint32_t wait = SD_CMD_TIMEOUT;
	uint32_t res = SDR_Success;
	uint32_t respR1;

	// Wait for response, error or timeout
	while (!(SDMMC1->STA & (SDMMC_STA_CCRCFAIL | SDMMC_STA_CMDREND | SDMMC_STA_CTIMEOUT)) && --wait);

	// Timeout?
	if ((SDMMC1->STA & SDMMC_STA_CTIMEOUT) && (wait == 0)) {
		SDMMC1->ICR = SDMMC_ICR_CTIMEOUTC;
		return SDR_Timeout;
	}

	// CRC fail?
	if (SDMMC1->STA & SDMMC_STA_CCRCFAIL) {
		SDMMC1->ICR = SDMMC_ICR_CCRCFAILC;
		return SDR_CRCError;
	}

	// Illegal command?
	if (SDMMC1->RESPCMD != cmd) {
		return SDR_IllegalCommand;
	}

	// Clear the static SDIO flags
	SDMMC1->ICR = SDIO_ICR_STATIC;

	// Get a R1 response and analyze it for errors
	respR1 = SDMMC1->RESP1;
	if (!(respR1 & SD_OCR_ALL_ERRORS))      return SDR_Success;
	if (respR1 & SD_OCR_OUT_OF_RANGE)       return SDR_AddrOutOfRange;
	if (respR1 & SD_OCR_ADDRESS_ERROR)      return SDR_AddrMisaligned;
	if (respR1 & SD_OCR_BLOCK_LEN_ERROR)    return SDR_BlockLenError;
	if (respR1 & SD_OCR_ERASE_SEQ_ERROR)    return SDR_EraseSeqError;
	if (respR1 & SD_OCR_ERASE_PARAM)        return SDR_EraseParam;
	if (respR1 & SD_OCR_WP_VIOLATION)       return SDR_WPViolation;
	if (respR1 & SD_OCR_LOCK_UNLOCK_FAILED) return SDR_LockUnlockFailed;
	if (respR1 & SD_OCR_COM_CRC_ERROR)      return SDR_ComCRCError;
	if (respR1 & SD_OCR_ILLEGAL_COMMAND)    return SDR_IllegalCommand;
	if (respR1 & SD_OCR_CARD_ECC_FAILED)    return SDR_CardECCFailed;
	if (respR1 & SD_OCR_CC_ERROR)           return SDR_CCError;
	if (respR1 & SD_OCR_ERROR)              return SDR_GeneralError;
	if (respR1 & SD_OCR_STREAM_R_UNDERRUN)  return SDR_StreamUnderrun;
	if (respR1 & SD_OCR_STREAM_W_OVERRUN)   return SDR_StreamOverrun;
	if (respR1 & SD_OCR_CSD_OVERWRITE)      return SDR_CSDOverwrite;
	if (respR1 & SD_OCR_WP_ERASE_SKIP)      return SDR_WPEraseSkip;
	if (respR1 & SD_OCR_CARD_ECC_DISABLED)  return SDR_ECCDisabled;
	if (respR1 & SD_OCR_ERASE_RESET)        return SDR_EraseReset;
	if (respR1 & SD_OCR_AKE_SEQ_ERROR)      return SDR_AKESeqError;

	return SDR_Success;
}

// Check R2 response
// input:
//   pBuf - pointer to the data buffer to store the R2 response
// return: SDResult value
static SDResult SD_GetR2Resp(uint32_t *pBuf) {
	volatile uint32_t wait = SD_CMD_TIMEOUT;

	// Wait for response, error or timeout
	while (!(SDMMC1->STA & (SDMMC_STA_CCRCFAIL | SDMMC_STA_CMDREND | SDMMC_STA_CTIMEOUT)) && --wait);

	// Timeout?
	if ((SDMMC1->STA & SDMMC_STA_CTIMEOUT) && (wait == 0)) {
		SDMMC1->ICR = SDMMC_ICR_CTIMEOUTC;

		return SDR_Timeout;
	}

	// CRC fail?
	if (SDMMC1->STA & SDMMC_STA_CCRCFAIL) {
		SDMMC1->ICR = SDMMC_ICR_CCRCFAILC;
		return SDR_CRCError;
	}

	// Clear the static SDIO flags
	SDMMC1->ICR = SDIO_ICR_STATIC;

	// SDMMC_RESP[1..4] registers contains the R2 response
#ifdef __GNUC__
	// Use GCC built-in intrinsics (fastest, less code) (GCC v4.3 or later)
	*pBuf++ = __builtin_bswap32(SDMMC1->RESP1);
	*pBuf++ = __builtin_bswap32(SDMMC1->RESP2);
	*pBuf++ = __builtin_bswap32(SDMMC1->RESP3);
	*pBuf   = __builtin_bswap32(SDMMC1->RESP4);
#else
	// Use ARM 'REV' instruction (fast, a bit bigger code than GCC intrinsics)
	*pBuf++ = __REV(SDMMC1->RESP1);
	*pBuf++ = __REV(SDMMC1->RESP2);
	*pBuf++ = __REV(SDMMC1->RESP3);
	*pBuf   = __REV(SDMMC1->RESP4);
/*
	// Use SHIFT, AND and OR (slower, biggest code)
	*pBuf++ = SWAP_UINT32(SDMMC1->RESP1);
	*pBuf++ = SWAP_UINT32(SDMMC1->RESP2);
	*pBuf++ = SWAP_UINT32(SDMMC1->RESP3);
	*pBuf   = SWAP_UINT32(SDMMC1->RESP4);
*/
#endif

	return SDR_Success;
}

// Check R3 response
// return: SDResult value
static SDResult SD_GetR3Resp(void) {
	volatile uint32_t wait = SD_CMD_TIMEOUT;

	// Wait for response, error or timeout
	while (!(SDMMC1->STA & (SDMMC_STA_CCRCFAIL | SDMMC_STA_CMDREND | SDMMC_STA_CTIMEOUT)) && --wait);

	// Timeout?
	if ((SDMMC1->STA & SDMMC_STA_CTIMEOUT) && (wait == 0)) {
		SDMMC1->ICR = SDMMC_ICR_CTIMEOUTC;

		return SDR_Timeout;
	}

	// Clear the static SDIO flags
	SDMMC1->ICR = SDIO_ICR_STATIC;

	return SDR_Success;
}

// Check R6 response (RCA)
// return: SDResult value
static SDResult SD_GetR6Resp(uint8_t cmd, uint16_t *pRCA) {
	volatile uint32_t wait = SD_CMD_TIMEOUT;
	uint32_t respR6;

	// Wait for response, error or timeout
	while (!(SDMMC1->STA & (SDMMC_STA_CCRCFAIL | SDMMC_STA_CMDREND | SDMMC_STA_CTIMEOUT)) && --wait);

	// Timeout?
	if ((SDMMC1->STA & SDMMC_STA_CTIMEOUT) && (wait == 0)) {
		SDMMC1->ICR = SDMMC_ICR_CTIMEOUTC;

		return SDR_Timeout;
	}

	// CRC fail?
	if (SDMMC1->STA & SDMMC_STA_CCRCFAIL) {
		SDMMC1->ICR = SDMMC_ICR_CCRCFAILC;
		return SDR_CRCError;
	}

	// Illegal command?
	if (SDMMC1->RESPCMD != cmd) {
		return SDR_IllegalCommand;
	}

	// Clear the static SDIO flags
	SDMMC1->ICR = SDIO_ICR_STATIC;

	// Get a R6 response and analyze it for errors
	respR6 = SDMMC1->RESP1;
	if (!(respR6 & (SD_R6_ILLEGAL_CMD | SD_R6_GENERAL_UNKNOWN_ERROR | SD_R6_COM_CRC_FAILED))) {
		*pRCA = (uint16_t)(respR6 >> 16);
		return SDR_Success;
	}
	if (respR6 & SD_R6_GENERAL_UNKNOWN_ERROR) return SDR_UnknownError;
	if (respR6 & SD_R6_ILLEGAL_CMD)           return SDR_IllegalCommand;
	if (respR6 & SD_R6_COM_CRC_FAILED)        return SDR_ComCRCError;

	return SDR_Success;
}

// Check R7 response
// return: SDResult value
static SDResult SD_GetR7Resp(void) {
	volatile uint32_t wait = SD_CMD_TIMEOUT;

	// Wait for response, error or timeout
	while (!(SDMMC1->STA & (SDMMC_STA_CCRCFAIL | SDMMC_STA_CMDREND | SDMMC_STA_CTIMEOUT)) && --wait);

	// Timeout?
	if ((SDMMC1->STA & SDMMC_STA_CTIMEOUT) || (wait == 0)) {
		SDMMC1->ICR = SDMMC_ICR_CTIMEOUTC;

		return SDR_Timeout;
	}

	// Clear command response received flag
	if (SDMMC1->STA & SDMMC_STA_CMDREND) {
		SDMMC1->ICR = SDMMC_ICR_CMDRENDC;

		return SDR_Success;
	}

	return SDR_NoResponse;
}

// Retrieve the SD card SCR register value
// input:
//   pSCR - pointer to the buffer for SCR register (8 bytes)
// return: SDResult value
// note: card must be in transfer mode, not supported by MMC
static SDResult SD_GetSCR(uint32_t *pSCR) {
	SDResult cmd_res;

	// Set block size to 8 bytes
	SD_Cmd(SD_CMD_SET_BLOCKLEN, 8, SD_RESP_SHORT); // CMD16
	cmd_res = SD_GetR1Resp(SD_CMD_SET_BLOCKLEN);
	if (cmd_res != SDR_Success) {
		return cmd_res;
	}

	// Send leading command for ACMD<n> command
	SD_Cmd(SD_CMD_APP_CMD, SDCard.RCA << 16, SD_RESP_SHORT); // CMD55
	cmd_res = SD_GetR1Resp(SD_CMD_APP_CMD);
	if (cmd_res != SDR_Success) {
		return cmd_res;
	}

	// Clear the data flags
	SDMMC1->ICR = SDIO_ICR_DATA;

	// Configure the SDIO data transfer
	SDMMC1->DTIMER = SD_DATA_R_TIMEOUT; // Data read timeout
	SDMMC1->DLEN   = 8; // Data length in bytes
	// Data transfer:
	//   - type: block
	//   - direction: card -> controller
	//   - size: 2^3 = 8bytes
	//   - DPSM: enabled
	SDMMC1->DCTRL  = SDMMC_DCTRL_DTDIR | (3 << 4) | SDMMC_DCTRL_DTEN;

	// Send SEND_SCR command
	SD_Cmd(SD_CMD_SEND_SCR, 0, SD_RESP_SHORT); // ACMD51
	cmd_res = SD_GetR1Resp(SD_CMD_SEND_SCR);
	if (cmd_res != SDR_Success) {
		return cmd_res;
	}

	// Receive the SCR register value
	while (!(SDMMC1->STA & (SDMMC_STA_RXOVERR | SDMMC_STA_DCRCFAIL | SDMMC_STA_DTIMEOUT | SDMMC_STA_DBCKEND | SDMMC_STA_STBITERR))) {
		// Read word when data available in receive FIFO
		if (SDMMC1->STA & SDMMC_STA_RXDAVL) *pSCR++ = SDMMC1->FIFO;
	}

	// Check for errors
	if (SDMMC1->STA & (SDMMC_STA_DTIMEOUT | SDMMC_STA_DCRCFAIL | SDMMC_STA_RXOVERR | SDMMC_STA_STBITERR)) {
		if (SDMMC1->STA & SDMMC_STA_DTIMEOUT) cmd_res = SDR_DataTimeout;
		if (SDMMC1->STA & SDMMC_STA_DCRCFAIL) cmd_res = SDR_DataCRCFail;
		if (SDMMC1->STA & SDMMC_STA_RXOVERR)  cmd_res = SDR_RXOverrun;
		if (SDMMC1->STA & SDMMC_STA_STBITERR) cmd_res = SDR_StartBitError;
	}

	// Clear the static SDIO flags
	SDMMC1->ICR = SDIO_ICR_STATIC;

	return cmd_res;
}

// Set block size of the SD card
// input:
//   block_size - block length
// return: SDResult value
SDResult SD_SetBlockSize(uint32_t block_size) {
	// Send SET_BLOCKLEN command
	SD_Cmd(SD_CMD_SET_BLOCKLEN, block_size, SD_RESP_SHORT); // CMD16

	return SD_GetR1Resp(SD_CMD_SET_BLOCKLEN);
}

// Initialize the SD card
// return: SDResult value
// note: SDIO peripheral clock must be on and SDIO GPIO configured
SDResult SD_Init(void) {
	volatile uint32_t wait;
	uint32_t response[4];
	uint32_t sd_type = SD_STD_CAPACITY; // SD card capacity
	SDResult cmd_res;

	// Populate SDCard structure with default values
	SDCard = (SDCard_TypeDef){ 0 };
	SDCard.Type = SDCT_UNKNOWN;

	// Enable the SDIO clock
	SDMMC1->POWER = SD_PWR_ON;

	// CMD0
	wait = SD_CMD_TIMEOUT;
	SD_Cmd(SD_CMD_GO_IDLE_STATE, 0x00, SD_RESP_NONE);
	while (!(SDMMC1->STA & (SDMMC_STA_CTIMEOUT | SDMMC_STA_CMDSENT)) && --wait);
	if ((SDMMC1->STA & SDMMC_STA_CTIMEOUT) || !wait) return SDR_Timeout;

	// CMD8: SEND_IF_COND. Send this command to verify SD card interface operating condition
	// Argument: - [31:12]: Reserved (shall be set to '0')
	//           - [11:08]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
	//           - [07:00]: Check Pattern (recommended 0xAA)
	SD_Cmd(SD_CMD_HS_SEND_EXT_CSD, SD_CHECK_PATTERN, SD_RESP_SHORT); // CMD8
	cmd_res = SD_GetR7Resp();
	if (cmd_res == SDR_Success) {
		// SD v2.0 or later

		// Check echo-back of check pattern
		if ((SDMMC1->RESP1 & 0x01FF) != (SD_CHECK_PATTERN & 0x01FF)) {
			return SDR_Unsupported;
		}
		sd_type = SD_HIGH_CAPACITY; // SD v2.0 or later

		// Issue ACMD41 command
		wait = SD_ACMD41_TRIALS;
		while (--wait) {
			// Send leading command for ACMD<n> command
			SD_Cmd(SD_CMD_APP_CMD, 0, SD_RESP_SHORT); // CMD55 with RCA 0
			cmd_res = SD_GetR1Resp(SD_CMD_APP_CMD);
			if (cmd_res != SDR_Success) {
				return cmd_res;
			}
			// ACMD41 - initiate initialization process.
			// Set 3.0-3.3V voltage window (bit 20)
			// Set HCS bit (30) (Host Capacity Support) to inform card what host support high capacity
			// Set XPC bit (28) (SDXC Power Control) to use maximum performance (SDXC only)
			SD_Cmd(SD_CMD_SD_SEND_OP_COND,SD_OCR_VOLTAGE | sd_type,SD_RESP_SHORT);
			cmd_res = SD_GetR3Resp();
			if (cmd_res != SDR_Success) {
				return cmd_res;
			}
			if (SDMMC1->RESP1 & (1 << 31)) {
				// The SD card has finished the power-up sequence
				break;
			}
		}
		if (wait == 0) {
			// Unsupported voltage range
			return SDR_InvalidVoltage;
		}

		// This is SDHC/SDXC card?
		SDCard.Type = (SDMMC1->RESP1 & SD_HIGH_CAPACITY) ? SDCT_SDHC : SDCT_SDSC_V2;
	} else if (cmd_res == SDR_Timeout) {
		// SD v1.x or MMC

		// Issue CMD55 to reset 'Illegal command' bit of the SD card
		SD_Cmd(SD_CMD_APP_CMD, 0, SD_RESP_SHORT); // CMD55 with RCA 0
		SD_GetR1Resp(SD_CMD_APP_CMD);

		// Issue ACMD41 command with zero argument
		wait = SD_ACMD41_TRIALS;
		while (--wait) {
			// Send leading command for ACMD<n> command
			SD_Cmd(SD_CMD_APP_CMD, 0, SD_RESP_SHORT); // CMD55 with RCA 0
			cmd_res = SD_GetR1Resp(SD_CMD_APP_CMD);
			if (cmd_res != SDR_Success) {
				return cmd_res;
			}

			// Send ACMD41 - initiate initialization process (bit HCS = 0)
			SD_Cmd(SD_CMD_SD_SEND_OP_COND, SD_OCR_VOLTAGE, SD_RESP_SHORT); // ACMD41
			cmd_res = SD_GetR3Resp();
			if (cmd_res == SDR_Timeout) {
				// MMC will not respond to this command
				break;
			}
			if (cmd_res != SDR_Success) {
				return cmd_res;
			}
			if (SDMMC1->RESP1 & (1 << 31)) {
				// The SD card has finished the power-up sequence
				break;
			}
		}
		if (wait == 0) {
			// Unknown/Unsupported card type
			return SDR_UnknownCard;
		}
		if (cmd_res != SDR_Timeout) {
			// SD v1.x
			SDCard.Type = SDCT_SDSC_V1; // SDv1
		} else {
			// MMC or not SD memory card

			////////////////////////////////////////////////////////////////
			// This part has not been tested due to lack of MMCmicro card //
			////////////////////////////////////////////////////////////////

			wait = SD_ACMD41_TRIALS;
			while (--wait) {
				// Issue CMD1: initiate initialization process.
				SD_Cmd(SD_CMD_SEND_OP_COND, SD_OCR_VOLTAGE, SD_RESP_SHORT); // CMD1
				cmd_res = SD_GetR3Resp();
				if (cmd_res != SDR_Success) {
					return cmd_res;
				}
				if (SDMMC1->RESP1 & (1 << 31)) {
					// The SD card has finished the power-up sequence
					break;
				}
			}
			if (wait == 0) return SDR_UnknownCard;
			SDCard.Type = SDCT_MMC; // MMC
		}
	} else {
		return cmd_res;
	}

	// Now the CMD2 and CMD3 commands should be issued in cycle until timeout to enumerate all cards on the bus
	// Since this module suitable to work with single card, issue this commands one time only

	// Send ALL_SEND_CID command
	SD_Cmd(SD_CMD_ALL_SEND_CID, 0, SD_RESP_LONG); // CMD2
	cmd_res = SD_GetR2Resp((uint32_t *)SDCard.CID); // The response will be CID/CSD register value
	if (cmd_res != SDR_Success) {
		return cmd_res;
	}

	if (SDCard.Type != SDCT_MMC) {
		// Send SEND_REL_ADDR command to ask the SD card to publish a new RCA (Relative Card Address)
		// Once the RCA is received the card state changes to the stand-by state
		SD_Cmd(SD_CMD_SEND_REL_ADDR, 0, SD_RESP_SHORT); // CMD3
		cmd_res = SD_GetR6Resp(SD_CMD_SEND_REL_ADDR, (uint16_t *)(&SDCard.RCA));
		if (cmd_res != SDR_Success) {
			return cmd_res;
		}
	} else {
		////////////////////////////////////////////////////////////////
		// This part has not been tested due to lack of MMCmicro card //
		////////////////////////////////////////////////////////////////

		// For MMC card host should set a RCA value to the card by SET_REL_ADDR command
		SD_Cmd(SD_CMD_SEND_REL_ADDR, SDIO_MMC_RCA << 16, SD_RESP_SHORT); // CMD3
		cmd_res = SD_GetR1Resp(SD_CMD_SEND_REL_ADDR);
		if (cmd_res != SDR_Success) {
			return cmd_res;
		}
		SDCard.RCA = SDIO_MMC_RCA;
	}

	// Send SEND_CSD command to retrieve CSD register from the card
	SD_Cmd(SD_CMD_SEND_CSD, SDCard.RCA << 16, SD_RESP_LONG); // CMD9
	cmd_res = SD_GetR2Resp((uint32_t *)SDCard.CSD);
	if (cmd_res != SDR_Success) {
		return cmd_res;
	}

	// Parse the values of CID and CSD registers
	SD_GetCardInfo();

	// Now card must be in stand-by mode, from this point it is possible to increase bus speed
	SD_SetBusClock(SD_CLK_DIV_TRAN);

	// Put the SD card to the transfer mode
	SD_Cmd(SD_CMD_SEL_DESEL_CARD, SDCard.RCA << 16, SD_RESP_SHORT); // CMD7
	cmd_res = SD_GetR1Resp(SD_CMD_SEL_DESEL_CARD); // In fact R1b response here
	if (cmd_res != SDR_Success) {
		return cmd_res;
	}

	// Disable the pull-up resistor on CD/DAT3 pin of card
	// Send leading command for ACMD<n> command
	SD_Cmd(SD_CMD_APP_CMD, SDCard.RCA << 16, SD_RESP_SHORT); // CMD55
	cmd_res = SD_GetR1Resp(SD_CMD_APP_CMD);
	if (cmd_res != SDR_Success) {
		return cmd_res;
	}
	// Send SET_CLR_CARD_DETECT command
	SD_Cmd(SD_CMD_SET_CLR_CARD_DETECT, 0, SD_RESP_SHORT); // ACMD42
	cmd_res = SD_GetR1Resp(SD_CMD_SET_CLR_CARD_DETECT);
	if (cmd_res != SDR_Success) {
		return cmd_res;
	}

	// Read the SCR register
	if (SDCard.Type != SDCT_MMC) {
		// MMC card doesn't support this feature
		// Warning: this function set block size to 8 bytes
		SD_GetSCR((uint32_t *)SDCard.SCR);
	}

	// For SDv1,SDv2 and MMC card must set block size
	// The SDHC/SDXC always have fixed block size (512 bytes)
	if ((SDCard.Type == SDCT_SDSC_V1) || (SDCard.Type == SDCT_SDSC_V2) || (SDCard.Type == SDCT_MMC)) {
		SD_Cmd(SD_CMD_SET_BLOCKLEN, 512, SD_RESP_SHORT); // CMD16
		cmd_res = SD_GetR1Resp(SD_CMD_SET_BLOCKLEN);
		if (cmd_res != SDR_Success) {
			return SDR_SetBlockSizeFailed;
		}
	}

	return SDR_Success;
}

// Set SDIO bus width
// input:
//   BW - bus width (one of SDIO_BUS_xBIT constants)
// return: SDResult
// note: card must be in TRAN state and not locked, otherwise it will respond with 'illegal command'
SDResult SD_SetBusWidth(uint32_t BW) {
	SDResult cmd_res = SDR_Success;
	uint32_t clk;

	if (SDCard.Type != SDCT_MMC) {
		// Send leading command for ACMD<n> command
		SD_Cmd(SD_CMD_APP_CMD, SDCard.RCA << 16, SD_RESP_SHORT); // CMD55
		cmd_res = SD_GetR1Resp(SD_CMD_APP_CMD);
		if (cmd_res != SDR_Success) {
			return cmd_res;
		}

		// Send SET_BUS_WIDTH command
		SD_Cmd(SD_CMD_SET_BUS_WIDTH, (BW == SD_BUS_1BIT) ? 0x00000000 : 0x00000002, SD_RESP_SHORT); // ACMD6
		cmd_res = SD_GetR1Resp(SD_CMD_SET_BUS_WIDTH);
		if (cmd_res != SDR_Success) {
			return cmd_res;
		}
	} else {
		// MMC supports only 8-bit ?
	}

	// Configure new bus width
	clk  = SDMMC1->CLKCR;
	clk &= ~SDMMC_CLKCR_WIDBUS;
	clk |= (BW & SDMMC_CLKCR_WIDBUS);
	SDMMC1->CLKCR = clk;

	return cmd_res;
}

// Set SDIO bus clock
// input:
//   clk_div - bus clock divider (0x00..0xff -> bus_clock = SDIOCLK / (clk_div + 2))
void SD_SetBusClock(uint32_t clk_div) {
	uint32_t clk;

	clk  = SDMMC1->CLKCR;
	clk &= ~SDMMC_CLKCR_CLKDIV;
	clk |= (clk_div & SDMMC_CLKCR_CLKDIV);
	SDMMC1->CLKCR = clk;
}

// Parse information about specific card
// note: CSD/CID register values already must be in the SDCard structure
void SD_GetCardInfo(void) {
	uint32_t dev_size, dev_size_mul, rd_block_len;

	// Parse the CSD register
	SDCard.CSDVer = SDCard.CSD[0] >> 6; // CSD version
	if (SDCard.Type != SDCT_MMC) {
		// SD
		SDCard.MaxBusClkFreq = SDCard.CSD[3];
		rd_block_len = SDCard.CSD[5] & 0x0f; // Max. read data block length
		if (SDCard.CSDVer == 0) {
			// CSD v1.00 (SDSCv1, SDSCv2)
			dev_size  = (uint32_t)(SDCard.CSD[6] & 0x03) << 10; // Device size
			dev_size |= (uint32_t)SDCard.CSD[7] << 2;
			dev_size |= (SDCard.CSD[8] & 0xc0) >> 6;
			dev_size_mul  = (SDCard.CSD[ 9] & 0x03) << 1; // Device size multiplier
			dev_size_mul |= (SDCard.CSD[10] & 0x80) >> 7;
			SDCard.BlockCount  = (dev_size + 1);
			SDCard.BlockCount *= (1 << (dev_size_mul + 2));
			SDCard.BlockSize =  1 << (rd_block_len);
		} else {
			// CSD v2.00 (SDHC, SDXC)
			dev_size  = (SDCard.CSD[7] & 0x3f) << 16;
			dev_size |=  SDCard.CSD[8] << 8;
			dev_size |=  SDCard.CSD[9];
			SDCard.BlockSize = 512;
			SDCard.BlockCount = dev_size + 1;
		}
		SDCard.Capacity = SDCard.BlockCount * SDCard.BlockSize;
	} else {
		// MMC
		SDCard.MaxBusClkFreq = SDCard.CSD[3];
		dev_size  = (uint32_t)(SDCard.CSD[6] & 0x03) << 8; // C_SIZE
		dev_size += (uint32_t)SDCard.CSD[7];
		dev_size <<= 2;
		dev_size += SDCard.CSD[8] >> 6;
		SDCard.BlockSize = 1 << (SDCard.CSD[5] & 0x0f); // MMC read block length
		dev_size_mul = ((SDCard.CSD[9] & 0x03) << 1) + ((SDCard.CSD[10] & 0x80) >> 7);
		SDCard.BlockCount = (dev_size + 1) * (1 << (dev_size_mul + 2));
		SDCard.Capacity = SDCard.BlockCount * SDCard.BlockSize;
	}

	// Parse the CID register
	if (SDCard.Type != SDCT_MMC) {
		// SD card
		SDCard.MID = SDCard.CID[0];
		SDCard.OID = (SDCard.CID[1] << 8) | SDCard.CID[2];
		SDCard.PNM[0] = SDCard.CID[3];
		SDCard.PNM[1] = SDCard.CID[4];
		SDCard.PNM[2] = SDCard.CID[5];
		SDCard.PNM[3] = SDCard.CID[6];
		SDCard.PNM[4] = SDCard.CID[7];
		SDCard.PRV = SDCard.CID[8];
		SDCard.PSN = (SDCard.CID[9] << 24) | (SDCard.CID[10] << 16) | (SDCard.CID[11] << 8) | SDCard.CID[12];
		SDCard.MDT = ((SDCard.CID[13] << 8) | SDCard.CID[14]) & 0x0fff;
	} else {
		// MMC
		SDCard.MID = 0x00;
		SDCard.OID = 0x0000;
		SDCard.PNM[0] = '*';
		SDCard.PNM[1] = 'M';
		SDCard.PNM[2] = 'M';
		SDCard.PNM[3] = 'C';
		SDCard.PNM[4] = '*';
		SDCard.PRV = 0;
		SDCard.PSN = 0x00000000;
		SDCard.MDT = 0x0000;
	}
}

SDResult SD_HighSpeed(void) {
	SDResult cmd_res = SDR_Success;
	uint8_t CCCR[64];
	uint32_t *pCCCR = (uint32_t *)CCCR;
	uint32_t response;
	uint32_t i;
	register uint32_t STA;

USART_SendStr(USART1,">>>>> HS: Check HS");

	// Set block size to 64 bytes
	SD_Cmd(SD_CMD_SET_BLOCKLEN, 64, SD_RESP_SHORT); // CMD16
	cmd_res = SD_GetR1Resp(SD_CMD_SET_BLOCKLEN);
	if (cmd_res != SDR_Success) {
		return cmd_res;
	}

	// Clear the data flags
	SDMMC1->ICR = SDIO_ICR_DATA;

	// Data read timeout
	SDMMC1->DTIMER = SD_DATA_R_TIMEOUT;
	// Data length in bytes
	SDMMC1->DLEN   = 64;
	// Data transfer:
	//   transfer mode: block
	//   direction: to card
	//   DMA: enabled
	//   block size: 2^6 = 64 bytes
	//   DPSM: enabled
	SDMMC1->DCTRL  = SDMMC_DCTRL_DTDIR | (6 << 4) | SDMMC_DCTRL_DTEN;

	// Send SD_CMD_HS_SWITCH command
//	SD_Cmd(SD_CMD_SET_BUS_WIDTH,0x00000001,SD_RESP_SHORT); // CMD6
//	SD_Cmd(SD_CMD_SET_BUS_WIDTH,0x80000001,SD_RESP_SHORT); // CMD6
	SD_Cmd(SD_CMD_SET_BUS_WIDTH,0x80FFFFF1,SD_RESP_SHORT); // CMD6
	cmd_res = SD_GetR1Resp(SD_CMD_SET_BUS_WIDTH);
	if (cmd_res != SDR_Success) {
		return cmd_res;
	}

	// Read the CCCR register value
	while (!(SDMMC1->STA & (SDMMC_STA_RXOVERR | SDMMC_STA_DCRCFAIL | SDMMC_STA_DTIMEOUT | SDMMC_STA_DBCKEND | SDMMC_STA_STBITERR))) {
		// Read word when data available in receive FIFO
		if (SDMMC1->STA & SDMMC_STA_RXFIFOHF) {
			*pCCCR++ = SDMMC1->FIFO;
			*pCCCR++ = SDMMC1->FIFO;
			*pCCCR++ = SDMMC1->FIFO;
			*pCCCR++ = SDMMC1->FIFO;
			*pCCCR++ = SDMMC1->FIFO;
			*pCCCR++ = SDMMC1->FIFO;
			*pCCCR++ = SDMMC1->FIFO;
			*pCCCR++ = SDMMC1->FIFO;
		}
	}

	// Check for errors
	if (STA & SDIO_XFER_ERROR_FLAGS) {
		if (SDMMC1->STA & SDMMC_STA_DTIMEOUT) cmd_res = SDR_DataTimeout;
		if (SDMMC1->STA & SDMMC_STA_DCRCFAIL) cmd_res = SDR_DataCRCFail;
		if (SDMMC1->STA & SDMMC_STA_RXOVERR)  cmd_res = SDR_RXOverrun;
		if (SDMMC1->STA & SDMMC_STA_STBITERR) cmd_res = SDR_StartBitError;
	}

	// Read the data remnant from the SDIO FIFO (if any present)
	while (SDMMC1->STA & SDMMC_STA_RXDAVL) {
		*pCCCR++ = SDMMC1->FIFO;
	}

	// Clear the static SDIO flags
	SDMMC1->ICR = SDIO_ICR_STATIC;

	if (cmd_res == SDR_Success) {
		for (i = 0; i < 64; i++) {
			if ((i % 8) == 0) USART_printf(USART1,"\r\n");
			USART_printf(USART1,"%02u=%02X ",i,CCCR[i]);
		}
		USART_printf(USART1,"\r\n");
		USART_printf(USART1,"SHS: %s\r\n",(CCCR[63 - (400 / 8)] & 0x01) ? "YES" : "NO");
		USART_printf(USART1,"BSS: %03b\r\n",(CCCR[63 - (400 / 8)] & 0x0E) >> 1);
	} else {
		USART_printf(USART1,"GetCCCR error: %02X\r\n",cmd_res);
	}

USART_SendStr(USART1,"<<<< HS: End of check\r\n");

	return SDR_Success;
}

// Abort an ongoing data transfer
// return: SDResult value
SDResult SD_StopTransfer(void) {
	// Send STOP_TRANSMISSION command
	SD_Cmd(SD_CMD_STOP_TRANSMISSION, 0, SD_RESP_SHORT); // CMD12

	return SD_GetR1Resp(SD_CMD_STOP_TRANSMISSION);
}

// Get current SD card state
// input:
//   pState - pointer to the variable for current card state, one of SD_STATE_xx values
// return: SDResult value
SDResult SD_GetCardState(uint8_t *pState) {
	uint8_t cmd_res;

	// Send SEND_STATUS command
	SD_Cmd(SD_CMD_SEND_STATUS, SDCard.RCA << 16, SD_RESP_SHORT); // CMD13
	cmd_res = SD_GetR1Resp(SD_CMD_SEND_STATUS);
	if (cmd_res != SDR_Success) {
		*pState = SD_STATE_ERROR;
		return cmd_res;
	}

	// Find out a card status
	*pState = (SDMMC1->RESP1 & 0x1e00) >> 9;

	// Check for errors
	return SDR_Success;
}

// Read block of data from the SD card
// input:
//   addr - address of the block to be read
//   pBuf - pointer to the buffer that will contain the received data
//   length - buffer length (must be multiple of 512)
// return: SDResult value
SDResult SD_ReadBlock(uint32_t addr, uint32_t *pBuf, uint32_t length) {
	SDResult cmd_res = SDR_Success;
	uint32_t blk_count = length >> 9; // Sectors in block
	register uint32_t STA; // to speed up SDIO flags checking
	register uint32_t STA_mask; // mask for SDIO flags checking

	// Initialize the data control register
	SDMMC1->DCTRL = 0;

	// SDSC card uses byte unit address and
	// SDHC/SDXC cards use block unit address (1 unit = 512 bytes)
	// For SDHC card addr must be converted to block unit address
	if (SDCard.Type == SDCT_SDHC) addr >>= 9;

	// Clear the static SDIO flags
	SDMMC1->ICR = SDIO_ICR_STATIC;

	if (blk_count > 1) {
		// Prepare bit checking variable for multiple block transfer
		STA_mask = SDIO_RX_MB_FLAGS;
		// Send READ_MULT_BLOCK command
		SD_Cmd(SD_CMD_READ_MULT_BLOCK, addr, SD_RESP_SHORT); // CMD18
		cmd_res = SD_GetR1Resp(SD_CMD_READ_MULT_BLOCK);
	} else {
		// Prepare bit checking variable for single block transfer
		STA_mask = SDIO_RX_SB_FLAGS;
		// Send READ_SINGLE_BLOCK command
		SD_Cmd(SD_CMD_READ_SINGLE_BLOCK, addr, SD_RESP_SHORT); // CMD17
		cmd_res = SD_GetR1Resp(SD_CMD_READ_SINGLE_BLOCK);
	}
	if (cmd_res != SDR_Success) {
		return cmd_res;
	}

	// Data read timeout
	SDMMC1->DTIMER = SD_DATA_R_TIMEOUT;
	// Data length
	SDMMC1->DLEN   = length;
	// Data transfer:
	//   transfer mode: block
	//   direction: to card
	//   DMA: disabled
	//   block size: 2^9 = 512 bytes
	//   DPSM: enabled
	SDMMC1->DCTRL  = SDMMC_DCTRL_DTDIR | (9 << 4) | SDMMC_DCTRL_DTEN;

	// Receive a data block from the SDIO
	// ----> TIME CRITICAL SECTION BEGIN <----
	do {
		STA = SDMMC1->STA;
		if (STA & SDMMC_STA_RXFIFOHF) {
			// Receive FIFO half full, there are at least 8 words in it
			// This code is 80 bytes more than the 'for' loop, but faster
			*pBuf++ = SDMMC1->FIFO;
			*pBuf++ = SDMMC1->FIFO;
			*pBuf++ = SDMMC1->FIFO;
			*pBuf++ = SDMMC1->FIFO;
			*pBuf++ = SDMMC1->FIFO;
			*pBuf++ = SDMMC1->FIFO;
			*pBuf++ = SDMMC1->FIFO;
			*pBuf++ = SDMMC1->FIFO;
		}
	} while (!(STA & STA_mask));
	// <---- TIME CRITICAL SECTION END ---->

	// Send stop transmission command in case of multiple block transfer
	if ((SDCard.Type != SDCT_MMC) && (blk_count > 1)) {
		cmd_res = SD_StopTransfer();
	}

	// Check for errors
	if (STA & SDIO_XFER_ERROR_FLAGS) {
		if (STA & SDMMC_STA_DTIMEOUT) cmd_res = SDR_DataTimeout;
		if (STA & SDMMC_STA_DCRCFAIL) cmd_res = SDR_DataCRCFail;
		if (STA & SDMMC_STA_RXOVERR)  cmd_res = SDR_RXOverrun;
		if (STA & SDMMC_STA_STBITERR) cmd_res = SDR_StartBitError;
	}

	// Read the data remnant from RX FIFO (if there is still any data)
	while (SDMMC1->STA & SDMMC_STA_RXDAVL) {
		*pBuf++ = SDMMC1->FIFO;
	}

	// Clear the static SDIO flags
	SDMMC1->ICR = SDIO_ICR_STATIC;

	return cmd_res;
}

// Write block of data to the SD card
// input:
//   addr - address of the block to be written
//   pBuf - pointer to the buffer that will contain the received data
//   length - buffer length (must be multiple of 512)
// return: SDResult value
SDResult SD_WriteBlock(uint32_t addr, uint32_t *pBuf, uint32_t length) {
	SDResult cmd_res = SDR_Success;
	uint32_t blk_count = length >> 9; // Sectors in block
	uint32_t STA; // To speed up SDIO flags checking
	register uint32_t STA_mask; // Mask for SDIO flags checking
	uint32_t data_sent = 0; // Counter of transferred bytes
	uint32_t data_left; // Words counter in last portion of data
	uint8_t card_state; // Card state
	uint32_t cntr;


	// Initialize the data control register
	SDMMC1->DCTRL = 0;

	// SDSC card uses byte unit address and
	// SDHC/SDXC cards use block unit address (1 unit = 512 bytes)
	// For SDHC card addr must be converted to block unit address
	if (SDCard.Type == SDCT_SDHC) addr >>= 9;

	if (blk_count > 1) {
		// Prepare bit checking variable for multiple block transfer
		STA_mask = SDIO_TX_MB_FLAGS;
		// Send WRITE_MULTIPLE_BLOCK command
		SD_Cmd(SD_CMD_WRITE_MULTIPLE_BLOCK, addr, SD_RESP_SHORT); // CMD25
		cmd_res = SD_GetR1Resp(SD_CMD_WRITE_MULTIPLE_BLOCK);
	} else {
		// Prepare bit checking variable for single block transfer
		STA_mask = SDIO_TX_SB_FLAGS;
		// Send WRITE_BLOCK command
		SD_Cmd(SD_CMD_WRITE_BLOCK, addr, SD_RESP_SHORT); // CMD24
		cmd_res = SD_GetR1Resp(SD_CMD_WRITE_BLOCK);
	}
	if (cmd_res != SDR_Success) {
		return cmd_res;
	}

	// Clear the static SDIO flags
	SDMMC1->ICR = SDIO_ICR_STATIC;

	// Data write timeout
	SDMMC1->DTIMER = SD_DATA_W_TIMEOUT;
	// Data length
	SDMMC1->DLEN = length;
	// Data transfer:
	//   transfer mode: block
	//   direction: to card
	//   DMA: disabled
	//   block size: 2^9 = 512 bytes
	//   DPSM: enabled
	SDMMC1->DCTRL = (9 << 4) | SDMMC_DCTRL_DTEN;

	// Transfer data block to the SDIO
	// ----> TIME CRITICAL SECTION BEGIN <----
	if (!(length & 0x1F)) {
		// The block length is multiple of 32, simplified transfer procedure can be used
		do {
			if ((SDMMC1->STA & SDMMC_STA_TXFIFOHE) && (data_sent < length)) {
				// The TX FIFO is half empty, at least 8 words can be written
				SDMMC1->FIFO = *pBuf++;
				SDMMC1->FIFO = *pBuf++;
				SDMMC1->FIFO = *pBuf++;
				SDMMC1->FIFO = *pBuf++;
				SDMMC1->FIFO = *pBuf++;
				SDMMC1->FIFO = *pBuf++;
				SDMMC1->FIFO = *pBuf++;
				SDMMC1->FIFO = *pBuf++;
				data_sent += 32;
			}
		} while (!(SDMMC1->STA & STA_mask));
	} else {
		// Since the block length is not a multiple of 32, it is necessary to apply additional calculations
		do {
			if ((SDMMC1->STA & SDMMC_STA_TXFIFOHE) && (data_sent < length)) {
				// TX FIFO half empty, at least 8 words can be written
				if (length - data_sent < 32) {
					// Write last portion of data to the TX FIFO
					data_left = ((length - data_sent) % 4 == 0) ? ((length - data_sent) >> 2) : (((length - data_sent) >> 2) + 1);
					for (cntr = 0; cntr < data_left; cntr++) {
						SDMMC1->FIFO = *pBuf++;
					}
					data_sent += data_left << 2;
				} else {
					// Write 8 words to the TX FIFO
					SDMMC1->FIFO = *pBuf++;
					SDMMC1->FIFO = *pBuf++;
					SDMMC1->FIFO = *pBuf++;
					SDMMC1->FIFO = *pBuf++;
					SDMMC1->FIFO = *pBuf++;
					SDMMC1->FIFO = *pBuf++;
					SDMMC1->FIFO = *pBuf++;
					SDMMC1->FIFO = *pBuf++;
					data_sent += 32;
				}
			}
		} while (!(SDMMC1->STA & STA_mask));
	}
	// <---- TIME CRITICAL SECTION END ---->

	// Save STA register value for further analysis
	STA = SDMMC1->STA;

	// Send stop transmission command in case of multiple block transfer
	if ((SDCard.Type != SDCT_MMC) && (blk_count > 1)) {
		cmd_res = SD_StopTransfer();
	}

	// Check for errors
	if (STA & SDIO_XFER_ERROR_FLAGS) {
		if (STA & SDMMC_STA_DTIMEOUT) cmd_res = SDR_DataTimeout;
		if (STA & SDMMC_STA_DCRCFAIL) cmd_res = SDR_DataCRCFail;
		if (STA & SDMMC_STA_TXUNDERR) cmd_res = SDR_TXUnderrun;
		if (STA & SDMMC_STA_STBITERR) cmd_res = SDR_StartBitError;
	}

	// Wait while the card is in programming state
	do {
		if (SD_GetCardState(&card_state) != SDR_Success) {
			break;
		}
	} while ((card_state == SD_STATE_PRG) || (card_state == SD_STATE_RCV));

	// Clear the static SDIO flags
	SDMMC1->ICR = SDIO_ICR_STATIC;

	return cmd_res;
}

#if (SDIO_USE_DMA)

// Initialize the DMA channel for SDIO peripheral (DMA2 Channel4)
// input:
//   pBuf - pointer to the memory buffer
//   length - size of the memory buffer in bytes (must be a multiple of 4, since the SDIO operates with 32-bit words)
//   direction - DMA channel direction, one of SDIO_DMA_DIR_xx values
// note: the DMA peripheral (DMA2) must be already enabled
void SD_Configure_DMA(uint32_t *pBuf, uint32_t length, uint8_t direction) {
	uint32_t reg;

	// Populate SDIO DMA channel handle
	SDIO_DMA_CH.Channel  = SDIO_DMA_CHANNEL;
	SDIO_DMA_CH.Instance = DMA_GetChannelPeripheral(SDIO_DMA_CHANNEL);
	SDIO_DMA_CH.ChIndex  = DMA_GetChannelIndex(SDIO_DMA_CHANNEL);
	SDIO_DMA_CH.Request  = SDIO_DMA_REQUEST;
	SDIO_DMA_CH.State    = DMA_STATE_RESET;

	// DMA channel configuration:
	//   channel priority: medium
	//   memory increment: enabled
	//   peripheral increment: disabled
	//   circular mode: disabled
	//   IRQ: disabled
	//   memory size: 32-bit
	//   peripheral size: 32-bit
	//   channel: disabled
	SDIO_DMA_CH.Channel->CCR  = DMA_CCR_MINC | DMA_PRIORITY_MEDIUM | DMA_MALIGN_32BIT | DMA_PALIGN_32BIT | direction;
	SDIO_DMA_CH.Channel->CPAR = (uint32_t)(&(SDMMC1->FIFO)); // Address of the SDIO FIFO
	SDIO_DMA_CH.Channel->CMAR = (uint32_t)pBuf; // Memory address

	// Number of DMA transactions
	DMA_SetDataLength(SDIO_DMA_CH.Channel,length >> 2);

	// Map DMA request to DMA channel
	DMA_SetRequest(SDIO_DMA_CH.Instance,SDIO_DMA_CH.Request,SDIO_DMA_CH.ChIndex);

	// Clear SDIO DMA channel interrupt flags
	DMA_ClearFlags(SDIO_DMA_CH.Instance,SDIO_DMA_CH.ChIndex,DMA_CF_ALL);
}

// Start reading of data block from the SD card with DMA transfer
// input:
//   addr - address of the block to be read
//   pBuf - pointer to the buffer that will contain the received data
//   length - buffer length (must be multiple of 512)
// return: SDResult value
SDResult SD_ReadBlock_DMA(uint32_t addr, uint32_t *pBuf, uint32_t length) {
	SDResult cmd_res = SDR_Success;
	uint32_t blk_count = length >> 9;
	uint32_t response;

	// Initialize the data control register
	SDMMC1->DCTRL = 0;

	// Clear the static SDIO flags
	SDMMC1->ICR = SDIO_ICR_STATIC;

	// Configure number of transactions and enable the SDIO DMA channel
	DMA_SetDataLength(SDIO_DMA_CH.Channel, length >> 2);
	DMA_EnableChannel(SDIO_DMA_CH.Channel);

	// SDSC card uses byte unit address and
	// SDHC/SDXC cards use block unit address (1 unit = 512 bytes)
	// For SDHC card addr must be converted to block unit address
	if (SDCard.Type == SDCT_SDHC) addr >>= 9;

	if (blk_count > 1) {
		// Send READ_MULT_BLOCK command
		SD_Cmd(SD_CMD_READ_MULT_BLOCK, addr, SD_RESP_SHORT); // CMD18
		cmd_res = SD_GetR1Resp(SD_CMD_READ_MULT_BLOCK);
	} else {
		// Send READ_SINGLE_BLOCK command
		SD_Cmd(SD_CMD_READ_SINGLE_BLOCK, addr, SD_RESP_SHORT); // CMD17
		cmd_res = SD_GetR1Resp(SD_CMD_READ_SINGLE_BLOCK);
	}
	if (cmd_res == SDR_Success) {
		// Data read timeout
		SDMMC1->DTIMER = SD_DATA_R_TIMEOUT;
		// Data length
		SDMMC1->DLEN   = length;
		// Data transfer:
		//   transfer mode: block
		//   direction: from card
		//   DMA: enabled
		//   block size: 2^9 = 512 bytes
		//   DPSM: enabled
		SDMMC1->DCTRL = SDMMC_DCTRL_DMAEN | SDMMC_DCTRL_DTDIR | (9 << 4) | SDMMC_DCTRL_DTEN;
	}

	return cmd_res;
}

// Start writing block of data to the SD card with DMA transfer
// input:
//   addr - address of the block to be written
//   pBuf - pointer to the buffer that will contain the received data
//   length - buffer length (must be multiple of 512)
// return: SDResult value
SDResult SD_WriteBlock_DMA(uint32_t addr, uint32_t *pBuf, uint32_t length) {
	SDResult cmd_res = SDR_Success;
	uint32_t blk_count = length >> 9;
	uint32_t response;

	// Initialize the data control register
	SDMMC1->DCTRL = 0;

	// Clear the static SDIO flags
	SDMMC1->ICR = SDIO_ICR_STATIC;

	// Configure number of transactions and enable the SDIO DMA channel
	DMA_SetDataLength(SDIO_DMA_CH.Channel,length >> 2);
	DMA_EnableChannel(SDIO_DMA_CH.Channel);

	// SDSC card uses byte unit address and
	// SDHC/SDXC cards use block unit address (1 unit = 512 bytes)
	// For SDHC card addr must be converted to block unit address
	if (SDCard.Type == SDCT_SDHC) addr >>= 9;

	if (blk_count > 1) {
		// Send WRITE_MULTIPLE_BLOCK command
		SD_Cmd(SD_CMD_WRITE_MULTIPLE_BLOCK,addr,SD_RESP_SHORT); // CMD25
		cmd_res = SD_GetR1Resp(SD_CMD_WRITE_MULTIPLE_BLOCK);
	} else {
		// Send WRITE_BLOCK command
		SD_Cmd(SD_CMD_WRITE_BLOCK,addr,SD_RESP_SHORT); // CMD24
		cmd_res = SD_GetR1Resp(SD_CMD_WRITE_BLOCK);
	}
	if (cmd_res == SDR_Success) {
		// Data write timeout
		SDMMC1->DTIMER = SD_DATA_W_TIMEOUT;
		// Data length
		SDMMC1->DLEN   = length;
		// Data transfer:
		//   transfer mode: block
		//   direction: to card
		//   DMA: enabled
		//   block size: 2^9 = 512 bytes
		//   DPSM: enabled
		SDMMC1->DCTRL = SDMMC_DCTRL_DMAEN | (9 << 4) | SDMMC_DCTRL_DTEN;
	}

	return cmd_res;
}

// This function waits until the SDIO DMA data read transfer is finished
// It must be called after SD_ReadBlock_DMA() function to ensure that all
// data sent by the SD card is already transfered by the DMA and send STOP
// command in case of multiple block transfer
// input:
//   length - buffer length (must be a multiple of 512)
// return: SDResult value
// note: length passed here to determine if it was multiple block transfer
SDResult SD_CheckRead(uint32_t length) {
	SDResult cmd_res = SDR_Success;
	uint32_t blk_count = length >> 9;
	volatile uint32_t wait = SD_DATA_R_TIMEOUT;
	uint32_t STA;

	// Wait for SDIO receive complete or error occurred
	if (blk_count > 1) {
		// Multiple block transfer
		while (!(SDMMC1->STA & SDIO_RX_MB_FLAGS) && --wait);
	} else {
		// Single block transfer
		while (!(SDMMC1->STA & SDIO_RX_SB_FLAGS) && --wait);
	}
	STA = SDMMC1->STA;

	// This is a READ operation, SDIO transfer must be completed before DMA transaction,
	// therefore will be enough to ensure only DMA transfer completed, without TXACT polling?

	// Wait for DMA transfer complete and then disable the DMA channel
	while (!(DMA_GetFlags(SDIO_DMA_CH.Instance,SDIO_DMA_CH.ChIndex,DMA_FLAG_TC)) && --wait);
	DMA_DisableChannel(SDIO_DMA_CH.Channel);

	// Send stop transmission command in case of multiple block transfer
	if ((SDCard.Type != SDCT_MMC) && (blk_count > 1)) {
		cmd_res = SD_StopTransfer();
	}

	// Clear the static SDIO flags
	SDMMC1->ICR = SDIO_ICR_STATIC;

	// Timeout?
	if (wait == 0) return SDR_Timeout;

	// Error happened?
	if (STA & SDIO_XFER_ERROR_FLAGS) {
		if (STA & SDMMC_STA_DTIMEOUT) cmd_res = SDR_DataTimeout;
		if (STA & SDMMC_STA_DCRCFAIL) cmd_res = SDR_DataCRCFail;
		if (STA & SDMMC_STA_RXOVERR)  cmd_res = SDR_RXOverrun;
		if (STA & SDMMC_STA_STBITERR) cmd_res = SDR_StartBitError;
	}

	return cmd_res;
}

// This function waits until the SDIO DMA data write transfer is finished
// It must be called after SD_WriteBlock_DMA() function to ensure that all
// data is already transfered by the DMA to SD card and send STOP command
// in case of multiple block transfer
// input:
//   length - buffer length (must be a multiple of 512)
// return: SDResult value
// note: length passed here to determine if it was multiple block transfer
SDResult SD_CheckWrite(uint32_t length) {
	SDResult cmd_res = SDR_Success;
	uint32_t blk_count = length >> 9;
	volatile uint32_t wait = SD_DATA_W_TIMEOUT;
	volatile uint32_t STA;
	uint8_t card_state;

	uint32_t ds,ss;

	// Wait for SDIO receive complete or error occurred
	if (blk_count > 1) {
		// Multiple block transfer
		while (!(SDMMC1->STA & SDIO_TX_MB_FLAGS) && --wait);
	} else {
		// Single block transfer
		while (!(SDMMC1->STA & SDIO_TX_SB_FLAGS) && --wait);
	}

	// Wait while the SDIO transfer active
	while ((SDMMC1->STA & SDMMC_STA_TXACT) && --wait);
	STA = SDMMC1->STA;

	// Disable the DMA channel
	DMA_DisableChannel(SDIO_DMA_CH.Channel);

	// Send stop transmission command in case of multiple block transfer
	if ((SDCard.Type != SDCT_MMC) && (blk_count > 1)) {
		cmd_res = SD_StopTransfer();
	}

	// Clear the static SDIO flags
	SDMMC1->ICR = SDIO_ICR_STATIC;

	// Timeout?
	if (wait == 0) return SDR_Timeout;

	// Error happened?
	if (STA & SDIO_XFER_ERROR_FLAGS) {
		if (STA & SDMMC_STA_DTIMEOUT) cmd_res = SDR_DataTimeout;
		if (STA & SDMMC_STA_DCRCFAIL) cmd_res = SDR_DataCRCFail;
		if (STA & SDMMC_STA_TXUNDERR) cmd_res = SDR_TXUnderrun;
		if (STA & SDMMC_STA_STBITERR) cmd_res = SDR_StartBitError;
	}

	// Wait while the card is in programming state
	do {
		if (SD_GetCardState(&card_state) != SDR_Success) break;
	} while ((card_state == SD_STATE_PRG) || (card_state == SD_STATE_RCV));

	// Clear the static SDIO flags
	SDMMC1->ICR = SDIO_ICR_STATIC;

	return cmd_res;
}

#endif // SDIO_USE_DMA
