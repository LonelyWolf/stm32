#include <stm32l1xx_rcc.h>
#include <stm32l1xx_gpio.h>
#include <misc.h>

#include <spi.h>
#include <sdcard.h>


SDCard_TypeDef SDCard;                 // SD card parameters


// Calculate CRC7
// It's a 7 bit CRC with polynomial x^7 + x^3 + 1
// input:
//   crcIn - the CRC before (0 for first step)
//   data - byte for CRC calculation
// return: the new CRC7
uint8_t CRC7_one(uint8_t crcIn, uint8_t data) {
	const uint8_t g = 0x89;
	uint8_t i;

	crcIn ^= data;
	for (i = 0; i < 8; i++) {
		if (crcIn & 0x80) crcIn ^= g;
		crcIn <<= 1;
	}

	return crcIn;
}

// Calculate CRC7 value of the buffer
// input:
//   pBuf - pointer to the buffer
//   len - length of the buffer
// return: the CRC7 value
uint8_t CRC7_buf(uint8_t *pBuf, uint8_t len) {
	uint8_t crc = 0;

	while (len--) crc = CRC7_one(crc,*pBuf++);

	return crc;
}

// Calculate CRC16 CCITT
// It's a 16 bit CRC with polynomial x^16 + x^12 + x^5 + 1
// input:
//   crcIn - the CRC before (0 for rist step)
//   data - byte for CRC calculation
// return: the CRC16 value
uint16_t CRC16_one(uint16_t crcIn, uint8_t data) {
	crcIn  = (uint8_t)(crcIn >> 8)|(crcIn << 8);
	crcIn ^=  data;
	crcIn ^= (uint8_t)(crcIn & 0xff) >> 4;
	crcIn ^= (crcIn << 8) << 4;
	crcIn ^= ((crcIn & 0xff) << 4) << 1;

	return crcIn;
}

// Calculate CRC16 CCITT value of the buffer
// input:
//   pBuf - pointer to the buffer
//   len - length of the buffer
// return: the CRC16 value
uint16_t CRC16_buf(const uint8_t * pBuf, uint16_t len) {
	uint16_t crc = 0;

	while (len--) crc = CRC16_one(crc,*pBuf++);

	return crc;
}

// Send buffer to the SD card
// input:
//   pBuf - pointer to the buffer
//   len - length of the buffer
// return: last response from SD card
void SD_WriteBuf(uint8_t *pBuf, uint16_t len) {
	while (len--) SPIx_SendRecv(SDCARD_SPI_PORT,*pBuf++);
}

// Receive buffer from the SD card
// input:
//   pBuf - pointer to the buffer
//   len - length of the buffer
void SD_ReadBuf(uint8_t *pBuf, uint16_t len) {
	while (len--) *pBuf++ = SPIx_SendRecv(SDCARD_SPI_PORT,0xff);
}

// Send command to the SD card and get response
// input:
//   cmd - SD card command
//   arg - 32-bit argument for SD card command
// return: SDR_xxx
SDResult_TypeDef SD_Cmd(uint8_t cmd, uint32_t arg, SDCmdResp_TypeDef resp_type, uint8_t *resp) {
	uint8_t buf[6];
	uint32_t wait = 2000; // response timeout
	uint8_t rdLen = 0; // response length
	uint8_t response;

	// Determine response length
	switch (resp_type) {
		case SD_R1:
		case SD_R1b:
			rdLen = 1;
			break;
		case SD_R2:
			rdLen = 2;
			break;
		case SD_R3:
		case SD_R7:
			rdLen = 5;
			break;
		default:
			return SDR_BadResponse;
			break;
	}

	// '01' start bits -> [6b]command -> [32b]argument -> [7b]CRC -> '1' end bit
	buf[0] =  cmd | 0x40; // command
	buf[1] = (arg >> 24); // argument is packed big-endian
	buf[2] = (arg >> 16) & 0xff;
	buf[3] = (arg >>  8) & 0xff;
	buf[4] =  arg & 0xff;
	buf[5] = CRC7_buf(&buf[0],5) | 0x01; // CRC (last bit always '1')

	// Select SD card
	SDCARD_CS_L();

	// Send CMD#
	SD_WriteBuf(&buf[0],6);

	// Wait for a valid response
	do {
		response = SPIx_SendRecv(SDCARD_SPI_PORT,0xff);
	} while ((response & 0x80) && --wait);

	// Timeout
	if (wait == 0) {
		SPIx_SendRecv(SDCARD_SPI_PORT,0xff);
		SDCARD_CS_H();
		return SDR_Timeout;
	}

	// Read the response
	while (rdLen--) {
		*resp++ = response;
		response = SPIx_SendRecv(SDCARD_SPI_PORT,0xff);
	}

	// Release SD card
	SDCARD_CS_H();

	return SDR_Success;
}

// Initialize the SD card
// return: SDR_xxx
// note: SPI peripheral must be initialized before
SDResult_TypeDef SD_Init(void) {
	uint32_t wait;
	uint8_t resp[5]; // buffer for card response
	GPIO_InitTypeDef PORT;

	// Populate SDCard structure with default values
	SDCard.CardCapacity = 0;
	SDCard.CardMaxBusClkFreq = 0;
	SDCard.CardBlockSize = 0;
	SDCard.CardCSDVer = 0;
	SDCard.CardType = SDCT_UNKNOWN;

	// Configure SDCARD CS control line as push-pull output with pullup
	RCC_AHBPeriphClockCmd(SDCARD_PORT_PERIPH,ENABLE);
	PORT.GPIO_Mode  = GPIO_Mode_OUT;
	PORT.GPIO_Speed = GPIO_Speed_40MHz;
	PORT.GPIO_OType = GPIO_OType_PP;
	PORT.GPIO_PuPd  = GPIO_PuPd_UP;
	PORT.GPIO_Pin = SDCARD_CS_PIN;
	GPIO_Init(SDCARD_CS_PORT,&PORT);

	// Set low SPI speed (32MHz -> 125kHz)
	SPIx_SetSpeed(SDCARD_SPI_PORT,SPI_BR_256);

	SDCard.CardType = SDCT_UNKNOWN;

	// Must wait at least 74 clock ticks after reset
	wait = 10;
	while (wait--) SPIx_SendRecv(SDCARD_SPI_PORT,0xff);

	// Software SD card reset (wait for SD card to enter IDLE state)
	// Some cards requires many IDLE commands, so do it several times
	wait = 100;
	do {
		SD_Cmd(SD_CMD_GO_IDLE_STATE,0x00,SD_R1,resp);
	} while (!(resp[0] & SD_R1_IDLE) && --wait);
	if (!wait) return SDR_NoResponse;

	// CMD8: SEND_IF_COND. Send this command to verify SD card interface operating condition
    // Argument: - [31:12]: Reserved (shall be set to '0')
	//           - [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
	//           - [7:0]: Check Pattern (recommended 0xAA)
	SD_Cmd(SD_CMD_HS_SEND_EXT_CSD,SD_CHECK_PATTERN,SD_R7,resp); // CMD8
	if (resp[0] == 0x05) {
		// SDv1 or MMC

		// Issue ACMD41 with zero argument while card returns idle state
		wait = 100;
		do {
			SD_Cmd(SD_CMD_APP_CMD,0,SD_R1,resp); // CMD55
			if (resp[0] & SD_R1_ILLEGAL_CMD) break; // MMC will respond with R1 'illegal command'
			SD_Cmd(SD_CMD_SD_APP_OP_COND,0,SD_R3,resp); // ACMD41
		} while (resp[0] != 0 && --wait);

		if (!wait || resp[0] & SD_R1_ILLEGAL_CMD) {
			// MMC or bad card
			// Issue CMD1: initiate initialization process.
			wait = 20;
			do {
				SD_Cmd(SD_CMD_SEND_OP_COND,0,SD_R1,resp); // CMD1
			} while (resp[0] != 0 && --wait);
			if (!wait) {
				return SDR_UnknownCard;
			} else {
				SDCard.CardType = SDCT_MMC; // MMC
			}
		} else {
			// SDv1 card
			SDCard.CardType = SDCT_SDSC_V1; // SDv1
		}
	} else {
		// SDv2 or later

		// Check for SDv2 pattern
		wait  = resp[1] << 24;
		wait |= resp[2] << 16;
		wait |= resp[3] << 8;
		wait |= resp[4];
		if ((wait & 0x01ff) != (SD_CHECK_PATTERN & 0x01ff)) return SDR_Unsupported;

		// Issue ACMD41 (this can be up to one second long)
		wait = 0x2710;
		do {
			// CMD55: Send leading command for ACMD<n> command.
			SD_Cmd(SD_CMD_APP_CMD,0,SD_R1,resp); // CMD55
			// ACMD41 - initiate initialization process.
			// Set HCS bit (Host Capacity Support) to inform card what
			// host support high capacity
			SD_Cmd(SD_CMD_SD_APP_OP_COND,1 << 30,SD_R3,resp); // ACMD41
		} while (resp[0] && --wait);
		if (!wait) return SDR_Timeout;

		// This is SDv2
		SDCard.CardType = SDCT_SDSC_V2;

		// Read the OCR register
		SD_Cmd(SD_CMD_READ_OCR,0,SD_R3,resp); // CMD58
		if (resp[0] == 0x00) {
			// R3 response
			wait  = resp[1] << 24;
			wait |= resp[2] << 16;
			wait |= resp[3] << 8;
			wait |= resp[4];
		} else {
			SDCard.CardType = SDCT_UNKNOWN;
			return SDR_BadResponse; // bad CMD58 response
		}
		// If CCS (Card Capacity Status) bit set -> this is SDHC or SDXC card
		if (wait & (1 << 30)) SDCard.CardType = SDCT_SDHC; // SDHC or SDXC
	}

	// Unknown or bad card
	if (SDCard.CardType == SDCT_UNKNOWN) return SDR_UnknownCard;

	// Set SPI to higher speed (32MHz -> 16MHz)
	SPIx_SetSpeed(SDCARD_SPI_PORT,SPI_BR_2);

	// Turn off CRC checks
//	SD_Cmd(SD_CMD_CRC_ON_OFF,0,SD_R1,resp); // CMD59
	// Turn on CRC checks
//	SD_Cmd(SD_CMD_CRC_ON_OFF,1,SD_R1,resp); // CMD59

	// Must set block size for SDv1,SDv2 and MMC
	// SDHC and SDXC always have fixed size 512
	if ((SDCard.CardType == SDCT_SDSC_V1) || (SDCard.CardType == SDCT_SDSC_V2) ||
			(SDCard.CardType == SDCT_MMC)) {
		SD_Cmd(SD_CMD_SET_BLOCKLEN,512,SD_R1,resp); // CMD16
		if (resp[0] != 0x00) return SDR_SetBlockSizeFailed;
	}

	return SDR_Success;
}

// Read register data after sending register read request
// input:
//   buf - pointer to the buffer to store the register value
//   length - length of the register (bytes)
// return: SDR_xxx
SDResult_TypeDef SD_ReadReg(uint8_t *buf, uint16_t length) {
	uint32_t wait;
	uint8_t cmdres;
	uint16_t CRC_rcv; // Received CRC16 value of the register
//	uint16_t CRC_loc; // Calculated CRC16 value of the register

	// Select SD card
	SDCARD_CS_L();

	// Waiting for a start block token
	wait = 0xfff; // Recommended timeout is 100ms FIXME: 0xfff is set by sight, need calculate more adequate value
	do {
		cmdres = SPIx_SendRecv(SDCARD_SPI_PORT,0xff);
	} while (cmdres == 0xff && --wait);

	if (!wait) {
		// It was timeout
		SPIx_SendRecv(SDCARD_SPI_PORT,0xff);
		SDCARD_CS_H();
		return SDR_Timeout;
	}

	if (cmdres != SD_TOKEN_START_BLOCK)	{
		// The card respond is not a start token
		SPIx_SendRecv(SDCARD_SPI_PORT,0xff);
		SDCARD_CS_H();
		return SDR_ReadError;
	}

	// Receive the register value
	SD_ReadBuf(buf,length);

	// 16-bit CRC (some cards demand to receive this)
	// Since some SD cards give wrong CRC values, we don't check if they are correct
	CRC_rcv  = SPIx_SendRecv(SDCARD_SPI_PORT,0xff) << 8;
	CRC_rcv |= SPIx_SendRecv(SDCARD_SPI_PORT,0xff);
//	CRC_loc  = CRC16_buf(buf,16);
//	if (CRC_rcv != CRC_loc) return SD_CRCErorr;

	// Provide extra 8 clocks for the card (from SanDisk specification)
	SPIx_SendRecv(SDCARD_SPI_PORT,0xff);

	// Release SD card
	SDCARD_CS_H();

	return SDR_Success;
}

// Read the CSD register from the SD card
// return: SDR_xxx
SDResult_TypeDef SD_ReadCSD(void) {
	uint8_t cmdres; // result of SD_Cmd
	uint8_t resp[5]; // buffer for card response
	uint32_t dev_size, dev_size_mul, rd_block_len;

	cmdres = SD_Cmd(SD_CMD_SEND_CSD,0,SD_R1,resp); // CMD9
	if (cmdres != SDR_Success) return cmdres;
	if (resp[0] != 0x00) return SDR_BadResponse;
	cmdres = SD_ReadReg(SDCard.CSD,16);
	if (cmdres != SDR_Success) return cmdres;

	// Parse the CSD register
	SDCard.CardCSDVer = SDCard.CSD[0] >> 6; // CSD version
	if (SDCard.CardType != SDCT_MMC) {
		// SD
		SDCard.CardMaxBusClkFreq = SDCard.CSD[3];
		rd_block_len = SDCard.CSD[5] & 0x0f; // Max. read data block length
		if (SDCard.CardCSDVer == 0) {
			// CSD v1.00 (SDSCv1, SDSCv2)
			dev_size  = (uint32_t)(SDCard.CSD[6] & 0x03) << 10; // Device size
			dev_size |= (uint32_t)SDCard.CSD[7] << 2;
			dev_size |= (SDCard.CSD[8] & 0xc0) >> 6;
			dev_size_mul  = (SDCard.CSD[9] & 0x03) << 1; // Device size multiplier
			dev_size_mul |= (SDCard.CSD[10] & 0x80) >> 7;
			SDCard.CardCapacity  = (dev_size + 1);
			SDCard.CardCapacity *= (1 << (dev_size_mul + 2));
			SDCard.CardBlockSize = 1 << (rd_block_len);
			SDCard.CardCapacity *= SDCard.CardBlockSize;
		} else {
			// CSD v2.00 (SDHC, SDXC)
			dev_size  = (SDCard.CSD[7] & 0x3f) << 16;
			dev_size |=  SDCard.CSD[8] << 8;
			dev_size |=  SDCard.CSD[9];
			SDCard.CardBlockSize = 512;
			SDCard.CardCapacity = (dev_size + 1) * SDCard.CardBlockSize;
		}
	} else {
		// MMC
		SDCard.CardMaxBusClkFreq = SDCard.CSD[3];
		dev_size  = (uint32_t)(SDCard.CSD[6] & 0x03) << 8; // C_SIZE
		dev_size += (uint32_t)SDCard.CSD[7];
		dev_size <<= 2;
		dev_size += SDCard.CSD[8] >> 6;
		SDCard.CardBlockSize = 1 << (SDCard.CSD[5] & 0x0f); // MMC read block length
		dev_size_mul = ((SDCard.CSD[9] & 0x03) << 1) + ((SDCard.CSD[10] & 0x80) >> 7);
		SDCard.CardCapacity = (dev_size + 1) * (1 << (dev_size_mul + 2)) * SDCard.CardBlockSize;
	}

	return SDR_Success;
}

// Read the CID register from the SD card
// return: SDR_xxx
SDResult_TypeDef SD_ReadCID(void) {
	uint8_t cmdres; // result of SD_Cmd
	uint8_t resp[5]; // buffer for card response

	cmdres = SD_Cmd(SD_CMD_SEND_CID,0,SD_R1,resp); // CMD10
	if (cmdres != SDR_Success) return cmdres;
	if (resp[0] != 0x00) return SDR_BadResponse;
	cmdres = SD_ReadReg(SDCard.CID,16);
	if (cmdres != SDR_Success) return cmdres;

	// Parse CID register
	// ...

	return SDR_Success;
}

// Read block of data from the SD card
// input:
//   addr - start address of the block (must be power of two)
//   pBuf - pointer to the buffer for received data
//   len - buffer length
// return: SDR_xxx
SDResult_TypeDef SD_ReadBlock(uint32_t addr, uint8_t *pBuf, uint32_t len) {
	uint32_t wait;
	uint8_t cmdres;
	uint8_t resp[5];
	uint16_t CRC_rcv; // Received CRC16 of the block
	uint16_t CRC_loc; // Calculated CRC16 of the block

	// SDSC card uses byte unit address and
	// SDHC/SDXC cards use block unit address (1 unit = 512 bytes)
	// For SDSC card addr must be converted to byte address
	if (SDCard.CardType != SDCT_SDHC) addr <<= 9;
	cmdres = SD_Cmd(SD_CMD_READ_SINGLE_BLOCK,addr,SD_R1,resp); // CMD17
	if (cmdres != SDR_Success) return cmdres;
	if (resp[0] == 0x00) {
		// Select SD card
		SDCARD_CS_L();

		// Waiting for a start block token
		wait = 0xfff; // Recommended timeout is 100ms FIXME: 0xfff is set by sight, need calculate more adequate value
		do {
			cmdres = SPIx_SendRecv(SDCARD_SPI_PORT,0xff);
		} while (cmdres == 0xff && --wait);

		if (!wait) {
			// It was timeout
			SPIx_SendRecv(SDCARD_SPI_PORT,0xff);
			SDCARD_CS_H();
			return SDR_Timeout;
		}

		if (cmdres != SD_TOKEN_START_BLOCK)	{
			// The card respond is not a start token
			SPIx_SendRecv(SDCARD_SPI_PORT,0xff);
			SDCARD_CS_H();

			if (cmdres & SD_TOKEN_READ_RANGE_ERROR) {
				// Specified address out of range
				return SDR_AddrError;
			}

			return SDR_ReadError;
		}

		// Receive data block
		SD_ReadBuf(pBuf,len);

		// 16-bit CRC (it must be received even if CRC is off)
		CRC_rcv  = SPIx_SendRecv(SDCARD_SPI_PORT,0xff) << 8;
		CRC_rcv |= SPIx_SendRecv(SDCARD_SPI_PORT,0xff);
		CRC_loc  = CRC16_buf(pBuf,len);

		// Provide extra 8 clocks for the card (from SanDisk specification)
		SPIx_SendRecv(SDCARD_SPI_PORT,0xff);

		// Release SD card
		SDCARD_CS_H();
	} else {
		// CMD17 has been rejected
		if (resp[0] & SD_R1_ADDR_ERROR) {
			// Given address is misaligned
			return SDR_AddrError;
		}
		if (resp[0] & SD_R1_PARAM_ERROR) {
			// Given address out of bounds
			return SDR_AddrError;
		}
		// Some unknown error
		return SDR_ReadError;
	}

	return (CRC_rcv == CRC_loc) ? SDR_Success : SDR_CRCError;
}


// Write block of data to the SD card
// input:
//   addr - start address of the block (must be power of two)
//   pBuf - pointer to the buffer with data
//   len - buffer length
// return: SDR_xxx
SDResult_TypeDef SD_WriteBlock(uint32_t addr, uint8_t *pBuf, uint32_t len) {
	uint32_t wait;
	uint8_t cmdres;
	uint8_t resp[5];
	uint16_t CRC_loc; // Calculated CRC16 of the block

	// Calculate 16-bit CRC
	CRC_loc = CRC16_buf(pBuf,len);

	// SDSC card uses byte unit address and
	// SDHC/SDXC cards use block unit address (1 unit = 512 bytes)
	// For SDSC card addr must be converted to byte address
	if (SDCard.CardType != SDCT_SDHC) addr <<= 9;
	cmdres = SD_Cmd(SD_CMD_WRITE_SINGLE_BLOCK,addr,SD_R1,resp); // CMD24
	if (cmdres != SDR_Success) return cmdres;
	if (resp[0] == 0x00) {
		// Select SD card
		SDCARD_CS_L();

		// Send start block token
		SPIx_SendRecv(SDCARD_SPI_PORT,SD_TOKEN_START_BLOCK);

		// Send data block
		SD_WriteBuf(pBuf,len);

		// Send CRC
		SPIx_SendRecv(SDCARD_SPI_PORT,CRC_loc >> 8);
		SPIx_SendRecv(SDCARD_SPI_PORT,(uint8_t)CRC_loc);

		// Get response from the SD card
		cmdres = SPIx_SendRecv(SDCARD_SPI_PORT,0xff);
		cmdres &= 0x1f;
		if (cmdres != SD_TOKEN_DATA_ACCEPTED) {
			// Data block rejected by SD card for some reason
			// Release SD card
			SPIx_SendRecv(SDCARD_SPI_PORT,0xff);
			SDCARD_CS_H();

			if (cmdres & SD_TOKEN_WRITE_CRC_ERROR) return SDR_WriteCRCError;
			if (cmdres & SD_TOKEN_WRITE_ERROR) return SDR_WriteErrorInternal;
			return SDR_WriteError;
		}

		// Wait while the SD card is busy by data programming
		wait = 0x7fff; // Recommended timeout is 250ms (500ms for SDXC) FIXME: 0x7fff is set by sight, need calculate more adequate value
		do {
			cmdres = SPIx_SendRecv(SDCARD_SPI_PORT,0xff);
		} while (cmdres == 0 && --wait);

		// Provide extra 8 clocks for the card (from SanDisk specification)
		SPIx_SendRecv(SDCARD_SPI_PORT,0xff);

		// Release SD card
		SDCARD_CS_H();
	} else {
		// Card rejected CMD24
		if ((resp[0] & SD_R1_ADDR_ERROR) || (resp[0] & SD_R1_PARAM_ERROR)) return SDR_AddrError;
		return SDR_WriteError;
	}

	return SDR_Success;
}
