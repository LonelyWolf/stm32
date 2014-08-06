#include <stm32l1xx_rcc.h>
#include <stm32l1xx_gpio.h>

#include <spi1.h>
#include <sdcard.h>
#include <uart.h>


SDCard_TypeDef SDCard;                 // SD card parameters
//uint8_t  SD_sector[512];               // Buffer for SD card sector


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
	while (len--) SPI1_SendRecv(*pBuf++);
}

// Receive buffer from the SD card
// input:
//   pBuf - pointer to the buffer
//   len - length of the buffer
void SD_ReadBuf(uint8_t *pBuf, uint16_t len) {
	while (len--) *pBuf++ = SPI1_SendRecv(0xff);
}

// Send command to the SD card
// input:
//   cmd - SD card command
//   arg - 32-bit argument for SD card command
// return: SD card response (0xff if it was a timeout)
uint8_t SD_Cmd(uint8_t cmd, uint32_t arg) {
	uint8_t buf[6];
	uint32_t wait = 2000;
	uint8_t response;

	buf[0] =  cmd | 0x40; // command
	buf[1] = (arg >> 24); // argument is packed big-endian
	buf[2] = (arg >> 16) & 0xff;
	buf[3] = (arg >>  8) & 0xff;
	buf[4] =  arg & 0xff;
	buf[5] = CRC7_buf(&buf[0],5) | 0x01; // CRC (end bit always '1')

	// Dummy read is necessary for some cards
	SPI1_SendRecv(0xff);

//	SPI1_SendRecv(buf[0]);
//	SPI1_SendRecv(buf[1]);
//	SPI1_SendRecv(buf[2]);
//	SPI1_SendRecv(buf[3]);
//	SPI1_SendRecv(buf[4]);
//	SPI1_SendRecv(buf[5]);

	// Send: '01' start bits -> [6b]command -> [32b]argument -> [7b]CRC -> '1' end bit
	SD_WriteBuf(&buf[0],6);

	// Wait for response from SD card
	do {
		response = SPI1_SendRecv(0xff);
	} while (((response & 0x80) != 0) && --wait);

	return response;
}

SDResult_TypeDef SD_Init(void) {
	uint32_t wait;
	uint8_t response;
	uint32_t long_response;
	GPIO_InitTypeDef PORT;

	// Configure SDCARD CS control line as push-pull output with pullup
	RCC_AHBPeriphClockCmd(SDCARD_PORT_PERIPH,ENABLE);
	PORT.GPIO_Mode  = GPIO_Mode_OUT;
	PORT.GPIO_Speed = GPIO_Speed_40MHz;
	PORT.GPIO_OType = GPIO_OType_PP;
	PORT.GPIO_PuPd  = GPIO_PuPd_UP;
	PORT.GPIO_Pin = SDCARD_CS_PIN;
	GPIO_Init(SDCARD_CS_PORT,&PORT);

	//////////////////////////////////////////////////
	// ****** SPI must be initialized before ****** //
	//////////////////////////////////////////////////

	// Set low SPI speed (32MHz -> 125kHz)
	SPI1_InitSpeed(0x38); // 0x38 = SPI_BaudRatePrescaler_256

	// Pull CS low - select SD card
	SDCARD_CS_L();

	SDCard.CardType = SDCT_UNKNOWN;

	// Must wait at least 74 clock ticks after reset
	wait = 10;
	while (wait--) SPI1_SendRecv(0xff);

	// Software SD card reset (wait for SD card to enter IDLE state)
	// Some cards requires many IDLE commands, so do it several times
	wait = 50;
	while (SD_Cmd(SD_CMD_GO_IDLE_STATE,0x00) != 0x01 && --wait);
	if (!wait) {
		SDCARD_CS_H();
		return SDR_NoResponse;
	}

	// CMD8: SEND_IF_COND. Send this command to verify SD card interface operating condition
    // Argument: - [31:12]: Reserved (shall be set to '0')
	//           - [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
	//           - [7:0]: Check Pattern (recommended 0xAA)
	response = SD_Cmd(SD_CMD_HS_SEND_EXT_CSD,SD_CHECK_PATTERN); // CMD8
	if (response == 0x05) {
		// SDv1 or MMC

		// Issue ACMD41 with zero argument while card returns idle state
		wait = 100;
		do {
			response = SD_Cmd(SD_CMD_APP_CMD,0); // CMD55
			if (response & 0x04) break; // MMC will respond with R1 'illegal command'
			response = SD_Cmd(SD_CMD_SD_APP_OP_COND,0); // ACMD41
		} while (response != 0 && --wait);

		if (!wait || response & 0x04) {
			// MMC or bad card
			// Issue CMD1: initiate initialization process.
			wait = 20;
			do {
				response = SD_Cmd(SD_CMD_SEND_OP_COND,0); // CMD1
			} while (response != 0 && --wait);
			if (!wait) {
				SDCARD_CS_H();
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

		// R7 response
		long_response  = SPI1_SendRecv(0xff) << 24;
		long_response |= SPI1_SendRecv(0xff) << 16;
		long_response |= SPI1_SendRecv(0xff) <<  8;
		long_response |= SPI1_SendRecv(0xff);

		// SDv2 pattern mismatch -> unsupported SD card
		if ((long_response & 0x01ff) != (SD_CHECK_PATTERN & 0x01ff)) {
			SDCARD_CS_H();
			return SDR_Unsupported;
		}

		// Issue ACMD41 (this can be up to one second long)
		wait = 0x2710;
		do {
			// CMD55: Send leading command for ACMD<n> command.
			SD_Cmd(SD_CMD_APP_CMD,0); // CMD55
			// ACMD41 - initiate initialization process.
			// Set HCS bit (Host Capacity Support) to inform card what
			// host support high capacity
			response = SD_Cmd(SD_CMD_SD_APP_OP_COND,1 << 30); // ACMD41
		} while (response && --wait);
		if (!wait) {
			SDCARD_CS_H();
			return SDR_Timeout;
		}

		SDCard.CardType = SDCT_SDSC_V2; // SDv2

		// Read the OCR register
		response = SD_Cmd(SD_CMD_READ_OCR,0); // CMD58
		if (response == 0x00) {
			// R3 response
			long_response  = SPI1_SendRecv(0xff) << 24;
			long_response |= SPI1_SendRecv(0xff) << 16;
			long_response |= SPI1_SendRecv(0xff) <<  8;
			long_response |= SPI1_SendRecv(0xff);
		} else {
			SDCard.CardType = SDCT_UNKNOWN;
			SDCARD_CS_H();
			return SDR_BadResponse; // bad CMD58 response
		}
		// If CCS (Card Capacity Status) bit set -> this is SDHC or SDXC card
		if (long_response & (1 << 30)) SDCard.CardType = SDCT_SDHC; // SDHC or SDXC
	}

	// Unknown or bad card
	if (SDCard.CardType == SDCT_UNKNOWN) {
		SDCARD_CS_H();
		return SDR_UnknownCard;
	}

	// Set SPI to higher speed (32MHz -> 16MHz)
	SPI1_InitSpeed(0x00); // 0x00 = SPI_BaudRatePrescaler_2

	// Turn off CRC checks
	//SD_Cmd(SD_CMD_CRC_ON_OFF,0x00000001); // CMD59

	// For SDv2,SDv1,MMC must set block size
	// SDHC and SDXC have fixed size 512
	if ((SDCard.CardType == SDCT_SDSC_V1) || (SDCard.CardType == SDCT_SDSC_V2) || (SDCard.CardType == SDCT_MMC)) {
		// CMD16: block size = 512 bytes
		if (SD_Cmd(SD_CMD_SET_BLOCKLEN,512) != 0x00) {
			SDCARD_CS_H();
			return SDR_SetBlockSizeFailed;
		}
	}

	// Release SD card
	SDCARD_CS_H();

	// Read and parse card information
	SD_ReadCSD();
	SD_ReadCID();

	return SDR_Success;
}

// Read the CSD register from the SD card
// return:
//   SD_Success - CSD register received and parsed
//   SD_Timeout - it was a timeout
//   SD_BadResponse - it was bad response for CMD9 command
SDResult_TypeDef SD_ReadCSD(void) {
	uint32_t wait;
	uint8_t response;
	uint16_t CSD_CRC_rcv; // Received CRC16 value of a CSD
//	uint16_t CSD_CRC;     // Calculated CRC16 value of a CSD
	uint32_t dev_size, dev_size_mul, rd_block_len;

	// Select SD card
	SDCARD_CS_L();

	response = SD_Cmd(SD_CMD_SEND_CSD,0); // CMD9
	if (response != 0x00) {
		// Something wrong happened
		SDCARD_CS_H();
		return SDR_BadResponse;
	} else {
		// Wait start block token
		wait = 0xfff; // Recommended timeout is 100ms
		do response = SPI1_SendRecv(0xff); while (response == 0xff && --wait);
		if (!wait) {
			// Timeout occurred
			SDCARD_CS_H();
			return SDR_Timeout;
		}
		if (response != SD_TOKEN_START_BLOCK) {
			// Card responded but it was not the start block token
			SDCARD_CS_H();
			return SDR_ReadError;
		}
		// Receive block
		SD_ReadBuf(&SDCard.CSD[0],16);
	}

	// 16-bit CRC (some cards demand to receive this)
	// But some cards are not able to calculate a valid CRC16 value
	// of CID and CSD reads.
	CSD_CRC_rcv  = SPI1_SendRecv(0xff) << 8;
	CSD_CRC_rcv |= SPI1_SendRecv(0xff);
//	CSD_CRC      = CRC16_buf(&SDCard.CSD[0],16);

	// Release SD card
	SDCARD_CS_H();

//	if (CSD_CRC_rcv != CSD_CRC) return SD_CRCErorr;

	// Parse CSD register
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
// return:
//   SD_Success - CID register received and parsed
//   SD_Timeout - it was a timeout
//   SD_BadResponse - it was bad response for CMD10 command
SDResult_TypeDef SD_ReadCID(void) {
	uint32_t wait;
	uint8_t response;
	uint16_t CID_CRC_rcv; // Received CRC16 value of a CID
//	uint16_t CID_CRC;     // Calculated CRC16 value of a CID

	// Select SD card
	SDCARD_CS_L();

	response = SD_Cmd(SD_CMD_SEND_CID,0); // CMD10
	if (response != 0x00) {
		// Something wrong happened
		SDCARD_CS_H();
		return SDR_BadResponse;
	} else {
		// Wait start block token
		wait = 0xfff; // Recommended timeout is 100ms
		do response = SPI1_SendRecv(0xff); while (response == 0xff && --wait);
		if (!wait) {
			// Timeout occurred
			SDCARD_CS_H();
			return SDR_Timeout;
		}
		if (response != SD_TOKEN_START_BLOCK) {
			// Card responded but it was not the start block token
			SDCARD_CS_H();
			return SDR_ReadError;
		}
		// Receive block
		SD_ReadBuf(&SDCard.CSD[0],16);
	}

	// 16-bit CRC (some cards demand to receive this)
	CID_CRC_rcv  = SPI1_SendRecv(0xff) << 8;
	CID_CRC_rcv |= SPI1_SendRecv(0xff);
//	CID_CRC      = CRC16_buf(&SDCard.CID[0],16);

	// Release SD card
	SDCARD_CS_H();

//	if (CID_CRC_rcv != CID_CRC) return SD_CRCErorr;

	// Parse CID register
	// ...

	return SDR_Success;
}

// Read block of data from the SD card
// input:
//   addr - start address of the block (must be power of two)
//   pBuf - pointer to the buffer for received data
//   len - buffer length
// return:
//   SD_Success - block of data received and stored in the buffer
//   SD_Timeout - it was a timeout
//   SD_ReadError - it was bad response for CMD17 command
//   SD_CRCErorr - in case of mismatch computed CRC and received
SDResult_TypeDef SD_ReadBlock(uint32_t addr, uint8_t *pBuf, uint32_t len) {
	uint32_t wait;
	uint8_t response;
	uint16_t Blk_CRC_rcv; // Received CRC16 of the block
	uint16_t Blk_CRC;     // Calculated CRC16 of the block

	// Select SD card
	SDCARD_CS_L();

	// SD card accepts byte address while SDHC accepts block address in multiples of 512
	// so for SD card block address must be converted into corresponding byte address
	if (SDCard.CardType != SDCT_SDHC) addr <<= 9;
	response = SD_Cmd(SD_CMD_READ_SINGLE_BLOCK,addr); // CMD17
	if (response == 0x00) {
		// Wait for start block token
		wait = 0xfff; // recommended timeout is 100ms
		do response = SPI1_SendRecv(0xff); while (response == 0xff && --wait);
		if (!wait) {
			// Timeout occurred
			SDCARD_CS_H();
			return SDR_Timeout;
		}
		if (response != SD_TOKEN_START_BLOCK) {
			// Card responded but it was not the start block token
			SDCARD_CS_H();
			return SDR_ReadError; // READ_SINGLE_BLOCK failed
		}
		// Receive data block
		SD_ReadBuf(pBuf,len);
	} else {
		// Card rejected CMD17
		// TODO: check for address error
		SDCARD_CS_H();
		return SDR_ReadError; // READ_SINGLE_BLOCK failed
	}

	// 16-bit CRC (some cards demand to receive this even if CRC is off)
	Blk_CRC_rcv  = SPI1_SendRecv(0xff) << 8;
	Blk_CRC_rcv |= SPI1_SendRecv(0xff);
	Blk_CRC = CRC16_buf(pBuf,len);

	// Release SD card
	SDCARD_CS_H();

	return (Blk_CRC_rcv == Blk_CRC) ? SDR_Success : SDR_CRCError;
}

// Write block of data to the SD card
// input:
//   addr - start address of the block (must be power of two)
//   pBuf - pointer to the buffer with data
//   len - buffer length
// return:
//   SD_Success - block of data received and stored in the buffer
//   SD_Timeout - it was a timeout
//   SD_WriteError - it was bad response for CMD24 command
//   SD_CRCErorr - in case of mismatch computed CRC and received
SDResult_TypeDef SD_WriteBlock(uint32_t addr, uint8_t *pBuf, uint32_t len) {
	uint32_t wait;
	uint8_t response;
	uint16_t Blk_CRC; // Calculated CRC16 of the block

	// Calculate 16-bit CRC
	Blk_CRC = CRC16_buf(pBuf,len);

	// Select SD card
	SDCARD_CS_L();

	// SD card accepts byte address while SDHC accepts block address in multiples of 512
	// so for SD card block address must be converted into corresponding byte address
	if (SDCard.CardType != SDCT_SDHC) addr <<= 9;
	response = SD_Cmd(SD_CMD_WRITE_SINGLE_BLOCK,addr); // CMD24
	if (response == 0x00) {
		SPI1_SendRecv(SD_TOKEN_START_BLOCK); // Send start block token
		SD_WriteBuf(pBuf,len); // Send data block
		// Send CRC
		SPI1_SendRecv(Blk_CRC >> 8);
		SPI1_SendRecv((uint8_t)Blk_CRC);
		// Get data response from SD card
		response = SPI1_SendRecv(0xff);
		if ((response & 0x1f) != SD_TOKEN_DATA_ACCEPTED) {
			// Data block rejected by SD card for some reason
			SDCARD_CS_H();
			if ((response & 0x1f) == SD_TOKEN_DATA_WRITE_ERROR) {
				// Data rejected due to a write error (internal SD card error)
				return SDR_WriteErrorInternal;
			} else {
				// Data rejected due to a CRC error
				// Actually this should not happen, one reason
			    // is noise on SPI lines
				return SDR_CRCError;
			}
		}
		// Wait while the SD card is busy by data programming
		wait = 0x2fff; // Recommended timeout is 250ms
		do {
			response = SPI1_SendRecv(0xff);
		} while (!response && --wait);

		// Release SD card
		SDCARD_CS_H();

		if (!wait) return SDR_Timeout; // Timeout occurred
	} else {
		// Card rejected CMD24
		// TODO: check for address error
		SDCARD_CS_H();
		return SDR_WriteError; // WRITE_SINGLE_BLOCK failed
	}

	return SDR_Success;
}

uint32_t DFS_ReadSector(uint8_t unit, uint8_t *buffer, uint32_t sector, uint32_t count) {
	SDResult_TypeDef Status = SDR_Success;

    Status = SD_ReadBlock(sector,buffer,512);

    return (Status == SDR_Success) ? 0 : 1;
}

uint32_t DFS_WriteSector(uint8_t unit, uint8_t *buffer, uint32_t sector, uint32_t count) {
	SDResult_TypeDef Status = SDR_Success;

    Status = SD_WriteBlock(sector,buffer,512);

    return (Status == SDR_Success) ? 0 : 1;
}
