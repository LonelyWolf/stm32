#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_spi.h>
#include <sdcard.h>


uint8_t  SD_CardType = SD_UNKNOWN_SD_CARD;         // SD Card type
uint32_t SD_CardCapacity;                          // SD Card capacity
uint8_t  SD_MaxBusClkFreq = 0;                     // Max. card bus frequency
uint8_t  SD_MID = 0;                               // SD Card Maunufacturer ID
uint16_t SD_OID = 0;                               // SD Card OEM/Application ID


uint8_t  SD_CSD[16];                               // CSD buffer
uint8_t  SD_CID[16];                               // CID buffer
uint8_t  SD_sector[512];                           // sector buffer
uint16_t SD_CRC16_rcv;                             // last received CRC16
uint16_t SD_CRC16_cmp;                             // last computed CRC16


static uint8_t CRC7_one(uint8_t t, uint8_t data) {
	const uint8_t g = 0x89;
	uint8_t i;

	t ^= data;
	for (i = 0; i < 8; i++) {
		if (t & 0x80) t ^= g;
		t <<= 1;
	}

	return t;
}

uint8_t CRC7_buf(const uint8_t * p, uint8_t len) {
	uint8_t j,crc = 0;

	for (j = 0; j < len; j++) crc = CRC7_one(crc,p[j]);

	return crc >> 1;
}

static uint16_t CRC16_one(uint16_t crc, uint8_t ser_data) {
	crc  = (uint8_t)(crc >> 8)|(crc << 8);
	crc ^= ser_data;
	crc ^= (uint8_t)(crc & 0xff) >> 4;
	crc ^= (crc << 8) << 4;
	crc ^= ((crc & 0xff) << 4) << 1;

	return crc;
}

uint16_t CRC16_buf(const uint8_t * p, uint32_t len) {
	uint32_t i;
	uint16_t crc = 0;

	for (i = 0; i < len; i++) crc = CRC16_one(crc,p[i]);

	return crc;
}

void SD_SPI_Init(uint16_t prescaler) {
	SPI_InitTypeDef SPI;
	SPI.SPI_Mode = SPI_Mode_Master;
	SPI.SPI_BaudRatePrescaler = prescaler;
	SPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI.SPI_CPOL = SPI_CPOL_High;
	SPI.SPI_CPHA = SPI_CPHA_2Edge;
	SPI.SPI_CRCPolynomial = 7;
	SPI.SPI_DataSize = SPI_DataSize_8b;
	SPI.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI.SPI_NSS = SPI_NSS_Soft;
	SPI_Init(SD_SPI,&SPI);

	// NSS must be set to '1' due to NSS_Soft settings (otherwise it will be Multimaster mode).
	SPI_NSSInternalSoftwareConfig(SD_SPI,SPI_NSSInternalSoft_Set);
}

void SD_Init(void) {
#if _SD_SPI == 1
	// SPI1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOA,ENABLE);
#elif _SD_SPI == 2
	// SPI2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
#elif _SD_SPI == 3
	// SPI3
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); // Disable JTAG for use PB3
#endif
	GPIO_InitTypeDef PORT;
	// Configure SPI pins
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	PORT.GPIO_Pin = SD_SCK_PIN | SD_MISO_PIN | SD_MOSI_PIN;
	PORT.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SD_PORT,&PORT);
	// Configure CS pin as output with Push-Pull
	PORT.GPIO_Pin = SD_CS_PIN;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(SD_CS_PORT,&PORT);

	SD_SPI_Init(SPI_BaudRatePrescaler_256); // set SPI at lowest possible speed
	SPI_Cmd(SD_SPI,ENABLE);

	GPIO_WriteBit(SD_CS_PORT,SD_CS_PIN,Bit_SET); // pull SD pin -> Deselect SD
}

uint8_t SD_SendRecv(uint8_t data) {
	uint8_t miso;

	while (SPI_I2S_GetFlagStatus(SD_SPI,SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SD_SPI,data);
	while (SPI_I2S_GetFlagStatus(SD_SPI,SPI_I2S_FLAG_RXNE) == RESET);
	miso = SPI_I2S_ReceiveData(SD_SPI);

	return miso;
}

uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg) {
	uint8_t wait, response, crc = 0;

	SD_SendRecv(0xff); // This dummy send necessary for some cards

	// Send: [8b]Command -> [32b]Argument -> [8b]CRC
	SD_SendRecv(cmd | 0x40);
	crc = CRC7_one(crc,cmd | 0x40);
	SD_SendRecv(arg >> 24);
	crc = CRC7_one(crc,arg >> 24);
	SD_SendRecv(arg >> 16);
	crc = CRC7_one(crc,arg >> 16);
	SD_SendRecv(arg >> 8);
	crc = CRC7_one(crc,arg >> 8);
	SD_SendRecv(arg);
	crc = CRC7_one(crc,arg);
	SD_SendRecv(crc | 0x01); // Bit 1 always must be set to "1" in CRC

	// Wait for response from SD Card
	wait = 0;
	while ((response = SD_SendRecv(0xff)) == 0xff) if (wait++ > 200) break;

	return response;
}

// return values:
//   0xff - SD card timeout
//   0xfe - Unknown or bad SD/MMC card
//   0xfd - Set block size command failed
//   0xfc - bad response for CMD58
//   0xfb - SDv2 pattern mismatch (in response to CMD8)
uint8_t SD_CardInit(void) {
	uint32_t wait = 0, r3;
	uint8_t response;

	GPIO_WriteBit(SD_CS_PORT,SD_CS_PIN,Bit_RESET); // pull CS to low

	// Must send at least 74 clock ticks to SD Card
	for (wait = 0; wait < 8; wait++) SD_SendRecv(0xff);

	// Software SD Card reset
	wait = 0; response = 0x00;
	while (wait < 0x20 && response != 0x01) {
		// Wait for SD card enters idle state (R1 response = 0x01)
		response = SD_SendCmd(SD_CMD_GO_IDLE_STATE,0x00);
		wait++;
	}
	if (wait >= 0x20 && response != 0x01) return 0xff; // SD card timeout

	// CMD8: SEND_IF_COND. Send this command to verify SD card interface operating condition
    /* Argument: - [31:12]: Reserved (shall be set to '0')
	             - [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
	             - [7:0]: Check Pattern (recommended 0xAA) */
	response = SD_SendCmd(SD_CMD_HS_SEND_EXT_CSD,SD_CHECK_PATTERN); // CMD8

	if (response == 0x01) {
		// SDv2 or later
		// Read R7 responce
		r3 =  SD_SendRecv(0xff) << 24;
		r3 |= SD_SendRecv(0xff) << 16;
		r3 |= SD_SendRecv(0xff) << 8;
		r3 |= SD_SendRecv(0xff);

		if ((r3 & 0x01ff) != (SD_CHECK_PATTERN & 0x01ff)) return 0xfb; // SDv2 pattern mismatch -> unsupported SD card

		// CMD55: Send leading command for ACMD<n> command.
		// CMD41: APP_SEND_OP_COND. For only SDC - initiate initialization process.
		wait = 0; response = 0xff;
		while (++wait < 0x2710 && response != 0x00) {
			SD_SendCmd(SD_CMD_APP_CMD,0); // CMD55
			response = SD_SendCmd(SD_CMD_SD_APP_OP_COND,0x40000000); // ACMD41: HCS flag set
		}
		if (wait >= 0x2710 || response != 0x00) return 0xff; // SD card timeout

		SD_CardType = SD_STD_CAPACITY_SD_CARD_V2_0; // SDv2;

		// Read OCR register
		response = SD_SendCmd(SD_CMD_READ_OCR,0x00000000); // CMD58
		if (response == 0x00) {
			// Get R3 response
			r3  = SD_SendRecv(0xff) << 24;
			r3 |= SD_SendRecv(0xff) << 16;
			r3 |= SD_SendRecv(0xff) << 8;
			r3 |= SD_SendRecv(0xff);
		} else {
			SD_CardType = SD_UNKNOWN_SD_CARD;
			return 0xfc; // bad CMD58 response
		}
		if (r3 & (1<<30)) SD_CardType = SD_HIGH_CAPACITY_SD_CARD; // SDHC or SDXC
	} else {
		// SDv1 or MMC
		wait = 0; response = 0xff;
		while (++wait < 0xfe) {
			SD_SendCmd(SD_CMD_APP_CMD,0); // CMD55
			response = SD_SendCmd(SD_CMD_SD_APP_OP_COND,0x00000000); // CMD41
			if (response == 0x00) {
				SD_CardType = SD_STD_CAPACITY_SD_CARD_V1_0; // SDv1
				break;
			}
		}

		if (response == 0x05 && wait >= 0xfe) {
			// MMC or bad card
			// CMD1: Initiate initialization process.
			wait = 0; response = 0xff;
			while (++wait < 0xfe) {
				response = SD_SendCmd(SD_CMD_SEND_OP_COND,0x00000000); // CMD1
				if (response == 0x00) {
					SD_CardType = SD_MULTIMEDIA_CARD; // MMC
					break;
				}
			}
		}
	}

	if (SD_CardType == 0) return 0xfe; // Unknown or bad SD/MMC card

	// Set SPI to higher speed
	SD_SPI_Init(SPI_BaudRatePrescaler_8);

	// Turn off CRC
	response = SD_SendCmd(SD_CMD_CRC_ON_OFF,0x00000001); // CMD59

	// For SDv2,SDv1,MMC must set block size. For SDHC/SDXC it fixed to 512.
	if ((SD_CardType == SD_STD_CAPACITY_SD_CARD_V1_0) ||
		(SD_CardType == SD_STD_CAPACITY_SD_CARD_V2_0) ||
		(SD_CardType == SD_MULTIMEDIA_CARD))
	{
		response = SD_SendCmd(SD_CMD_SET_BLOCKLEN,0x00000200); // CMD16: block size = 512 bytes
		if (response != 0x00) return 0xfd; // Set block size failed
	}

	GPIO_WriteBit(SD_CS_PORT,SD_CS_PIN,Bit_SET); // pull CS to high

	return 0;
}

// return:
// 0x00 -- read OK
// 0x01..0xfe -- error response to CMD9
// 0xff -- timeout
uint8_t SD_Read_CSD(void) {
	uint32_t wait;
	uint8_t i, response;

	GPIO_WriteBit(SD_CS_PORT,SD_CS_PIN,Bit_RESET); // pull CS to low

	response = SD_SendCmd(SD_CMD_SEND_CSD,0); // CMD9
	if (response != 0x00) {
		// Something wrong happened, fill buffer with zeroes
		for (i = 0; i < 16; i++) SD_CSD[i] = 0x00;
		return response;
	} else {
		wait = 0; response = 0;
		while (++wait <= 0x1ff && response != 0xfe)	response = SD_SendRecv(0xff);
		if (wait >= 0x1ff) return 0xff;
		// Read 16 bytes of CSD register
		for (i = 0; i < 16; i++) SD_CSD[i] = SD_SendRecv(0xff);
	}

	// Receive 16-bit CRC (some cards demand this)
	SD_CRC16_rcv  = SD_SendRecv(0xff) << 8;
	SD_CRC16_rcv |= SD_SendRecv(0xff);

	// // Calculate CRC16 of received buffer
	SD_CRC16_cmp = CRC16_buf(&SD_CSD[0],16);

	GPIO_WriteBit(SD_CS_PORT,SD_CS_PIN,Bit_SET); // pull CS to high

	// Parse some stuff from CID
	SD_MaxBusClkFreq = SD_CSD[3];
	uint32_t c_size,c_size_mult;
	if (SD_CardType != SD_MULTIMEDIA_CARD) {
		if (SD_CSD[0] >> 6 == 0x01) {
			// CSD Version 2.0
			c_size  = (SD_CSD[7] & 0x3f) << 16;
			c_size |= SD_CSD[8] << 8;
			c_size |= SD_CSD[7];
			SD_CardCapacity = c_size << 9; // = c_size * 512
		} else {
			// CSD Version 1.0
			c_size  = (SD_CSD[6] & 0x03) << 10;
			c_size |= (uint32_t)SD_CSD[7] << 2;
			c_size |= SD_CSD[8] >> 6;
			c_size_mult  = (SD_CSD[9] & 0x03) << 1;
			c_size_mult |= SD_CSD[10] >> 7;
			SD_CardCapacity = c_size << 10;
		}
	} else {
		SD_CardCapacity = 0;
	}

	return 0;
}

// return:
// 0x00 -- read OK
// 0x01..0xfe -- error response to CMD10
// 0xff -- timeout
uint8_t SD_Read_CID(void) {
	uint32_t wait;
	uint8_t i, response;

	GPIO_WriteBit(SD_CS_PORT,SD_CS_PIN,Bit_RESET); // pull CS to low

	response = SD_SendCmd(SD_CMD_SEND_CID,0); // CMD10
	if (response != 0x00) {
		// Something wrong happened, fill buffer with zeroes
		for (i = 0; i < 16; i++) SD_CID[i] = 0x00;
		return response;
	} else {
		wait = 0; response = 0;
		while (++wait <= 0x1ff && response != 0xfe)	response = SD_SendRecv(0xff);
		if (wait >= 0x1ff) return 0xff;
		// Read 16 bytes of CID register
		for (i = 0; i < 16; i++) SD_CID[i] = SD_SendRecv(0xff);
	}

	// Receive 16-bit CRC (some cards demand this)
	SD_CRC16_rcv  = SD_SendRecv(0xff) << 8;
	SD_CRC16_rcv |= SD_SendRecv(0xff);

	// // Calculate CRC16 of received buffer
	SD_CRC16_cmp = CRC16_buf(&SD_CID[0],16);

	GPIO_WriteBit(SD_CS_PORT,SD_CS_PIN,Bit_SET); // pull CS to high

	return 0;
}

// return:
// 0x00 -- read OK
// 0x01..0xfe -- error response from CMD17
// 0xff -- timeout
uint8_t SD_Read_Block(uint32_t addr) {
	uint32_t wait;
	uint16_t i;
	uint8_t response;

	GPIO_WriteBit(SD_CS_PORT,SD_CS_PIN,Bit_RESET); // pull CS to low

	if (SD_CardType != SD_HIGH_CAPACITY_SD_CARD) addr <<= 9; // Convert block number to byte offset
	response = SD_SendCmd(SD_CMD_READ_SINGLE_BLOCK,addr); // CMD17
	if (response != 0x00) {
		// Something wrong happened, fill buffer with zeroes
		for (i = 0; i < 512; i++) SD_sector[i] = 0;
		return response; // SD_CMD_READ_SINGLE_BLOCK command returns bad response
	} else {
		wait = 0; response = 0;
		while (++wait <= 0x1ff && response != 0xfe)	response = SD_SendRecv(0xff);
		if (wait >= 0x1ff) return 0xff;
		// Read 512 bytes of sector
		for (i = 0; i < 512; i++) SD_sector[i] = SD_SendRecv(0xff);
	}

	// Receive 16-bit CRC (some cards demand this)
	SD_CRC16_rcv  = SD_SendRecv(0xff) << 8;
	SD_CRC16_rcv |= SD_SendRecv(0xff);

	// // Calculate CRC16 of received buffer
	SD_CRC16_cmp = CRC16_buf(&SD_sector[0],512);

	GPIO_WriteBit(SD_CS_PORT,SD_CS_PIN,Bit_SET); // pull CS to high

	return 0;
}
