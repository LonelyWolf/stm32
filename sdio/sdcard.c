#include <sdcard.h>


static uint8_t SD_CardType = SD_UNKNOWN_SD_CARD;
static uint8_t CSD_tab[16];     // CSD buffer
static uint8_t CID_tab[16];     // CID buffer
static uint8_t sector[512];     // sector buffer
static uint8_t SD_CardSize = 0; // Card capacity


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
	int j;

	uint8_t crc = 0;
	for (j = 0; j < len; j++) crc = CRC7_one(crc,p[j]);

	return crc >> 1;
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
	SPI_Init(SPI1,&SPI);
	// NSS must be set to '1' due to NSS_Soft settings (otherwise it will be Multimaster mode).
	SPI_NSSInternalSoftwareConfig(SPI1,SPI_NSSInternalSoft_Set);
}

void SD_Init(void) {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOA,ENABLE);
	// Configure SPI1 pins (PA5 = SCK, PA6 = MISO, PA7 = MOSI)
	GPIO_InitTypeDef PORT;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	PORT.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7; // SCK,MISO,MOSI AF with PP
	PORT.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&PORT);
	// Configure CS (PA3) pin for output with Push-Pull
	PORT.GPIO_Pin = GPIO_Pin_3;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA,&PORT);

	GPIO_WriteBit(GPIOA,GPIO_Pin_3,Bit_SET); // pull SD pin -> Deselect SD

	SD_SPI_Init(SPI_BaudRatePrescaler_256); // set SPI at lowest possible speed
	SPI_Cmd(SPI1,ENABLE);
}

uint8_t SD_SendRecv(uint8_t data) {
	uint8_t miso;

	while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1,data);
	while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE) == RESET);
	miso = SPI_I2S_ReceiveData(SPI1);

	return miso;
}

uint8_t SD_SendCmd(uint8_t cmd, uint32_t arg) {
	uint8_t wait = 0, response, crc = 0;

	SD_SendRecv(0xff); // This dummy send necessary for some cards

	/* NOTE: for SDHC address must be corrected from byte to block! */

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
	uint32_t wait = 0,r3;
	uint8_t response;

	GPIO_WriteBit(GPIOA,GPIO_Pin_3,Bit_RESET); // pull CS to low

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

		if ((r3 & 0x01ff) != 0x01aa) return 0xfb; // SDv2 pattern mismatch -> unsupported SD card

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
		//wait = 0; response = 0x00;
		//while (++wait < 0xfe && response != 0x00) response = SD_SendCmd(SD_CMD_READ_OCR,0x00000000); // CMD58
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

	GPIO_WriteBit(GPIOA,GPIO_Pin_3,Bit_SET); // pull CS to high

	return 0;
}

uint8_t SD_GetVersion(void) {
	return SD_CardType;
}

uint8_t* SD_Read_CSD(void) {
	uint8_t i, response;

	GPIO_WriteBit(GPIOA,GPIO_Pin_3,Bit_RESET); // pull CS to low

	response = SD_SendCmd(SD_CMD_SEND_CSD,0); // CMD9
	if (response != 0x00) {
		// Something wrong happened, fill buffer with zeroes
		for (i = 0; i < 16; i++) CSD_tab[i] = response;
	} else {
		// Read 16 bytes of CSD register
		for (i = 0; i < 16; i++) CSD_tab[i] = SD_SendRecv(0xff);
	}

	GPIO_WriteBit(GPIOA,GPIO_Pin_3,Bit_SET); // pull CS to high

	return &CSD_tab[0];
}

uint8_t* SD_Read_CID(void) {
	uint8_t i, response;

	GPIO_WriteBit(GPIOA,GPIO_Pin_3,Bit_RESET); // pull CS to low

	response = SD_SendCmd(SD_CMD_SEND_CID,0); // CMD10
	if (response != 0x00) {
		// Something wrong happened, fill buffer with zeroes
		for (i = 0; i < 16; i++) CID_tab[i] = response;
	} else {
		// Read 16 bytes of CID register
		for (i = 0; i < 16; i++) CID_tab[i] = SD_SendRecv(0xff);
	}

	GPIO_WriteBit(GPIOA,GPIO_Pin_3,Bit_SET); // pull CS to high

	return &CID_tab[0];
}

uint8_t* SD_Read_Block(uint32_t addr) {
	uint16_t i;
	uint8_t response;

	GPIO_WriteBit(GPIOA,GPIO_Pin_3,Bit_RESET); // pull CS to low

	response = SD_SendCmd(SD_CMD_READ_SINGLE_BLOCK,addr); // CMD17
	if (response != 0x00) {
		// Something wrong happened, fill buffer with zeroes
		for (i = 0; i < 512; i++) sector[i] = 0;
	} else {
		// Read 512 bytes of sector
		for (i = 0; i < 512; i++) sector[i] = SD_SendRecv(0xff);
	}

	GPIO_WriteBit(GPIOA,GPIO_Pin_3,Bit_SET); // pull CS to high

	return &sector[0];
}
