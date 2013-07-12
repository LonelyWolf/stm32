#include <sdcard.h>


static uint8_t SD_Version = 0;


static uint8_t CRC7_one(uint8_t t, uint8_t data) {
	uint8_t i;
	const uint8_t g = 0x89;

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
	for ( j = 0; j < len; j++ ) crc = CRC7_one (crc, p[ j ]);

	return crc >> 1;
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

	SPI_InitTypeDef SPI;
	SPI.SPI_Mode = SPI_Mode_Master;
	SPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
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
	uint8_t wait = 0, responce, crc = 0;

	GPIO_WriteBit(GPIOA,GPIO_Pin_3,Bit_RESET); // pull CS to low

	SD_SendRecv(0xff);

	/* NOTE: for SDHC address must be corrected from byte to block! */

	// Send: [8b]Command -> [32b]Argument -> [8b]CRC
	SD_SendRecv(cmd | 0x40); // WTF 0x40?
	crc = CRC7_one(crc,cmd | 0x40);
	SD_SendRecv(arg >> 24);
	crc = CRC7_one(crc,arg >> 24);
	SD_SendRecv(arg >> 16);
	crc = CRC7_one(crc,arg >> 16);
	SD_SendRecv(arg >> 8);
	crc = CRC7_one(crc,arg >> 8);
	SD_SendRecv(arg);
	crc = CRC7_one(crc,arg);
	//SD_SendRecv(cmd != SD_CMD_HS_SEND_EXT_CSD ? 0x95 : 0x87);
	SD_SendRecv(crc | 0x01); // Bit 1 always must be set to "1"

	// Wait for responce from SD Card
	wait = 0;
	while ((responce = SD_SendRecv(0xff)) == 0xff) if (wait++ > 200) break;

	GPIO_WriteBit(GPIOA,GPIO_Pin_3,Bit_SET); // pull CS to high

	return responce;
}

uint8_t SD_CardInit(void) {
	uint32_t wait = 0;
	uint8_t responce;

	// Must send at least 74 clock ticks to SD Card
	GPIO_WriteBit(GPIOA,GPIO_Pin_3,Bit_RESET); // pull CS to low
	for (wait = 0; wait < 8; wait++) SD_SendRecv(0xff);
	GPIO_WriteBit(GPIOA,GPIO_Pin_3,Bit_SET); // pull CS to high

	// Software SD Card reset
	wait = 0; responce = 0x00;
	while (wait < 0x20 && responce != 0x01) {
		// Wait for SD card enters idle state (R1 responce = 0x01)
		responce = SD_SendCmd(SD_CMD_GO_IDLE_STATE,0x00);
		wait++;
	}
	if (wait >= 0x20 && responce != 0x01) return 0xff; // SD card timeout

	// CMD8: SEND_IF_COND. Send this command to verify SD card interface operating condition
    /* Argument: - [31:12]: Reserved (shall be set to '0')
	             - [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
	             - [7:0]: Check Pattern (recommended 0xAA) */
	wait = 0; responce = 0x00;
	while (wait++ < 0xfe && (responce != 0x01 && responce != 0x05)) responce = SD_SendCmd(SD_CMD_HS_SEND_EXT_CSD,SD_CHECK_PATTERN); // CMD8
	if (wait >= 0xfe) return 0xff; // SD card timeout

	if (responce == 0x01) SD_Version = 2; // SDv2 or later

	wait = 0; responce = 0xff;
	if (SD_Version == 2) {
		// CMD55: Send leading command for ACMD<n> command.
		// CMD41: APP_SEND_OP_COND. For only SDC - initiate initialization process.
		while (wait++ < 0x2710 && responce != 0x00) {
			SD_SendCmd(SD_CMD_APP_CMD,0); // CMD55
			responce = SD_SendCmd(SD_CMD_SD_APP_OP_COND,0x40000000); // CMD41
		}
		if (wait >= 0x2710) return 0xff; // SD card timeout

		// This is SDv2 or later
		// Read OCR register here?
	} else {
		while (wait++ < 0xfe) {
			SD_SendCmd(SD_CMD_APP_CMD,0); // CMD55
			responce = SD_SendCmd(SD_CMD_SD_APP_OP_COND,0x00000000); // CMD41
			if (responce == 0x00) {
				SD_Version = 1; // SDv1
				return 0;
			}
		}

		if (responce == 0x05 && wait >= 0xfe) {
			// MMC or bad card
			// CMD1: Initiate initialization process.
			wait = 0; responce = 0xff;
			while (++wait < 0xfe) {
				responce = SD_SendCmd(SD_CMD_SEND_OP_COND,0x00000000); // CMD1
				if (responce == 0x00) {
					SD_Version = 3; // MMC
					return 0;
				}
			}

			// WTF is here?
		}

		wait = responce;
/*
		// CMD55: Send leading command for any ACMD<n> command.
 		responce = SD_SendCmd(SD_CMD_APP_CMD,0); // CMD55
		wait = 0;
		if ((responce & 0x04) != 0) {
			// Invalid command return - this is MMC or bad card.
			// CMD1: Initiate initialization process.
			responce = 0xff;
			while (wait++ < 0xfe && responce != 0x00) responce = SD_SendCmd(SD_CMD_SEND_OP_COND,0x00000000); // CMD1
			if (wait >= 0xfe) return 0xff; // SD card timeout -> bad card or unknown type
			SD_Version = 3;
		} else {
			// CMD55: Send leading command for ACMD<n> command. (! already sent)
			// CMD41: APP_SEND_OP_COND. For only SDC - initiate initialization process.
			responce = 0xff;
			while (wait++ < 0xfe && responce != 0x00) {
				SD_SendCmd(SD_CMD_APP_CMD,0); // CMD55
				responce = SD_SendCmd(SD_CMD_SD_APP_OP_COND,0x00000000); // CMD41
			}
			if (wait >= 0xfe) return 0xff; // SD card timeout -> bad card or unknown type
			SD_Version = 1;
		}
*/
	}

	return responce;
}

uint8_t SD_GetVersion(void) {
	return SD_Version;
}
