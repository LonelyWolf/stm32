///////////////////
// STM32L151RBT6 //
///////////////////


#include <misc.h>
#include <stm32l1xx_rcc.h>
#include <stm32l1xx_gpio.h>
#include <stm32l1xx_syscfg.h>
#include <string.h> // For memset, memmove

#include <wolk.h>
#include <delay.h>
#include <spi.h>
#include <uart.h>
#include <uc1701.h>
#include <sdcard.h>
#include <log.h>
#include <RTC.h>

#include <font5x7.h>
#include <font7x10.h>

#include <dosfs/dosfs.h>


uint32_t i,j,k;

uint8_t sector[SECTOR_SIZE];
uint8_t transferbuffer[SECTOR_SIZE];
uint32_t pstart, psize;
uint8_t pactive, ptype;
VOLINFO vi;
DIRINFO di;
DIRENT de;
uint32_t cache;
FILEINFO fi;
uint8_t *p;

typedef struct {
	uint32_t v1;
	uint16_t v2;
	uint8_t  v3;
} theStruct;

theStruct ts;


int main(void) {
	SystemCoreClockUpdate();

	UART2_Init(1382400); // Hell yeah, it works!

	Delay_Init(NULL);

	SPIx_Init(SPI1);
	SPIx_Init(SPI2);

	RTC_Config();

	UC1701_Init();
	UC1701_Contrast(4,24);
	UC1701_Orientation(scr_normal);
	UC1701_SetBacklight(0);
	UC1701_Fill(0x00);
	PutStr(0,0,"SD card...",fnt7x10);
	UC1701_Flush();

	UC1701_Fill(0x00);
	i = PutStr(0,0,"SD init:",fnt5x7) - 1;
	j = (uint32_t)SD_Init();
	i += PutHex(i,0,j,fnt5x7) + 10;
	UC1701_Flush();

	UART_SendStr("SD init: ");
	UART_SendHex8(j);
	UART_SendChar('\n');

	if (j != SDR_Success) while(1);

	j = SD_ReadCSD();
	UART_SendStr("CSD: ");
	if (j == SDR_Success) {
		UART_SendBufHex((char *)SDCard.CSD,16);
	} else {
		UART_SendHex8(j);
	}
	UART_SendChar('\n');

	j = SD_ReadCID();
	UART_SendStr("CID: ");
	if (j == SDR_Success) {
		UART_SendBufHex((char *)SDCard.CID,16);
	} else {
		UART_SendHex8(j);
	}
	UART_SendChar('\n');

	UART_SendStr("Card capacity = ");
	UART_SendInt(SDCard.CardCapacity);
	UART_SendChar('\n');
	UART_SendStr("Card block size = ");
	UART_SendInt(SDCard.CardBlockSize);
	UART_SendChar('\n');

/*
	j = (uint32_t)SD_ReadBlock(111,sector,512);
	UART_SendStr("SD Read: ");
	UART_SendHex8(j);
	UART_SendChar('\n');
	if (j == SDR_Success) UART_SendBufHexFancy((char *)sector,512,32,'.');

	for (j = 0; j < 512; j++) sector[j] = j >> 1;
	j = (uint32_t)SD_WriteBlock(111,sector,512);
	UART_SendStr("SD Write: ");
	UART_SendHex8(j);
	UART_SendChar('\n');
*/

	//while(1);

	i += PutStr(i,0,"Type:",fnt5x7) - 1;
	PutInt(i,0,(uint32_t)SDCard.CardType,fnt5x7);
	UC1701_Flush();
	i = PutStr(0,8,"Capacity:",fnt5x7) - 1;
	if (SDCard.CardType == SDCT_SDHC) {
		i += PutInt(i,8,SDCard.CardCapacity / 1024,fnt5x7);
	} else {
		i += PutInt(i,8,SDCard.CardCapacity / 1048576,fnt5x7);
	}
	PutStr(i,8,"Mb",fnt5x7);
	UC1701_Flush();
	i = PutStr(0,16,"Bus:",fnt5x7) - 1;
    i += PutInt(i,16,SDCard.CardMaxBusClkFreq,fnt5x7) + 6;
	i += PutStr(i,16,"Block:",fnt5x7) - 1;
	PutInt(i,16,SDCard.CardBlockSize,fnt5x7);
	UC1701_Flush();

	UART_SendStr("DOSFS in action...\n");

	j = LOG_Init();
	UART_SendStr("LOG_Init(): ");
	UART_SendInt(j);
	UART_SendChar('\n');
	if (j != LOG_OK) while(1);

	uint32_t cntr = 0;

	// Create files for test
	for (k = 0; k < 1; k++) {
		// ------ Text file
		j = LOG_NewFile(&cntr);

		UART_SendStr("LOG_NewFile(): ");
		UART_SendInt(j);
		UART_SendChar('\n');
		UART_SendStr("Log number: ");
		UART_SendInt(cntr);
		UART_SendChar('\n');

		// Write header to the log file
		LOG_WriteStr("Wolk Bike Computer log file\r\n");
		LOG_WriteStr("--- BEGIN ---\r\n");

		cntr = 0;

		j = 500;
		for (i = 0; i < j; i++) {
			if (!(i % 50)) {
				UART_SendStr("write ... ");
				UART_SendInt(i);
				UART_SendChar('\\');
				UART_SendInt(j);
				UART_SendChar('\n');
			}
			LOG_WriteStr("Checkpoint #");
			LOG_WriteInt(i + 1);
			LOG_WriteStr(" of ");
			LOG_WriteInt(j);
			LOG_WriteStr("\r\n");

			cntr++;
//				cntr = 4294967295;
			LOG_WriteIntU(cntr);
			LOG_WriteStr(" ");
			LOG_WriteIntF(cntr,1);
			LOG_WriteStr(" ");
			LOG_WriteIntF(cntr,2);
			LOG_WriteStr(" ");
			LOG_WriteIntF(cntr,3);
			LOG_WriteStr(" ");
			LOG_WriteIntF(cntr,4);
			LOG_WriteStr(" ");
			LOG_WriteIntF(cntr,5);
			LOG_WriteStr(" ");
			LOG_WriteIntF(cntr,6);
			LOG_WriteStr(" ");
			LOG_WriteIntF(cntr,7);
			LOG_WriteStr(" ");
			LOG_WriteIntF(cntr,8);
			LOG_WriteStr(" ");
			LOG_WriteIntF(cntr,9);
			LOG_WriteStr("\r\n");
		}

		// Write file ending
		LOG_WriteStr("---- EOF ----\r\n");

		// Save data buffer to SD card
		LOG_FileSync();

		// ------ Binary file
		cntr = 0;
		j = LOG_NewFile(&cntr);

		UART_SendStr("LOG_NewFile(): ");
		UART_SendInt(j);
		UART_SendChar('\n');
		UART_SendStr("Log number: ");
		UART_SendInt(cntr);
		UART_SendChar('\n');

		// Write header to the log file
		LOG_WriteStr("WBC!");

		j = 500;
		for (i = 0; i < j; i++) {
			if (!(i % 50)) {
				UART_SendStr("write ... ");
				UART_SendInt(i);
				UART_SendChar('\\');
				UART_SendInt(j);
				UART_SendChar('\n');
			}
			ts.v1 = i + j;
			ts.v2 = i;
			ts.v3 = j;
			LOG_WriteBin((uint8_t *)&i,sizeof(i));
			LOG_WriteBin((uint8_t *)&j,sizeof(j));
			LOG_WriteBin((uint8_t *)&ts,sizeof(ts));
		}

		// Write file ending
		LOG_WriteStr("!WBC");

		// Save data buffer to SD card
		LOG_FileSync();
	}

	UART_SendStr("--- File write end ---\n");





	// Find partition start
	pstart = DFS_GetPtnStart(0,sector,0,&pactive,&ptype,&psize);
	if (pstart == DFS_ERRMISC) {
		UART_SendStr("Cannot find first partition\n");
	} else {
		UART_SendStr("Partition start: ");
		UART_SendHex32(pstart);
		UART_SendChar('\n');
		UART_SendStr("Active: ");
		UART_SendHex8(pactive);
		UART_SendChar('\n');
		UART_SendStr("Type: ");
		UART_SendHex8(ptype);
		UART_SendChar('\n');
		UART_SendStr("Size: ");
		UART_SendHex32(psize);
		UART_SendChar('\n');
	}

	if (DFS_GetVolInfo(0,sector,pstart,&vi)) {
		UART_SendStr("Error getting volume information\n");
	} else {
		UART_SendStr("Volume label: ");
		UART_SendStr((char *)vi.label);
		UART_SendChar('\n');
		UART_SendStr("File system: ");
		if (vi.filesystem == FAT12)
			UART_SendStr("FAT12\n");
		else if (vi.filesystem == FAT16)
			UART_SendStr("FAT16\n");
		else if (vi.filesystem == FAT32)
			UART_SendStr("FAT32\n");
		else
			UART_SendStr("[unknown]\n");

		di.scratch = sector;
		if (DFS_OpenDir(&vi,(uint8_t *)LOG_DIR_LOGS,&di)) {
			UART_SendStr("Error opening root directory\n");
		} else {
			while (!DFS_GetNext(&vi, &di, &de)) {
				if (de.name[0]) {
					UART_SendStr("file: ");
					UART_SendBuf((char *)de.name,11); // There is no zero terminator in name
					UART_SendChar('\t');
					UART_SendHex8(de.attr);
					UART_SendChar('\t');
					UART_SendInt((de.filesize_3 << 24) | (de.filesize_2 << 16) | (de.filesize_1 << 8) | de.filesize_0);
					UART_SendChar('\n');
				}
			}
		}
	}

	UART_SendStr("--------- It's done! ---------\n");

	while(1) {
		UC1701_SetBacklight(25);
		Delay_ms(1000);
		UC1701_SetBacklight(0);
		Delay_ms(1000);
	}
}
