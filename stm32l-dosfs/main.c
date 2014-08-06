///////////////////
// STM32L151RBT6 //
///////////////////


#include <misc.h>
#include <stm32l1xx_rcc.h>
#include <stm32l1xx_gpio.h>
#include <stm32l1xx_tim.h>
#include <stm32l1xx_syscfg.h>
#include <string.h> // For memset, memmove

#include <delay.h>
#include <spi.h>
#include <spi1.h>
#include <uart.h>
#include <uc1701.h>
#include <sdcard.h>
#include <log.h>
#include <wolk.h>

#include <font5x7.h>
#include <font7x10.h>

#include <dosfs/dosfs.h>


uint32_t i,j;

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



int main(void) {
	SystemCoreClockUpdate();

	UART2_Init(921600);

	Delay_Init(NULL);

	SPI2_Init();

	UC1701_Init();
	UC1701_Contrast(4,24);
	UC1701_Orientation(scr_normal);
	UC1701_SetBacklight(0);
	UC1701_Fill(0x00);
	PutStr(0,0,"SD card...",fnt7x10);
	UC1701_Flush();

	SPI1_Init();

	UC1701_Fill(0x00);
	i = PutStr(0,0,"SD init:",fnt5x7) - 1;
	j = (uint32_t)SD_Init();
	i += PutHex(i,0,j,fnt5x7) + 10;
	UC1701_Flush();
	if (j == SDR_Success) {
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
		if (j != LOG_OK) {
			UART_SendStr("LOG_Init() error:");
			UART_SendInt(j);
			UART_SendChar('\n');
			while(1);
		}
//		while(1);

		uint32_t k = 0;

		for (; k < 500; k++) {
			j = LOG_NewFile();
			UART_SendStr("Log number: ");
			UART_SendInt(j);
			UART_SendChar('\n');

			j = 10;
			for (i = 0; i < j; i++) {
				if (!(i % 25)) {
					UART_SendStr("write... ");
					UART_SendInt(i);
					UART_SendChar('\\');
					UART_SendInt(j);
					UART_SendChar('\n');
				}
				LOG_WriteInt(-100 * (i + 1));
				LOG_WriteStr(" of ");
				LOG_WriteInt(-100 * j);
				LOG_WriteStr(" checkpoint!\r\n");
	//			Delay_ms(50);
			}
		}

		UART_SendStr("---------------------------\n");
		UART_SendStr("---  LOG PROCEDURES END ---\n");
		UART_SendStr("---------------------------\n");
		Delay_ms(100);

		if (SD_ReadBlock(0,sector,SECTOR_SIZE) != SDR_Success) {
			UART_SendStr("Error reading sector 0\n");
		} else {
			if ((sector[0x1FE] == 0x55) && (sector[0x1FF] == 0xAA)) {
				UART_SendStr("Sector 0 contains FAT or MBR delimiter\n");
				if (((sector[0x36] << 16) | (sector[0x37] << 8) | sector[0x38]) == 0x464154) {
					UART_SendStr("FAT12/16 header\n");
					pstart = 0;
					pactive = 0x80; // Assume what partition active
					ptype = 0x06; // Assume what partition type is FAT16
					psize = 0xFFFFFFFF;
				} else if (((sector[0x52] << 16) | (sector[0x53] << 8) | sector[0x54]) == 0x464154) {
					UART_SendStr("FAT32 header\n");
					pstart = 0;
					pactive = 0x80; // Assume what partition active
					ptype = 0x0b; // Assume what partition type is FAT32 with CHS
					psize = 0xFFFFFFFF;
				} else {
					// Sector 0 is MBR, find partition start
					pstart = DFS_GetPtnStart(0,sector,0,&pactive,&ptype,&psize);
				}

				if (pstart == 0xffffffff) {
					UART_SendStr("Cannot find first partition\n");
				} else {
					UART_SendStr("Partition start sector: ");
					UART_SendHex32(pstart);
					UART_SendChar('\n');
					UART_SendStr("Active: ");
					UART_SendHex32(pactive);
					UART_SendChar('\n');
					UART_SendStr("Type: ");
					UART_SendHex32(ptype);
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
//					if (DFS_OpenDir(&vi,(uint8_t *)"",&di)) {
						UART_SendStr("Error opening root directory\n");
					} else {
						UART_SendStr("Directory enumeration:\n");
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
						UART_SendStr("End of enumeration\n");
					}

/*
// FILE WRITE
					if (DFS_OpenFile(&vi,(uint8_t *)"LOGS/TEST.TXT",DFS_WRITE,sector,&fi)) {
						UART_SendStr("Error opening file TEST.TXT\n");
					} else {
						for (j = 0; j < SECTOR_SIZE; j++) transferbuffer[j] = j >> 1;
						for (j = 0; j < 100; j++) {
							DFS_WriteFile(&fi,sector,transferbuffer,&cache,SECTOR_SIZE);
							if (j % 5 == 0) {
								FillRect(0,24,scr_width - 1,scr_height - 1,PReset);
								PutInt(0,24,j,fnt7x10);
								UC1701_Flush();
							}
						}
					}
*/

/*
// FILE READ
					UART_SendStr("=====Read file\n");
					if (DFS_OpenFile(&vi,(uint8_t *)"BLUE.MP3",DFS_READ,sector,&fi)) {
						UART_SendStr("Error opening file\n");
					} else {
						i = 0;
						do {
							j = DFS_ReadFile(&fi,sector,transferbuffer,&i,512);
//							UART_SendStr("DFS_ReadFile()=");
//							UART_SendInt(j);
//							UART_SendStr(", read: ");
//							UART_SendInt(i);
//							UART_SendStr("bytes (expected ");
//							UART_SendInt(fi.filelen);
//							UART_SendStr("), pointer ");
//							UART_SendInt(fi.pointer);
//							UART_SendChar('\n');
//							for (j = 0; j < i; j++) UART_SendChar(transferbuffer[j]);
//							j = DFS_OK;
//							UART_SendBufHexFancy((char *)&transferbuffer[0],i,32,'.');
//							UART_SendChar('\n');
						} while (j == DFS_OK && fi.pointer != fi.filelen);
						UART_SendStr("=====EOF");
						UART_SendChar('\n');
					}
*/
		}
			}
		}
	}

	UART_SendStr("--------- It's done! ---------\n");
	while(1);
}
