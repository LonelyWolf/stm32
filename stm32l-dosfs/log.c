#include <string.h>

#include <dosfs/dosfs.h>
#include <uart.h>
#include <wolk.h>
#include <log.h>


VOLINFO vol_info; // Volume information
DIRINFO dir_info; // Directory information
uint8_t sector[SECTOR_SIZE]; // Buffer to store sector from SD card
uint32_t pstart; // Partition start sector
FILEINFO log_file; // Current log file handler


uint32_t fn_atoi(uint8_t *filename) {
	uint8_t ch;
	uint32_t result = 0;

	ch = *filename++;
	while (ch >= '0' && ch <= '9') {
		result += ch - '0';
		result *= 10;
		ch = *filename++;
	}
	result /= 10;

	return result;
}

void fn_itoa(uint32_t num, uint8_t *filename) {
	uint8_t i = 7;

	memcpy(filename,LOG_FILENAME_TEMPLATE,sizeof(LOG_FILENAME_TEMPLATE));
	while (i > 2 && num > 0) {
		filename[i--] = (num % 10) + '0';
		num /= 10;
	}
}

// Read first sector from SD card and find first partition entry
// return: LOG_Result (0 if everything went good)
// note: LOG_Init() must be called before calling any other routines
LOG_Result LOG_Init(void) {
	uint32_t i;

	if (SD_ReadBlock(0,sector,SECTOR_SIZE) == SDR_Success) {
		if ((sector[0x1FE] == 0x55) && (sector[0x1FF] == 0xAA)) {
			// Sector 0 contains FAT or MBR delimiter
			if (((sector[0x36] << 16) | (sector[0x37] << 8) | sector[0x38]) == 0x464154) {
				// This is FAT12 or FAT16 header
				pstart = 0;
			} else if (((sector[0x52] << 16) | (sector[0x53] << 8) | sector[0x54]) == 0x464154) {
				// This is FAT32 header
				pstart = 0;
			} else {
				// Sector 0 is MBR, find partition start
				pstart = DFS_GetPtnStart(0,sector,0,NULL,NULL,NULL);
			}

			if (pstart == 0xffffffff) {
				// Partition not found
				return LOG_NOPARTITION;
			}

			if (DFS_GetVolInfo(0,sector,pstart,&vol_info)) {
				// Error getting volume information
				return LOG_VIERROR;
			}

			dir_info.scratch = sector;
			i = DFS_OpenDir(&vol_info,(uint8_t *)LOG_DIR_LOGS,&dir_info);
			if (i == DFS_NOTFOUND) {
				// LOGS directory not found, try to create it
				i = DFS_OpenFile(&vol_info,(uint8_t *)LOG_DIR_LOGS,DFS_CREATEDIR,sector,&log_file);
			};
			if (i != DFS_OK) return LOG_ROOTERROR;
		}
	} else return LOG_S0ERROR;

	// Looks like everything went good
	return LOG_OK;
}

// Create new log file
// return: new log file number or LOG_ERROR in case of error
uint32_t LOG_NewFile(void) {
	DIRENT dir_entry; // Directory entry
	uint8_t filename[12]; // Buffer for directory entry
	uint8_t path[64]; // Full file path
	uint32_t log_num = 0;
	uint32_t i;

	i = DFS_OpenDir(&vol_info,(uint8_t *)LOG_DIR_LOGS,&dir_info);
	if (i == DFS_NOTFOUND) {
		// Unable to open logs directory, try to create it
		// In fact, this should not be because the directory had to be created in LOG_Init()
		i = DFS_OpenFile(&vol_info,(uint8_t *)LOG_DIR_LOGS,DFS_CREATEDIR,sector,&log_file);
		if (i == DFS_OK) {
			// Open created directory
			i = DFS_OpenDir(&vol_info,(uint8_t *)LOG_DIR_LOGS,&dir_info);
		}
	}
	if (i != DFS_OK) return LOG_ERROR;

	// Enumerate *.LOG files and determine highest number
	while (!DFS_GetNext(&vol_info,&dir_info,&dir_entry)) {
		if (dir_entry.name[0] && !(dir_entry.attr & ATTR_DIRECTORY)) {
			DFS_DirToCanonical(filename,dir_entry.name);
			if (!memcmp(&filename[0],&LOG_FILENAME_TEMPLATE[0],3) &&
					!memcmp(&filename[8],&LOG_FILENAME_TEMPLATE[8],4)) {
				i = fn_atoi(&filename[3]);
				if (i > log_num) log_num = i;
			}
		}
	}

	// Create .LOG file with next number
	log_num++;
	fn_itoa(log_num,filename);
	strcpy((char *)path,"LOGS/");
	strcat((char *)path,(char *)filename);
	i = DFS_OpenFile(&vol_info,(uint8_t *)path,DFS_WRITE,sector,&log_file);
	if (i != DFS_OK) return LOG_CREATEERROR;

	// Write header to log file
	i = LOG_WriteStr("Wolk Bike Computer log file\r\n");

	return log_num;
}

// Write string to log file
// input:
//   str - pointer to the null-terminated string
// return: number of written bytes
// note: string length must not exceed SECTOR_SIZE
uint32_t LOG_WriteStr(char *str) {
	uint32_t cache;

	DFS_WriteFile(&log_file,sector,(uint8_t *)str,&cache,strlen(str));

	return cache;
}

// Write signed 32-bit integer value to log file
// input:
//   num - integer value to write
// return: number of written bytes
uint32_t LOG_WriteInt(int32_t num) {
	uint32_t cache;
	uint8_t txt[10];
	uint8_t neg;
	uint8_t i;

	memset(&txt,0,10);
	i = numlen(num) - 1;
	if (num < 0) {
		neg = 1;
		num *= -1;
	} else neg = 0;
	do txt[i--] = num % 10 + '0'; while ((num /= 10) > 0);
	if (neg) txt[i--] = '-';
	DFS_WriteFile(&log_file,sector,txt,&cache,strlen((char *)txt));

	return cache;
}
