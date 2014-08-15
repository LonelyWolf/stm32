#include <string.h>

#include <dosfs/dosfs.h>
#include <uart.h>
#include <wolk.h>
#include <log.h>


#define LOG_DATA_BUF_SIZE    SECTOR_SIZE // Size of data buffer


VOLINFO vol_info; // Volume information
DIRINFO dir_info; // Directory information
uint8_t sector[SECTOR_SIZE]; // Buffer to store sector from SD card
uint32_t pstart; // Partition start sector
FILEINFO log_file; // Current log file handler
uint8_t log_data[LOG_DATA_BUF_SIZE]; // Buffer for data to write
uint32_t log_data_pos; // Position in data buffer


uint32_t fn_atoi(char *filename) {
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

void fn_itoa(uint32_t num, char *filename) {
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
	uint32_t result;

	pstart = DFS_GetPtnStart(0,sector,0,NULL,NULL,NULL);
	if (pstart == DFS_ERRMISC) return LOG_NOPARTITION; // Partition not found

	// Get volume information
	if (DFS_GetVolInfo(0,sector,pstart,&vol_info)) return LOG_VIERROR;

	dir_info.scratch = sector;
	result = DFS_OpenDir(&vol_info,(uint8_t *)LOG_DIR_LOGS,&dir_info);
	if (result == DFS_NOTFOUND) {
		// LOGS directory not found, try to create it
		result = DFS_OpenFile(&vol_info,(uint8_t *)LOG_DIR_LOGS,DFS_CREATEDIR,sector,&log_file);
	};
	if (result != DFS_OK) return LOG_ROOTERROR;

	// Looks like everything went good
	return LOG_OK;
}

// Create new log file
// return: new log file number or LOG_ERROR in case of error
uint32_t LOG_NewFile(void) {
	DIRENT dir_entry; // Directory entry
	char filename[12]; // Buffer for directory entry
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
			DFS_DirToCanonical((uint8_t *)filename,dir_entry.name);
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
	strcat((char *)path,filename);
	i = DFS_OpenFile(&vol_info,(uint8_t *)path,DFS_WRITE,sector,&log_file);
	if (i != DFS_OK) return LOG_CREATEERROR;

	// Clear data buffer
	memset(log_data,0,sizeof(log_data));
	log_data_pos = 0;

	return log_num;
}

// Write data buffer to SD card
// return: number of bytes written
uint32_t LOG_FileSync(void) {
	uint32_t cache = 0;

	// TODO: check for DFS_WriteFile() error
	DFS_WriteFile(&log_file,sector,log_data,&cache,log_data_pos);
	log_data_pos = 0;

	return cache;
}

// Write binary data to data buffer
// input:
//   buf - pointer to the buffer with binary data
//   len - length of the buffer
// return: number of bytes copied into data buffer
uint32_t LOG_WriteBin(uint8_t *buf, uint32_t len) {
	uint32_t part_len;

    if (log_data_pos + len >= LOG_DATA_BUF_SIZE) {
    	// Copy part of data into data buffer and write it to SD card
		part_len = LOG_DATA_BUF_SIZE - log_data_pos;
		memcpy(&log_data[log_data_pos],buf,part_len);
		log_data_pos += part_len;
		LOG_FileSync(); // FIXME: check for error here
		// Copy rest of data into data buffer
		len -= part_len;
		memcpy(&log_data[log_data_pos],&buf[part_len],len);
		log_data_pos += len;
	} else {
		// Copy data into data buffer
		memcpy(&log_data[log_data_pos],buf,len);
		log_data_pos += len;
	}

	return len;
}

// Write string to data buffer
// input:
//   str - pointer to the null-terminated string
// return: number of bytes copied into data buffer
// note: string length must not exceed LOG_DATA_BUF_SIZE
uint32_t LOG_WriteStr(char *str) {
	uint32_t len;

	len = strlen(str);
	len = LOG_WriteBin((uint8_t *)str,len);

	return len;
}

// Write signed 32-bit integer value to data buffer
// input:
//   num - integer value to write
// return: number of bytes copied into data buffer
uint32_t LOG_WriteInt(int32_t num) {
	uint32_t len;
	uint8_t txt[10];
	uint8_t neg;
	uint8_t i;

	memset(&txt,0,10);
	len = numlen(num);
	i = len - 1;
	if (num < 0) {
		neg = 1;
		num *= -1;
	} else neg = 0;
	do txt[i--] = num % 10 + '0'; while ((num /= 10) > 0);
	if (neg) txt[i--] = '-';

	len = LOG_WriteBin(txt,len);

	return len;
}
