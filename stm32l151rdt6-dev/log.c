#include <string.h>

#include "wolk.h"
#include "dosfs.h"
#include "log.h"


#define LOG_DATA_BUF_SIZE    SECTOR_SIZE    // Size of data buffer
                                            // for optimal performance must be equal to sector size


bool _SD_present;                           // TRUE if SD card present
bool _logging;                              // TRUE if log file was created

VOLINFO vol_info;                           // Volume information
DIRINFO dir_info;                           // Directory information
uint8_t sector[SECTOR_SIZE];                // Buffer to store one sector
uint32_t pstart;                            // Partition start sector
FILEINFO log_file;                          // Current log file handler
uint8_t log_data[LOG_DATA_BUF_SIZE];        // Buffer for data to write
uint32_t log_data_pos;                      // Position in data buffer


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

	_logging = FALSE;

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
// input:
//   pNum - pointer to variable to store log number
// return: LOG_XXX value (LOG_OK if file created)
// note: pNum changed only if new log file created
uint32_t LOG_NewFile(uint32_t *pNum) {
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

	// Return log number
	*pNum = log_num;

	return LOG_OK;
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
	uint8_t txt[11]; // Maximum length of signed int32
	uint8_t neg;
	uint8_t i;

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

// Write unsigned 32-bit integer value to data buffer
// input:
//   num - integer value to write
// return: number of bytes copied into data buffer
uint32_t LOG_WriteIntU(uint32_t num) {
	uint32_t len;
	uint8_t txt[10]; // Maximum length of unsigned int32
	uint8_t i;

	len = numlen(num);
	i = len - 1;
	do txt[i--] = num % 10 + '0'; while ((num /= 10) > 0);

	len = LOG_WriteBin(txt,len);

	return len;
}

// Write unsigned 32-bit integer value as float to data buffer
// input:
//   num - integer value to write
//   decimals - number of digits in fractional part [1..9]
// return: number of bytes copied into data buffer
uint32_t LOG_WriteIntF(uint32_t num, uint8_t decimals) {
	uint32_t len;
	uint8_t txt[11]; // Maximum length of unsigned int32 with decimal point
	int8_t i;

	if (num == 0) {
		// Special case for '0.0'
		txt[10] = '0';
		txt[9]  = '.';
		txt[8]  = '0';
		len = 3;
		i = 8;
	} else {
		i = 10;
		len = numlenu(num) + 1;
		do {
			if (10 - i == decimals) {
				txt[i--] = '.';
			} else {
				txt[i--] = num % 10 + '0';
				num /= 10;
			}
		} while (num > 0);
		if (10 - i <= decimals) {
			while (10 - i < decimals) {
				txt[i--] = '0';
				len++;
			}
			txt[i--] = '.';
			txt[i--] = '0';
			len++;
		}
		i++;
	}

	len = LOG_WriteBin(&txt[i],len);

	return len;
}

// Write date to data buffer (format DDMMYY)
// input:
//   day - date day [1..31]
//   month - date month [1..12]
//   year - date year [0..99]
// return: number of bytes copied into data buffer
uint32_t LOG_WriteDate(uint8_t day, uint8_t month, uint8_t year) {
	uint8_t txt[8];

	// Day
	txt[0] = (day / 10) + '0';
	txt[1] = (day % 10) + '0';
	txt[2] = '.';

	// Month
	txt[3] = (month / 10) + '0';
	txt[4] = (month % 10) + '0';
	txt[5] = '.';

	// Year
	txt[6] = (year / 10) + '0';
	txt[7] = (year % 10) + '0';

	return LOG_WriteBin(txt,8);
}

// Write time to data buffer (format HHMMSS)
// input:
//   hours - time hours [0..23]
//   minutes - date month [0..59]
//   seconds - date year [0..59]
// return: number of bytes copied into data buffer
uint32_t LOG_WriteTime(uint8_t hours, uint8_t minutes, uint8_t seconds) {
	uint8_t txt[8];

	// Hours
	txt[0] = (hours / 10) + '0';
	txt[1] = (hours % 10) + '0';
	txt[2] = ':';

	// Minutes
	txt[3] = (minutes / 10) + '0';
	txt[4] = (minutes % 10) + '0';
	txt[5] = ':';

	// Seconds
	txt[6] = (seconds / 10) + '0';
	txt[7] = (seconds % 10) + '0';

	return LOG_WriteBin(txt,8);
}

// Write date and time to data buffer in TZ format
// input:
//   time - seconds from midnight
//   date - date in format DDMMYYYY
//   tz - time zone offset (hours)
// return: number of bytes copied into data buffer
// note: year must be greater than 1999
//       result for zero tz (means UTC time): 2014-01-01T23:59:59Z
//       result for non zero tz: 2014-01-01T23:59:59+0300
uint32_t LOG_WriteDateTimeTZ(uint32_t time, uint32_t date, int8_t tz) {
	uint8_t txt[24];
	uint32_t i;

	// Date
	// Year
	i = (date % 10000) - 2000;
	txt[0] = '2';
	txt[1] = '0';
	txt[2] = (i / 10) + '0';
	txt[3] = (i % 10) + '0';
	txt[4] = '-';

	// Month
	i = (date - ((date / 1000000) * 1000000)) / 10000; // Ugly :(
	txt[5] = (i / 10) + '0';
	txt[6] = (i % 10) + '0';
	txt[7] = '-';

	// Day
	i = date / 1000000;
	txt[8]  = (i / 10) + '0';
	txt[9]  = (i % 10) + '0';
	txt[10] = 'T';

	// Time
	// Hours
	i =  time / 3600;
	txt[11] = (i / 10) + '0';
	txt[12] = (i % 10) + '0';
	txt[13] = ':';

	// Minutes
	i = (time / 60) % 60;
	txt[14] = (i / 10) + '0';
	txt[15] = (i % 10) + '0';
	txt[16] = ':';

	// Seconds
	i = time % 60;
	txt[17] = (i / 10) + '0';
	txt[18] = (i % 10) + '0';

	if (tz == 0) {
		// This is UTC time
		txt[19] = 'Z';

		return LOG_WriteBin(txt,20);
	} else {
		// This is local time with time zone offset
		if (tz > 0) {
			txt[19] = '+';
		} else {
			txt[19] = '-';
			tz *= -1;
		}
		txt[20] = (tz / 10) + '0';
		txt[21] = (tz % 10) + '0';
		txt[22] = '0';
		txt[23] = '0';

		return LOG_WriteBin(txt,24);
	}
}
