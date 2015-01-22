// Define to prevent recursive inclusion -------------------------------------
#ifndef __LOG_H
#define __LOG_H


#define LOG_DIR_LOGS             "LOGS"             // Directory to store log files
#define LOG_FILENAME_TEMPLATE    "WBC00000.LOG"     // Template for log file name
#define LOG_FILE_EXTENSION       ".LOG"             // Log files extension


typedef enum {
	LOG_OK          = 0x00,        // The operation is completed successfully
	LOG_NOPARTITION = 0x01,        // Partition not found
	LOG_VIERROR     = 0x02,        // Error getting volume information
	LOG_ROOTERROR   = 0x03,        // Error opening root directory
	LOG_CREATEERROR = 0x04,        // File create error
	LOG_ERROR       = 0xff         // Unknown log error
} LOG_Result;


// Public variables
extern bool _SD_present;                           // TRUE if SD card present
extern bool _logging;                              // TRUE if log file was created


// Function prototypes
LOG_Result LOG_Init(void);
uint32_t LOG_NewFile(uint32_t *pNum);
uint32_t LOG_FileSync(void);

uint32_t LOG_WriteBin(uint8_t *buf, uint32_t len);
uint32_t LOG_WriteStr(char *str);
uint32_t LOG_WriteInt(int32_t num);
uint32_t LOG_WriteIntU(uint32_t num);
uint32_t LOG_WriteIntF(uint32_t num, uint8_t decimals);
uint32_t LOG_WriteDate(uint8_t day, uint8_t month, uint8_t year);
uint32_t LOG_WriteTime(uint8_t hours, uint8_t minutes, uint8_t seconds);
uint32_t LOG_WriteDateTimeTZ(uint32_t time, uint32_t date, int8_t tz);

#endif // __LOG_H
