// Define to prevent recursive inclusion -------------------------------------
#ifndef __LOG_H
#define __LOG_H


#define LOG_DIR_LOGS             "LOGS"             // Directory to store log files
#define LOG_FILENAME_TEMPLATE    "WBC00000.LOG"     // Template for log file name
#define LOG_FILE_EXTENSION       ".LOG"             // Log files extension


typedef enum {
	LOG_OK          = 0,                 // The operation is completed successfully
	LOG_S0ERROR     = 1,                 // Error reading sector 0
	LOG_NOPARTITION = 2,                 // Partition not found
	LOG_VIERROR     = 3,                 // Error getting volume information
	LOG_ROOTERROR   = 4,                 // Error opening root directory
	LOG_CREATEERROR = 5,                 // File create error
	LOG_ERROR       = 0xffffffff         // Unknown log error
} LOG_Result;


LOG_Result LOG_Init(void);
uint32_t LOG_NewFile(void);

uint32_t LOG_WriteStr(char *str);
uint32_t LOG_WriteInt(int32_t num);

#endif // __LOG_H
