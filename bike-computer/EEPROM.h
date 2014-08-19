// Define to prevent recursive inclusion -------------------------------------
#ifndef __EEPROM_H
#define __EEPROM_H


typedef enum {
	EEPROM_COMPLETE = 0,
	EEPROM_TIMEOUT  = 1
} EEPROM_Status;


// Sequences used to unlock the data EEPROM block and FLASH_PECR register
#define FLASH_PEKEY1               (uint32_t)0x89ABCDEF
#define FLASH_PEKEY2               (uint32_t)0x02030405

// Data EEPROM start and end address
#define DATA_EEPROM_START_ADDR     0x08080000
#define DATA_EEPROM_END_ADDR       0x08080FFF

// Timeout value for EEPROM operations
#define EEPROM_PRG_TIMEOUT         (uint32_t)0x8000


// Function prototypes
void EEPROM_Unlock(void);
void EEPROM_Lock(void);
EEPROM_Status EEPROM_WaitForLastOperation(uint32_t Timeout);
EEPROM_Status EEPROM_Write(uint32_t Address, uint32_t Data);
uint32_t EEPROM_Read(uint32_t address);

void ReadBuffer_EEPROM(uint32_t addr, volatile uint32_t *buffer, uint32_t len);
void SaveBuffer_EEPROM(uint32_t addr, volatile uint32_t *buffer, uint32_t len);

#endif // __EEPROM_H
