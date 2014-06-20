#include <stm32l1xx_rcc.h>

#include <EEPROM.h>


// Unlocks the data memory and FLASH_PECR register
void EEPROM_Unlock(void) {
	if (FLASH->PECR & FLASH_PECR_PELOCK) {
		// Unlock only if it locked
		FLASH->PEKEYR = FLASH_PEKEY1;
		FLASH->PEKEYR = FLASH_PEKEY2;
	}
}

// Lock the data memory and FLASH_PECR register
void EEPROM_Lock(void) {
	FLASH->PECR |= FLASH_PECR_PELOCK;
}

// Wait for a EEPROM operation complete or a timeout occur
// input:
//   Timeout - timeout interval
// return: EEPROM status: EEPROM_COMPLETE or EEPROM_TIMEOUT
EEPROM_Status EEPROM_WaitForLastOperation(uint32_t Timeout) {
	volatile EEPROM_Status status = EEPROM_COMPLETE;

	// Wait for a EEPROM operation to complete or a timeout occur
	while (FLASH->SR & FLASH_SR_BSY && Timeout) Timeout--;
	if (!Timeout) status = EEPROM_TIMEOUT;

	return status;
}

// Write long at specified address in data memory (32-bit)
// input:
//   Address - address of long to be written
//   Data - data to be written
// return: EEPROM status: EEPROM_COMPLETE or EEPROM_TIMEOUT
EEPROM_Status EEPROM_Write(uint32_t Address, uint32_t Data) {
	EEPROM_Status status = EEPROM_COMPLETE;

	// Wait for a EEPROM operation to complete or a timeout occur
	status = EEPROM_WaitForLastOperation(EEPROM_PRG_TIMEOUT);

	if (status == EEPROM_COMPLETE) {
		// Clear the FTDW bit (data will be erased before write if it non zero)
		FLASH->PECR &= (uint32_t)(~(uint32_t)FLASH_PECR_FTDW);
		*(__IO uint32_t *)Address = Data; // Program the new data

		// Wait for a EEPROM operation to complete or a timeout occur
		status = EEPROM_WaitForLastOperation(EEPROM_PRG_TIMEOUT);
	}

	return status;
}

// Read 32-bit value from EEPROM data memory
// input:
//   Address - address in data memory
// return: value from EEPROM data memory
// note: Address must be between DATA_EEPROM_START_ADDR and DATA_EEPROM_END_ADDR
uint32_t EEPROM_Read(uint32_t Address) {
	return (*(__IO uint32_t*)Address);
}
