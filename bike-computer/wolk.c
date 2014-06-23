#include <stm32l1xx_rcc.h>
#include <stm32l1xx_rtc.h>

#include <wolk.h>
#include <EEPROM.h>


nRF24_Packet_TypeDef nRF24_Packet;          // nRF24L01 last received packet
Cur_Data_TypeDef CurData;                   // Current data (Speed, Cadence, etc.)
BTN_TypeDef BTN[4];                         // Buttons
Settings_TypeDef Settings;                  // Settings which stored in EEPROM


// Convert characters to number with fixed length
// input:
//   buf - point to character buffer
//   len - length of number to convert
// return: converted number
// note: only positive values
uint32_t atos_len(uint8_t *buf, uint8_t len) {
	uint32_t value = 0;
	uint8_t digit;
	uint8_t i;
	char c;

	for (i = 0; i < len; i++) {
		c = buf[i];
		if (c >= '0' && c <= '9') digit = (uint8_t)(c - '0'); else return 0;
		value = (value * 10) + digit;
    }

    return value;
}

// Convert characters to number
// from specified position until first non digital character
// input:
//   buf - point to character buffer
//   pos - point to position inside buffer
// return: converted number
// note: pos value will be changed to the next character after converted number
int32_t atos_char(uint8_t *buf, uint16_t *pos) {
	uint32_t value = 0;
	uint8_t digit;
	uint8_t i = 0;
	int8_t neg = 1;
	char c;

	if (buf[i] == '-') {
		neg = -1;
		i++;
	}
	for (; i < 11; i++) {
		c = buf[i];
		if (c >= '0' && c <= '9') digit = (uint8_t)(c - '0'); else {
			*pos += i + 1;
			return value * neg;
		}
		value = (value * 10) + digit;
    }
	*pos += i + 1;

    return value * neg;
}

// BSD implementation of strlen
// input:
//   str - pointer to string
// return: string length
uint32_t stringlen(const char *str) {
	const char *s;

	for (s = str; *s; ++s);

	return (s - str);
}

// Compute character length of numeric value
// input:
//   num - numeric value
// return: length in characters (3456 -> 4)
uint8_t numlen(int32_t num) {
	uint8_t len = 1;

	if (num < 0) {
		num *= -1;
		len++;
	}
	while ((num /= 10) > 0) len++;

	return len;
}

// Load settings from EEPROM
void ReadSettings_EEPROM(void) {
	Settings_TypeDef settings;
	uint32_t *ptr = (uint32_t *)&settings;
	uint8_t i;

	for (i = 0; i < sizeof(settings); i += 4) *ptr++ = EEPROM_Read(DATA_EEPROM_START_ADDR + i);
}

// Save settings to EEPROM
void SaveSettings_EEPROM(void) {
	Settings_TypeDef settings;
	uint32_t *ptr = (uint32_t *)&settings;
	uint8_t i;
	uint32_t data;

	EEPROM_Unlock();
	for (i = 0; i < sizeof(settings); i += 4) {
		data = *ptr++;
		if (data != EEPROM_Read(DATA_EEPROM_START_ADDR + i)) EEPROM_Write(DATA_EEPROM_START_ADDR + i,data);
	}
	EEPROM_Lock();
}
