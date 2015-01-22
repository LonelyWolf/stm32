#include <wolk.h>


nRF24_Packet_TypeDef nRF24_Packet;          // nRF24L01 last received packet
Cur_Data_TypeDef CurData;                   // Current data (Speed, Cadence, etc.)
BTN_TypeDef BTN[4];                         // Buttons
Settings_TypeDef Settings;                  // Settings which stored in EEPROM
bool _screensaver;                          // TRUE if screensaver active or must be activated
uint32_t _time_scr_timeout;                 // Timeout for screensaver start
uint32_t _time_no_signal;                   // Time since last packet received (seconds)
uint32_t _time_idle;                        // Time from last user event (button press)


// Execute WFI instruction
void SleepWait(void) {
	// Enable sleep-on-exit
	// After IRQ the MCU will fall asleep without returning to main loop
	SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;

	// Clear the WUF wake-up flag
	PWR->CR |= PWR_CR_CWUF;

	// Ensure effect of last store takes effect
	__DSB();

	// Enter sleep mode
	__WFI();
}

// Put MCU into STOP mode
void SleepStop(void) {
	uint32_t tmp_reg;

	// If PDDS bit is set -> STANDBY mode, STOP otherwise
	PWR->CR &= (uint32_t)~((uint32_t)~PWR_CR_PDDS);

	// Voltage regulator on during sleep mode
	PWR->CR &= (uint32_t)~((uint32_t)~PWR_CR_LPSDSR);

	// Clear the wake-up flag (WUF)
	PWR->CR |= PWR_CR_CWUF;

	// Disable sleep-on-exit
	SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;

	// Set SLEEPDEEP bit of Cortex-M System Control Register
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

	// Ensure effect of last store takes effect
	__DSB();

	// Enter STOP mode
	__WFI();

	// Clear SLEEPDEEP bit of Cortex-M System Control Register
	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

	// Restore system clocks only when screensaver deactivated
	if (!_screensaver) {
		// After wake-up from STOP mode system clocks are feed from HSI
		// and must be reconfigured to use HSE and PLL
		RCC->CR |= RCC_CR_HSEON; // Turn on HSE
		while (!(RCC->CR & RCC_CR_HSERDY)); // Wait till HSE ready
		RCC->CR |= RCC_CR_PLLON; // Turn on PLL
		while (!(RCC->CR & RCC_CR_PLLRDY)); // Wait till PLL ready
		// Select PLL as system clock source
		tmp_reg  = RCC->CFGR;
		tmp_reg &= ~RCC_CFGR_SW;
		tmp_reg |= RCC_SYSCLKSource_PLLCLK;
		RCC->CFGR = tmp_reg;
		// Wait till PLL is used as system clock source
		while ((uint8_t)(RCC->CFGR & RCC_CFGR_SWS) != 0x0c);

		// Update SystemCoreClock according to Clock Register Values (for any case)
		SystemCoreClockUpdate();
	}
}

// Put MCU into Standby mode
void SleepStandby(void) {
	// Clear the wake-up and standby flags
	PWR->CR |= (PWR_CR_CWUF | PWR_CR_CSBF);

	// Enter STANDBY mode when the CPU enters deepsleep
	PWR->CR |= PWR_CR_PDDS;

	// Disable sleep-on-exit
	SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;

	// Set SLEEPDEEP bit of Cortex-M System Control Register
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

	// Ensure effect of last store takes effect
	__DSB();

	// Enter STANDBY mode
	__WFI();

	// After waking up from STANDBY mode, program execution restarts in the same way as after a Reset
}

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

// Compute character length of unsigned integer numeric value
// input:
//   num - numeric value
// return: length in characters (3456 -> 4)
uint8_t numlenu(uint32_t num) {
	uint8_t len = 1;

	while ((num /= 10) > 0) len++;

	return len;
}

// CRC8-CCITT calculation for buffer (polynomial: x^8 + x^2 + x + 1 (0xE0))
// input:
//   buf - pointer to the buffer to calc CRC
//   len - length of the buffer (bytes)
// return: CRC8 of the buffer
uint8_t CRC8_CCITT(uint8_t *buf, uint8_t len) {
    uint8_t i;
    uint8_t data = 0;

    while (len--) {
        data ^= *buf++;
        for (i = 0; i < 8; i++ ) {
            if (data & 0x80) data ^= 0x07;
            data <<= 1;
        }
    }

    return data;
}

// Wait for key press
// input:
//   Sleep - execute SleepWait if TRUE
//   WaitFlag - pointer to bool variable with flag (function exits wait loop when flag set to TRUE)
//   Timeout - wait timeout (seconds), no timeout if this parameter is zero
// note: WaitFlag ignored if it NULL
void WaitForKeyPress(bool Sleep, bool *WaitFlag, uint32_t Timeout) {
	bool key_pressed = FALSE;

	do {
		key_pressed = BTN[0].cntr || BTN[1].cntr || BTN[2].cntr || BTN[3].cntr ||
				BTN[0].state == BTN_Hold || BTN[1].state == BTN_Hold ||
				BTN[2].state == BTN_Hold || BTN[3].state == BTN_Hold;
		if (WaitFlag) key_pressed |= *WaitFlag;
		if (Sleep) SleepWait();
		if (_time_idle > Timeout && Timeout) key_pressed = TRUE;
	} while (!key_pressed);
}

// Clear buttons state
void ClearKeys(void) {
	uint8_t i;

	for (i = 0; i < 4; i++)	BTN[i].cntr = 0;
}
