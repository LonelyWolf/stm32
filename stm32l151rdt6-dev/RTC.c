#include <stm32l1xx_rcc.h>

#include "RTC.h"


RTC_TimeTypeDef RTC_Time; // Current RTC time
RTC_DateTypeDef RTC_Date; // Current RTC date


// Waits until the RTC Time and Date registers (RTC_TR and RTC_DR) are synchronized with RTC APB clock
// return: SUCCESS if RTC registers are synchronized, ERROR otherwise
// note: write protection to RTC registers must be disabled (RTC_WPR = 0xCA,0x53)
// note: access to the RTC registers must be enabled (bit DBP set in PWR_CR register)
ErrorStatus RTC_WaitForSynchro(void) {
	uint32_t wait = RTC_SYNC_TIMEOUT;

	// Clear the RSF flag
	RTC->ISR &= ~RTC_ISR_RSF;

	// Wait the registers to be synchronized
	while (!(RTC->ISR & RTC_ISR_RSF) && --wait);

	return (RTC->ISR & RTC_ISR_RSF) ? SUCCESS : ERROR;
}

// Enters the RTC Initialization mode
// return: SUCCESS if RTC is in initialization mode, ERROR otherwise
// note: write protection to RTC registers must be disabled (RTC_WPR = 0xCA,0x53)
// note: access to the RTC registers must be enabled (bit DBP set in PWR_CR register)
ErrorStatus RTC_EnterInitMode(void) {
	uint32_t wait = RTC_INIT_TIMEOUT;

	if (!(RTC->ISR & RTC_ISR_INITF)) {
	    // Set the initialization mode
	    RTC->ISR = RTC_ISR_INIT;
		wait = RTC_INIT_TIMEOUT;

		// Wait till RTC is in INIT state or timeout
		while (!(RTC->ISR & RTC_ISR_INITF) && --wait);
	}

	return (RTC->ISR & RTC_ISR_INITF) ? SUCCESS : ERROR;
}

// Initialize and configure the RTC peripheral
// return: SUCCESS if RTC configured, ERROR otherwise
ErrorStatus RTC_Config(void) {
	uint32_t wait;

	// Enable the PWR peripheral
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	// Access to RTC, RTC Backup and RCC CSR registers enabled
	PWR->CR |= PWR_CR_DBP;

	// Turn on LSE and wait until it become stable
	RCC_LSEConfig(RCC_LSE_ON);
	while(!(RCC->CSR & RCC_CSR_LSERDY));

	// Select LSE as RTC clock source
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);
	// Enable RTC clock
	RCC_RTCCLKCmd(ENABLE);

	// Configure the EXTI line connected internally to the RTC WKUP
	EXTI->PR    =  RTC_WKUP_EXTI; // Clear IT pending bit
	EXTI->IMR  |=  RTC_WKUP_EXTI; // Enable interrupt request from EXTI line
	EXTI->EMR  &= ~RTC_WKUP_EXTI; // Disable event on EXTI line
	EXTI->RTSR |=  RTC_WKUP_EXTI; // Trigger rising edge enabled
	EXTI->FTSR &= ~RTC_WKUP_EXTI; // Trigger falling edge disabled

	// Configure the EXTI line connected internally to the RTC ALARM
	EXTI->PR    =  RTC_ALARM_EXTI; // Clear IT pending bit
	EXTI->IMR  |=  RTC_ALARM_EXTI; // Enable interrupt request from EXTI line
	EXTI->EMR  &= ~RTC_ALARM_EXTI; // Disable event on EXTI line
	EXTI->RTSR |=  RTC_ALARM_EXTI; // Trigger rising edge enabled
	EXTI->FTSR &= ~RTC_ALARM_EXTI; // Trigger falling edge disabled

	// Disable the write protection for RTC registers
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;

	// Wait for the RTC APB registers synchronization
	if (RTC_WaitForSynchro() != SUCCESS) {
		// Enable the write protection for RTC registers
		RTC->WPR = 0xFF;
		// Access to RTC, RTC Backup and RCC CSR registers disabled
		PWR->CR &= ~PWR_CR_DBP;

		return ERROR;
	}

	// Enter the RTC initialization mode
	if (RTC_EnterInitMode() != SUCCESS) {
		// Enable the write protection for RTC registers
		RTC->WPR = 0xFF;
		// Access to RTC, RTC Backup and RCC CSR registers disabled
		PWR->CR &= ~PWR_CR_DBP;

		return ERROR;
	}

	// Clear RTC CR FMT Bit (24-hour format)
	RTC->CR &= ~RTC_CR_FMT;

	// Configure the RTC prescaler
	RTC->PRER = 0x007f00ff; // Asynch = 128, Synch = 256

	// Exit the RTC Initialization mode
	RTC->ISR &= ~RTC_ISR_INIT;

	// Configure the wake-up clock source (ck_spre = 1Hz, 16bits)
	RTC->CR &= ~RTC_CR_WUCKSEL;
	RTC->CR |=  RTC_CR_WUCKSEL_2;

	// Disable the wake-up timer
	RTC->CR &= ~RTC_CR_WUTE;
	// Wait for the RTC WUTWF flag is set or timeout
	wait = RTC_INIT_TIMEOUT;
	while (!(RTC->ISR & RTC_ISR_WUTWF) && --wait);
	if (!(RTC->ISR & RTC_ISR_WUTWF)) {
		// Enable the write protection for RTC registers
		RTC->WPR = 0xFF;
		// Access to RTC, RTC Backup and RCC CSR registers disabled
		PWR->CR &= ~PWR_CR_DBP;

		return ERROR;
	}
	// Set interval to 1 second (the wake-up counter is disabled)
	RTC->WUTR = 0;

	// Enable the write protection for RTC registers
	RTC->WPR = 0xFF;
	// Access to RTC, RTC Backup and RCC CSR registers disabled
	PWR->CR &= ~PWR_CR_DBP;

	return SUCCESS;
}

// Configure wake-up interval
// input:
//   interval - wake-up timer counter interval
// return: SUCCESS if wake-up
// note: interval can be a value from range [0x0..0xFFFF]
//       wake-up will be disabled if interval is zero
ErrorStatus RTC_SetWakeUp(uint32_t interval) {
	uint32_t wait;

	// Access to RTC, RTC Backup and RCC CSR registers enabled
	PWR->CR |= PWR_CR_DBP;
	// Disable the write protection for RTC registers
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;

	// Disable the wake-up counter
	RTC->CR &= ~RTC_CR_WUTE; // Disable the wake-up timer
	// Wait for the RTC WUTWF flag is set or timeout
	wait = RTC_INIT_TIMEOUT;
	while (!(RTC->ISR & RTC_ISR_WUTWF) && --wait);
	if (!(RTC->ISR & RTC_ISR_WUTWF)) {
		// Enable the write protection for RTC registers
		RTC->WPR = 0xFF;
		// Access to RTC, RTC Backup and RCC CSR registers disabled
		PWR->CR &= ~PWR_CR_DBP;

		return ERROR;
	}

	if (interval) {
		// Set specified interval and enable the wake-up counter
		RTC->WUTR = interval - 1;
		// Enable the wake-up timer
		RTC->CR |= RTC_CR_WUTE;
	} else {
		// Set interval to 1 second and left a wake-up counter disabled
		RTC->WUTR = 0;
	}

	// Enable the write protection for RTC registers
	RTC->WPR = 0xFF;
	// Access to RTC, RTC Backup and RCC CSR registers disabled
	PWR->CR &= ~PWR_CR_DBP;

	return SUCCESS;
}

// Set date and time from RTC_Date and RTC_Time variables
// input:
//   Time - pointer to RTC time structure
//   Date - pointer to RTC date structure
// return: SUCCESS if date and time set, ERROR otherwise
ErrorStatus RTC_SetDateTime(RTC_TimeTypeDef *time, RTC_DateTypeDef *date) {
	uint32_t TR,DR;

	// Calculate value for time register
	TR =   (((time->RTC_Hours   / 10) << 20) + ((time->RTC_Hours   % 10) << 16) +
			((time->RTC_Minutes / 10) << 12) + ((time->RTC_Minutes % 10) <<  8) +
			((time->RTC_Seconds / 10) <<  4) +  (time->RTC_Seconds % 10) +
			 (time->RTC_H12 << 12)) & RTC_TR_RESERVED_MASK;
	// Calculate value for date register
	DR =   (((date->RTC_Year  / 10) << 20) + ((date->RTC_Year  % 10) << 16) +
			((date->RTC_Month / 10) << 12) + ((date->RTC_Month % 10) <<  8) +
			((date->RTC_Date  / 10) <<  4) +  (date->RTC_Date  % 10) +
			 (date->RTC_WeekDay << 13)) & RTC_DR_RESERVED_MASK;

	// Access to RTC, RTC Backup and RCC CSR registers enabled
	PWR->CR |= PWR_CR_DBP;
	// Disable the write protection for RTC registers
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;

	// Enter the RTC initialization mode
	if (RTC_EnterInitMode() != SUCCESS) {
		// Enable the write protection for RTC registers
		RTC->WPR = 0xFF;
		// Access to RTC, RTC Backup and RCC CSR registers disabled
		PWR->CR &= ~PWR_CR_DBP;

		return ERROR;
	}

	// Write date and time to the RTC registers
	RTC->TR = TR;
	RTC->DR = DR;

	// Exit the RTC Initialization mode
	RTC->ISR &= ~RTC_ISR_INIT;

	// Wait for synchronization if BYPSHAD bit is not set in the RTC_CR register
	TR = SUCCESS;
	if (!(RTC->CR & RTC_CR_BYPSHAD)) {
		TR = (RTC_WaitForSynchro() == ERROR) ? ERROR : SUCCESS;
	}

	// Enable the write protection for RTC registers
	RTC->WPR = 0xFF;
	// Access to RTC, RTC Backup and RCC CSR registers disabled
	PWR->CR &= ~PWR_CR_DBP;

	return TR;
}

// Get current date and time
// input:
//   Time - pointer to RTC time structure
//   Date - pointer to RTC date structure
// return: date and time in Time and Date structures
void RTC_GetDateTime(RTC_TimeTypeDef *time, RTC_DateTypeDef *date) {
	uint32_t TR,DR;

	// Get date and time (clear reserved bits just for any case)
	TR = RTC->TR & RTC_TR_RESERVED_MASK;
	DR = RTC->DR & RTC_DR_RESERVED_MASK;

	// Convert BCD to human readable format
	time->RTC_Hours   = (((TR >> 20) & 0x03) * 10) + ((TR >> 16) & 0x0f);
	time->RTC_Minutes = (((TR >> 12) & 0x07) * 10) + ((TR >>  8) & 0x0f);
	time->RTC_Seconds = (((TR >>  4) & 0x07) * 10) +  (TR & 0x0f);
	time->RTC_H12     =   (TR & RTC_TR_PM) >> 16;
	date->RTC_Year    = (((DR >> 20) & 0x07) * 10) + ((DR >> 16) & 0x0f);
	date->RTC_Month   = (((DR >> 12) & 0x01) * 10) + ((DR >>  8) & 0x0f);
	date->RTC_Date    = (((DR >>  4) & 0x03) * 10) +  (DR & 0x0f);
	date->RTC_WeekDay = (DR & RTC_DR_WDU) >> 13;
}

// Set the specified RTC alarm
// input:
//   Alarm - which alarm to configure (RTC_ALARM_A or RTC_ALARM_B)
//   time - pointer to RTC time structure
//   AlarmDateDay - alarm date (value must be in range [1..31], don't care if RTC_ALARM_MASK_DAY bit set in AlarmMask)
//   AlarmMask - mask for the alarm (combination of RTC_ALARM_MASK_XXX values)
void RTC_SetAlarm(uint32_t Alarm, RTC_TimeTypeDef *time, uint8_t AlarmDateDay, uint32_t AlarmMask) {
	uint32_t ALRM;

	// Compute value for the ALRMAR register
	ALRM =	((AlarmDateDay      / 10) << 28) + ((AlarmDateDay      % 10) << 24) +
			((time->RTC_Hours   / 10) << 20) + ((time->RTC_Hours   % 10) << 16) +
			((time->RTC_Minutes / 10) << 12) + ((time->RTC_Minutes % 10) <<  8) +
			((time->RTC_Seconds / 10) <<  4) +  (time->RTC_Seconds % 10) +
			 (time->RTC_H12 << 16) +
			  AlarmMask;

	// Access to RTC, RTC Backup and RCC CSR registers enabled
	PWR->CR |= PWR_CR_DBP;
	// Disable the write protection for RTC registers
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;

	if (Alarm == RTC_ALARM_A) {
		// Set alarm A
		RTC->ALRMAR = ALRM;
	} else {
		// Set alarm B
		RTC->ALRMBR = ALRM;
	}

	// Enable the write protection for RTC registers
	RTC->WPR = 0xFF;
	// Access to RTC, RTC Backup and RCC CSR registers disabled
	PWR->CR &= ~PWR_CR_DBP;
}

// Enable or disable the specified RTC alarm
// input:
//   Alarm - which alarm to configure (RTC_ALARM_A or RTC_ALARM_B)
//   NewState - new state of alarm (ENABLED or DISABLED)
// return: SUCCESS if alarm enabled/disabled, ERROR in case of timeout while disabling alarm
ErrorStatus RTC_AlarmCmd(uint32_t Alarm, FunctionalState NewState) {
	uint32_t wait = RTC_INIT_TIMEOUT;

	// Access to RTC, RTC Backup and RCC CSR registers enabled
	PWR->CR |= PWR_CR_DBP;
	// Disable the write protection for RTC registers
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;

	if (NewState == ENABLE) {
		// Enable the specified alarm
		RTC->CR |= Alarm;
		wait = SUCCESS;
	} else {
		// Disable the specified alarm
		RTC->CR &= ~Alarm;

		// Wait till ALRxWF flag set in RTC_ISR register or timeout
		while (!(RTC->ISR & (Alarm >> 8)) && --wait);
		wait = (RTC->ISR & (Alarm >> 8)) ? SUCCESS : ERROR;
	}

	// Enable the write protection for RTC registers
	RTC->WPR = 0xFF;
	// Access to RTC, RTC Backup and RCC CSR registers disabled
	PWR->CR &= ~PWR_CR_DBP;

	return wait;
}

// Enable or disable the specified RTC interrupts
// input:
//   IT - interrupts to be enabled or disabled (combination of RTC_IT_XXX values)
//   NewState - new state of interrupt (ENABLED or DISABLED)
void RTC_ITConfig(uint32_t IT, FunctionalState NewState) {
	// Access to RTC, RTC Backup and RCC CSR registers enabled
	PWR->CR |= PWR_CR_DBP;
	// Disable the write protection for RTC registers
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;

	if (NewState == ENABLE) {
		// Enable the specified interrupts
		RTC->CR |=  IT;
	} else {
		// Disable the specified interrupts
		RTC->CR &= ~IT;
	}

	// Enable the write protection for RTC registers
	RTC->WPR = 0xFF;
	// Access to RTC, RTC Backup and RCC CSR registers disabled
	PWR->CR &= ~PWR_CR_DBP;
}

// Convert Date/Time structures to epoch time
// input:
//   time - pointer to the RTC time structure
//   date - pointer to the RTC date structure
// return: 32-bit epoch value (seconds)
uint32_t RTC_ToEpoch(RTC_TimeTypeDef *time, RTC_DateTypeDef *date) {
	uint8_t  a;
	uint16_t y;
	uint8_t  m;
	uint32_t JDN;

	// These hardcore math's are taken from http://en.wikipedia.org/wiki/Julian_day

	// Calculate some coefficients
	a = (14 - date->RTC_Month) / 12;
	y = (date->RTC_Year + 2000) + 4800 - a; // years since 1 March, 4801 BC
	m = date->RTC_Month + (12 * a) - 3; // since 1 March, 4801 BC

	// Gregorian calendar date compute
	JDN  = date->RTC_Date;
	JDN += (153 * m + 2) / 5;
	JDN += 365 * y;
	JDN += y / 4;
	JDN += -y / 100;
	JDN += y / 400;
	JDN  = JDN - 32045;
	JDN  = JDN - JULIAN_DATE_BASE;    // Calculate from base date
	JDN *= 86400;                     // Days to seconds
	JDN += time->RTC_Hours * 3600;    // ... and today seconds
	JDN += time->RTC_Minutes * 60;
	JDN += time->RTC_Seconds;

	return JDN;
}

// Convert epoch time to Date/Time structures
// input:
//   epoch - 32-bit epoch value (seconds)
//   time - pointer to the RTC time structure
//   date - pointer to the RTC date structure
void RTC_FromEpoch(uint32_t epoch, RTC_TimeTypeDef *time, RTC_DateTypeDef *date) {
	uint32_t tm;
	uint32_t t1;
	uint32_t a;
	uint32_t b;
	uint32_t c;
	uint32_t d;
	uint32_t e;
	uint32_t m;
	int16_t  year  = 0;
	int16_t  month = 0;
	int16_t  dow   = 0;
	int16_t  mday  = 0;
	int16_t  hour  = 0;
	int16_t  min   = 0;
	int16_t  sec   = 0;
	uint64_t JD    = 0;
	uint64_t JDN   = 0;

	// These hardcore math's are taken from http://en.wikipedia.org/wiki/Julian_day

	JD  = ((epoch + 43200) / (86400 >>1 )) + (2440587 << 1) + 1;
	JDN = JD >> 1;

	tm = epoch; t1 = tm / 60; sec  = tm - (t1 * 60);
	tm = t1;    t1 = tm / 60; min  = tm - (t1 * 60);
	tm = t1;    t1 = tm / 24; hour = tm - (t1 * 24);

	dow   = JDN % 7;
	a     = JDN + 32044;
	b     = ((4 * a) + 3) / 146097;
	c     = a - ((146097 * b) / 4);
	d     = ((4 * c) + 3) / 1461;
	e     = c - ((1461 * d) / 4);
	m     = ((5 * e) + 2) / 153;
	mday  = e - (((153 * m) + 2) / 5) + 1;
	month = m + 3 - (12 * (m / 10));
	year  = (100 * b) + d - 4800 + (m / 10);

	date->RTC_Year    = year - 2000;
	date->RTC_Month   = month;
	date->RTC_Date    = mday;
	date->RTC_WeekDay = dow;
	time->RTC_Hours   = hour;
	time->RTC_Minutes = min;
	time->RTC_Seconds = sec;
}

// Adjust time with time zone offset
// input:
//   time - pointer to RTC_Time structure with time to adjust
//   date - pointer to RTC_Date structure with date to adjust
//   offset - hours offset to add or subtract from date/time (hours)
void RTC_AdjustTimeZone(RTC_TimeTypeDef *time, RTC_DateTypeDef *date, int8_t offset) {
	uint32_t epoch;

	epoch = RTC_ToEpoch(time,date) + (offset * 3600);
	RTC_FromEpoch(epoch,time,date);
}

// Day Of Week calculation from specified date
// input:
//   date - pointer to RTC_Date structure with date
// return: RTC_WeekDay field of date structure will be modified
RTC_CalcDOW(RTC_DateTypeDef *date) {
	int16_t adjustment, mm, yy;

	adjustment = (14 - date->RTC_Month) / 12;
	mm = date->RTC_Month + (12 * adjustment) - 2;
	yy = date->RTC_Year - adjustment;

	date->RTC_WeekDay = (date->RTC_Date + ((13 * mm - 1) / 5) + yy + (yy / 4) - (yy / 100) + (yy / 400)) % 7;
}
