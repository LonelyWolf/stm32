// Real-Time Clock (RTC) peripheral management


#include <stm32l1xx_rcc.h>
#include "rtc.h"


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

#if (USE_WKUP)
	// Configure the EXTI line connected internally to the RTC WKUP
	EXTI->PR    =  RTC_WKUP_EXTI; // Clear IT pending bit
	EXTI->IMR  |=  RTC_WKUP_EXTI; // Enable interrupt request from EXTI line
	EXTI->EMR  &= ~RTC_WKUP_EXTI; // Disable event on EXTI line
	EXTI->RTSR |=  RTC_WKUP_EXTI; // Trigger rising edge enabled
	EXTI->FTSR &= ~RTC_WKUP_EXTI; // Trigger falling edge disabled
#endif // USE_WKUP

#if (USE_ALARMS)
	// Configure the EXTI line connected internally to the RTC ALARM
	EXTI->PR    =  RTC_ALARM_EXTI; // Clear IT pending bit
	EXTI->IMR  |=  RTC_ALARM_EXTI; // Enable interrupt request from EXTI line
	EXTI->EMR  &= ~RTC_ALARM_EXTI; // Disable event on EXTI line
	EXTI->RTSR |=  RTC_ALARM_EXTI; // Trigger rising edge enabled
	EXTI->FTSR &= ~RTC_ALARM_EXTI; // Trigger falling edge disabled
#endif // USE_ALARMS

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

#if (USE_WKUP)
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
#endif // USE_WKUP

	// Enable the write protection for RTC registers
	RTC->WPR = 0xFF;
	// Access to RTC, RTC Backup and RCC CSR registers disabled
	PWR->CR &= ~PWR_CR_DBP;

	return SUCCESS;
}

#if (USE_WKUP)
// Configure wake-up interval
// input:
//   interval - wake-up timer counter interval
// return: SUCCESS if wake-up
// note: interval can be a value in range [0x0000..0xFFFF]
//       wake-up will be disabled if a specified interval is zero
ErrorStatus RTC_SetWakeUp(uint32_t interval) {
	uint32_t wait;

	// Access to RTC, RTC Backup and RCC CSR registers enabled
	PWR->CR |= PWR_CR_DBP;
	// Disable the write protection for RTC registers
	RTC->WPR = 0xCA;
	RTC->WPR = 0x53;

	// Disable the wake-up counter
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
#endif // USE_WKUP

#if (USE_ALARMS)
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
#endif // USE_ALARMS

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

// Set date and time from RTC_Date and RTC_Time structures
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
	if (RTC_EnterInitMode() == SUCCESS) {
		// Write date and time to the RTC registers
		RTC->TR = TR;
		RTC->DR = DR;

		// Exit the RTC Initialization mode
		RTC->ISR &= ~RTC_ISR_INIT;

		// Wait for synchronization if BYPSHAD bit is not set in the RTC_CR register
		TR = SUCCESS;
		if (!(RTC->CR & RTC_CR_BYPSHAD)) {
			TR = RTC_WaitForSynchro();
		}
	} else TR = ERROR;

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

	// Calculate some coefficients
	a = (14 - date->RTC_Month) / 12;
	y = date->RTC_Year + 6800 - a; // years since 1 March, 4801 BC
	m = date->RTC_Month + (12 * a) - 3;

	// Compute Julian day number (from Gregorian calendar date)
	JDN  = date->RTC_Date;
	JDN += ((153 * m) + 2) / 5; // Number of days since 1 march
	JDN += 365 * y;
	JDN += y / 4;
	JDN -= y / 100;
	JDN += y / 400;
	JDN -= 32045;

	// Subtract number of days passed before base date from Julian day number
	JDN -= JULIAN_DATE_BASE;

	// Convert days to seconds
	JDN *= 86400;

	// Add to epoch specified time in seconds
	JDN += time->RTC_Hours * 3600;
	JDN += time->RTC_Minutes * 60;
	JDN += time->RTC_Seconds;

	// Number of seconds passed since the base date
	return JDN;
}

// Convert epoch time to RTC date/time
// input:
//   epoch - 32-bit epoch value (seconds)
//   time - pointer to the RTC time structure
//   date - pointer to the RTC date structure
void RTC_FromEpoch(uint32_t epoch, RTC_TimeTypeDef *time, RTC_DateTypeDef *date) {
	uint32_t a,b,c,d;

	// Calculate JDN (Julian day number) from a specified epoch value
	a = (epoch / 86400) + JULIAN_DATE_BASE;

	// Day of week
	date->RTC_WeekDay = (a % 7) + 1;

	// Calculate intermediate values
	a += 32044;
	b  = ((4 * a) + 3) / 146097;
	a -= (146097 * b) / 4;
	c  = ((4 * a) + 3) / 1461;
	a -= (1461 * c) / 4;
	d  = ((5 * a) + 2) / 153;

	// Date
	date->RTC_Date  = a - (((153 * d) + 2) / 5) + 1;
	date->RTC_Month = d + 3 - (12 * (d / 10));
	date->RTC_Year  = (100 * b) + c - 6800 + (d / 10);

	// Time
	time->RTC_Hours   = (epoch / 3600) % 24;
	time->RTC_Minutes = (epoch / 60) % 60;
	time->RTC_Seconds =  epoch % 60;
}

// Adjust time with time zone offset
// input:
//   time - pointer to RTC_Time structure with time to adjust
//   date - pointer to RTC_Date structure with date to adjust
//   offset - hours offset to add or subtract from date/time (hours)
void RTC_AdjustTimeZone(RTC_TimeTypeDef *time, RTC_DateTypeDef *date, int8_t offset) {
	uint32_t epoch;

	// Convert date/time to epoch
	epoch  = RTC_ToEpoch(time,date);
	// Add or subtract offset in seconds
	epoch += offset * 3600;
	// Convert updated epoch back to date/time
	RTC_FromEpoch(epoch,time,date);
}

// Day Of Week calculation from specified date
// input:
//   date - pointer to RTC_Date structure with date
// return: RTC_WeekDay field of date structure will be modified
// note: works for dates after 1583 A.D.
void RTC_CalcDOW(RTC_DateTypeDef *date) {
	int16_t adjustment,mm,yy;

	// Calculate intermediate values
	adjustment = (14 - date->RTC_Month) / 12;
	mm = date->RTC_Month + (12 * adjustment) - 2;
	yy = date->RTC_Year - adjustment;

	// Calculate day of week (0 = Sunday ... 6 = Saturday)
	date->RTC_WeekDay = (date->RTC_Date + ((13 * mm - 1) / 5) + yy + (yy / 4) - (yy / 100) + (yy / 400)) % 7;

	// Sunday?
	if (!date->RTC_WeekDay) date->RTC_WeekDay = 7;
}
