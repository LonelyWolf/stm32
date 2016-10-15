// Real-Time Clock (RTC) peripheral management


#include "rtc.h"


// Timeouts
#define RTC_TIMEOUT_INIT           100000U // enter initialization mode, about 1s
#define RTC_TIMEOUT_SYNC           100000U // wait for synchronization, about 1s
#define RTC_TIMEOUT                  1000U // timeout for various operations, about 10ms


// Count rough delay for timeouts
static uint32_t RTC_CalcDelay(uint32_t delay) {
	uint32_t cnt;

	if (SystemCoreClock > 1000000U) {
		cnt = (delay * ((SystemCoreClock / 1000000U) + 1U));
	} else {
		cnt = (((delay / 100U) + 1U) * ((SystemCoreClock / 10000U) + 1U));
	}

	return cnt;
}

// Waits until the RTC Time and Date registers (RTC_TR and RTC_DR) are synchronized with RTC APB clock
// return: SUCCESS if RTC registers are synchronized, ERROR otherwise
// note: write protection to RTC registers must be disabled (RTC_WPR = 0xCA,0x53)
// note: access to the RTC registers must be enabled (bit DBP set in PWR_CR register)
ErrorStatus RTC_WaitForSynchro(void) {
	volatile uint32_t wait = RTC_CalcDelay(RTC_TIMEOUT_SYNC);

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
	volatile uint32_t wait = RTC_CalcDelay(RTC_TIMEOUT_INIT);

	// Check if the initialization mode is already set
	if (!(RTC->ISR & RTC_ISR_INITF)) {
	    // Set the initialization mode
	    RTC->ISR = RTC_ISR_INIT;

		// Wait till RTC is in INIT state or timeout
		while (!(RTC->ISR & RTC_ISR_INITF) && --wait);
	}

	return (wait) ? SUCCESS : ERROR;
}

// Initialize the RTC peripheral
// input:
//   psc_asynch - asynchronous prescaler value (7-bit), possible values 0x00..0x7F
//   psc_synch - synchronous prescaler value (15-bit), possible values 0x00..0x7FFF
// return: SUCCESS or ERROR
// note: access to the backup domain must be enabled
ErrorStatus RTC_Init(uint32_t psc_asynch, uint32_t psc_synch) {
	// Disable the write protection for RTC registers
	RTC_WriteProtectionDisable();

	// Enter the RTC initialization mode
	if (RTC_EnterInitMode() == ERROR) {
		// Enable the write protection for RTC registers
		RTC_WriteProtectionEnable();

		return ERROR;
	}

	// Configure 24-hour format
	RTC->CR &= ~RTC_CR_FMT;

	// Configure synchronous and asynchronous prescalers
	RTC->PRER = ((psc_asynch << 16) & RTC_PRER_PREDIV_A) | (psc_synch & RTC_PRER_PREDIV_S);

	// Exit the RTC initialization mode
	RTC_ExitInitMode();

	// Enable the write protection for RTC registers
	RTC_WriteProtectionEnable();

	return SUCCESS;
}

#if (USE_RTC_WAKEUP)
// Configure the RTC wakeup clock
// input:
//   clk_cfg - new wakeup clock selection, one of RTC_WUCLCK_xx values
// note: access to the backup domain must be enabled
// note: must called only when RTC_CR WUTE bit = 0 and RTC_ISR WUTWF bit = 1
ErrorStatus RTC_SetWakeupClock(uint32_t clk_cfg) {
	volatile uint32_t wait;

	// Disable the write protection for RTC registers
	RTC_WriteProtectionDisable();

	// Clear the WUTF and check the WUTWF flags only if wakeup is enabled
	if ((RTC->CR & RTC_CR_WUTE) || !(RTC->ISR & RTC_ISR_WUTWF)) {
		// Disable the wakeup timer
		RTC->CR &= ~RTC_CR_WUTE;

		// Clear the wakeup timer flag (WUTF)
		RTC_ClearWUTF();

		// Wait until WUTWF flag is set
		wait = RTC_CalcDelay(RTC_TIMEOUT);
		while (!(RTC->ISR & RTC_ISR_WUTWF) && --wait);
		if (!(RTC->ISR & RTC_ISR_WUTWF)) {
			// Enable the write protection for RTC registers
			RTC_WriteProtectionEnable();

			return ERROR;
		}
	}

	// Configure new wakeup clock
	RTC->CR &= ~RTC_CR_WUCKSEL;
	RTC->CR |= clk_cfg & RTC_CR_WUCKSEL;

	// Enable the write protection for RTC registers
	RTC_WriteProtectionEnable();

	return SUCCESS;
}

// Configure wakeup interval
// input:
//   interval - wakeup timer counter interval
// return: SUCCESS if wakeup
// note: interval can be a value in range [0x0000..0xFFFF]
//       wakeup will be disabled if a specified interval is zero
ErrorStatus RTC_SetWakeup(uint32_t interval) {
	volatile uint32_t wait = RTC_CalcDelay(RTC_TIMEOUT);

	// Disable the write protection for RTC registers
	RTC_WriteProtectionDisable();

	// Disable the wakeup counter
	RTC->CR &= ~RTC_CR_WUTE;

	// Clear the wakeup timer flag (WUTF)
	RTC_ClearWUTF();

	// Wait for the RTC WUTWF flag is set or timeout
	while (!(RTC->ISR & RTC_ISR_WUTWF) && --wait);
	if (!(RTC->ISR & RTC_ISR_WUTWF)) {
		// Enable the write protection for RTC registers
		RTC_WriteProtectionEnable();

		return ERROR;
	}

	if (interval) {
		// Configure wakeup auto-reload to specified interval and enable the counter
		RTC->WUTR = interval - 1;
		RTC->CR |= RTC_CR_WUTE;
	} else {
		// Set wakeup counter to zero and left the wakeup counter disabled
		RTC->WUTR = 0;
	}

	// Enable the write protection for RTC registers
	RTC_WriteProtectionEnable();

	return SUCCESS;
}
#endif // USE_RTC_WAKEUP

#if (USE_RTC_ALARMS)
// Configure the RTC alarm
// input:
//   alarm - which alarm to configure (RTC_ALARM_A or RTC_ALARM_B)
//   alarm_time - pointer to RTC time structure
//   alarm_mask - mask for the alarm (combination of RTC_ALARM_MASK_XXX values)
//   alarm_dateday - alarm date (value must be in range [1..31], this value ignored if RTC_ALARM_MASK_DAY bit set in mask value)
void RTC_AlarmInit(uint32_t alarm, RTC_TimeTypeDef *alarm_time, uint32_t alarm_mask, uint8_t alarm_dateday) {
	uint32_t ALRM;

	// Prepare value for the ALRMAR register
	ALRM =	((alarm_time->RTC_Hours   / 10) << 20) + ((alarm_time->RTC_Hours   % 10) << 16) +
			((alarm_time->RTC_Minutes / 10) << 12) + ((alarm_time->RTC_Minutes % 10) <<  8) +
			((alarm_time->RTC_Seconds / 10) <<  4) +  (alarm_time->RTC_Seconds % 10) +
			((alarm_dateday / 10) << 28) + ((alarm_dateday % 10) << 24) +
			 (alarm_time->RTC_H12 << 16) +
			  alarm_mask;

	// Disable the write protection for RTC registers
	RTC_WriteProtectionDisable();

	// Which of the alarms, A or B?
	if (alarm == RTC_ALARM_A) {
		RTC->ALRMAR = ALRM;
	} else {
		RTC->ALRMBR = ALRM;
	}

	// Enable the write protection for RTC registers
	RTC_WriteProtectionEnable();
}

// Enable or disable the specified RTC alarm
// input:
//   alarm - which alarm to configure (RTC_ALARM_A or RTC_ALARM_B)
//   NewState - new state of alarm (ENABLED or DISABLED)
// return: SUCCESS if alarm enabled/disabled, ERROR in case of timeout while disabling alarm
ErrorStatus RTC_AlarmSet(uint32_t alarm, FunctionalState NewState) {
	volatile uint32_t wait = RTC_CalcDelay(RTC_TIMEOUT);

	// Disable the write protection for RTC registers
	RTC_WriteProtectionDisable();

	if (NewState == ENABLE) {
		// Enable the specified alarm
		RTC->CR |= alarm;
		wait = SUCCESS;
	} else {
		// Disable the specified alarm
		RTC->CR &= ~alarm;

		// Wait till ALRxWF flag set in RTC_ISR register or timeout
		while (!(RTC->ISR & (alarm >> 8)) && --wait);
		wait = (RTC->ISR & (alarm >> 8)) ? SUCCESS : ERROR;
	}

	// Enable the write protection for RTC registers
	RTC_WriteProtectionEnable();

	return wait;
}
#endif // USE_RTC_ALARMS

// Enable or disable the specified RTC interrupts
// input:
//   IT - interrupts to be enabled or disabled (combination of RTC_IT_XXX values)
//   NewState - new state of interrupt (ENABLED or DISABLED)
void RTC_ITConfig(uint32_t IT, FunctionalState NewState) {
	// Disable the write protection for RTC registers
	RTC_WriteProtectionDisable();

	if (NewState == ENABLE) {
		// Enable the specified interrupts
		RTC->CR |=  IT;
	} else {
		// Disable the specified interrupts
		RTC->CR &= ~IT;
	}

	// Enable the write protection for RTC registers
	RTC_WriteProtectionEnable();
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

	// Disable the write protection for RTC registers
	RTC_WriteProtectionDisable();

	// Enter the RTC initialization mode
	if (RTC_EnterInitMode() == SUCCESS) {
		// Write date and time to the RTC registers
		RTC->TR = TR;
		RTC->DR = DR;

		// Exit the initialization mode
		RTC->ISR &= ~RTC_ISR_INIT;

		// Wait for synchronization if BYPSHAD bit is not set in the RTC_CR register
		TR = SUCCESS;
		if (!(RTC->CR & RTC_CR_BYPSHAD)) {
			TR = RTC_WaitForSynchro();
		}
	} else TR = ERROR;

	// Enable the write protection for RTC registers
	RTC_WriteProtectionEnable();

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
	JDN -= RTC_JDN;

	// Convert days to seconds
	JDN *= 86400;

	// Increase epoch time by specified time (in seconds)
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
	a = (epoch / 86400) + RTC_JDN;

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

// Calculate Day Of Week for specified date
// input:
//   date - pointer to the RTC_Date structure with date
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

// Write a data in a specified RTC backup data register
// input:
//   bkup_reg - RTC backup data register number
//   data - data to be written
// note: access to backup domain must be enabled
void RTC_BKUPWrite(uint32_t bkup_reg, uint32_t data) {
	__IO uint32_t addr;

	addr = ((uint32_t)&RTC->BKP0R) + (bkup_reg << 2);
	*(__IO uint32_t *)addr = data;
}

// Read a data from a specified RTC backup data register
// input:
//   bkup_reg - RTC backup data register number
// return: value of a specified register
uint32_t RTC_BKUPRead(uint32_t bkup_reg) {
	__IO uint32_t addr;

	addr = ((uint32_t)&RTC->BKP0R) + (bkup_reg << 2);

	return (*(__IO uint32_t *)addr);
}
