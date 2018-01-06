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

// Wait until the RTC Time and Date registers (RTC_TR and RTC_DR) are synchronized with RTC APB clock
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

// Enter RTC Initialization mode
// return: SUCCESS if RTC is in initialization mode, ERROR otherwise
// note: write protection to RTC registers must be disabled (RTC_WPR = 0xCA,0x53)
// note: access to the RTC registers must be enabled (bit DBP set in PWR_CR register)
ErrorStatus RTC_EnterInitMode(void) {
	volatile uint32_t wait = RTC_CalcDelay(RTC_TIMEOUT_INIT);

	// Check if initialization mode is already set
	if (!(RTC->ISR & RTC_ISR_INITF)) {
		// Set the initialization mode and wait till RTC is in INIT state or timeout
		RTC->ISR = RTC_ISR_INIT;
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
ErrorStatus RTC_Init(uint32_t asynch, uint32_t synch) {
	ErrorStatus result;

	// Disable the write protection for RTC registers and enter RTC initialization mode
	RTC_WriteProtectionDisable();
	result = RTC_EnterInitMode();
	if (result == SUCCESS) {
		// Configure 24-hour format
		RTC->CR &= ~RTC_CR_FMT;

		// Configure synchronous and asynchronous prescalers
		RTC->PRER = ((asynch << 16) & RTC_PRER_PREDIV_A) | (synch & RTC_PRER_PREDIV_S);

		// Exit RTC initialization mode
		RTC_ExitInitMode();
		if (!(RTC->CR & RTC_CR_BYPSHAD)) {
			// Need to wait for synchronization of RTC registers
			// when shadow registers are enabled
			result = RTC_WaitForSynchro();
		}
	}

	// Enable write protection
	RTC_WriteProtectionEnable();

	return result;
}

#if (RTC_USE_WAKEUP)
// Configure the RTC wakeup clock
// input:
//   clk_cfg - new wakeup clock selection, one of RTC_WUCLCK_xx values
// note: access to the backup domain must be enabled
// note: must called only when RTC_CR WUTE bit = 0 and RTC_ISR WUTWF bit = 1
ErrorStatus RTC_SetWakeupClock(uint32_t clk_cfg) {
	// Disable the write protection for RTC registers
	RTC_WriteProtectionDisable();

	// Clear the WUTF and check the WUTWF flags only if wakeup is enabled
	if ((RTC->CR & RTC_CR_WUTE) || !(RTC->ISR & RTC_ISR_WUTWF)) {
		// Disable the wakeup timer
		RTC->CR &= ~RTC_CR_WUTE;

		// Clear the wakeup timer flag (WUTF)
		RTC_ClearWUTF();

		// Wait until WUTWF flag is set
		volatile uint32_t wait = RTC_CalcDelay(RTC_TIMEOUT);
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
		RTC->WUTR = interval - 1U;
		RTC->CR |= RTC_CR_WUTE;
	} else {
		// Set wakeup counter to zero and left the wakeup counter disabled
		RTC->WUTR = 0U;
	}

	// Enable the write protection for RTC registers
	RTC_WriteProtectionEnable();

	return SUCCESS;
}
#endif // RTC_USE_WAKEUP

#if (RTC_USE_ALARMS)
// Configure the RTC alarm
// input:
//   alarm - which alarm to configure (RTC_ALARM_A or RTC_ALARM_B)
//   alarm_time - pointer to RTC time structure
//   alarm_mask - mask for the alarm (combination of RTC_ALARM_MASK_XXX values)
//   alarm_dateday - alarm date (value must be in range [1..31], this value ignored if RTC_ALARM_MASK_DAY bit set in mask value)
void RTC_AlarmInit(uint32_t alarm, RTC_TimeTypeDef *alarm_time, uint32_t alarm_mask, uint8_t alarm_dateday) {
	register uint32_t ALRM;

	// Prepare value for the ALRMAR register
	ALRM = ((alarm_time->RTC_Hours / 10U) << RTC_ALRMAR_HT_Pos)     | \
			((alarm_time->RTC_Hours % 10U) << RTC_ALRMAR_HU_Pos)    | \
			((alarm_time->RTC_Minutes / 10U) << RTC_ALRMAR_MNT_Pos) | \
			((alarm_time->RTC_Minutes % 10U) << RTC_ALRMAR_MNU_Pos) | \
			((alarm_time->RTC_Seconds / 10U) << RTC_ALRMAR_ST_Pos)  | \
			((alarm_time->RTC_Seconds % 10U) << RTC_ALRMAR_SU_Pos)  | \
			((alarm_dateday / 10U) << RTC_ALRMAR_DT_Pos)            | \
			((alarm_dateday % 10U) << RTC_ALRMAR_DU_Pos)            | \
			(alarm_time->RTC_H12 << RTC_ALRMAR_PM_Pos)              | \
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
#endif // RTC_USE_ALARMS

// Enable or disable the specified RTC interrupts
// input:
//   IT - interrupts to be enabled or disabled (combination of RTC_IT_XXX values)
//   NewState - new state of interrupt (ENABLED or DISABLED)
void RTC_ITConfig(uint32_t IT, FunctionalState NewState) {
	// Disable the write protection for RTC registers
	RTC_WriteProtectionDisable();

	// Configure new state of interrupts
	if (NewState == ENABLE) {
		RTC->CR |=  IT;
	} else {
		RTC->CR &= ~IT;
	}

	// Enable the write protection for RTC registers
	RTC_WriteProtectionEnable();
}

// Enable or disable bypass the RTC shadow registers
// input:
//   NewState - new state of the bypass (ENABLED or DISABLED)
void RTC_BypassShadowConfig(FunctionalState NewState) {
	// Disable the write protection for RTC registers
	RTC_WriteProtectionDisable();

	// Configure new state of bypass the shadow registers
	if (NewState == ENABLE) {
		RTC->CR |=  RTC_CR_BYPSHAD;
	} else {
		RTC->CR &= ~RTC_CR_BYPSHAD;
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
	register uint32_t TR;
	register uint32_t DR;

	// Disable write protection for RTC registers and enter initialization mode
	RTC_WriteProtectionDisable();

#if (RTC_USE_SETDATETIME == 0)
	// Variant #1: enter INIT mode, compose new values for date and time registers, then write them

	// Enter initialization mode
	RTC_WriteProtectionDisable();
	if (RTC_EnterInitMode() == SUCCESS) {
		// Compose new value for time register
		TR = (((time->RTC_Hours / 10U) << RTC_TR_HT_Pos)      | \
				((time->RTC_Hours % 10U) << RTC_TR_HU_Pos)    | \
				((time->RTC_Minutes / 10U) << RTC_TR_MNT_Pos) | \
				((time->RTC_Minutes % 10U) << RTC_TR_MNU_Pos) | \
				((time->RTC_Seconds / 10U) << RTC_TR_ST_Pos)  | \
				((time->RTC_Seconds % 10U) << RTC_TR_SU_Pos)  | \
				(time->RTC_H12 << RTC_TR_PM_Pos))            & \
						RTC_TR_RESERVED_MASK;

		// Compose new value for date register
		if (date->RTC_WeekDay == 0U) {
			// Value '000' is forbidden for WDU bits in RTC_DR register
			date->RTC_WeekDay = 7U;
		}
		DR = (((date->RTC_Year / 10U) << RTC_DR_YT_Pos)    | \
				((date->RTC_Year % 10U) << RTC_DR_YU_Pos)  | \
				((date->RTC_Month / 10U) << RTC_DR_MT_Pos) | \
				((date->RTC_Month % 10U) << RTC_DR_MU_Pos) | \
				((date->RTC_Date / 10U) << RTC_DR_DT_Pos)  | \
				((date->RTC_Date % 10U) << RTC_DR_DU_Pos)  | \
				(date->RTC_WeekDay << RTC_DR_WDU_Pos))     & \
						RTC_DR_RESERVED_MASK;

		// Write date and time to the RTC registers and exit initialization mode
		RTC->TR = TR;
		RTC->DR = DR;
		RTC_ExitInitMode();

		if (!(RTC->CR & RTC_CR_BYPSHAD)) {
			// Need to wait for synchronization of RTC registers
			TR = RTC_WaitForSynchro();
		} else {
			TR = SUCCESS;
		}
	} else {
		// Timeout while entering initialization mode
		TR = ERROR;
	}
#else
	// Variant #2: initialize entry to the INIT mode, compose new values for the date/time
	// registers and then wait for the INIT mode, then write new values to the RTC registers
	// Looks less pretty than the variant #1, but gives a performance gain in those cases
	// when the frequency of the MCU is rather low

	// Activate the initialization mode if it is not already on
	if (!(RTC->ISR & RTC_ISR_INITF)) {
		RTC->ISR = RTC_ISR_INIT;
	}

	// Compose new value for time register
	TR = (((time->RTC_Hours / 10U) << RTC_TR_HT_Pos)      | \
			((time->RTC_Hours % 10U) << RTC_TR_HU_Pos)    | \
			((time->RTC_Minutes / 10U) << RTC_TR_MNT_Pos) | \
			((time->RTC_Minutes % 10U) << RTC_TR_MNU_Pos) | \
			((time->RTC_Seconds / 10U) << RTC_TR_ST_Pos)  | \
			((time->RTC_Seconds % 10U) << RTC_TR_SU_Pos)  | \
			(time->RTC_H12 << RTC_TR_PM_Pos))            & \
					RTC_TR_RESERVED_MASK;

	// Compose new value for date register
	if (date->RTC_WeekDay == 0) {
		// Value '000' is forbidden for WDU bits in RTC_DR register
		date->RTC_WeekDay = 7U;
	}
	DR = (((date->RTC_Year / 10U) << RTC_DR_YT_Pos)    | \
			((date->RTC_Year % 10U) << RTC_DR_YU_Pos)  | \
			((date->RTC_Month / 10U) << RTC_DR_MT_Pos) | \
			((date->RTC_Month % 10U) << RTC_DR_MU_Pos) | \
			((date->RTC_Date / 10U) << RTC_DR_DT_Pos)  | \
			((date->RTC_Date % 10U) << RTC_DR_DU_Pos)  | \
			(date->RTC_WeekDay << RTC_DR_WDU_Pos))     & \
					RTC_DR_RESERVED_MASK;

	// Poll the flag to ensure that RTC is in INIT state
	if (!(RTC->ISR & RTC_ISR_INITF)) {
		volatile uint32_t wait = RTC_CalcDelay(RTC_TIMEOUT_INIT);
		while (!(RTC->ISR & RTC_ISR_INITF) && --wait);
		if (wait == 0U) {
			// Enable the write protection for RTC registers
			RTC_WriteProtectionEnable();

			return ERROR;
		}
	}

	// Write date and time to the RTC registers and exit initialization mode
	RTC->TR = TR;
	RTC->DR = DR;
	RTC_ExitInitMode();

	if (!(RTC->CR & RTC_CR_BYPSHAD)) {
		// Need to wait for synchronization of RTC registers
		TR = RTC_WaitForSynchro();
	} else {
		TR = SUCCESS;
	}

#endif // RTC_USE_SETDATETIME

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
	register uint32_t TR;
	register uint32_t DR;

	// Read a values of date and time registers, clear reserved bits just for any case
	TR = RTC->TR & RTC_TR_RESERVED_MASK;
	DR = RTC->DR & RTC_DR_RESERVED_MASK;

	// Convert BCD to human readable format
	time->RTC_Hours   = (((TR & RTC_TR_HT)  >> RTC_TR_HT_Pos)  * 10U) + ((TR & RTC_TR_HU)  >> RTC_TR_HU_Pos);
	time->RTC_Minutes = (((TR & RTC_TR_MNT) >> RTC_TR_MNT_Pos) * 10U) + ((TR & RTC_TR_MNU) >> RTC_TR_MNU_Pos);
	time->RTC_Seconds = (((TR & RTC_TR_ST)  >> RTC_TR_ST_Pos)  * 10U) + ((TR & RTC_TR_SU)  >> RTC_TR_SU_Pos);
	time->RTC_H12     = (TR & RTC_TR_PM) >> RTC_TR_PM_Pos;

	date->RTC_Year    = (((DR & RTC_DR_YT) >> RTC_DR_YT_Pos) * 10U) + ((DR & RTC_DR_YU) >> RTC_DR_YU_Pos);
	date->RTC_Month   = (((DR & RTC_DR_MT) >> RTC_DR_MT_Pos) * 10U) + ((DR & RTC_DR_MU) >> RTC_DR_MU_Pos);
	date->RTC_Date    = (((DR & RTC_DR_DT) >> RTC_DR_DT_Pos) * 10U) + ((DR & RTC_DR_DU) >> RTC_DR_DU_Pos);
	date->RTC_WeekDay = (DR & RTC_DR_WDU) >> RTC_DR_WDU_Pos;
}

#if (RTC_USE_EPOCH)
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
	a = (14U - date->RTC_Month) / 12U;
	y = date->RTC_Year + 6800U - a; // years since 1 March, 4801 BC
	m = date->RTC_Month + (12U * a) - 3U;

	// Compute Julian day number (from Gregorian calendar date)
	JDN  = date->RTC_Date;
	JDN += ((153U * m) + 2U) / 5U; // Number of days since 1 march
	JDN += 365U * y;
	JDN += y / 4U;
	JDN -= y / 100U;
	JDN += y / 400U;
	JDN -= 32045U;

	// Subtract number of days passed before base date from Julian day number
	JDN -= RTC_JDN;

	// Convert days to seconds
	JDN *= 86400U;

	// Increase epoch time by specified time (in seconds)
	JDN += time->RTC_Hours * 3600U;
	JDN += time->RTC_Minutes * 60U;
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
	uint32_t a;
	uint32_t b;
	uint32_t c;
	uint32_t d;

	// Calculate JDN (Julian day number) from a specified epoch value
	a = (epoch / 86400U) + RTC_JDN;

	// Day of week
	date->RTC_WeekDay = (a % 7U) + 1U;

	// Calculate intermediate values
	a += 32044U;
	b  = ((4U * a) + 3U) / 146097U;
	a -= (146097U * b) / 4U;
	c  = ((4U * a) + 3U) / 1461U;
	a -= (1461U * c) / 4U;
	d  = ((5U * a) + 2U) / 153U;

	// Date
	date->RTC_Date  = a - (((153U * d) + 2U) / 5U) + 1U;
	date->RTC_Month = d + 3U - (12U * (d / 10U));
	date->RTC_Year  = (100U * b) + c - 6800U + (d / 10U);

	// Time
	time->RTC_Hours   = (epoch / 3600U) % 24U;
	time->RTC_Minutes = (epoch / 60U) % 60U;
	time->RTC_Seconds =  epoch % 60U;
}

// Adjust time and date by time zone offset
// input:
//   time - pointer to RTC_Time structure with time to adjust
//   date - pointer to RTC_Date structure with date to adjust
//   offset - hours offset to add or subtract from date/time (hours)
void RTC_AdjustTimeZone(RTC_TimeTypeDef *time, RTC_DateTypeDef *date, int8_t offset) {
	uint32_t epoch;

	// Convert date/time to epoch
	epoch = RTC_ToEpoch(time, date);
	// Add or subtract offset in seconds
	epoch += offset * 3600;
	// Convert updated epoch back to date/time
	RTC_FromEpoch(epoch, time, date);
}
#endif // RTC_USE_EPOCH

// Calculate Day Of Week for specified date
// input:
//   date - pointer to the RTC_Date structure with date
// return: RTC_WeekDay field of date structure will be modified
// note: works for dates after 1583 A.D.
void RTC_CalcDOW(RTC_DateTypeDef *date) {
	int16_t adjustment;
	int16_t month;
	int16_t year;
	register uint8_t wd;

	// Calculate intermediate values
	adjustment = (14 - date->RTC_Month) / 12;
	month = date->RTC_Month + (12 * adjustment) - 2;
	year = date->RTC_Year - adjustment;

	// Calculate day of week (0 = Sunday ... 6 = Saturday)
	wd  = date->RTC_Date;
	wd += (uint8_t)((((13 * month) - 1) / 5) + year + (year / 4) - (year / 100) + (year / 400));
	wd %= 7;

	// Is it Sunday?
	if (wd == 0U) {
		wd = 7U;
	}

	date->RTC_WeekDay = wd;
}

// Write a data in a specified RTC backup data register
// input:
//   bkup_reg - RTC backup data register number
//   data - data to be written
// note: access to backup domain must be enabled
void RTC_BKUPWrite(uint32_t bkup_reg, uint32_t data) {
	__IO uint32_t addr = ((uint32_t)&RTC->BKP0R) + (bkup_reg << 2);

	*(__IO uint32_t *)addr = data;
}

// Read a data from a specified RTC backup data register
// input:
//   bkup_reg - RTC backup data register number
// return: value of a specified register
uint32_t RTC_BKUPRead(uint32_t bkup_reg) {
	__IO uint32_t addr = ((uint32_t)&RTC->BKP0R) + (bkup_reg << 2);

	return (*(__IO uint32_t *)addr);
}
