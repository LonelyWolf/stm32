#include <stm32l1xx_rcc.h>
#include <stm32l1xx_rtc.h>
#include <stm32l1xx_exti.h>
#include <misc.h>

#include <RTC.h>


RTC_TimeTypeDef RTC_Time;                   // Current RTC time
RTC_DateTypeDef RTC_Date;                   // Current RTC date


// Initialize and configure the RTC peripheral
void RTC_Config(void) {
	RTC_InitTypeDef RTCInit;
	NVIC_InitTypeDef NVICInit;
	EXTI_InitTypeDef EXTIInit;

	RCC->APB1ENR |= RCC_APB1Periph_PWR; // Enable the PWR peripheral
	PWR->CR |= PWR_CR_DBP; // Access to RTC, RTC Backup and RCC CSR registers enabled

	// Turn on LSE and wait until it become stable
	RCC_LSEConfig(RCC_LSE_ON);
	while(!(RCC->CSR & RCC_CSR_LSERDY));

	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE); // Select LSE as RTC clock source
	RCC_RTCCLKCmd(ENABLE); // Enable RTC clock
	RTC_WaitForSynchro(); // Wait for RTC APB registers synchronization

	// ck_spre = 1Hz
	RTCInit.RTC_AsynchPrediv = 0x7f; // div128
	RTCInit.RTC_SynchPrediv  = 0xff; // div256
	RTCInit.RTC_HourFormat   = RTC_HourFormat_24;
	RTC_Init(&RTCInit);

	// RTC wake-up -> EXTI line 20
	EXTI_ClearITPendingBit(EXTI_Line20);
	EXTIInit.EXTI_Line = EXTI_Line20;
	EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising; // Must be rising edge
	EXTIInit.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTIInit);

	// Enable the RTC wake-up interrupt
	NVICInit.NVIC_IRQChannel = RTC_WKUP_IRQn;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 0x0f; // 0x0f - lowest priority
	NVICInit.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Init(&NVICInit);

	// Configure the wake-up clock source
	RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);

	// Disable the wake-up counter and configure it default value
	RTC_WakeUpCmd(DISABLE);
	RTC_SetWakeUpCounter(0); // wake-up every second (value = 1s - 1)

	// Enable the wake-up interrupt
	RTC_ClearITPendingBit(RTC_IT_WUT);
	RTC_ITConfig(RTC_IT_WUT,ENABLE);

	PWR->CR &= ~PWR_CR_DBP; // Access to RTC, RTC Backup and RCC CSR registers disabled
}

// Configure wake-up interrupt
// input:
//   interval - wake-up timer counter interval
// note: interval can be a value from 0x0000 to 0xFFFF
//       if interval is zero then wake-up is disabled
void RTC_SetWakeUp(uint32_t interval) {
	PWR->CR |= PWR_CR_DBP; // Access to RTC, RTC Backup and RCC CSR registers enabled
	// Wake-up counter can be set only when wake-up disabled
	RTC_WakeUpCmd(DISABLE);
	if (interval) {
		// Set specified interval and enable wake-up counter
		RTC_SetWakeUpCounter(interval - 1);
		RTC_WakeUpCmd(ENABLE);
	} else {
		// Set interval to 1 second and leave a wake-up counter disabled
		RTC_SetWakeUpCounter(0);
	}
	PWR->CR &= ~PWR_CR_DBP; // Access to RTC, RTC Backup and RCC CSR registers disabled
}

// Set date and time from RTC_Date and RTC_Time variables
// input:
//   Time - pointer to RTC time structure
//   Date - pointer to RTC date structure
void RTC_SetDateTime(RTC_TimeTypeDef *time, RTC_DateTypeDef *date) {
	PWR->CR |= PWR_CR_DBP; // Access to RTC, RTC Backup and RCC CSR registers enabled
	RTC_SetTime(RTC_Format_BIN,time);
	RTC_SetDate(RTC_Format_BIN,date);
	PWR->CR &= ~PWR_CR_DBP; // Access to RTC, RTC Backup and RCC CSR registers disabled
}

// Get current date and time
// input:
//   Time - pointer to RTC time structure
//   Date - pointer to RTC date structure
// return: date and time in Time and Date structures
void RTC_GetDateTime(RTC_TimeTypeDef *time, RTC_DateTypeDef *date) {
	RTC_GetTime(RTC_Format_BIN,time);
	RTC_GetDate(RTC_Format_BIN,date);
}

// Convert Date/Time structures to epoch time
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

	epoch  = RTC_ToEpoch(time,date);
	epoch += offset * 3600;
	RTC_FromEpoch(epoch,time,date);
}
