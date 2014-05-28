#include <stm32l1xx_rcc.h>
#include <stm32l1xx_rtc.h>
#include <stm32l1xx_pwr.h>
#include <RTC.h>


const uint16_t week_day[] = { 0x4263, 0xA8BD, 0x42BF, 0x4370, 0xABBF, 0xA8BF, 0x43B2};


// Init RTC
void RTC_Config(void) {
	RTC_InitTypeDef RTCInit;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR,ENABLE); // Enable the PWR peripheral
	PWR_RTCAccessCmd(ENABLE); // Enable access to RTC and BKP registers
	RCC_RTCResetCmd(ENABLE); // Reset RTC time
	RCC_RTCResetCmd(DISABLE);
	RCC_LSEConfig(RCC_LSE_ON); // Turn on LSE and wait until it's become stable
//	while(RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);
	while(!(RCC->CSR & RCC_CSR_LSERDY));
	RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE); // Select RTC clock source
	RCC_RTCCLKCmd(ENABLE); // Enable RTC clock
	RTC_WaitForSynchro(); // Wait for RTC APB registers synchronization

	// ck_spre = 1Hz
	RTCInit.RTC_AsynchPrediv = 0x7f; // div128
//	RTCInit.RTC_AsynchPrediv = 0x0f; // ----------------------- HARDER, FASTER!
	RTCInit.RTC_SynchPrediv  = 0xff; // div256
	RTCInit.RTC_HourFormat   = RTC_HourFormat_24;
	RTC_Init(&RTCInit);

	RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);
	RTC_WakeUpCmd(DISABLE);
	RTC_SetWakeUpCounter(0); // Counter can be set only when wakeup disabled
	RTC_ITConfig(RTC_IT_WUT,ENABLE); // Enable wakeup interrupt
	RTC_WakeUpCmd(ENABLE);
}

// Convert Date/Time structures to epoch time
uint32_t RTC_ToEpoch(RTC_TimeTypeDef *time, RTC_DateTypeDef *date) {
	uint8_t  a;
	uint16_t y;
	uint8_t  m;
	uint32_t JDN;

	// These hardcore math taken from http://en.wikipedia.org/wiki/Julian_day

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
    JDN  = JDN - JULIAN_DATE_BASE;    // Calcuate from base date
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

	// These hardcore math taken from http://en.wikipedia.org/wiki/Julian_day

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

void RTC_AdjustTimeZone(RTC_TimeTypeDef *time, RTC_DateTypeDef *date, int8_t offset) {
	uint32_t epoch;

	epoch  = RTC_ToEpoch(time,date);
	epoch += offset * 3600;
	RTC_FromEpoch(epoch,time,date);
}
