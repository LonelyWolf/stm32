#ifndef __RTC_H
#define __RTC_H


#include <stm32l1xx.h>


// Сode related to RTC wake-up
//   0 - RTC wake-up is not used
//   1 - RTC wake-up code enabled
#define USE_WKUP              1

// Сode related to RTC alarms
//   0 - RTC alarms is not used
//   1 - RTC alarms code enabled
#define USE_ALARMS            0


// RTC ALARM internally connected to EXTI17
#define RTC_ALARM_EXTI        (1 << 17)
// RTC WKUP internally connected to EXTI20
#define RTC_WKUP_EXTI         (1 << 20)

// The RTC initialization timeout
#define RTC_INIT_TIMEOUT      ((uint32_t)0x00002000)
// The RTC synchronization timeout
#define RTC_SYNC_TIMEOUT      ((uint32_t)0x00008000)
// Reserved bits in the RTC_TR register
#define RTC_TR_RESERVED_MASK  ((uint32_t)0x007F7F7F)
// Reserved bits in the RTC_DR register
#define RTC_DR_RESERVED_MASK  ((uint32_t)0x00FFFF3F)

// Alarms
#define RTC_ALARM_A           ((uint32_t)RTC_CR_ALRAE)
#define RTC_ALARM_B           ((uint32_t)RTC_CR_ALRBE)

// Alarm mask
#define RTC_ALARM_MASK_WD     ((uint32_t)RTC_ALRMAR_WDSEL) // If set then date units, day of week otherwise
#define RTC_ALARM_MASK_DAY    ((uint32_t)RTC_ALRMAR_MSK4)  // If set date/day don't care
#define RTC_ALARM_MASK_HRS    ((uint32_t)RTC_ALRMAR_MSK3)  // If set hours don't care
#define RTC_ALARM_MASK_MIN    ((uint32_t)RTC_ALRMAR_MSK2)  // If set minutes don't care
#define RTC_ALARM_MASK_SEC    ((uint32_t)RTC_ALRMAR_MSK1)  // If set seconds don't care

// RTC interrupts
#define RTC_IT_TS             ((uint32_t)RTC_CR_TSIE)   // Time stamp
#define RTC_IT_WUT            ((uint32_t)RTC_CR_WUTIE)  // Wake-up
#define RTC_IT_ALRA           ((uint32_t)RTC_CR_ALRAIE) // Alarm A
#define RTC_IT_ALRB           ((uint32_t)RTC_CR_ALRBIE) // Alarm B

// Days of week
#define RTC_DOW_MONDAY        ((uint8_t)0x01)
#define RTC_DOW_TUESDAY       ((uint8_t)0x02)
#define RTC_DOW_WEDNESDAY     ((uint8_t)0x03)
#define RTC_DOW_THURSDAY      ((uint8_t)0x04)
#define RTC_DOW_FRIDAY        ((uint8_t)0x05)
#define RTC_DOW_SATURDAY      ((uint8_t)0x06)
#define RTC_DOW_SUNDAY        ((uint8_t)0x07)

// Months
#define RTC_MONTH_JANUARY     ((uint8_t)0x01)
#define RTC_MONTH_FEBRUARY    ((uint8_t)0x02)
#define RTC_MONTH_MARCH       ((uint8_t)0x03)
#define RTC_MONTH_APRIL       ((uint8_t)0x04)
#define RTC_MONTH_MAY         ((uint8_t)0x05)
#define RTC_MONTH_JUNE        ((uint8_t)0x06)
#define RTC_MONTH_JULY        ((uint8_t)0x07)
#define RTC_MONTH_AUGUST      ((uint8_t)0x08)
#define RTC_MONTH_SEPTEMBER   ((uint8_t)0x09)
#define RTC_MONTH_OCTOBER     ((uint8_t)0x10)
#define RTC_MONTH_NOVEMBER    ((uint8_t)0x11)
#define RTC_MONTH_DECEMBER    ((uint8_t)0x12)

// Count epoch value from 01.01.2015 (this value represents days JDN of 01.01.2015)
// To user Unix epoch time define '2440588' here
#define JULIAN_DATE_BASE      2457023


// Days of week text notation (first element is empty, because RTC count days from '1')
static char const * const RTC_DOW_STR[] = {
		"",
		"MON",
		"TUE",
		"WED",
		"THU",
		"FRI",
		"SAT",
		"SUN"
};


// RTC time structure
typedef struct {
	uint8_t RTC_Hours;   // RTC time hour, the value range is [0..23] or [0..12] depending of hour format
	uint8_t RTC_Minutes; // RTC time minutes, the value range is [0..59]
	uint8_t RTC_Seconds; // RTC time minutes, the value range is [0..59]
	uint8_t RTC_H12;     // RTC AM/PM time
} RTC_TimeTypeDef;

// RTC date structure
typedef struct {
	uint8_t RTC_WeekDay; // RTC date week day (one of RTC_DOW_XXX definitions)
	uint8_t RTC_Month;   // RTC date month (in BCD format, one of RTC_MONTH_XXX definitions)
	uint8_t RTC_Date;    // RTC date, the value range is [1..31]
	uint8_t RTC_Year;    // RTC date year, the value range is [0..99]
} RTC_DateTypeDef;


extern RTC_TimeTypeDef RTC_Time; // Current RTC time
extern RTC_DateTypeDef RTC_Date; // Current RTC date


// Function prototypes
ErrorStatus RTC_WaitForSynchro(void);
ErrorStatus RTC_EnterInitMode(void);
ErrorStatus RTC_Config(void);
#if (USE_WKUP)
ErrorStatus RTC_SetWakeUp(uint32_t interval);
#endif // USE_WKUP
#if (USE_ALARMS)
void RTC_SetAlarm(uint32_t Alarm, RTC_TimeTypeDef *time, uint8_t AlarmDateDay, uint32_t AlarmMask);
ErrorStatus RTC_AlarmCmd(uint32_t Alarm, FunctionalState NewState);
#endif // USE_ALARMS
void RTC_ITConfig(uint32_t IT, FunctionalState NewState);

ErrorStatus RTC_SetDateTime(RTC_TimeTypeDef *time, RTC_DateTypeDef *date);
void RTC_GetDateTime(RTC_TimeTypeDef *time, RTC_DateTypeDef *date);

uint32_t RTC_ToEpoch(RTC_TimeTypeDef *time, RTC_DateTypeDef *date);
void RTC_FromEpoch(uint32_t epoch, RTC_TimeTypeDef *time, RTC_DateTypeDef *date);
void RTC_AdjustTimeZone(RTC_TimeTypeDef *time, RTC_DateTypeDef *date, int8_t offset);
void RTC_CalcDOW(RTC_DateTypeDef *date);

#endif // __RTC_H
