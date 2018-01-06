#ifndef __RTC_H
#define __RTC_H


#include <stm32l4xx.h>
#include "rcc.h"


// Memo for STM32L4:
// RTC ALARM internally connected to EXTI18
// RTC WKUP internally connected to EXTI20


// Compilation parameters

// Code related to RTC wake-up:
//   0 - RTC wake-up is not used
//   1 - RTC wake-up code enabled
#define RTC_USE_WAKEUP             1

// Code related to RTC alarms:
//   0 - RTC alarms is not used
//   1 - RTC alarms code enabled
#define RTC_USE_ALARMS             0

// Code related to epoch calculations
//   0 - no epoch related functions
//   1 - epoch related functions enabled
#define RTC_USE_EPOCH              1

// Code related to set date and time
//   0 - Variant #1: enter INIT mode,
//       compose new values for date and time registers,
//       then write them
//   1 - Variant #2: initialize entry to the INIT mode, compose new values for the date/time
//       registers and then wait for the INIT mode, then write new values to the RTC registers
//       Looks less pretty than the variant #1, but gives a performance gain in those cases
//       when the frequency of the MCU is rather low
#define RTC_USE_SETDATETIME        0


// Reserved bits in the RTC_TR register
#define RTC_TR_RESERVED_MASK       ((uint32_t)0x007F7F7FU)
// Reserved bits in the RTC_DR register
#define RTC_DR_RESERVED_MASK       ((uint32_t)0x00FFFF3FU)

#if (RTC_USE_ALARMS)
// Alarms
#define RTC_ALARM_A                RTC_CR_ALRAE
#define RTC_ALARM_B                RTC_CR_ALRBE

// Alarm mask
#define RTC_ALARM_MASK_WD          RTC_ALRMAR_WDSEL // If set then date units, day of week otherwise
#define RTC_ALARM_MASK_DAY         RTC_ALRMAR_MSK4  // If set date/day don't care
#define RTC_ALARM_MASK_HRS         RTC_ALRMAR_MSK3  // If set hours don't care
#define RTC_ALARM_MASK_MIN         RTC_ALRMAR_MSK2  // If set minutes don't care
#define RTC_ALARM_MASK_SEC         RTC_ALRMAR_MSK1  // If set seconds don't care
#endif // RTC_USE_ALARMS

// RTC interrupts
#define RTC_IT_TS                  RTC_CR_TSIE   // Time stamp
#define RTC_IT_WUT                 RTC_CR_WUTIE  // Wake-up
#define RTC_IT_ALRA                RTC_CR_ALRAIE // Alarm A
#define RTC_IT_ALRB                RTC_CR_ALRBIE // Alarm B

// RTC wakeup clock selection
#define RTC_WUCLCK_DIV_16          ((uint32_t)0x00000000U)               // RTC/16
#define RTC_WUCLCK_DIV_8           RTC_CR_WUCKSEL_0                      // RTC/8
#define RTC_WUCLCK_DIV_4           RTC_CR_WUCKSEL_1                      // RTC/4
#define RTC_WUCLCK_DIV_2           (RTC_CR_WUCKSEL_1 | RTC_CR_WUCKSEL_0) // RTC/2
#define RTC_WUCLCK_CKSPRE          RTC_CR_WUCKSEL_2                      // ck_spre
#define RTC_WUCLCK_CKSPRE_WUT      (RTC_CR_WUCKSEL_2 | RTC_CR_WUCKSEL_1) // ck_spre and 2^16 is added to the WUT counter

// Days of week
#define RTC_DOW_MONDAY             ((uint8_t)0x01U)
#define RTC_DOW_TUESDAY            ((uint8_t)0x02U)
#define RTC_DOW_WEDNESDAY          ((uint8_t)0x03U)
#define RTC_DOW_THURSDAY           ((uint8_t)0x04U)
#define RTC_DOW_FRIDAY             ((uint8_t)0x05U)
#define RTC_DOW_SATURDAY           ((uint8_t)0x06U)
#define RTC_DOW_SUNDAY             ((uint8_t)0x07U)

// Months
#define RTC_MONTH_JANUARY          ((uint8_t)0x01U)
#define RTC_MONTH_FEBRUARY         ((uint8_t)0x02U)
#define RTC_MONTH_MARCH            ((uint8_t)0x03U)
#define RTC_MONTH_APRIL            ((uint8_t)0x04U)
#define RTC_MONTH_MAY              ((uint8_t)0x05U)
#define RTC_MONTH_JUNE             ((uint8_t)0x06U)
#define RTC_MONTH_JULY             ((uint8_t)0x07U)
#define RTC_MONTH_AUGUST           ((uint8_t)0x08U)
#define RTC_MONTH_SEPTEMBER        ((uint8_t)0x09U)
#define RTC_MONTH_OCTOBER          ((uint8_t)0x10U)
#define RTC_MONTH_NOVEMBER         ((uint8_t)0x11U)
#define RTC_MONTH_DECEMBER         ((uint8_t)0x12U)

// Definition of the Julian day number
// Epoch will start at 31 Dec 1999 12:00:00 (since the STM32 RTC start counting from 01 Jan 2000 00:00:00)
// note: to use Unix epoch time define '2440588' here
#define RTC_JDN                    ((uint32_t)2451544U) // 31 Dec 1999 12:00:00


// Days of week text notation
// The first element of array is empty because RTC counts days from '1'
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


// Public macros/functions

// Enable the write protection for RTC registers
__STATIC_INLINE void RTC_WriteProtectionEnable(void) {
	RTC->WPR = (uint8_t)0xFFU;
}

// Disable the write protection for RTC registers
__STATIC_INLINE void RTC_WriteProtectionDisable(void) {
	RTC->WPR = (uint8_t)0xCAU;
	RTC->WPR = (uint8_t)0x53U;
}

// Disable the RTC initialization mode
__STATIC_INLINE void RTC_ExitInitMode(void) {
	RTC->ISR &= ~RTC_ISR_INIT;
}

#if (RTC_USE_WAKEUP)
// Clear the RTC wakeup timer flag (WUTF)
__STATIC_INLINE void RTC_ClearWUTF(void) {
	RTC->ISR = ~((RTC_ISR_WUTF | RTC_ISR_INIT) & 0x0000FFFFU) | (RTC->ISR & RTC_ISR_INIT);
}
#endif // RTC_USE_WAKEUP

#if (RTC_USE_ALARMS)
// Clear the RTC alarm A flag
__STATIC_INLINE void RTC_ClearALRAF(void) {
	RTC->ISR = ~((RTC_ISR_ALRAF | RTC_ISR_INIT) & 0x0000FFFFU) | (RTC->ISR & RTC_ISR_INIT);
}

// Clear the RTC alarm B flag
__STATIC_INLINE void RTC_ClearALRBF(void) {
	RTC->ISR = ~((RTC_ISR_ALRBF | RTC_ISR_INIT) & 0x0000FFFFU) | (RTC->ISR & RTC_ISR_INIT);
}
#endif // RTC_USE_ALARMS


// Function prototypes
ErrorStatus RTC_WaitForSynchro(void);
ErrorStatus RTC_EnterInitMode(void);
ErrorStatus RTC_Init(uint32_t asynch, uint32_t synch);

#if (RTC_USE_WAKEUP)
ErrorStatus RTC_SetWakeupClock(uint32_t clk_cfg);
ErrorStatus RTC_SetWakeup(uint32_t interval);
#endif // RTC_USE_WAKEUP

#if (RTC_USE_ALARMS)
void RTC_AlarmInit(uint32_t alarm, RTC_TimeTypeDef *alarm_time, uint32_t alarm_mask, uint8_t alarm_dateday);
ErrorStatus RTC_AlarmSet(uint32_t Alarm, FunctionalState NewState);
#endif // RTC_USE_ALARMS

void RTC_ITConfig(uint32_t IT, FunctionalState NewState);
void RTC_BypassShadowConfig(FunctionalState NewState);

ErrorStatus RTC_SetDateTime(RTC_TimeTypeDef *time, RTC_DateTypeDef *date);
void RTC_GetDateTime(RTC_TimeTypeDef *time, RTC_DateTypeDef *date);

#if (RTC_USE_EPOCH)
uint32_t RTC_ToEpoch(RTC_TimeTypeDef *time, RTC_DateTypeDef *date);
void RTC_FromEpoch(uint32_t epoch, RTC_TimeTypeDef *time, RTC_DateTypeDef *date);
void RTC_AdjustTimeZone(RTC_TimeTypeDef *time, RTC_DateTypeDef *date, int8_t offset);
#endif // RTC_USE_EPOCH

void RTC_CalcDOW(RTC_DateTypeDef *date);

void RTC_BKUPWrite(uint32_t bkup_reg, uint32_t data);
uint32_t RTC_BKUPRead(uint32_t bkup_reg);

#endif // __RTC_H
