// Define to prevent recursive inclusion -------------------------------------
#ifndef __RTC_H
#define __RTC_H


#include <stm32l1xx_rtc.h>


#define RTC_EXTI_LINE        1 << 20   // RTC connected to EXTI_Line20

#define JULIAN_DATE_BASE     2440588   // Unix epoch time in Julian calendar (UnixTime = 00:00:00 01.01.1970 => JDN = 2440588)

static const uint16_t week_day[] = { 0x4263, 0xA8BD, 0x42BF, 0x4370, 0xABBF, 0xA8BF, 0x43B2 };


extern RTC_TimeTypeDef RTC_Time;                   // Current RTC time
extern RTC_DateTypeDef RTC_Date;                   // Current RTC date


// Function prototypes
void RTC_Config(void);
void RTC_SetWakeUp(uint32_t interval);
void RTC_SetDateTime(RTC_TimeTypeDef *time, RTC_DateTypeDef *date);
void RTC_GetDateTime(RTC_TimeTypeDef *time, RTC_DateTypeDef *date);

uint32_t RTC_ToEpoch(RTC_TimeTypeDef *time, RTC_DateTypeDef *date);
void RTC_FromEpoch(uint32_t epoch, RTC_TimeTypeDef *time, RTC_DateTypeDef *date);
void RTC_AdjustTimeZone(RTC_TimeTypeDef *time, RTC_DateTypeDef *date, int8_t offset);

#endif // __RTC_H
