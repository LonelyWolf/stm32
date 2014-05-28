// Define to prevent recursive inclusion -------------------------------------
#ifndef __RTC_H
#define __RTC_H


#define JULIAN_DATE_BASE     2440588   // Unix epoch time in Julian calendar (UnixTime = 00:00:00 01.01.1970 => JDN = 2440588)


// Function prototypes
void RTC_Config(void);
uint32_t RTC_ToEpoch(RTC_TimeTypeDef *time, RTC_DateTypeDef *date);
void RTC_FromEpoch(uint32_t epoch, RTC_TimeTypeDef *time, RTC_DateTypeDef *date);
void RTC_AdjustTimeZone(RTC_TimeTypeDef *time, RTC_DateTypeDef *date, int8_t offset);

#endif // __RTC_H
