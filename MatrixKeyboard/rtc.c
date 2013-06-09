#include <stm32f10x_rcc.h>
#include <stm32f10x_pwr.h>
#include <stm32f10x_rtc.h>
#include <stm32f10x_bkp.h>
#include <rtc.h>

// Convert Time to RTC format
uint32_t TimeToRTC(RTC_Time *time) {
	uint32_t result;
	result  = (uint32_t)time->hour * 3600;
	result += (uint32_t)time->min  * 60;
	result +=           time->sec;
	return result;
}

// Convert RTC to time format
void RTCToTime(uint32_t cnt, RTC_Time *time) {
	time->sec  = cnt % 60;
	cnt /= 60;
	time->min  = cnt % 60;
	cnt /= 60;
	time->hour = cnt % 24;
}

// Init RTC
void RTC_Init(void) {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP,ENABLE); // Enable power control and backup domain
	PWR_BackupAccessCmd(ENABLE); // Enable BKP and RTC registers

	// Init and enable RTC if it is not enabled
	if ((RCC->BDCR & RCC_BDCR_RTCEN) != RCC_BDCR_RTCEN) {
		RCC_LSEConfig(RCC_LSE_ON); // Turn on LSE oscillator
		while(!RCC_GetFlagStatus(RCC_FLAG_LSERDY)) {} // Wait till LSE is ready
		while((RCC->BDCR & RCC_BDCR_LSEON) != RCC_BDCR_LSEON) {}
		RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE); // Set LSE as clock source
		RCC_RTCCLKCmd(ENABLE); // Enable RTC clock
		RTC_WaitForSynchro();
		RTC_WaitForLastTask();
		RTC_SetPrescaler(32768); // Set prescaler --> 1Hz with 32.768KHz quartz on demoboard
		RTC_WaitForLastTask();
	}
	RTC_ITConfig(RTC_IT_SEC,ENABLE); // Enable RTC IRQ
	RTC_WaitForLastTask();
}
