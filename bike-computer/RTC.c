#include <stm32l1xx_rcc.h>
#include <stm32l1xx_rtc.h>
#include <stm32l1xx_pwr.h>
#include <RTC.h>

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
