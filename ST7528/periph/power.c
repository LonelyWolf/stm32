// Various power control functions:
//   - put the MCU to sleep modes
//   - detect a source of reset


#include "power.h"


// Execute WFI instruction
void SleepWait(void) {
	// Enable sleep-on-exit (after IRQ processing the MCU will fall asleep without returning to main loop)
	SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;

	// Clear the WUF wake-up flag
	PWR->CR |= PWR_CR_CWUF;

	// Ensure effect of last store takes effect
	__DSB();

	// Enter sleep mode
	__WFI();
}

// Put MCU into STOP mode
// input:
//   RestoreClocks - restore or not system clocks after wake-up (can be one of CLOCK_XXX values)
void SleepStop(ClockAction RestoreClocks) {
	uint32_t tmp_reg;

	// If PDDS bit is set -> STANDBY mode, STOP otherwise
	PWR->CR &= (uint32_t)~((uint32_t)~PWR_CR_PDDS);

	// Voltage regulator on during sleep mode
	PWR->CR &= (uint32_t)~((uint32_t)~PWR_CR_LPSDSR);

	// Clear the wake-up flag (WUF)
	PWR->CR |= PWR_CR_CWUF;

	// Disable sleep-on-exit
	SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;

	// Set SLEEPDEEP bit of Cortex-M System Control Register
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

	// Ensure effect of last store takes effect
	__DSB();

	// Enter STOP mode
	__WFI();

	// Clear SLEEPDEEP bit of Cortex-M System Control Register
	SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;

	// Return if restore of system clocks is not needed
	if (!RestoreClocks) return;

	// After wake-up from STOP mode system clocks are feed from HSI
	// Reconfigure them to use HSE and PLL

	// Turn on the HSE and wait till it ready
	RCC->CR |= RCC_CR_HSEON;
	while (!(RCC->CR & RCC_CR_HSERDY));

	// Turn on the PLL and wait till it ready
	RCC->CR |= RCC_CR_PLLON;
	while (!(RCC->CR & RCC_CR_PLLRDY));

	// Select PLL as system clock source
	tmp_reg  = RCC->CFGR;
	tmp_reg &= ~RCC_CFGR_SW;
	tmp_reg |= RCC_CFGR_SW_PLL;
	RCC->CFGR = tmp_reg;

	// Wait till PLL is used as system clock source
	while ((uint8_t)(RCC->CFGR & RCC_CFGR_SWS) != 0x0c);

	// Update SystemCoreClock according to clock register values
	SystemCoreClockUpdate();
}

// Put MCU into Standby mode
void SleepStandby(void) {
	// Clear the wake-up and standby flags
	PWR->CR |= (PWR_CR_CWUF | PWR_CR_CSBF);

	// Enter STANDBY mode when the CPU enters deepsleep
	PWR->CR |= PWR_CR_PDDS;

	// Set SLEEPDEEP bit of Cortex-M System Control Register
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

	// Ensure effect of last store takes effect
	__DSB();

	// Enter STANDBY mode
	__WFI();

	// After waking up from STANDBY mode, program execution restarts in the same way as after a Reset
}

// Determine source of reset
uint32_t GetResetSource(void) {
	uint32_t result = RESET_SRC_UNKNOWN;
	uint32_t reg;

	// Reset source
	reg = RCC->CSR;
	if (reg & RCC_CSR_SFTRSTF) {
		result |= RESET_SRC_SOFT;
	} else if (reg & RCC_CSR_PORRSTF) {
		result |= RESET_SRC_POR;
	} else if (reg & RCC_CSR_PINRSTF) {
		result |= RESET_SRC_PIN;
	}

	// Clear the reset flags
	RCC->CSR |= RCC_CSR_RMVF;

	// Enable the PWR peripheral to deal with it CSR register
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	__DSB();

	// WKUP pin or RTC alarm
	reg = PWR->CSR;
	if (reg & PWR_CSR_WUF) {
		// Is this wake from Standby?
		if (reg & PWR_CSR_SBF) result |= RESET_SRC_STBY;

		// Clear the wake-up and standby flags
		PWR->CR |= PWR_CR_CWUF | PWR_CR_CSBF;

#if (POWER_USE_WKUP_PINS)
		// Remember value of the AHBENR register
		reg = RCC->AHBENR;

		// Determine a state of the WKUP# pins
#if (POWER_USE_WKUP1)
		RCC->AHBENR = reg | WKUP1_AHB_PERIPHERAL;
		__DSB();
		if (WKUP1_STATE) result |= RESET_SRC_STBY_WP1;
#endif

#if (POWER_USE_WKUP2)
		RCC->AHBENR = reg | WKUP2_AHB_PERIPHERAL;
		__DSB();
		if (WKUP2_STATE) result |= RESET_SRC_STBY_WP2;
#endif

#if (POWER_USE_WKUP3)
		RCC->AHBENR = reg | WKUP3_AHB_PERIPHERAL;
		__DSB();
		if (WKUP3_STATE) result |= RESET_SRC_STBY_WP3;
#endif

		// Restore value of the AHBENR register
		RCC->AHBENR = reg;
#endif // POWER_USE_WKUP_PINS
	}

	return result;
}
