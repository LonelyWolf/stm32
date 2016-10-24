// Various power control functions:
//   - change voltage scaling mode
//   - put the MCU to sleep modes
//   - detect a source of reset


#include "pwr.h"


// Determine source of reset/wakeup
// return: combination of PWR_RESET_SRC_xx values (bitmap)
uint32_t PWR_GetResetSource(void) {
	uint32_t result = PWR_RESET_SRC_UNKNOWN;
	uint32_t reg;
	uint32_t pwrstate = 0;

	// The source of reset
	reg = RCC->CSR;
	if (reg & RCC_CSR_SFTRSTF) {
		// Software reset
		result |= PWR_RESET_SRC_SOFT;
	} else if (reg & RCC_CSR_BORRSTF) {
		// BOR reset
		result |= PWR_RESET_SRC_BOR;
	} else if (reg & RCC_CSR_PINRSTF) {
		// Reset from NRST pin
		result |= PWR_RESET_SRC_PIN;
	}
	if (reg & RCC_CSR_LPWRRSTF) {
		// Reset after illegal Stop, Standby or Shutdown mode entry
		result |= PWR_RESET_SRC_LPWR;
	}
	if (reg & RCC_CSR_WWDGRSTF) {
		// Window watchdog reset
		result |= PWR_RESET_SRC_WWDG;
	}
	if (reg & RCC_CSR_IWDGRSTF) {
		// Independent watchdog reset
		result |= PWR_RESET_SRC_IWDG;
	}
	if (reg & RCC_CSR_FWRSTF) {
		// Firewall reset
		result |= PWR_RESET_SRC_FWR;
	}
	if (reg & RCC_CSR_OBLRSTF) {
		// Option byte load reset
		result |= PWR_RESET_SRC_OBL;
	}

// Clear the reset flags
RCC->CSR |= RCC_CSR_RMVF;

	// Remember state of the PWR peripheral and enable it
	pwrstate = RCC->APB1ENR1;
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
	reg = PWR->SR1;

	if (reg & PWR_SR1_SBF) {
		// Device has been woken from standby mode
		result |= PWR_RESET_SRC_STBY;
	}
	if (reg & PWR_SR1_WUFI) {
		// Device has been woken by internal wakeup line
		result |= PWR_RESET_SRC_STBY_WUFI;
	}
	if (reg & PWR_SR1_WUF1) {
		// Device has been woken by WKUP1 pin
		result |= PWR_RESET_SRC_STBY_WKUP1;
	}
	if (reg & PWR_SR1_WUF2) {
		// Device has been woken by WKUP2 pin
		result |= PWR_RESET_SRC_STBY_WKUP2;
	}
	if (reg & PWR_SR1_WUF3) {
		// Device has been woken by WKUP3 pin
		result |= PWR_RESET_SRC_STBY_WKUP3;
	}
	if (reg & PWR_SR1_WUF4) {
		// Device has been woken by WKUP4 pin
		result |= PWR_RESET_SRC_STBY_WKUP4;
	}
	if (reg & PWR_SR1_WUF5) {
		// Device has been woken by WKUP5 pin
		result |= PWR_RESET_SRC_STBY_WKUP5;
	}

	// Clear all the wakeup flags
	PWR->SCR = PWR_SCR_CSBF | PWR_SCR_CWUF;

	// Restore state of PWR peripheral
	RCC->APB1ENR1 = pwrstate;

	return result;
}

// Enter SLEEP or low-power SLEEP mode
// input:
//   entry - specifies which instruction is used to entry to SLEEP mode (WFI or WFE),
//           can be one of PWR_SENTRY_xx values
// note: Low-power SLEEP mode is entered from the low-power run mode
void PWR_EnterSLEEPMode(uint32_t entry) {
	// Clear SLEEPDEEP bit of Cortex system control register
	SCB->SCR &= ~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);

	// Enter to the SLEEP mode
	if (entry == PWR_SENTRY_WFI) {
		// Wait for interrupt
		__WFI();
	} else {
		// Wait for event
		__SEV(); // Set event register
		__WFE(); // This fires immediately due to previous SEV
		__WFE();
	}
}

// Enter STOP# mode
// input:
//   entry - specifies which instruction is used to entry to STOP mode (WFI or WFE),
//           can be one of PWR_SENTRY_xx values
//   mode - specifies which STOP mode to enter, can be one of PWR_STOP_MODEx values
// note: After exiting from STOP mode the system clock source is determined by
//       the STOPWUCK bit in the RCC_CFGR register:
//         - the HSI oscillator if bit is set
//         - the MSI oscillator if bit is cleared
void PWR_EnterSTOPMode(uint32_t entry, uint32_t mode) {
	// Select low power mode: STOP#
	PWR->CR1 &= ~PWR_CR1_LPMS;
	PWR->CR1 |= mode & (PWR_CR1_LPMS_STOP0 | PWR_CR1_LPMS_STOP1 | PWR_CR1_LPMS_STOP2);

	// Set SLEEPDEEP bit of Cortex system control register
	SCB->SCR |= (uint32_t)SCB_SCR_SLEEPDEEP_Msk;

	// Enter to the selected STOP mode
	if (entry == PWR_SENTRY_WFI) {
		// Wait for interrupt
		__WFI();
	} else {
		// Wait for event
		__SEV(); // Set event register
		__WFE(); // This fires immediately due to previous SEV
		__WFE();
	}

	// Clear SLEEPDEEP bit of Cortex system control register
	SCB->SCR &= ~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);
}

// Enter STANDBY mode
void PWR_EnterSTANDBYMode(void) {
	// Select low power mode: SHUTDOWN
	PWR->CR1 &= ~PWR_CR1_LPMS;
	PWR->CR1 |= PWR_CR1_LPMS_STANDBY;

	// Set SLEEPDEEP bit of Cortex system control register
	SCB->SCR |= (uint32_t)SCB_SCR_SLEEPDEEP_Msk;

	// Clear all the wakeup flags
	PWR->SCR = PWR_SCR_CSBF | PWR_SCR_CWUF;

	// Enter to the SHUTDOWN mode
	__WFI();

	// After waking up from STANDBY mode, program execution restarts in the same way as after a Reset
}

// Enter SHUTDOWN mode
void PWR_EnterSHUTDOWNMode(void) {
	// Select low power mode: SHUTDOWN
	PWR->CR1 &= ~PWR_CR1_LPMS;
	PWR->CR1 |= PWR_CR1_LPMS_SHUTDOWN;

	// Set SLEEPDEEP bit of Cortex system control register
	SCB->SCR |= (uint32_t)SCB_SCR_SLEEPDEEP_Msk;

	// Clear all the wakeup flags
	PWR->SCR = PWR_SCR_CSBF | PWR_SCR_CWUF;

	// Enter to the SHUTDOWN mode
	__WFI();

	// After waking up from STANDBY mode, program execution restarts in the same way as after a Reset
}
