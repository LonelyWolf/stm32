#include "rcc.h"


// Private variables
static __I uint8_t PLLMulTable[9] = {3, 4, 6, 8, 12, 16, 24, 32, 48};
static __I uint8_t APBAHBPrescTable[16] = {0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9};


// Configure the external low speed oscillator (LSE)
// input:
//   LSE - new state of the LSE
// return: SUCCESS if configuration is successful, ERROR otherwise
// note: as the LSE is in the RTC domain and write access is denied to this domain after
//       reset, write access must be enabled before calling this function
ErrorStatus RCC_LSE_cfg(uint32_t LSE) {
	volatile uint32_t wait;

	// Clear LSE bits (LSE disabled)
	RCC->CSR &= ~(RCC_CSR_LSEON | RCC_CSR_LSEBYP | RCC_CSR_LSECSSON);

	if (LSE != _RCC_LSE_OFF) {
		// Turn on LSE and wait until it stable
		RCC->CSR |= LSE;
		wait = RCC_LSE_TIMEOUT;
		while (!(RCC->CSR & RCC_CSR_LSERDY) && --wait);
		if (!(RCC->CSR & RCC_CSR_LSERDY)) return ERROR;
	}

	return SUCCESS;
}

// Enable or disable the internal low speed oscillator (LSI)
// input:
//   LSI - new state of LSI (ENABLE or DISABLE)
// return: SUCCESS if configuration is successful, ERROR otherwise
ErrorStatus RCC_LSI_cfg(FunctionalState LSI) {
	volatile uint32_t timeout = RCC_LSI_TIMEOUT;

	if (LSI == ENABLE) {
		// Enable LSI
		RCC_CSR_LSION_BB = 1;

		// Wait until LSI stable
		while (!(RCC->CSR & RCC_CSR_LSIRDY) && --timeout);
		if (!(RCC->CSR & RCC_CSR_LSIRDY)) return ERROR;
	} else {
		// Disable LSI
		RCC_CSR_LSION_BB = 0;
	}

	return SUCCESS;
}

// Configure the RTC clock source
// input:
//   source - specifies the RTC clock source (one of RCC_RTCCLK_Source_XXX values)
// note: as the RTC clock configuration bits are in the RTC domain and write access
//       to this domain is denied after reset, write access must be enabled before
//       calling this function
void RCC_RTCCLK_cfg(uint32_t source) {
	uint32_t reg;

	// The HSE specified as RTC clock source?
	if ((source & RCC_CSR_RTCSEL_HSE) == RCC_CSR_RTCSEL_HSE) {
		// Configure HSE division for RTC clock
		reg = RCC->CR;
		reg &= ~RCC_CR_RTCPRE; // Clear RTCPRE[1:0] bits
		reg |= source & RCC_CR_RTCPRE;
		RCC->CR = reg; // Write new HSE division factor
	}

	// Which clock source already selected?
	if ((RCC->CSR & RCC_CSR_RTCSEL) != RCC_CSR_RTCSEL_NOCLOCK) {
		// Once the RTC clock source has been selected it cannot be switched until
		// RTC domain is reset or POR occurred
		// Therefore if new RTC clock source differs from currently selected, the RTC
		// domain must be reseted
		if ((RCC->CSR & RCC_CSR_RTCSEL) != (source & RCC_CSR_RTCSEL)) {
			RCC->CSR |=  RCC_CSR_RTCRST;
			__DSB();
			RCC->CSR &= ~RCC_CSR_RTCRST;
			__DSB();
		}
	}

	// Clear RTCSEL[1:0] bits
	reg = RCC->CSR & ~RCC_CSR_RTCSEL;

	// Select new RTC clock source
	RCC->CSR = reg | (source & RCC_CSR_RTCSEL);
}

// Return the frequencies of the System, AHB and APB bus clocks
// input:
//   RCC_Clocks - pointer to a RCC_ClocksTypeDef structure which will hold the
//                clocks frequencies
// note: The frequencies returned by this functions is not the real frequencies of the chip.
//       It is calculated based on the predefined constants and current clocks configuration:
//         If SYSCLK source is MSI, function returns values based on the MSI_VALUE
//         If SYSCLK source is HSI, function returns values based on the HSI_VALUE
//         If SYSCLK source is HSE, function returns values based on the HSE_VALUE
//         If SYSCLK source is PLL, function returns values based on the HSE_VALUE
//           or HSI_VALUE multiplied/divided by the PLL factors.
// note: For compatibility with some SPL functions
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks) {
	uint32_t reg;
	uint32_t divider;

	// Get the SYSCLK source
	reg = RCC->CFGR;
	switch (reg & RCC_CFGR_SWS) {
	case RCC_CFGR_SWS_0:
		// HSI used as system clock
		RCC_Clocks->SYSCLK_Frequency = HSI_VALUE;

		break;
	case RCC_CFGR_SWS_1:
		// HSE used as system clock
		RCC_Clocks->SYSCLK_Frequency = HSE_VALUE;

		break;
	case RCC_CFGR_SWS:
		// PLL used as system clock
		divider = ((reg & RCC_CFGR_PLLDIV) >> 22) + 1;
		RCC_Clocks->SYSCLK_Frequency  = (reg & RCC_CFGR_PLLSRC) ? HSE_VALUE : HSI_VALUE;
		RCC_Clocks->SYSCLK_Frequency *= PLLMulTable[(reg & RCC_CFGR_PLLMUL) >> 18];
		RCC_Clocks->SYSCLK_Frequency /= divider;

		break;
	default:
		// MSI used as system clock
		RCC_Clocks->SYSCLK_Frequency  = (1 << (((RCC->ICSCR & RCC_ICSCR_MSIRANGE) >> 13) + 1)) * 32768;

		break;
	}

	// HCLK clock frequency
	divider = APBAHBPrescTable[(reg & RCC_CFGR_HPRE) >> 4];
	RCC_Clocks->HCLK_Frequency  = RCC_Clocks->SYSCLK_Frequency >> divider;

	// PCLK1 clock frequency
	divider = APBAHBPrescTable[(reg & RCC_CFGR_PPRE1) >> 8];
	RCC_Clocks->PCLK1_Frequency = RCC_Clocks->HCLK_Frequency   >> divider;

	// PCLK2 clock frequency
	divider = APBAHBPrescTable[(reg & RCC_CFGR_PPRE2) >> 11];
	RCC_Clocks->PCLK2_Frequency = RCC_Clocks->HCLK_Frequency   >> divider;
}
