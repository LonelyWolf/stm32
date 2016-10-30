#include "rcc.h"


// Startup timeouts
#define RCC_TIMEOUT_MSI            100U    // MSI, about 1ms
#define RCC_TIMEOUT_HSI            100U    // HSI, about 1ms
#define RCC_TIMEOUT_HSE            50000U  // HSE, about 0.5s
#define RCC_TIMEOUT_PLL            10U     // PLL, about 100us
#define RCC_TIMEOUT_LSI            50U     // LSI, about 500us
#define RCC_TIMEOUT_LSE            500000U // LSE, about 5s


// Count rough delay for timeouts
// input:
//   delay - desired delay duration
// return: value for timeout counter
static uint32_t RCC_CalcDelay(uint32_t delay) {
	uint32_t cnt;

	if (SystemCoreClock > 1000000U) {
		cnt = (delay * ((SystemCoreClock / 1000000U) + 1U));
	} else {
		cnt = (((delay / 100U) + 1U) * ((SystemCoreClock / 10000U) + 1U));
	}

	return cnt;
}

// Get current PLLM divider value
// return: value of PLLM divider (one of RCC_PLLM_DIVx values)
__STATIC_INLINE uint32_t RCC_GetPLLMDiv(void) {
	return (RCC->PLLCFGR & RCC_PLLCFGR_PLLM);
}

// Calculate the output PLL frequency according to given settings
// input:
//   inPLL - frequency of PLL input clock (Hz)
//   cfgPLL - pointer to the structure what contains the configuration for the PLL
// return: output PLL frequency (Hz)
static uint32_t RCC_CalcPLLFreq(uint32_t inPLL, uint32_t pllm, RCC_PLLInitTypeDef *cfgPLL) {
	uint32_t pllfreq;

	pllfreq  = inPLL / ((pllm >> 4) + 1);
	pllfreq *= cfgPLL->PLLN;
	pllfreq /= ((cfgPLL->PLLR >> 25) + 1) << 1;

	return pllfreq;
}

// Configure the FLASH wait states according to a specified HCLK frequency
// and current voltage range
// input:
//   hclk - HCLK clock frequency
static void RCC_SetFlashLatency(uint32_t hclk) {
	uint32_t latency = FLASH_ACR_LATENCY_0WS; // default value is 0WS
	uint32_t reg;

	if ((PWR->CR1 & PWR_CR1_VOS) == PWR_CR1_VOS_0) {
		// Voltage scaling range 1
		if (hclk > RCC_SCALE1_LATENCY4) {
			// 64..80MHz => 4WS
			latency = FLASH_ACR_LATENCY_4WS;
		} else if (hclk > RCC_SCALE1_LATENCY3) {
			// 48..64MHz => 3WS
			latency = FLASH_ACR_LATENCY_3WS;
		} else if (hclk > RCC_SCALE1_LATENCY2) {
			// 32..48MHz => 2WS
			latency = FLASH_ACR_LATENCY_2WS;
		} else if (hclk > RCC_SCALE1_LATENCY1) {
			// 16..32MHz => 1WS
			latency = FLASH_ACR_LATENCY_1WS;
		}
		// else default latency 0WS
	} else {
		// Voltage scaling range 2
		if (hclk > RCC_SCALE2_LATENCY3) {
			// 18..26MHz => 3WS
			latency = FLASH_ACR_LATENCY_3WS;
		} else if (hclk > RCC_SCALE2_LATENCY2) {
			// 12..18MHz => 2WS
			latency = FLASH_ACR_LATENCY_2WS;
		} else if (hclk > RCC_SCALE2_LATENCY1) {
			//  6..12MHz => 1WS
			latency = FLASH_ACR_LATENCY_1WS;
		}
		// else default latency 0WS
	}

	// Configure the new flash latency
	reg  = FLASH->ACR;
	reg &= ~FLASH_ACR_LATENCY;
	reg |= latency;
	FLASH->ACR = reg;
}

// Turn on PLL and switch the system clock to it
// input:
//   SysClkFreq - frequency of the System clock (Hz)
//   cfgCLK - pointer to the structure what contains the configuration for the bus prescalers
// return: ERROR in case when the PLL had not lock, SUCCESS otherwise
// note: the PLL must be already configured
static ErrorStatus RCC_SwitchToPLL(uint32_t SysClkFreq, RCC_CLKInitTypeDef *cfgCLK) {
	volatile uint32_t wait;
	uint32_t hclk;
	uint32_t reg;
	uint32_t bkp;

	// Calculate the new HCLK frequency
	hclk = SysClkFreq >> AHBPrescTable[(cfgCLK->AHBdiv & RCC_CFGR_HPRE) >> 4];

	// In case of increasing the frequency the number of wait states should be
	// increased before the clocks configuration
	if (hclk > SystemCoreClock) {
		RCC_SetFlashLatency(hclk);
	}

	// Configure new AHB prescaler and system clock source
	reg  = RCC->CFGR;
	bkp  = reg;
	reg &= ~(RCC_CFGR_HPRE | RCC_CFGR_SW);
	reg |= RCC_CFGR_SW_PLL | (cfgCLK->AHBdiv & RCC_CFGR_HPRE);
	RCC->CFGR = reg;

	// Wait for the system clock to switch to PLL
	wait = RCC_CalcDelay(RCC_TIMEOUT_PLL);
	while (((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) && --wait);
	if (wait == 0) {
		// The PLL had not lock
		// Restore previous AHB prescaler and flash latency
		RCC->CFGR = bkp;
		RCC_SetFlashLatency(SystemCoreClock);

		return ERROR;
	}

	// In case of decreasing the frequency, the number of wait states should be
	// decreased after the clocks configuration
	if (hclk < SystemCoreClock) {
		RCC_SetFlashLatency(hclk);
	}

	// Update the SystemCoreClock variable
	SystemCoreClock = hclk;

	// Configure the new APB1 and APB2 prescalers
	reg  = RCC->CFGR;
	reg &= ~(RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
	reg |= (cfgCLK->APB1div & RCC_CFGR_PPRE1) | (cfgCLK->APB2div & RCC_CFGR_PPRE2);
	RCC->CFGR = reg;

	return SUCCESS;
}

// Get the current system clock source
// return: clock source, one of RCC_SYSCLK_SRC_xx values
uint32_t RCC_GetSysClockSource(void) {
	uint32_t result = RCC_SYSCLK_SRC_UNKNOWN;

	// Get the SYSCLK source
	switch (RCC->CFGR & RCC_CFGR_SWS) {
		case RCC_CFGR_SWS_HSI:
			result = RCC_SYSCLK_SRC_HSI;

			break;
		case RCC_CFGR_SWS_HSE:
			result = RCC_SYSCLK_SRC_HSE;

			break;
		case RCC_CFGR_SWS_PLL:
			switch (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) {
				case RCC_PLLCFGR_PLLSRC_HSI:
					result = RCC_SYSCLK_SRC_HSIPLL;

					break;
				case RCC_PLLCFGR_PLLSRC_HSE:
					result = RCC_SYSCLK_SRC_HSEPLL;

					break;
				default:
					result = RCC_SYSCLK_SRC_MSIPLL;

					break;
			}

			break;
		default:
			result = RCC_SYSCLK_SRC_MSI;

			break;
	}

	return result;
}

// Get current MSI clock frequency
// return: MSI frequency (Hz)
uint32_t RCC_GetMSIFreq(void) {
	uint32_t reg = RCC->CR;

	if (reg & RCC_CR_MSIRGSEL) {
		// MSI Range set by MSIRANGE[3:0] in the RCC_CR register
		return MSIRangeTable[(reg & RCC_CR_MSIRANGE) >> 4];
	} else {
		// MSI Range set by MSISRANGE[3:0] in RCC_CSR register
		return MSIRangeTable[(RCC->CSR & RCC_CSR_MSISRANGE) >> 8];
	}
}

// Get current SYSCLK clock frequency
// return: SYSCLK frequency (Hz)
uint32_t RCC_GetSYSCLKFreq(void) {
	uint32_t freq;
	uint32_t reg;

	// Get the SYSCLK source
	switch (RCC->CFGR & RCC_CFGR_SWS) {
	case RCC_CFGR_SWS_HSI:
		// HSI used as system clock
		freq = HSI_VALUE;

		break;
	case RCC_CFGR_SWS_HSE:
		// HSE used as system clock
		freq = HSE_VALUE;

		break;
	case RCC_CFGR_SWS_PLL:
		// PLL used as system clock
		reg = RCC->PLLCFGR;
		switch (reg & RCC_PLLCFGR_PLLSRC) {
		case RCC_PLLCFGR_PLLSRC_HSI:
			// HSI used as PLL clock source
			freq = HSI_VALUE;

			break;
		case RCC_PLLCFGR_PLLSRC_HSE:
			// HSE used as PLL clock source
			freq = HSE_VALUE;

			break;
		default:
			// MSI used as PLL clock source
			freq = RCC_GetMSIFreq();

			break;
		}

		// PLLM divider
		freq /= ((reg & RCC_PLLCFGR_PLLM) >> 4) + 1;
		// PLLN multiplier
		freq *= (reg & RCC_PLLCFGR_PLLN) >> 8;
		// PLLR divider
		freq /= (((reg & RCC_PLLCFGR_PLLR) >> 25) + 1) * 2;

		break;
	default:
		// MSI used as system clock
		freq = RCC_GetMSIFreq();

		break;
	}

	return freq;
}

// Get current HCLK clock frequency
// input:
//   sysclk - SYSCLK frequency (Hz)
// return: HCLK frequency (Hz)
uint32_t RCC_GetHCLKFreq(uint32_t sysclk) {
	return (sysclk >> AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE) >> 4]);
}

// Get current PCLK1 (APB1) clock frequency
// input:
//   hclk - HCLK frequency (Hz)
// return: PCLK1 frequency (Hz)
uint32_t RCC_GetPCLK1Freq(uint32_t hclk) {
	return (hclk >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1) >> 8]);
}

// Get current PCLK2 (APB2) clock frequency
// input:
//   hclk - HCLK frequency (Hz)
// return: PCLK2 frequency (Hz)
uint32_t RCC_GetPCLK2Freq(uint32_t hclk) {
	return (hclk >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE2) >> 11]);
}

// Return the frequencies of the System, AHB and APB bus clocks
// input:
//   RCC_Clocks - pointer to a RCC_ClocksTypeDef structure which will hold the
//                clocks frequencies
// note: The frequencies returned by this functions is not the real frequencies of the chip.
//       It is calculated based on the predefined constants and current clocks configuration:
//         If SYSCLK source is MSI, function returns values based on the current MSI settings
//         If SYSCLK source is HSI, function returns values based on the HSI_VALUE
//         If SYSCLK source is HSE, function returns values based on the HSE_VALUE
//         If SYSCLK source is PLL, function returns values based on the HSE_VALUE, HSI_VALUE or
//           current MSI values multiplied/divided by the PLL factors.
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks) {
	RCC_Clocks->SYSCLK_Frequency = RCC_GetSYSCLKFreq();
	RCC_Clocks->HCLK_Frequency   = RCC_GetHCLKFreq(RCC_Clocks->SYSCLK_Frequency);
	RCC_Clocks->PCLK1_Frequency  = RCC_GetPCLK1Freq(RCC_Clocks->HCLK_Frequency);
	RCC_Clocks->PCLK2_Frequency  = RCC_GetPCLK2Freq(RCC_Clocks->HCLK_Frequency);
}

// Configure the PLLs clock source
// input:
//   pll_src - new clock source for PLLs, one of RCC_PLLSRC_xxx values
// note: PLLs clock can be changed only when all PLLs are disabled
// note: if RCC_PLLSRC_NONE values specified then all PLLs will be disabled
void RCC_PLLSrcConfig(uint32_t pll_src) {
	// Check if all PLLs are disabled
	if (RCC->CR & (RCC_CR_PLLON | RCC_CR_PLLSAI1ON | RCC_CR_PLLSAI2ON)) {
		// Modifying the PLL input clock when one of PLLs are enabled is bad idea...
	} else {
		RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLSRC);
		if (pll_src != RCC_PLLSRC_NONE) {
			RCC->PLLCFGR |= pll_src & RCC_PLLCFGR_PLLSRC;
		} else {
			// Since no clock will be sent to PLLs, disable them all to save power
			RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_PLLSAI1ON | RCC_CR_PLLSAI2ON);
		}
	}
}

// Configure the PLLM divider (for PLL, PLLSAI1 and PLLSAI2 input clock)
// input:
//   pllm - input clock frequency divider, one of RCC_PLLM_DIVx values
// note: PLLM can be changed only when all PLLs are disabled
void RCC_PLLMConfig(uint32_t pllm) {
	// Check if all PLLs are disabled
	if (RCC->CR & (RCC_CR_PLLON | RCC_CR_PLLSAI1ON | RCC_CR_PLLSAI2ON)) {
		// Modifying the PLL input clock divider when one of PLLs are enabled is bad idea...
	} else {
		RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLM);
		RCC->PLLCFGR |= pllm & RCC_PLLCFGR_PLLM;
	}
}

// Enable the PLL outputs
// input:
//   pll - which PLL must be configured, one of RCC_PLL_xxx values
//   pll_out - which PLL outputs must be enabled, combination of RCC_PLL_OUTx values
// note: PLLSAI2 doesn't have PLLQ output
void RCC_PLLOutEnable(uint32_t pll, uint32_t pll_out) {
	switch (pll) {
		case RCC_PLL_MAIN:
			if (pll_out & RCC_PLL_OUTR) RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;
			if (pll_out & RCC_PLL_OUTQ) RCC->PLLCFGR |= RCC_PLLCFGR_PLLQEN;
			if (pll_out & RCC_PLL_OUTP) RCC->PLLCFGR |= RCC_PLLCFGR_PLLPEN;

			break;
		case RCC_PLL_SAI1:
			if (pll_out & RCC_PLL_OUTR) RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1REN;
			if (pll_out & RCC_PLL_OUTQ) RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1QEN;
			if (pll_out & RCC_PLL_OUTP) RCC->PLLSAI1CFGR |= RCC_PLLSAI1CFGR_PLLSAI1PEN;

			break;
		case RCC_PLL_SAI2:
			if (pll_out & RCC_PLL_OUTR) RCC->PLLSAI2CFGR |= RCC_PLLSAI2CFGR_PLLSAI2REN;
			if (pll_out & RCC_PLL_OUTP) RCC->PLLSAI2CFGR |= RCC_PLLSAI2CFGR_PLLSAI2PEN;

			break;
		default:
			// Wrong PLL specified, do nothing

			break;
	}

}

// Disable the PLL outputs
// input:
//   pll - which PLL must be configured, one of RCC_PLL_xxx values
//   pll_out - which PLL outputs must be disabled, combination of RCC_PLL_OUTx values
// note: PLLSAI2 doesn't have PLLQ output
// note: PLLR out of the main PLL must not be disabled when PLL used as system clock source
void RCC_PLLOutDisable(uint32_t pll, uint32_t pll_out) {
	switch (pll) {
		case RCC_PLL_MAIN:
			if (pll_out & RCC_PLL_OUTR) {
				// Just as foolproof: check what PLL is not used as system clock source
				if ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
					RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLREN;
				}
			}
			if (pll_out & RCC_PLL_OUTQ) RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLQEN;
			if (pll_out & RCC_PLL_OUTP) RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLPEN;

			break;
		case RCC_PLL_SAI1:
			if (pll_out & RCC_PLL_OUTR) RCC->PLLSAI1CFGR &= ~RCC_PLLSAI1CFGR_PLLSAI1REN;
			if (pll_out & RCC_PLL_OUTQ) RCC->PLLSAI1CFGR &= ~RCC_PLLSAI1CFGR_PLLSAI1QEN;
			if (pll_out & RCC_PLL_OUTP) RCC->PLLSAI1CFGR &= ~RCC_PLLSAI1CFGR_PLLSAI1PEN;

			break;
		case RCC_PLL_SAI2:
			if (pll_out & RCC_PLL_OUTR) RCC->PLLSAI2CFGR &= ~RCC_PLLSAI2CFGR_PLLSAI2REN;
			if (pll_out & RCC_PLL_OUTP) RCC->PLLSAI2CFGR &= ~RCC_PLLSAI2CFGR_PLLSAI2PEN;

			break;
		default:
			// Wrong PLL specified, do nothing

			break;
	}

}

// Disable the PLL
// input:
//   pll - which PLL must be configured, one of RCC_PLL_xxx values
// note: the main PLL must not be disabled when it is used as system clock source
void RCC_PLLDisable(uint32_t pll) {
	volatile uint32_t wait = RCC_CalcDelay(RCC_TIMEOUT_PLL);

	switch (pll) {
		case RCC_PLL_MAIN:
			// Check if the PLL used as system clock
			if ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
				// Disable the PLL and wait till it turned off
				RCC->CR &= ~(RCC_CR_PLLON);
				while ((RCC->CR & RCC_CR_PLLRDY) && --wait);
			} else {
				// Modifying the PLL settings when it is used as the system clock source is bad idea...
			}

			break;
		case RCC_PLL_SAI1:
			RCC->CR &= ~RCC_CR_PLLSAI1ON;
			while ((RCC->CR & RCC_CR_PLLSAI1RDY) && --wait);

			break;
		case RCC_PLL_SAI2:
			RCC->CR &= ~RCC_CR_PLLSAI2ON;
			while ((RCC->CR & RCC_CR_PLLSAI2RDY) && --wait);

			break;
		default:
			// Wrong PLL specified, do nothing

			break;
	}
}

// Configure and turn on the PLL
// input:
//   pll - which PLL must be configured, one of RCC_PLL_xxx values
//   cfgPLL - pointer to the structure what contains the configuration for the PLL
// return: SUCCESS if operation was successful, ERROR in case of a timeout
ErrorStatus RCC_PLLConfig(uint32_t pll, RCC_PLLInitTypeDef *cfgPLL) {
	volatile uint32_t wait = 0;

	switch (pll) {
		case RCC_PLL_MAIN:
			// Check if the PLL used as system clock
			if ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
				// Disable the PLL and wait till it turned off
				wait = RCC_CalcDelay(RCC_TIMEOUT_PLL);
				RCC->CR &= ~(RCC_CR_PLLON);
				while ((RCC->CR & RCC_CR_PLLRDY) && --wait);
				if (wait == 0) return ERROR;

				// Configure the PLL multiplication and division factors
				RCC->PLLCFGR &= ~(RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLR | RCC_PLLCFGR_PLLQ | RCC_PLLCFGR_PLLP);
				RCC->PLLCFGR |= (cfgPLL->PLLN << 8) | (cfgPLL->PLLR & RCC_PLLCFGR_PLLR) |
						(cfgPLL->PLLP & RCC_PLLCFGR_PLLP) | (cfgPLL ->PLLQ & RCC_PLLCFGR_PLLQ);

				// Enable the PLL and its system clock output
				RCC->CR |= RCC_CR_PLLON;

				// Wait till the PLL is ready
				wait = RCC_CalcDelay(RCC_TIMEOUT_PLL);
				while (!(RCC->CR & RCC_CR_PLLRDY) && --wait);
			} else {
				// Modifying the PLL settings when it is used as the system clock source is bad idea...
			}

			break;
		case RCC_PLL_SAI1:
			// Disable the PLLSAI1 and wait till it turned off
			wait = RCC_CalcDelay(RCC_TIMEOUT_PLL);
			RCC->CR &= ~(RCC_CR_PLLSAI1ON);
			while ((RCC->CR & RCC_CR_PLLSAI1RDY) && --wait);
			if (wait == 0) return ERROR;

			// Configure the PLLSAI1 multiplication and division factors
			RCC->PLLSAI1CFGR &= ~(RCC_PLLSAI1CFGR_PLLSAI1N | RCC_PLLSAI1CFGR_PLLSAI1R | RCC_PLLSAI1CFGR_PLLSAI1Q |
					RCC_PLLSAI1CFGR_PLLSAI1P);
			RCC->PLLSAI1CFGR |= (cfgPLL->PLLN << 8) | (cfgPLL->PLLR & RCC_PLLSAI1CFGR_PLLSAI1R) |
					(cfgPLL->PLLP & RCC_PLLSAI1CFGR_PLLSAI1P) | (cfgPLL ->PLLQ & RCC_PLLSAI1CFGR_PLLSAI1Q);

			// Enable the PLLSAI1 and its system clock output
			RCC->CR |= RCC_CR_PLLSAI1ON;

			// Wait till the PLL is ready
			wait = RCC_CalcDelay(RCC_TIMEOUT_PLL);
			while (!(RCC->CR & RCC_CR_PLLSAI1RDY) && --wait);

			break;
		case RCC_PLL_SAI2:
			// Disable the PLLSAI2 and wait till it turned off
			wait = RCC_CalcDelay(RCC_TIMEOUT_PLL);
			RCC->CR &= ~(RCC_CR_PLLSAI2ON);
			while ((RCC->CR & RCC_CR_PLLSAI2RDY) && --wait);
			if (wait == 0) return ERROR;

			// Configure the PLLSAI2 multiplication and division factors
			RCC->PLLSAI2CFGR &= ~(RCC_PLLSAI2CFGR_PLLSAI2N | RCC_PLLSAI2CFGR_PLLSAI2R | RCC_PLLSAI2CFGR_PLLSAI2P);
			RCC->PLLSAI2CFGR |= (cfgPLL->PLLN << 8) | (cfgPLL->PLLR & RCC_PLLSAI2CFGR_PLLSAI2R) |
					(cfgPLL->PLLP & RCC_PLLSAI2CFGR_PLLSAI2P);

			// Enable the PLLSAI2 and its system clock output
			RCC->CR |= RCC_CR_PLLSAI1ON;

			// Wait till the PLL is ready
			wait = RCC_CalcDelay(RCC_TIMEOUT_PLL);
			while (!(RCC->CR & RCC_CR_PLLSAI2RDY) && --wait);

			break;
		default:
			// Wrong PLL specified, do nothing

			break;
	}

	return (wait) ? SUCCESS : ERROR;
}

// Configure the MSI oscillator
// input:
//   state - new state of MSI, one of RCC_MSI_xxx values
// return: SUCCESS if operation was successful, ERROR in case of a timeout
// note: when the MSI is used as system clock or clock source for PLL, in these
//       cases it is not allowed to be disabled
ErrorStatus RCC_MSIConfig(uint32_t state) {
	volatile uint32_t wait;
	uint32_t msifreq;
	uint32_t reg;

	if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_MSI) {
		// The MSI is current system clock, only configure new frequency range allowed
		if (state != RCC_MSI_OFF) {
			// Get new MSI frequency
			msifreq = MSIRangeTable[(state & RCC_CR_MSIRANGE) >> 4];

			// In case the MSI is the current system clock source and the new MSI clock frequency is
			// increasing then the FLASH wait states should be adjusted
			if (((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_MSI) && (msifreq > SystemCoreClock)) {
				RCC_SetFlashLatency(msifreq);
				SystemCoreClock = msifreq;
			}

			// Set MSIRGSEL bit to switch range selection to the RCC_CR register
			RCC->CR |= RCC_CR_MSIRGSEL;

			// Configure new MSI frequency
			reg  = RCC->CR;
			reg &= ~(RCC_CR_MSIRANGE);
			reg |= state & RCC_CR_MSIRANGE;
			RCC->CR = reg;

			// In case the MSI is the current system clock source and the new MSI clock frequency is
			// decreased then the FLASH wait states should be adjusted
			if (((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_MSI) && (msifreq < SystemCoreClock)) {
				RCC_SetFlashLatency(msifreq);
				SystemCoreClock = msifreq;
			}
			// To return SUCCESS
			wait = 0xDEADBEEF;
		} else {
			// Turning off the HSI when it is the system clock source is bad idea...
			wait = 0;
		}
	} else {
		wait = RCC_CalcDelay(RCC_TIMEOUT_MSI);
		if (state != RCC_MSI_OFF) {
			// Enable the MSI and wait till it ready
			RCC->CR |= RCC_CR_MSION;
			while (!(RCC->CR & RCC_CR_MSIRDY) && --wait);
			if (wait) {
				// Set MSIRGSEL bit to switch range selection to the RCC_CR register
				RCC->CR |= RCC_CR_MSIRGSEL;

				// Configure new MSI frequency
				reg  = RCC->CR;
				reg &= ~(RCC_CR_MSIRANGE);
				reg |= state & RCC_CR_MSIRANGE;
				RCC->CR = reg;
			}
		} else {
			// Disable the MSI and wait till it turned off
			RCC->CR &= ~RCC_CR_MSION;
			while ((RCC->CR & RCC_CR_MSIRDY) && --wait);
		}
	}

	return (wait) ? SUCCESS : ERROR;
}

// Configure the HSI oscillator
// input:
//   state - new state of HSI, one of RCC_HSI_xxx values
// return: SUCCESS if operation was successful, ERROR in case of a timeout
// note: when the HSI is used as system clock or clock source for PLL, in these
//       cases it is not allowed to be disabled
ErrorStatus RCC_HSIConfig(uint32_t state) {
	volatile uint32_t wait = RCC_CalcDelay(RCC_TIMEOUT_HSI);

	if (state == RCC_HSI_ON) {
		// Enable HSI and wait till it ready
		RCC->CR |= RCC_CR_HSION;
		while (!(RCC->CR & RCC_CR_HSIRDY) && --wait);
	} else {
		// Disable HSI and wait till it turned off
		RCC->CR &= ~RCC_CR_HSION;
		while (!(RCC->CR & RCC_CR_HSIRDY) && --wait);
	}

	return (wait) ? SUCCESS : ERROR;
}

// Configure the HSE
// input:
//   state - new state of HSE, one of RCC_HSE_xxx values
// return: SUCCESS if operation was successful, ERROR in case of a timeout
// note: when the HSE is used as system clock or clock source for PLL, in these
//       cases it is not allowed to be disabled
ErrorStatus RCC_HSEConfig(uint32_t state) {
	volatile uint32_t wait = RCC_CalcDelay(RCC_TIMEOUT_HSE);

	// Before configuring the HSE both HSEON and HSEBYP bits must be reset
	RCC->CR &= ~(RCC_CR_HSEBYP | RCC_CR_HSEON);

	// Wait till HSE is turned off
	while ((RCC->CR & RCC_CR_HSERDY) && --wait);
	if (wait == 0) return ERROR;

	// Configure new state of the HSE and wait till it ready
	if (state != RCC_HSE_OFF) {
		RCC->CR |= RCC_CR_HSEON;
		if (state == RCC_HSE_BYPASS) RCC->CR |= RCC_CR_HSEBYP;
		wait = RCC_CalcDelay(RCC_TIMEOUT_HSE);
		while (!(RCC->CR & RCC_CR_HSERDY) && --wait);
	}

	return (wait) ? SUCCESS : ERROR;
}

// Configure the LSI oscillator
// input:
//   state - new state of LSI, one of RCC_LSI_xxx values
// return: SUCCESS if operation was successful, ERROR in case of a timeout
ErrorStatus RCC_LSIConfig(uint32_t state) {
	volatile uint32_t wait = RCC_CalcDelay(RCC_TIMEOUT_LSI);

	if (state == RCC_LSI_ON) {
		// Enable LSI and wait till it ready
		RCC->CSR |= RCC_CSR_LSION;
		while (!(RCC->CSR & RCC_CSR_LSIRDY) && --wait);
	} else {
		// Disable LSI and wait till it turned off
		RCC->CSR &= ~RCC_CSR_LSION;
		while ((RCC->CSR & RCC_CSR_LSIRDY) && --wait);
	}

	return (wait) ? SUCCESS : ERROR;
}

// Configure the LSE oscillator
// input:
//   state - new state of LSE, one of RCC_LSE_xxx values
// return: SUCCESS if operation was successful, ERROR in case of a timeout
ErrorStatus RCC_LSEConfig(uint32_t state) {
	uint32_t pwrstate;
	uint32_t dbpstate;
	volatile uint32_t wait = RCC_CalcDelay(RCC_TIMEOUT_LSE);

	// LSE is configured via backup domain control register (RCC_BDCR).
	// It is write-protected (after reset), thus to gain access to it
	// the PWR peripheral must be enabled.
	pwrstate = RCC->APB1ENR1;
	if (!(pwrstate & RCC_APB1ENR1_PWREN)) {
		// Enable PWR peripheral
		RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
	}

	// Save state of DBP bit and enable write access to the backup domain
	dbpstate = ((PWR->CR1 & PWR_CR1_DBP) != PWR_CR1_DBP);
	PWR->CR1 |= PWR_CR1_DBP;

	if (state == RCC_LSE_ON) {
		// Enable LSE
		RCC->BDCR |= RCC_BDCR_LSEON;
	} else if (state == RCC_LSE_BYPASS) {
		// LSE bypassed by external clock
		RCC->BDCR &= ~RCC_BDCR_LSEON;
		RCC->BDCR |= RCC_BDCR_LSEBYP | RCC_BDCR_LSEON;
	} else {
		// Disable LSE
		RCC->BDCR &= ~(RCC_BDCR_LSEON | RCC_BDCR_LSEBYP);
	}

	if (state == RCC_LSE_OFF) {
		// Wait till LSE is turned off
		while ((RCC->BDCR & RCC_BDCR_LSERDY) && --wait);
	} else {
		// Wait till LSE is ready
		while (!(RCC->BDCR & RCC_BDCR_LSERDY) && --wait);
	}

	// Restore states of the DBP bit and PWR peripheral
	if (dbpstate) PWR->CR1 &= ~PWR_CR1_DBP;
	RCC->APB1ENR1 = pwrstate;

	return (wait) ? SUCCESS : ERROR;
}

// Configure the MSI clock as system clock
// input:
//   range - new MSI clock range, one of RCC_MSI_xxx values (except of RCC_MSI_OFF)
//   cfgCLK - pointer to the structure what contains the configuration for the bus prescalers
// note: the MSI must be already enabled
// note: do not pass RCC_MSI_OFF value in range variable because that's nonsense
ErrorStatus RCC_SetClockMSI(RCC_CLKInitTypeDef *cfgCLK) {
	volatile uint32_t wait = RCC_CalcDelay(RCC_TIMEOUT_MSI);
	uint32_t reg;
	uint32_t msifreq;

	// Get the current frequency of MSI
	msifreq = MSIRangeTable[(RCC->CR & RCC_CR_MSIRANGE) >> 4];

	// If the MSI is not the current system clock source and its frequency is higher than
	// the current frequency of system clock then the FLASH wait states should be
	// adjusted before switching to MSI
	if (((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI) && (msifreq > SystemCoreClock)) {
		RCC_SetFlashLatency(msifreq);
		SystemCoreClock = msifreq;
	}

	// Configure the new AHB prescaler and system clock source
	reg  = RCC->CFGR;
	reg &= ~(RCC_CFGR_HPRE | RCC_CFGR_SW);
	RCC->CFGR = (reg | RCC_CFGR_SW_MSI | (cfgCLK->AHBdiv & RCC_CFGR_HPRE));

	// Wait for the system clock to switch to MSI
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_MSI);
	if (wait == 0) {
		// Restore configuration and flash latency
		RCC->CFGR = reg;
		RCC_SetFlashLatency(SystemCoreClock);

		return ERROR;
	}

	// At that moment the MSI is the system clock source and if its frequency is higher than
	// the old frequency of system clock then the FLASH wait states should be adjusted
	// according to the new frequency
	if (msifreq < SystemCoreClock) {
		RCC_SetFlashLatency(msifreq);
		SystemCoreClock = msifreq;
	}

	// Configure the new APB1 and APB2 prescalers
	reg  = RCC->CFGR;
	reg &= ~(RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
	reg |= (cfgCLK->APB1div & RCC_CFGR_PPRE1) | (cfgCLK->APB2div & RCC_CFGR_PPRE2);
	RCC->CFGR = reg;

	return SUCCESS;
}

// Configure the HSI clock as system clock
// input:
//   cfgCLK - pointer to the structure what contains the configuration for the bus prescalers
// note: the HSI must be already enabled
ErrorStatus RCC_SetClockHSI(RCC_CLKInitTypeDef *cfgCLK) {
	volatile uint32_t wait = RCC_CalcDelay(RCC_TIMEOUT_HSI);
	uint32_t reg;

	// In case of increasing the frequency the number of wait states should be
	// increased before the clocks configuration
	if (HSI_VALUE > SystemCoreClock) {
		RCC_SetFlashLatency(HSI_VALUE);
	}

	// Configure the new AHB prescaler and system clock source
	reg  = RCC->CFGR;
	reg &= ~(RCC_CFGR_HPRE | RCC_CFGR_SW);
	RCC->CFGR = (reg | RCC_CFGR_SW_HSI | (cfgCLK->AHBdiv & RCC_CFGR_HPRE));

	// Wait for the system clock to switch to HSI
	while (((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSI) && --wait);
	if (wait == 0) {
		// Restore configuration and flash latency
		RCC->CFGR = reg;
		RCC_SetFlashLatency(SystemCoreClock);

		return ERROR;
	}

	// Configure the new APB1 and APB2 prescalers
	reg  = RCC->CFGR;
	reg &= ~(RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
	reg |= (cfgCLK->APB1div & RCC_CFGR_PPRE1) | (cfgCLK->APB2div & RCC_CFGR_PPRE2);
	RCC->CFGR = reg;

	// In case of decreasing the frequency, the number of wait states should be
	// decreased after the clocks configuration
	if (HSI_VALUE < SystemCoreClock) {
		RCC_SetFlashLatency(HSI_VALUE);
	}

	// Update the SystemCoreClock variable
	SystemCoreClock = HSI_VALUE;

	return SUCCESS;
}

// Configure the HSE clock as system clock
// input:
//   cfgCLK - pointer to the structure what contains the configuration for the bus prescalers
// note: the HSE must be already enabled
ErrorStatus RCC_SetClockHSE(RCC_CLKInitTypeDef *cfgCLK) {
	volatile uint32_t wait = RCC_CalcDelay(RCC_TIMEOUT_HSE);
	uint32_t reg;

	// In case of increasing the frequency the number of wait states should be
	// increased before the clocks configuration
	if (HSE_VALUE > SystemCoreClock) {
		RCC_SetFlashLatency(HSE_VALUE);
	}

	// Configure the new AHB prescaler and system clock source
	reg  = RCC->CFGR;
	reg &= ~(RCC_CFGR_HPRE | RCC_CFGR_SW);
	RCC->CFGR = (reg | RCC_CFGR_SW_HSE | (cfgCLK->AHBdiv & RCC_CFGR_HPRE));

	// Wait for the system clock to switch to HSE
	while (((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_HSE) && --wait);
	if (wait == 0) {
		// Restore configuration and flash latency
		RCC->CFGR = reg;
		RCC_SetFlashLatency(SystemCoreClock);

		return ERROR;
	}

	// Configure the new APB1 and APB2 prescalers
	reg  = RCC->CFGR;
	reg &= ~(RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
	reg |= (cfgCLK->APB1div & RCC_CFGR_PPRE1) | (cfgCLK->APB2div & RCC_CFGR_PPRE2);
	RCC->CFGR = reg;

	// In case of decreasing the frequency, the number of wait states should be
	// decreased after the clocks configuration
	if (HSE_VALUE < SystemCoreClock) {
		RCC_SetFlashLatency(HSE_VALUE);
	}

	// Update the SystemCoreClock variable
	SystemCoreClock = HSE_VALUE;

	return SUCCESS;
}

// Switch main system clock to main PLL
// input:
//   clock_source - main PLL clock source
//   cfgPLL - pointer to the structure what contains the configuration for the PLL
//   cfgCLK - pointer to the structure what contains the configuration for the bus prescalers
// return: SUCCESS in case of success, ERROR if switch to PLL failed
// note: the PLL clock source and PLLM divider must be already configured, clock source must be ready
ErrorStatus RCC_SetClockPLL(uint32_t clock_source, RCC_PLLInitTypeDef *cfgPLL, RCC_CLKInitTypeDef *cfgCLK) {
	uint32_t freq;
	ErrorStatus result = ERROR;

	// Determine PLL source clock frequency
	switch (clock_source) {
		case RCC_PLLSRC_MSI:
			freq = RCC_GetMSIFreq();

			break;
		case RCC_PLLSRC_HSI:
			freq = HSI_VALUE;

			break;
		case RCC_PLLSRC_HSE:
			freq = HSE_VALUE;

			break;
		default:
			// Invalid clock source specified
			freq = 0;

			break;
	}

	if (freq != 0) {
		// Enable main PLL R output
		RCC_PLLOutEnable(RCC_PLL_MAIN,RCC_PLL_OUTR);

		// Calculate PLL output frequency
		freq = RCC_CalcPLLFreq(freq,RCC_GetPLLMDiv(),cfgPLL);

		// Configure the main PLL and enable it
		RCC_PLLConfig(RCC_PLL_MAIN,cfgPLL);

		// Switch system clock to it
		result = RCC_SwitchToPLL(freq,cfgCLK);

		if (result != SUCCESS) {
			// Disable main PLL R output
			RCC_PLLOutDisable(RCC_PLL_MAIN,RCC_PLL_OUTR);
		}
	}

	return result;
}

// Return USART/UART peripheral clock frequency
// input:
//   periph_sel - indicates for which peripheral frequency will be returned
//                can be one of RCC_USARTx_CLK_SRC or RCC_UARTx_CLK_SRC values
// return: clock frequency (Hz)
//         zero if the clock is not ready (HSI/LSE) or invalid value of periph_sel has been specified
uint32_t RCC_GetClockUSART(uint32_t periph_sel) {
	uint32_t frequency = 0;
	uint32_t clk_src;

	// Get clock source for specified peripheral
	switch (periph_sel) {
		case RCC_USART1_CLK_SRC: clk_src =  RCC->CCIPR & RCC_CCIPR_USART1SEL;       break;
		case RCC_USART2_CLK_SRC: clk_src = (RCC->CCIPR & RCC_CCIPR_USART2SEL) >> 2; break;
		case RCC_USART3_CLK_SRC: clk_src = (RCC->CCIPR & RCC_CCIPR_USART3SEL) >> 4; break;
		case RCC_UART4_CLK_SRC:  clk_src = (RCC->CCIPR & RCC_CCIPR_UART4SEL)  >> 6; break;
		case RCC_UART5_CLK_SRC:  clk_src = (RCC->CCIPR & RCC_CCIPR_UART5SEL)  >> 8; break;
		default:
			// Wrong peripheral selector was given
			clk_src = 0xFFFFFFFF;

			break;
	}

	// Check if clock selected correctly, return zero frequency otherwise
	if (clk_src != 0xFFFFFFFF) {
		switch (clk_src) {
			case RCC_PERIPH_CLK_PCLK:
				// USART clock is APBx clock
				if (periph_sel == RCC_USART1_CLK_SRC) {
					// USART1 clocked from APB2
					frequency = RCC_GetPCLK2Freq(RCC_GetHCLKFreq(RCC_GetSYSCLKFreq()));
				} else {
					// All except USART1 clocked from APB1
					frequency = RCC_GetPCLK1Freq(RCC_GetHCLKFreq(RCC_GetSYSCLKFreq()));
				}

				break;
			case RCC_PERIPH_CLK_SYSCLK:
				// USART clock is system clock
				frequency = RCC_GetSYSCLKFreq();

				break;
			case RCC_PERIPH_CLK_HSI:
				// USART clock is HSI
				if (RCC->CR & RCC_CR_HSIRDY) frequency = HSI_VALUE;

				break;
			case RCC_PERIPH_CLK_LSE:
				// USART clock is LSE
				if (RCC->BDCR & RCC_BDCR_LSERDY) frequency = LSE_VALUE;

				break;
			default:
				// Do nothing, this is wrong clock source (frequency for return is zero)
				break;
		}
	}

	return frequency;
}

// Configure USART/UART peripheral clock source
// input:
//   periph_sel - indicates for which peripheral frequency will be returned
//                can be either one of RCC_USARTx_CLK_SRC or RCC_UARTx_CLK_SRC values
//   clock_src - new clock source, one of RCC_PERIPH_CLK_xxx values
void RCC_SetClockUSART(uint32_t periph_sel, uint32_t clock_src) {
	uint32_t reg;

	// Configure new clock source for specified peripheral
	reg  = RCC->CCIPR;
	reg &= ~periph_sel;
	clock_src &= RCC_PERIPH_CLK_MASK;
	switch (periph_sel) {
		case RCC_USART1_CLK_SRC: reg |= clock_src;      break;
		case RCC_USART2_CLK_SRC: reg |= clock_src << 2; break;
		case RCC_USART3_CLK_SRC: reg |= clock_src << 4; break;
		case RCC_UART4_CLK_SRC:  reg |= clock_src << 6; break;
		case RCC_UART5_CLK_SRC:  reg |= clock_src << 8; break;
		default:
			// Wrong peripheral specified, do nothing
			break;
	}
	RCC->CCIPR = reg;
}

// Configure RTC peripheral clock source
// input:
//   clock_src - new clock source, one of RCC_RTC_CLK_xxx values
void RCC_SetClockRTC(uint32_t clock_src) {
	RCC->BDCR &= ~RCC_BDCR_RTCSEL;
	RCC->BDCR |= clock_src & RCC_BDCR_RTCSEL;
}

// Configure USART/UART peripheral clock source
// input:
//   periph_sel - indicates for which peripheral frequency will be returned
//                can be one of RCC_I2Cx_CLK_SRC values
//   clock_src - new clock source, one of RCC_PERIPH_CLK_xxx values except RCC_PERIPH_CLK_LSE
void RCC_SetClockI2C(uint32_t periph_sel, uint32_t clock_src) {
	uint32_t reg;

	// LSE clock is to slow for I2C
	if (clock_src == RCC_PERIPH_CLK_LSE) return;

	// Configure new clock source for specified peripheral
	reg  = RCC->CCIPR;
	reg &= ~periph_sel;
	clock_src &= RCC_PERIPH_CLK_MASK;
	switch (periph_sel) {
		case RCC_I2C1_CLK_SRC: reg |= clock_src << 12; break;
		case RCC_I2C2_CLK_SRC: reg |= clock_src << 14; break;
		case RCC_I2C3_CLK_SRC: reg |= clock_src << 16; break;
		default:
			// Wrong peripheral specified, do nothing
			break;
	}
	RCC->CCIPR = reg;
}

// Configure CLK48 (USB, RNG and SDMMC) clock source
// input:
//   clock_src - new clock source, one of RCC_CLK48_CLK_xxx values
void RCC_SetClock48M(uint32_t clock_src) {
	RCC->CCIPR &= ~RCC_CCIPR_CLK48SEL;
	RCC->CCIPR |= clock_src & RCC_CCIPR_CLK48SEL;
}
