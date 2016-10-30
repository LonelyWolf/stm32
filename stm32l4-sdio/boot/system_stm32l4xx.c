/**
  ******************************************************************************
  * @file    system_stm32l4xx.c
  * @author  MCD Application Team
  * @version V1.1.2
  * @date    12-September-2016
  * @brief   CMSIS Cortex-M4 Device Peripheral Access Layer System Source File
  *
  *   This file provides two functions and one global variable to be called from
  *   user application:
  *      - SystemInit(): This function is called at startup just after reset and
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32l4xx.s" file.
  *
  *      - SystemCoreClock variable: Contains the core clock (HCLK), it can be used
  *                                  by the user application to setup the SysTick
  *                                  timer or configure other parameters.
  *
  *      - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
  *                                 be called whenever the core clock is changed
  *                                 during program execution.
  *
  *   After each device reset the MSI (4 MHz) is used as system clock source.
  *   Then SystemInit() function is called, in "startup_stm32l4xx.s" file, to
  *   configure the system clock before to branch to main program.
  *
  *   This file configures the system clock as follows:
  *=============================================================================
  *-----------------------------------------------------------------------------
  *        System Clock source                    | MSI
  *-----------------------------------------------------------------------------
  *        SYSCLK(Hz)                             | 4000000
  *-----------------------------------------------------------------------------
  *        HCLK(Hz)                               | 4000000
  *-----------------------------------------------------------------------------
  *        AHB Prescaler                          | 1
  *-----------------------------------------------------------------------------
  *        APB1 Prescaler                         | 1
  *-----------------------------------------------------------------------------
  *        APB2 Prescaler                         | 1
  *-----------------------------------------------------------------------------
  *        PLL_M                                  | 1
  *-----------------------------------------------------------------------------
  *        PLL_N                                  | 8
  *-----------------------------------------------------------------------------
  *        PLL_P                                  | 7
  *-----------------------------------------------------------------------------
  *        PLL_Q                                  | 2
  *-----------------------------------------------------------------------------
  *        PLL_R                                  | 2
  *-----------------------------------------------------------------------------
  *        PLLSAI1_P                              | NA
  *-----------------------------------------------------------------------------
  *        PLLSAI1_Q                              | NA
  *-----------------------------------------------------------------------------
  *        PLLSAI1_R                              | NA
  *-----------------------------------------------------------------------------
  *        PLLSAI2_P                              | NA
  *-----------------------------------------------------------------------------
  *        PLLSAI2_Q                              | NA
  *-----------------------------------------------------------------------------
  *        PLLSAI2_R                              | NA
  *-----------------------------------------------------------------------------
  *        Require 48MHz for USB OTG FS,          | Disabled
  *        SDIO and RNG clock                     |
  *-----------------------------------------------------------------------------
  *=============================================================================
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */


#include "stm32l4xx.h"


// Uncomment the following line to relocate vector table in internal SRAM
//#define VECT_TAB_SRAM

// Vector Table base offset field (this value must be a multiple of 0x200)
#define VECT_TAB_OFFSET 0x00


// The core clock frequency (HCLK, Hz)
uint32_t SystemCoreClock = MSI_VALUE;

// AHB prescalers
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};

// APB prescalers
const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};

// Available MSI frequency ranges (Hz)
const uint32_t MSIRangeTable[12] = {
		  100000,   200000,   400000,   800000,
		 1000000,  2000000,  4000000,  8000000,
		16000000, 24000000, 32000000, 48000000
};


// Setup the microcontroller system (reset the clocks to the default reset state)
void SystemInit(void) {
	// FPU settings
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  // set CP10 and CP11 Full Access
#endif

  // Reset the RCC clock configuration to the default reset state

	// Set MSION bit
	RCC->CR |= RCC_CR_MSION;

	// Reset CFGR register
	RCC->CFGR = 0x00000000;

	// Reset HSEON, CSSON , HSION, and PLLON bits
	RCC->CR &= (uint32_t)0xEAF6FFFF;

	// Reset PLLCFGR register
	RCC->PLLCFGR = 0x00001000;

	// Reset HSEBYP bit
	RCC->CR &= (uint32_t)0xFFFBFFFF;

	// Disable all interrupts
	RCC->CIER = 0x00000000;

	// Configure the vector table relocation
#ifdef VECT_TAB_SRAM
	SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; // table in internal SRAM
#else
	SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; // table in Internal FLASH
#endif
}

// Calculate the value of SystemCoreClock according to clock register values
// note:
//   - The result of this function could be not correct when using fractional
//     value for HSE crystal.
//   - This function must be called whenever the core clock is changed
//     during program execution
void SystemCoreClockUpdate(void) {
	uint32_t msirange;
	uint32_t tmp;

	// MSI frequency (Hz)
	if (RCC->CR & RCC_CR_MSIRGSEL) {
		// MSIRGSEL=1 --> MSIRANGE from RCC_CR applies
		msirange = MSIRangeTable[(RCC->CR & RCC_CR_MSIRANGE) >> 4];
	} else {
		// MSIRGSEL=0 --> MSISRANGE from RCC_CSR applies
		msirange = MSIRangeTable[(RCC->CSR & RCC_CSR_MSISRANGE) >> 8];
	}

	// SYSCLK source
	switch (RCC->CFGR & RCC_CFGR_SWS) {
	case RCC_CFGR_SWS_HSI:
		// HSI used as system clock source
		SystemCoreClock = HSI_VALUE;
		break;
	case RCC_CFGR_SWS_HSE:
		// HSE used as system clock source
		SystemCoreClock = HSE_VALUE;
		break;
	case RCC_CFGR_SWS_PLL:
		// PLL used as system clock source

		// PLLM division factor
		tmp = ((RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> 4) + 1;

		// PLL source
		switch (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) {
		case RCC_PLLCFGR_PLLSRC_HSI:
        	// HSI used as PLL clock source
			SystemCoreClock = (HSI_VALUE / tmp);
			break;
		case RCC_PLLCFGR_PLLSRC_HSE:
			// HSE used as PLL clock source
			SystemCoreClock = (HSE_VALUE / tmp);
			break;
		case RCC_PLLCFGR_PLLSRC_MSI:
		default:
			// MSI used as PLL clock source
			SystemCoreClock = (msirange / tmp);
			break;
		}

		// PLL_VCO = (HSE_VALUE or HSI_VALUE or MSI_VALUE/PLLM) * PLLN
		// SYSCLK = PLL_VCO / PLLR
		SystemCoreClock *= (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 8;
		SystemCoreClock /= (((RCC->PLLCFGR & RCC_PLLCFGR_PLLR) >> 25) + 1) << 1;

		break;
	case RCC_CFGR_SWS_MSI:
	default:
		// MSI used as system clock source
		SystemCoreClock = msirange;

		break;
	}

	// HCLK clock frequency
	SystemCoreClock >>= AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
}
