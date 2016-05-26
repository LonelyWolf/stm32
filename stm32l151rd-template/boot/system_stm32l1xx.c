/**
  ******************************************************************************
  * @file    system_stm32l1xx.c
  * @author  MCD Application Team
  * @version V2.1.2
  * @date    09-October-2015
  * @brief   CMSIS Cortex-M3 Device Peripheral Access Layer System Source File.
  *             
  *   This file provides two functions and one global variable to be called from 
  *   user application:
  *      - SystemInit(): This function is called at startup just after reset and 
  *                      before branch to main program. This call is made inside
  *                      the "startup_stm32l1xx.s" file.
  *                        
  *      - SystemCoreClock variable: Contains the core clock (HCLK), it can be used
  *                                  by the user application to setup the SysTick 
  *                                  timer or configure other parameters.
  *                                     
  *      - SystemCoreClockUpdate(): Updates the variable SystemCoreClock and must
  *                                 be called whenever the core clock is changed
  *                                 during program execution.   
  *      
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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

/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32l1xx_system
  * @{
  */  
  
/** @addtogroup STM32L1xx_System_Private_Includes
  * @{
  */

#include "stm32l1xx.h"

/**
  * @}
  */

/** @addtogroup STM32L1xx_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/*!< Uncomment the following line if you need to use external SRAM mounted
     on STM32L152D_EVAL board as data memory  */
/* #define DATA_IN_ExtSRAM */
  
/*!< Uncomment the following line if you need to relocate your vector Table in
     Internal SRAM. */ 
/* #define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET  0x0 /*!< Vector Table base offset field. 
                                  This value must be a multiple of 0x200. */
/**
  * @}
  */

/** @addtogroup STM32L1xx_System_Private_Macros
  * @{
  */

/**
  * @}
  */

// The core clock frequency (HCLK, Hz)
uint32_t SystemCoreClock = MSI_VALUE;

// PLL multipliers table
const uint8_t PLLMulTable[9]    = {3, 4, 6, 8, 12, 16, 24, 32, 48};

// AHB prescalers table
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};


// FSMC initialization (if supported)
#if defined (STM32L151xD) || defined (STM32L152xD) || defined (STM32L162xD)
	#ifdef DATA_IN_ExtSRAM
		static void SystemInit_ExtMemCtl(void);
	#endif // DATA_IN_ExtSRAM
#endif // STM32L151xD || STM32L152xD || STM32L162xD


/**
  * @brief  Setup the microcontroller system.
  *         Initialize the Embedded Flash Interface, the PLL and update the 
  *         SystemCoreClock variable.
  * @param  None
  * @retval None
  */
void SystemInit (void) {
	/*!< Set MSION bit */
	RCC->CR |= (uint32_t)0x00000100;

	/*!< Reset SW[1:0], HPRE[3:0], PPRE1[2:0], PPRE2[2:0], MCOSEL[2:0] and MCOPRE[2:0] bits */
	RCC->CFGR &= (uint32_t)0x88FFC00C;
  
	/*!< Reset HSION, HSEON, CSSON and PLLON bits */
	RCC->CR &= (uint32_t)0xEEFEFFFE;

	/*!< Reset HSEBYP bit */
	RCC->CR &= (uint32_t)0xFFFBFFFF;

	/*!< Reset PLLSRC, PLLMUL[3:0] and PLLDIV[1:0] bits */
	RCC->CFGR &= (uint32_t)0xFF02FFFF;

	/*!< Disable all interrupts */
	RCC->CIR = 0x00000000;

#ifdef DATA_IN_ExtSRAM
	// Initialize the FSMC peripheral
	SystemInit_ExtMemCtl();
#endif // DATA_IN_ExtSRAM
    
	// Configure vector table relocation
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
	uint32_t reg;
	uint32_t divider;

	// Get the SYSCLK source
	reg = RCC->CFGR;
	switch (reg & RCC_CFGR_SWS) {
	case RCC_CFGR_SWS_0:
		// HSI used as system clock
		SystemCoreClock = HSI_VALUE;

		break;
	case RCC_CFGR_SWS_1:
		// HSE used as system clock
		SystemCoreClock = HSE_VALUE;

		break;
	case RCC_CFGR_SWS:
		// PLL used as system clock
		divider = ((reg & RCC_CFGR_PLLDIV) >> 22) + 1;
		SystemCoreClock  = (reg & RCC_CFGR_PLLSRC) ? HSE_VALUE : HSI_VALUE;
		SystemCoreClock *= PLLMulTable[(reg & RCC_CFGR_PLLMUL) >> 18];
		SystemCoreClock /= divider;

		break;
	default:
		// MSI used as system clock
		SystemCoreClock = (1 << (((reg & RCC_ICSCR_MSIRANGE) >> 13) + 1)) * 32768;

		break;
	}

	// HCLK clock frequency
	SystemCoreClock >>= AHBPrescTable[(reg & RCC_CFGR_HPRE) >> 4];
}

/**
  * @brief  Configures the System clock frequency, AHB/APBx prescalers and Flash
  *         settings.
  * @note   This function should be called only once the RCC clock configuration
  *         is reset to the default reset state (done in SystemInit() function).
  * @param  None
  * @retval None
  */
void SetSysClock(void) {
	__IO uint32_t StartUpCounter = 0, HSEStatus = 0;

	/* SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/

	/* Enable HSE */
	RCC->CR |= ((uint32_t)RCC_CR_HSEON);

	/* Wait till HSE is ready and if Time out is reached exit */
	do {
		HSEStatus = RCC->CR & RCC_CR_HSERDY;
		StartUpCounter++;
	} while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

	if ((RCC->CR & RCC_CR_HSERDY) != RESET) {
		HSEStatus = (uint32_t)0x01;
	} else {
		HSEStatus = (uint32_t)0x00;
	}

	if (HSEStatus == (uint32_t)0x01) {
		/* Enable 64-bit access */
		FLASH->ACR |= FLASH_ACR_ACC64;

		/* Enable Prefetch Buffer */
		FLASH->ACR |= FLASH_ACR_PRFTEN;

		/* Flash 1 wait state */
		FLASH->ACR |= FLASH_ACR_LATENCY;

		/* Power enable */
		RCC->APB1ENR |= RCC_APB1ENR_PWREN;

		/* Select the Voltage Range 1 (1.8 V) */
		PWR->CR = PWR_CR_VOS_0;

		/* Wait Until the Voltage Regulator is ready */
		while((PWR->CSR & PWR_CSR_VOSF) != RESET);

		/* HCLK = SYSCLK /1*/
		RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;

		/* PCLK2 = HCLK /1*/
		RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;

		/* PCLK1 = HCLK /1*/
		RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV1;

		/*  PLL configuration */
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL | RCC_CFGR_PLLDIV));
		RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMUL6 | RCC_CFGR_PLLDIV3);

		/* Enable PLL */
		RCC->CR |= RCC_CR_PLLON;

		/* Wait till PLL is ready */
		while((RCC->CR & RCC_CR_PLLRDY) == 0);

		/* Select PLL as system clock source */
		RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
		RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;

		/* Wait till PLL is used as system clock source */
		while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL);
	} else {
		/*
		 *
		 *  If HSE fails to start-up, the application will have wrong clock
		 * configuration. User can add here some code to deal with this error
		 *
		 */
	}
}

#if defined (STM32L151xD) || defined (STM32L152xD) || defined (STM32L162xD)
#ifdef DATA_IN_ExtSRAM
/**
  * @brief  Setup the external memory controller.
  *         Called in SystemInit() function before jump to main.
  *         This function configures the external SRAM mounted on STM32L152D_EVAL board
  *         This SRAM will be used as program data memory (including heap and stack).
  * @param  None
  * @retval None
  */
void SystemInit_ExtMemCtl(void)
{
  __IO uint32_t tmpreg = 0;

  /* Flash 1 wait state */
  FLASH->ACR |= FLASH_ACR_LATENCY;
  
  /* Power enable */
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->APB1ENR, RCC_APB1ENR_PWREN);

  /* Select the Voltage Range 1 (1.8 V) */
  PWR->CR = PWR_CR_VOS_0;
  
  /* Wait Until the Voltage Regulator is ready */
  while((PWR->CSR & PWR_CSR_VOSF) != RESET)
  {
  }
  
/*-- GPIOs Configuration -----------------------------------------------------*/
/*
 +-------------------+--------------------+------------------+------------------+
 +                       SRAM pins assignment                                   +
 +-------------------+--------------------+------------------+------------------+
 | PD0  <-> FSMC_D2  | PE0  <-> FSMC_NBL0 | PF0  <-> FSMC_A0 | PG0 <-> FSMC_A10 |
 | PD1  <-> FSMC_D3  | PE1  <-> FSMC_NBL1 | PF1  <-> FSMC_A1 | PG1 <-> FSMC_A11 |
 | PD4  <-> FSMC_NOE | PE7  <-> FSMC_D4   | PF2  <-> FSMC_A2 | PG2 <-> FSMC_A12 |
 | PD5  <-> FSMC_NWE | PE8  <-> FSMC_D5   | PF3  <-> FSMC_A3 | PG3 <-> FSMC_A13 |
 | PD8  <-> FSMC_D13 | PE9  <-> FSMC_D6   | PF4  <-> FSMC_A4 | PG4 <-> FSMC_A14 |
 | PD9  <-> FSMC_D14 | PE10 <-> FSMC_D7   | PF5  <-> FSMC_A5 | PG5 <-> FSMC_A15 |
 | PD10 <-> FSMC_D15 | PE11 <-> FSMC_D8   | PF12 <-> FSMC_A6 | PG10<-> FSMC_NE2 |
 | PD11 <-> FSMC_A16 | PE12 <-> FSMC_D9   | PF13 <-> FSMC_A7 |------------------+
 | PD12 <-> FSMC_A17 | PE13 <-> FSMC_D10  | PF14 <-> FSMC_A8 | 
 | PD13 <-> FSMC_A18 | PE14 <-> FSMC_D11  | PF15 <-> FSMC_A9 | 
 | PD14 <-> FSMC_D0  | PE15 <-> FSMC_D12  |------------------+
 | PD15 <-> FSMC_D1  |--------------------+ 
 +-------------------+
*/

  /* Enable GPIOD, GPIOE, GPIOF and GPIOG interface clock */
  RCC->AHBENR   = 0x000080D8;
  
  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->AHBENR, RCC_AHBENR_GPIODEN);
  
  /* Connect PDx pins to FSMC Alternate function */
  GPIOD->AFR[0]  = 0x00CC00CC;
  GPIOD->AFR[1]  = 0xCCCCCCCC;
  /* Configure PDx pins in Alternate function mode */  
  GPIOD->MODER   = 0xAAAA0A0A;
  /* Configure PDx pins speed to 40 MHz */  
  GPIOD->OSPEEDR = 0xFFFF0F0F;
  /* Configure PDx pins Output type to push-pull */  
  GPIOD->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PDx pins */ 
  GPIOD->PUPDR   = 0x00000000;

  /* Connect PEx pins to FSMC Alternate function */
  GPIOE->AFR[0]  = 0xC00000CC;
  GPIOE->AFR[1]  = 0xCCCCCCCC;
  /* Configure PEx pins in Alternate function mode */ 
  GPIOE->MODER   = 0xAAAA800A;
  /* Configure PEx pins speed to 40 MHz */ 
  GPIOE->OSPEEDR = 0xFFFFC00F;
  /* Configure PEx pins Output type to push-pull */  
  GPIOE->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PEx pins */ 
  GPIOE->PUPDR   = 0x00000000;

  /* Connect PFx pins to FSMC Alternate function */
  GPIOF->AFR[0]  = 0x00CCCCCC;
  GPIOF->AFR[1]  = 0xCCCC0000;
  /* Configure PFx pins in Alternate function mode */   
  GPIOF->MODER   = 0xAA000AAA;
  /* Configure PFx pins speed to 40 MHz */ 
  GPIOF->OSPEEDR = 0xFF000FFF;
  /* Configure PFx pins Output type to push-pull */  
  GPIOF->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PFx pins */ 
  GPIOF->PUPDR   = 0x00000000;

  /* Connect PGx pins to FSMC Alternate function */
  GPIOG->AFR[0]  = 0x00CCCCCC;
  GPIOG->AFR[1]  = 0x00000C00;
  /* Configure PGx pins in Alternate function mode */ 
  GPIOG->MODER   = 0x00200AAA;
  /* Configure PGx pins speed to 40 MHz */ 
  GPIOG->OSPEEDR = 0x00300FFF;
  /* Configure PGx pins Output type to push-pull */  
  GPIOG->OTYPER  = 0x00000000;
  /* No pull-up, pull-down for PGx pins */ 
  GPIOG->PUPDR   = 0x00000000;
  
/*-- FSMC Configuration ------------------------------------------------------*/
  /* Enable the FSMC interface clock */
  RCC->AHBENR    = 0x400080D8;

  /* Delay after an RCC peripheral clock enabling */
  tmpreg = READ_BIT(RCC->AHBENR, RCC_AHBENR_FSMCEN);
  
  (void)(tmpreg);
  
  /* Configure and enable Bank1_SRAM3 */
  FSMC_Bank1->BTCR[4]  = 0x00001011;
  FSMC_Bank1->BTCR[5]  = 0x00000300;
  FSMC_Bank1E->BWTR[4] = 0x0FFFFFFF;
/*
  Bank1_SRAM3 is configured as follow:

  p.FSMC_AddressSetupTime = 0;
  p.FSMC_AddressHoldTime = 0;
  p.FSMC_DataSetupTime = 3;
  p.FSMC_BusTurnAroundDuration = 0;
  p.FSMC_CLKDivision = 0;
  p.FSMC_DataLatency = 0;
  p.FSMC_AccessMode = FSMC_AccessMode_A;

  FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM3;
  FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
  FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
  FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
  FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_AsynchronousWait = FSMC_AsynchronousWait_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
  FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
  FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
  FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
  FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
  FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
  FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p;

  FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 

  FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM3, ENABLE);
*/
  
}
#endif /* DATA_IN_ExtSRAM */
#endif /* STM32L151xD || STM32L152xD || STM32L162xD */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
