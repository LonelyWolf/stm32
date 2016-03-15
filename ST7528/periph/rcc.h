#ifndef __RCC_H
#define __RCC_H


#include <stm32l1xx.h>


// The LSE initialization timeout (this can take a while)
#define RCC_LSE_TIMEOUT            ((uint32_t)0x00AFFFFF) // about 5 seconds on 32MHz

// The LSI initialization timeout
#define RCC_LSI_TIMEOUT            ((uint32_t)0x0000FFFF)

// LSE configuration
#define _RCC_LSE_OFF               ((uint32_t)0x00000000)
#define _RCC_LSE_ON                RCC_CSR_LSEON
#define _RCC_LSE_BYPASS            ((uint32_t)(RCC_CSR_LSEON | RCC_CSR_LSEBYP))

// RTC clock source
#define RCC_RTCCLK_LSE             RCC_CSR_RTCSEL_LSE // LSE
#define RCC_RTCCLK_LSI             RCC_CSR_RTCSEL_LSI // LSI
#define RCC_RTCCLK_HSE2            RCC_CSR_RTCSEL_HSE // HSE/2
#define RCC_RTCCLK_HSE4            ((uint32_t)(RCC_CSR_RTCSEL_HSE | RCC_CR_RTCPRE_0)) // HSE/4
#define RCC_RTCCLK_HSE8            ((uint32_t)(RCC_CSR_RTCSEL_HSE | RCC_CR_RTCPRE_1)) // HSE/8
#define RCC_RTCCLK_HSE16           ((uint32_t)(RCC_CSR_RTCSEL_HSE | RCC_CR_RTCPRE))   // HSE/16

// Alias word access to LSION bit in RCC_CSR register
#define RCC_CSR_LSION_BN           0 // bit number
#define RCC_CSR_LSION_BB           (*(__IO uint32_t *)(PERIPH_BB_BASE + ((((uint32_t)&(RCC->CSR)) - PERIPH_BASE) << 5) + (RCC_CSR_LSION_BN << 2)))

// Alias word address of bits of the PWR_CR register
#define PWR_CR_DBP_BN              8 // DBP bit number
#define PWR_CR_DBP_BB              (*(__IO uint32_t *)(PERIPH_BB_BASE + ((((uint32_t)&(PWR->CR)) - PERIPH_BASE) << 5) + (PWR_CR_DBP_BN << 2)))


// Structure definitions

// RCC_ClocksTypeDef (for SPL compatibility)
typedef struct {
	uint32_t SYSCLK_Frequency;
	uint32_t HCLK_Frequency;
	uint32_t PCLK1_Frequency;
	uint32_t PCLK2_Frequency;
} RCC_ClocksTypeDef;


// Function prototypes
ErrorStatus RCC_LSE_cfg(uint32_t LSE);
ErrorStatus RCC_LSI_cfg(FunctionalState LSI);
void RCC_RTCCLK_cfg(uint32_t source);

void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);

#endif // __RCC_H
