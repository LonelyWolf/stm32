#ifndef __RCC_H
#define __RCC_H


#include <stm32l4xx.h>


// Definitions of system clock source
#define RCC_SYSCLK_SRC_UNKNOWN     ((uint32_t)0x00000000U) // The system clock is unknown
#define RCC_SYSCLK_SRC_MSI         ((uint32_t)0x00000001U) // MSI
#define RCC_SYSCLK_SRC_HSI         ((uint32_t)0x00000002U) // HSI
#define RCC_SYSCLK_SRC_HSE         ((uint32_t)0x00000003U) // HSE
#define RCC_SYSCLK_SRC_MSIPLL      ((uint32_t)0x00000004U) // MSI -> PLL
#define RCC_SYSCLK_SRC_HSIPLL      ((uint32_t)0x00000005U) // HSI -> PLL
#define RCC_SYSCLK_SRC_HSEPLL      ((uint32_t)0x00000006U) // HSE -> PLL

// Definitions of PLLs clock source configuration
#define RCC_PLLSRC_NONE            ((uint32_t)0x00000000U) // No clock sent to PLLs
#define RCC_PLLSRC_MSI             RCC_PLLCFGR_PLLSRC_MSI  // MSI clock
#define RCC_PLLSRC_HSI             RCC_PLLCFGR_PLLSRC_HSI  // HSI clock
#define RCC_PLLSRC_HSE             RCC_PLLCFGR_PLLSRC_HSE  // HSE clock

// Definitions of PLL selection
#define RCC_PLL_MAIN               ((uint32_t)0x00000001U) // Main PLL
#define RCC_PLL_SAI1               ((uint32_t)0x00000002U) // PLLSAI1
#define RCC_PLL_SAI2               ((uint32_t)0x00000004U) // PLLSAI2

// Definitions of PLL outputs
#define RCC_PLL_OUTR               ((uint32_t)0x00000001U) // PLLR
#define RCC_PLL_OUTQ               ((uint32_t)0x00000002U) // PLLQ
#define RCC_PLL_OUTP               ((uint32_t)0x00000004U) // PLLP

// PLLM: PLL, PLLSAI1 and PLLSAI2 division factor
#define RCC_PLLM_DIV1              ((uint32_t)0x00000000U)                     // 1
#define RCC_PLLM_DIV2              RCC_PLLCFGR_PLLM_0                          // 2
#define RCC_PLLM_DIV3              RCC_PLLCFGR_PLLM_1                          // 3
#define RCC_PLLM_DIV4              ((RCC_PLLCFGR_PLLM_1 | RCC_PLLCFGR_PLLM_0)) // 4
#define RCC_PLLM_DIV5              RCC_PLLCFGR_PLLM_2                          // 5
#define RCC_PLLM_DIV6              ((RCC_PLLCFGR_PLLM_2 | RCC_PLLCFGR_PLLM_0)) // 6
#define RCC_PLLM_DIV7              ((RCC_PLLCFGR_PLLM_2 | RCC_PLLCFGR_PLLM_1)) // 7
#define RCC_PLLM_DIV8              RCC_PLLCFGR_PLLM                            // 8

// PLLR: main PLL division factor for PLLCLK
#define RCC_PLLR_DIV2              ((uint32_t)0x00000000U) // 2
#define RCC_PLLR_DIV4              RCC_PLLCFGR_PLLR_0      // 4
#define RCC_PLLR_DIV6              RCC_PLLCFGR_PLLR_1      // 6
#define RCC_PLLR_DIV8              RCC_PLLCFGR_PLLR        // 8

// PLLQ: main PLL division factor PLL48M1CLK (clock for USB, RNG and SDMMC)
#define RCC_PLLQ_DIV2              ((uint32_t)0x00000000U) // 2
#define RCC_PLLQ_DIV4              RCC_PLLCFGR_PLLQ_0      // 4
#define RCC_PLLQ_DIV6              RCC_PLLCFGR_PLLQ_1      // 6
#define RCC_PLLQ_DIV8              RCC_PLLCFGR_PLLQ        // 8

// PLLP: main PLL division factor for PLLSAI3CLK (clock for SAI1 and SAI2)
#define RCC_PLLP_DIV7              ((uint32_t)0x00000000U) // 7
#define RCC_PLLP_DIV17             RCC_PLLCFGR_PLLP        // 17

// PLLSAI1R: PLLSAI1 division factor for PLLADC1CLK (clock for ADC)
#define RCC_PLLSAI1R_DIV2          ((uint32_t)0x00000000U)    // 2
#define RCC_PLLSAI1R_DIV4          RCC_PLLSAI1CFGR_PLLSAI1R_0 // 4
#define RCC_PLLSAI1R_DIV6          RCC_PLLSAI1CFGR_PLLSAI1R_1 // 6
#define RCC_PLLSAI1R_DIV8          RCC_PLLSAI1CFGR_PLLSAI1R   // 8

// PLLSAI1Q: PLLSAI1 division factor for PLL48M2CLK (clock for USB, RNG and SDMMC)
#define RCC_PLLSAI1Q_DIV2          ((uint32_t)0x00000000U)    // 2
#define RCC_PLLSAI1Q_DIV4          RCC_PLLSAI1CFGR_PLLSAI1Q_0 // 4
#define RCC_PLLSAI1Q_DIV6          RCC_PLLSAI1CFGR_PLLSAI1Q_1 // 6
#define RCC_PLLSAI1Q_DIV8          RCC_PLLSAI1CFGR_PLLSAI1Q   // 8

// PLLPSAI1P: PLLSAI1 division factor for PLLSAI1CLK (clock for SAI1 and SAI2)
#define RCC_PLLSAI1P_DIV7          ((uint32_t)0x00000000U)  // 7
#define RCC_PLLSAI1P_DIV17         RCC_PLLSAI1CFGR_PLLSAI1P // 17

// PLLSAI1R: PLLSAI2 division factor for PLLADC2CLK (clock for ADC)
#define RCC_PLLSAI2R_DIV2          ((uint32_t)0x00000000U)    // 2
#define RCC_PLLSAI2R_DIV4          RCC_PLLSAI2CFGR_PLLSAI2R_0 // 4
#define RCC_PLLSAI2R_DIV6          RCC_PLLSAI2CFGR_PLLSAI2R_1 // 6
#define RCC_PLLSAI2R_DIV8          RCC_PLLSAI2CFGR_PLLSAI2R   // 8

// PLLPSAI1P: PLLSAI2 division factor for PLLSAI2CLK (clock for SAI1 and SAI2)
#define RCC_PLLSAI2P_DIV7          ((uint32_t)0x00000000U)  // 7
#define RCC_PLLSAI2P_DIV17         RCC_PLLSAI2CFGR_PLLSAI2P // 17

// Definitions of AHB prescaler
#define RCC_AHB_DIV1               RCC_CFGR_HPRE_DIV1   // SYSCLK not divided
#define RCC_AHB_DIV2               RCC_CFGR_HPRE_DIV2   // SYSCLK divided by 2
#define RCC_AHB_DIV4               RCC_CFGR_HPRE_DIV4   // SYSCLK divided by 4
#define RCC_AHB_DIV8               RCC_CFGR_HPRE_DIV8   // SYSCLK divided by 8
#define RCC_AHB_DIV16              RCC_CFGR_HPRE_DIV16  // SYSCLK divided by 16
#define RCC_AHB_DIV64              RCC_CFGR_HPRE_DIV64  // SYSCLK divided by 64
#define RCC_AHB_DIV128             RCC_CFGR_HPRE_DIV128 // SYSCLK divided by 128
#define RCC_AHB_DIV256             RCC_CFGR_HPRE_DIV256 // SYSCLK divided by 256
#define RCC_AHB_DIV512             RCC_CFGR_HPRE_DIV512 // SYSCLK divided by 512

// Definitions of APB low-speed (APB1) prescaler
#define RCC_APB1_DIV1              RCC_CFGR_PPRE1_DIV1  // HCLK not divided
#define RCC_APB1_DIV2              RCC_CFGR_PPRE1_DIV2  // HCLK divided by 2
#define RCC_APB1_DIV4              RCC_CFGR_PPRE1_DIV4  // HCLK divided by 4
#define RCC_APB1_DIV8              RCC_CFGR_PPRE1_DIV8  // HCLK divided by 8
#define RCC_APB1_DIV16             RCC_CFGR_PPRE1_DIV16 // HCLK divided by 16

// Definitions of APB high-speed prescaler (APB2)
#define RCC_APB2_DIV1              RCC_CFGR_PPRE2_DIV1  // HCLK not divided
#define RCC_APB2_DIV2              RCC_CFGR_PPRE2_DIV2  // HCLK divided by 2
#define RCC_APB2_DIV4              RCC_CFGR_PPRE2_DIV4  // HCLK divided by 4
#define RCC_APB2_DIV8              RCC_CFGR_PPRE2_DIV8  // HCLK divided by 8
#define RCC_APB2_DIV16             RCC_CFGR_PPRE2_DIV16 // HCLK divided by 16

// Defines used for FLASH latency selection according to HCLK frequency
#define RCC_SCALE1_LATENCY1        ((uint32_t)16000000U) // FLASH latency 1 in power scale 1
#define RCC_SCALE1_LATENCY2        ((uint32_t)32000000U) // FLASH latency 2 in power scale 1
#define RCC_SCALE1_LATENCY3        ((uint32_t)48000000U) // FLASH latency 3 in power scale 1
#define RCC_SCALE1_LATENCY4        ((uint32_t)64000000U) // FLASH latency 4 in power scale 1
#define RCC_SCALE2_LATENCY1        ((uint32_t)6000000U)  // FLASH latency 1 in power scale 2
#define RCC_SCALE2_LATENCY2        ((uint32_t)12000000U) // FLASH latency 2 in power scale 2
#define RCC_SCALE2_LATENCY3        ((uint32_t)18000000U) // FLASH latency 3 in power scale 2

// Definitions of MSI states
#define RCC_MSI_OFF                RCC_CR_MSIRANGE    // disabled
#define RCC_MSI_100K               RCC_CR_MSIRANGE_0  // 100 KHz
#define RCC_MSI_200K               RCC_CR_MSIRANGE_1  // 200 KHz
#define RCC_MSI_400K               RCC_CR_MSIRANGE_2  // 400 KHz
#define RCC_MSI_800K               RCC_CR_MSIRANGE_3  // 800 KHz
#define RCC_MSI_1M                 RCC_CR_MSIRANGE_4  // 1 MHz
#define RCC_MSI_2M                 RCC_CR_MSIRANGE_5  // 2 MHz
#define RCC_MSI_4M                 RCC_CR_MSIRANGE_6  // 4 MHz
#define RCC_MSI_8M                 RCC_CR_MSIRANGE_7  // 8 MHz
#define RCC_MSI_16M                RCC_CR_MSIRANGE_8  // 16 MHz
#define RCC_MSI_24M                RCC_CR_MSIRANGE_9  // 24 MHz
#define RCC_MSI_32M                RCC_CR_MSIRANGE_10 // 32 MHz
#define RCC_MSI_48M                RCC_CR_MSIRANGE_11 // 48 MHz

// Definitions of MSI ranges after standby mode
#define RCC_MSIS_1M                RCC_CSR_MSISRANGE_1 // 1 MHz
#define RCC_MSIS_2M                RCC_CSR_MSISRANGE_2 // 2 MHz
#define RCC_MSIS_4M                RCC_CSR_MSISRANGE_4 // 4 MHz
#define RCC_MSIS_8M                RCC_CSR_MSISRANGE_8 // 8 MHz

// Definitions of LSE oscillator state
#define RCC_LSE_OFF                ((uint32_t)0x00000000U) // disabled
#define RCC_LSE_ON                 RCC_CR_HSEON            // enabled
#define RCC_LSE_BYPASS             ((RCC_CR_HSEBYP | RCC_CR_HSEON)) // bypassed by external clock

// Definitions of LSE oscillator drive capability
#define RCC_LSE_DRV_LOW            ((uint32_t)0x00000000U) // Lower
#define RCC_LSE_DRV_MEDLOW         RCC_BDCR_LSEDRV_0       // Medium low
#define RCC_LSE_DRV_MEDHIGH        RCC_BDCR_LSEDRV_1       // Medium high
#define RCC_LSE_DRV_HIGH           RCC_BDCR_LSEDRV         // Higher

// Definitions of LSI oscillator state
#define RCC_LSI_OFF                ((uint32_t)0x00000000U) // disabled
#define RCC_LSI_ON                 RCC_CSR_LSION           // enabled

// Definitions of HSE state
#define RCC_HSE_OFF                ((uint32_t)0x00000000U) // disabled
#define RCC_HSE_ON                 RCC_CR_HSEON            // enabled
#define RCC_HSE_BYPASS             ((RCC_CR_HSEBYP | RCC_CR_HSEON)) // bypassed by external clock

// Definitions of HSI state
#define RCC_HSI_OFF                ((uint32_t)0x00000000U) // disabled
#define RCC_HSI_ON                 RCC_CR_HSION            // enabled

// Definitions of peripheral clock sources
#define RCC_PERIPH_CLK_PCLK        ((uint32_t)0x00000000U) // PCLK1/PCLK2 clock (APB1/APB2)
#define RCC_PERIPH_CLK_SYSCLK      ((uint32_t)0x00000001U) // SYSCLK clock
#define RCC_PERIPH_CLK_HSI         ((uint32_t)0x00000002U) // HSI clock
#define RCC_PERIPH_CLK_LSE         ((uint32_t)0x00000003U) // LSE clock
#define RCC_PERIPH_CLK_MASK        ((uint32_t)0x00000003U) // Mask bits

// Definitions of USART/UART peripheral clock selection
#define RCC_USART1_CLK_SRC         RCC_CCIPR_USART1SEL // USART1 clock source selection
#define RCC_USART2_CLK_SRC         RCC_CCIPR_USART2SEL // USART2 clock source selection
#define RCC_USART3_CLK_SRC         RCC_CCIPR_USART3SEL // USART3 clock source selection
#define RCC_UART4_CLK_SRC          RCC_CCIPR_UART4SEL  // UART4 clock source selection
#define RCC_UART5_CLK_SRC          RCC_CCIPR_UART5SEL  // UART5 clock source selection

// Definitions of RTC peripheral clock selection
#define RCC_RTC_CLK_NONE           ((uint32_t)0x00000000U) // No clock
#define RCC_RTC_CLK_LSE            RCC_BDCR_RTCSEL_0       // LSE oscillator
#define RCC_RTC_CLK_LSI            RCC_BDCR_RTCSEL_1       // LSI oscillator
#define RCC_RTC_CLK_HSE32          RCC_BDCR_RTCSEL         // HSE oscillator clock divided by 32

// Definitions of system clock selection after wake-up from STOP
#define RCC_WAKEUPCLOCK_MSI        ((uint32_t)0x00000000U) // MSI oscillator
#define RCC_WAKEUPCLOCK_HSI        RCC_CFGR_STOPWUCK       // HSI oscillator

// Definitions of I2C peripheral clock selection
#define RCC_I2C1_CLK_SRC           RCC_CCIPR_I2C1SEL // I2C1 clock source selection
#define RCC_I2C2_CLK_SRC           RCC_CCIPR_I2C2SEL // I2C2 clock source selection
#define RCC_I2C3_CLK_SRC           RCC_CCIPR_I2C3SEL // I2C3 clock source selection

// Definitions of CLK48 (USB, RNG and SDMMC) clock source selection
#define RCC_CLK48_CLK_NONE         ((uint32_t)0x00000000U) // No clock
#define RCC_CLK48_CLK_PLLSAI1Q     RCC_CCIPR_CLK48SEL_0    // PLLSAI1Q (PLL48M2CLK) clock
#define RCC_CLK48_CLK_PLLQ         RCC_CCIPR_CLK48SEL_1    // main PLLQ (PLL48M1CLK) clock
#define RCC_CLK48_CLK_MSI          RCC_CCIPR_CLK48SEL      // PLLSAI1Q (PLL48M2CLK) clock


// Structure definitions

// Structure to store clock frequencies (Hz)
typedef struct {
	uint32_t SYSCLK_Frequency; // SYSCLK
	uint32_t HCLK_Frequency;   // HCLK
	uint32_t PCLK1_Frequency;  // APB1
	uint32_t PCLK2_Frequency;  // APB2
} RCC_ClocksTypeDef;

// PLL configuration parameters
typedef struct {
	uint32_t PLLN; // PLLN multiplier, value in range between 8 and 86
	uint32_t PLLR; // PLLR divider:
				//   main PLL: one of RCC_PLLR_DIVx values
				//   PLLSAI1 : one of RCC_PLLSAI1R_DIVx values
				//   PLLSAI2 : one of RCC_PLLSAI2R_DIVx values
	uint32_t PLLQ; // PLLQ divider:
				//   main PLL: one of RCC_PLLQ_DIVx values
				//   PLLSAI1 : one of RCC_PLLSAI1Q_DIVx values
				//   PLLSAI2 : ignored
	uint32_t PLLP; // PLLP divider:
				//   main PLL: one of RCC_PLLP_DIVx values
				//   PLLSAI1 : one of RCC_PLLSAI1P_DIVx values
				//   PLLSAI2 : one of RCC_PLLSAI2P_DIVx values
} RCC_PLLInitTypeDef;

// Clock dividers
typedef struct {
	uint32_t AHBdiv;  // AHB divider, one of RCC_AHB_DIVx values
	uint32_t APB1div; // APB1 divider, one of RCC_APB1_DIVx values
	uint32_t APB2div; // APB2 divider, one of RCC_APB2_DIVx values
} RCC_CLKInitTypeDef;


// Public functions and macros

// Enable prefetch buffer
__STATIC_INLINE void RCC_PrefetchEnable(void) {
	FLASH->ACR |= FLASH_ACR_PRFTEN;
}

// Disable prefetch buffer
__STATIC_INLINE void RCC_PrefetchDisable(void) {
	FLASH->ACR &= ~FLASH_ACR_PRFTEN;
}

// Get flash latency
// return: a flash latency, one of FLASH_ACR_LATENCY_xx values
__STATIC_INLINE uint32_t RCC_GetLatency(void) {
	return (FLASH->ACR & FLASH_ACR_LATENCY);
}

// Enable RTC clock
__STATIC_INLINE void RCC_RTCEnable(void) {
	RCC->BDCR |= RCC_BDCR_RTCEN;
}

// Disable RTC clock
__STATIC_INLINE void RCC_RTCDisable(void) {
	RCC->BDCR &= ~RCC_BDCR_RTCEN;
}

// Backup domain software reset
__STATIC_INLINE void RCC_BkpReset(void) {
	RCC->BDCR |=  RCC_BDCR_BDRST;
	RCC->BDCR &= ~RCC_BDCR_BDRST;
}

// Check if LSE oscillator is ready
// return: state of oscillator (0 or 1)
__STATIC_INLINE uint32_t RCC_LSEIsReady(void) {
	return ((RCC->BDCR & RCC_BDCR_LSERDY) == RCC_BDCR_LSERDY);
}

// Check if LSI oscillator is ready
// return: state of oscillator (0 or 1)
// note: LSIRDY bit will be set if LSI is off but LSECSS is on
__STATIC_INLINE uint32_t RCC_LSIIsReady(void) {
	return ((RCC->CSR & RCC_CSR_LSIRDY) == RCC_CSR_LSIRDY);
}

// Check if HSE oscillator is ready
// return: state of oscillator (0 or 1)
__STATIC_INLINE uint32_t RCC_HSEIsReady(void) {
	return ((RCC->CR & RCC_CR_HSERDY) == RCC_CR_HSERDY);
}

// Check if LSE oscillator is on
// return: state of oscillator (0 or 1)
__STATIC_INLINE uint32_t RCC_LSEIsOn(void) {
	return ((RCC->BDCR & RCC_BDCR_LSEON) == RCC_BDCR_LSEON);
}

// Check if LSI oscillator is on
// return: state of oscillator (0 or 1)
__STATIC_INLINE uint32_t RCC_LSIIsOn(void) {
	return ((RCC->CSR & RCC_CSR_LSION) == RCC_CSR_LSION);
}

// Check if HSE oscillator is on
// return: state of oscillator (0 or 1)
__STATIC_INLINE uint32_t RCC_HSEIsOn(void) {
	return ((RCC->CR & RCC_CR_HSEON) == RCC_CR_HSEON);
}

// Configure system clock selection after wake-up from STOP mode
// input:
//   clock_src - specifies which clock to select, one of RCC_WAKEUPCLOCK_xx values
__STATIC_INLINE void RCC_SetWakeClock(uint32_t clock_src) {
	RCC->CFGR &= ~RCC_CFGR_STOPWUCK;
	RCC->CFGR |= clock_src & RCC_CFGR_STOPWUCK;
}

// Configure LSE oscillator drive capability
// input:
//   drv_cap - new LSE oscillator drive capability, one of RCC_LSE_DRV_xx values
// note: access to the backup domain must be enabled
__STATIC_INLINE void RCC_SetLSEDrive(uint32_t drv_cap) {
	RCC->BDCR &= ~RCC_BDCR_LSEDRV;
	RCC->BDCR |= drv_cap & RCC_BDCR_LSEDRV;
}

// Enable CSS for LSE
// note: must be called after LSE and LSI are enabled, ready, and after the RTC
//       clock has been selected
__STATIC_INLINE void RCC_CSSLSEEnable(void) {
	RCC->BDCR |= RCC_BDCR_LSECSSON;
}

// Disable CSS for LSE
// note: call to this function only makes sense after a failure detection on LSE
__STATIC_INLINE void RCC_CSSLSEDisable(void) {
	RCC->BDCR &= ~RCC_BDCR_LSECSSON;
}

// Get state of CSS for LSE
// return: state of CSS (0 or 1)
__STATIC_INLINE uint32_t RCC_IsCSSLSEOn(void) {
	return ((RCC->BDCR & RCC_BDCR_LSECSSON) == RCC_BDCR_LSECSSON);
}

// Enable LSE CSS interrupt
__STATIC_INLINE void RCC_CSSLSEEnableIRQ(void) {
	RCC->CIER |= RCC_CIER_LSECSSIE;
}

// Clear the CSS for LSE flag
__STATIC_INLINE void RCC_CSSLSEClearFlag(void) {
	RCC->CICR = RCC_CICR_LSECSSC;
}

// Retrieve current RTC clock source
// return: RTC clock source, one of RCC_RTC_CLK_xx values
__STATIC_INLINE uint32_t RCC_GetClockRTC(void) {
	return (RCC->BDCR & RCC_BDCR_RTCSEL);
}


// Function prototypes
uint32_t RCC_GetSysClockSource(void);

uint32_t RCC_GetMSIFreq(void);
uint32_t RCC_GetSYSCLKFreq(void);
uint32_t RCC_GetHCLKFreq(uint32_t sysclk);
uint32_t RCC_GetPCLK1Freq(uint32_t hclk);
uint32_t RCC_GetPCLK2Freq(uint32_t hclk);
void RCC_GetClocksFreq(RCC_ClocksTypeDef* RCC_Clocks);

void RCC_PLLSrcConfig(uint32_t pll_src);
void RCC_PLLMConfig(uint32_t pllm);
void RCC_PLLOutEnable(uint32_t pll, uint32_t pll_out);
void RCC_PLLOutDisable(uint32_t pll, uint32_t pll_out);
void RCC_PLLDisable(uint32_t pll);
ErrorStatus RCC_PLLConfig(uint32_t pll, RCC_PLLInitTypeDef *cfgPLL);
ErrorStatus RCC_MSIConfig(uint32_t state);
ErrorStatus RCC_HSIConfig(uint32_t state);
ErrorStatus RCC_HSEConfig(uint32_t state);
ErrorStatus RCC_LSIConfig(uint32_t state);
ErrorStatus RCC_LSEConfig(uint32_t state);

ErrorStatus RCC_SetClockMSI(RCC_CLKInitTypeDef *cfgCLK);
ErrorStatus RCC_SetClockHSI(RCC_CLKInitTypeDef *cfgCLK);
ErrorStatus RCC_SetClockHSE(RCC_CLKInitTypeDef *cfgCLK);
ErrorStatus RCC_SetClockPLL(uint32_t clock_source, RCC_PLLInitTypeDef *cfgPLL, RCC_CLKInitTypeDef *cfgCLK);

uint32_t RCC_GetClockUSART(uint32_t periph_sel);

void RCC_SetClockUSART(uint32_t periph_sel, uint32_t clock_src);
void RCC_SetClockRTC(uint32_t clock_src);
void RCC_SetClockI2C(uint32_t periph_sel, uint32_t clock_src);
void RCC_SetClock48M(uint32_t clock_src);

#endif // __RCC_H
