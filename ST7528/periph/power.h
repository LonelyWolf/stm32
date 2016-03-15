#ifndef __POWER_H
#define __POWER_H


#include <stm32l1xx.h>


// Define which WKUP pins will be used
//   0 - pin is not used
//   1 - pin used
#define POWER_USE_WKUP1           1
#define POWER_USE_WKUP2           1
#define POWER_USE_WKUP3           0


// Reset source
#define RESET_SRC_UNKNOWN         (uint32_t)0x00000000 // Unknown reset source
#define RESET_SRC_POR             (uint32_t)0x00000001 // POR/PDR (Power On Reset or Power Down Reset)
#define RESET_SRC_SOFT            (uint32_t)0x00000002 // Software reset
#define RESET_SRC_PIN             (uint32_t)0x00000004 // Reset from NRST pin
#define RESET_SRC_STBY            (uint32_t)0x00000008 // Reset after STANBY, wake-up flag set (RTC or WKUP# pins)
#define RESET_SRC_STBY_WP1        (uint32_t)0x00000010 // Reset after STANBY, wake-up from WKUP1 pin
#define RESET_SRC_STBY_WP2        (uint32_t)0x00000020 // Reset after STANBY, wake-up from WKUP2 pin
#define RESET_SRC_STBY_WP3        (uint32_t)0x00000040 // Reset after STANBY, wake-up from WKUP3 pin


// WKUP pins
#if (POWER_USE_WKUP1) || (POWER_USE_WKUP2) || (POWER_USE_WKUP3)
#define POWER_USE_WKUP_PINS       1
#endif

#if (POWER_USE_WKUP1)
// WKUP1 - PA0
#define WKUP1_AHB_PERIPHERAL      RCC_AHBENR_GPIOAEN
#define WKUP1_STATE               (*(__I uint32_t *)(PERIPH_BB_BASE + ((((uint32_t)&(GPIOA->IDR)) - PERIPH_BASE) << 5) + (0 << 2)))
#endif

#if (POWER_USE_WKUP2)
// WKUP2 - PC13
#define WKUP2_AHB_PERIPHERAL      RCC_AHBENR_GPIOCEN
#define WKUP2_STATE               (*(__I uint32_t *)(PERIPH_BB_BASE + ((((uint32_t)&(GPIOC->IDR)) - PERIPH_BASE) << 5) + (13 << 2)))
#endif

#if (POWER_USE_WKUP3)
// WKUP3 - PE6
#define WKUP3_AHB_PERIPHERAL      RCC_AHBENR_GPIOEEN
#define WKUP3_STATE               (*(__I uint32_t *)(PERIPH_BB_BASE + ((((uint32_t)&(GPIOE->IDR)) - PERIPH_BASE) << 5) + (6 << 2)))
#endif


// RestoreClocks enumeration
typedef enum {
	CLOCK_SKIP    = 0,
	CLOCK_RESTORE
} ClockAction;


// Function prototypes
void SleepWait(void);
void SleepStop(ClockAction RestoreClocks);
void SleepStandby(void);
uint32_t GetResetSource(void);

#endif // __POWER_H
