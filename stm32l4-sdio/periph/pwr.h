#ifndef __PWR_H
#define __PWR_H


#include <stm32l4xx.h>


// Definitions of reset source
#define PWR_RESET_SRC_UNKNOWN     (uint32_t)0x00000000U // Unknown reset source
#define PWR_RESET_SRC_SOFT        (uint32_t)0x00000001U // Software reset
#define PWR_RESET_SRC_BOR         (uint32_t)0x00000002U // BOR reset
#define PWR_RESET_SRC_PIN         (uint32_t)0x00000004U // Reset from NRST pin
#define PWR_RESET_SRC_LPWR        (uint32_t)0x00000008U // Reset after illegal Stop, Standby or Shutdown mode entry
#define PWR_RESET_SRC_WWDG        (uint32_t)0x00000010U // Window watchdog reset
#define PWR_RESET_SRC_IWDG        (uint32_t)0x00000020U // Independent watchdog reset
#define PWR_RESET_SRC_OBL         (uint32_t)0x00000040U // Option byte loading reset
#define PWR_RESET_SRC_FWR         (uint32_t)0x00000080U // Firewall reset
#define PWR_RESET_SRC_STBY        (uint32_t)0x00000100U // Wakeup from shutdown/standby
#define PWR_RESET_SRC_STBY_WUFI   (uint32_t)0x00000200U // Wakeup from shutdown/standby by internal wakeup line
#define PWR_RESET_SRC_STBY_WKUP1  (uint32_t)0x00000400U // Wakeup from shutdown/standby by WKUP1 pin
#define PWR_RESET_SRC_STBY_WKUP2  (uint32_t)0x00000800U // Wakeup from shutdown/standby by WKUP2 pin
#define PWR_RESET_SRC_STBY_WKUP3  (uint32_t)0x00001000U // Wakeup from shutdown/standby by WKUP3 pin
#define PWR_RESET_SRC_STBY_WKUP4  (uint32_t)0x00002000U // Wakeup from shutdown/standby by WKUP4 pin
#define PWR_RESET_SRC_STBY_WKUP5  (uint32_t)0x00004000U // Wakeup from shutdown/standby by WKUP5 pin

// Sleep/Stop mode entry definitions
#define PWR_SENTRY_WFI            (uint32_t)0x00000001U // Wait For Interrupt (WFI) instruction
#define PWR_SENTRY_WFE            (uint32_t)0x00000002U // Wait For Event (WFE) WFE instruction

// Stop mode definitions
#define PWR_STOP_MODE0            PWR_CR1_LPMS_STOP0 // Stop 0 mode
#define PWR_STOP_MODE1            PWR_CR1_LPMS_STOP1 // Stop 1 mode
#define PWR_STOP_MODE2            PWR_CR1_LPMS_STOP2 // Stop 2 mode


// Public macros/functions

// Enable access to the backup domain
__STATIC_INLINE void PWR_BkpAccessEnable(void) {
	PWR->CR1 |= PWR_CR1_DBP;
}

// Disable access to the backup domain
__STATIC_INLINE void PWR_BkpAccessDisable(void) {
	PWR->CR1 &= ~PWR_CR1_DBP;
}


// Function prototypes
uint32_t PWR_GetResetSource(void);

void PWR_EnterSLEEPMode(uint32_t entry);
void PWR_EnterSTOPMode(uint32_t entry, uint32_t mode);
void PWR_EnterSTANDBYMode(void);
void PWR_EnterSHUTDOWNMode(void);

#endif // __PWR_H
