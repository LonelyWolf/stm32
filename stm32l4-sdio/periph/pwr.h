#ifndef __PWR_H
#define __PWR_H


#include <stm32l4xx.h>


// Definitions of reset source
#define PWR_RESET_SRC_UNKNOWN      ((uint32_t)0x00000000U) // Unknown reset source
#define PWR_RESET_SRC_SOFT         ((uint32_t)0x00000001U) // Software reset
#define PWR_RESET_SRC_BOR          ((uint32_t)0x00000002U) // BOR reset
#define PWR_RESET_SRC_PIN          ((uint32_t)0x00000004U) // Reset from NRST pin
#define PWR_RESET_SRC_LPWR         ((uint32_t)0x00000008U) // Reset after illegal Stop, Standby or Shutdown mode entry
#define PWR_RESET_SRC_WWDG         ((uint32_t)0x00000010U) // Window watchdog reset
#define PWR_RESET_SRC_IWDG         ((uint32_t)0x00000020U) // Independent watchdog reset
#define PWR_RESET_SRC_OBL          ((uint32_t)0x00000040U) // Option byte loading reset
#define PWR_RESET_SRC_FWR          ((uint32_t)0x00000080U) // Firewall reset
#define PWR_RESET_SRC_STBY         ((uint32_t)0x00000100U) // Wakeup from shutdown/standby
#define PWR_RESET_SRC_STBY_WUFI    ((uint32_t)0x00000200U) // Wakeup from shutdown/standby by internal wakeup line
#define PWR_RESET_SRC_STBY_WKUP1   ((uint32_t)0x00000400U) // Wakeup from shutdown/standby by WKUP1 pin
#define PWR_RESET_SRC_STBY_WKUP2   ((uint32_t)0x00000800U) // Wakeup from shutdown/standby by WKUP2 pin
#define PWR_RESET_SRC_STBY_WKUP3   ((uint32_t)0x00001000U) // Wakeup from shutdown/standby by WKUP3 pin
#define PWR_RESET_SRC_STBY_WKUP4   ((uint32_t)0x00002000U) // Wakeup from shutdown/standby by WKUP4 pin
#define PWR_RESET_SRC_STBY_WKUP5   ((uint32_t)0x00004000U) // Wakeup from shutdown/standby by WKUP5 pin

// Sleep/Stop mode entry definitions
#define PWR_SENTRY_WFI             ((uint32_t)0x00000001U) // Wait for interrupt (WFI) instruction
#define PWR_SENTRY_WFE             ((uint32_t)0x00000002U) // Wait for event (WFE) instruction

// Stop mode definitions
#define PWR_STOP_MODE0             PWR_CR1_LPMS_STOP0 // Stop 0 mode
#define PWR_STOP_MODE1             PWR_CR1_LPMS_STOP1 // Stop 1 mode
#define PWR_STOP_MODE2             PWR_CR1_LPMS_STOP2 // Stop 2 mode

// Wakeup pins definitions
#define PWR_WKUP_PIN_1             PWR_CR3_EWUP1 // Wakeup pin #1
#define PWR_WKUP_PIN_2             PWR_CR3_EWUP2 // Wakeup pin #2
#define PWR_WKUP_PIN_3             PWR_CR3_EWUP3 // Wakeup pin #3
#define PWR_WKUP_PIN_4             PWR_CR3_EWUP4 // Wakeup pin #4
#define PWR_WKUP_PIN_5             PWR_CR3_EWUP5 // Wakeup pin #5

// Wakeup pin polarity definition
#define PWR_WKUP_PH                ((uint32_t)0x00000000U) // Wakeup event on rising edge
#define PWR_WKUP_PL                ((uint32_t)0x00000001U) // Wakeup event on falling edge

// GPIO port pull-up control definitions
#define PWR_GPIOA                  ((uint32_t)0x00000000U) // GPIOA
#define PWR_GPIOB                  ((uint32_t)0x00000001U) // GPIOB
#define PWR_GPIOC                  ((uint32_t)0x00000002U) // GPIOC
#if defined(GPIOD_BASE)
#define PWR_GPIOD                  ((uint32_t)0x00000003U) // GPIOD
#endif
#if defined(GPIOE_BASE)
#define PWR_GPIOE                  ((uint32_t)0x00000004U) // GPIOE
#endif
#if defined(GPIOF_BASE)
#define PWR_GPIOF                  ((uint32_t)0x00000005U) // GPIOF
#endif
#if defined(GPIOG_BASE)
#define PWR_GPIOG                  ((uint32_t)0x00000006U) // GPIOG
#endif
#define PWR_GPIOH                  ((uint32_t)0x00000007U) // GPIOH
#if defined(GPIOI_BASE)
#define PWR_GPIOI                  ((uint32_t)0x00000008U) // GPIOI
#endif

// GPIO pin pull-up control definitions
#define PWR_GPIO_PIN_0             PWR_PUCRA_PA0  // Pin 0
#define PWR_GPIO_PIN_1             PWR_PUCRA_PA1  // Pin 1
#define PWR_GPIO_PIN_2             PWR_PUCRA_PA2  // Pin 2
#define PWR_GPIO_PIN_3             PWR_PUCRA_PA3  // Pin 3
#define PWR_GPIO_PIN_4             PWR_PUCRA_PA4  // Pin 4
#define PWR_GPIO_PIN_5             PWR_PUCRA_PA5  // Pin 5
#define PWR_GPIO_PIN_6             PWR_PUCRA_PA6  // Pin 6
#define PWR_GPIO_PIN_7             PWR_PUCRA_PA7  // Pin 7
#define PWR_GPIO_PIN_8             PWR_PUCRA_PA8  // Pin 8
#define PWR_GPIO_PIN_9             PWR_PUCRA_PA9  // Pin 9
#define PWR_GPIO_PIN_10            PWR_PUCRA_PA10 // Pin 10
#define PWR_GPIO_PIN_11            PWR_PUCRA_PA11 // Pin 11
#define PWR_GPIO_PIN_12            PWR_PUCRA_PA12 // Pin 12
#define PWR_GPIO_PIN_13            PWR_PUCRA_PA13 // Pin 13
#define PWR_GPIO_PIN_14            PWR_PDCRA_PA14 // Pin 14
#define PWR_GPIO_PIN_15            PWR_PUCRA_PA15 // Pin 15

// GPIO pin pull-up state definitions
#define PWR_GPIO_DISABLE           ((uint32_t)0x00000000U) // Pull-up/pull-down disabled
#define PWR_GPIO_PU                ((uint32_t)0x00000001U) // Pull-up
#define PWR_GPIO_PD                ((uint32_t)0x00000002U) // Pull-down


// Public macros/functions

// Enable access to the backup domain
__STATIC_INLINE void PWR_BkpAccessEnable(void) {
	PWR->CR1 |= PWR_CR1_DBP;
}

// Disable access to the backup domain
__STATIC_INLINE void PWR_BkpAccessDisable(void) {
	PWR->CR1 &= ~PWR_CR1_DBP;
}

// Enable Cortex sleep-on-exit mode
// After execution of all IRQ handlers the MCU will enter back to sleep mode if it was woken by an interrupt
__STATIC_INLINE void PWR_EnableSleepOnExit(void) {
	SCB->SCR |= (uint32_t)SCB_SCR_SLEEPONEXIT_Msk;
}

// Disable Cortex sleep-on-exit mode
// After execution of all IRQ handlers the MCU will back to main thread
__STATIC_INLINE void PWR_DisableSleepOnExit(void) {
	SCB->SCR &= ~((uint32_t)SCB_SCR_SLEEPONEXIT_Msk);
}

// Enable wakeup pin (WKUP#)
// input:
//   pin - wakeup pin to enable, one of PWR_WKUP_PIN_x values
__STATIC_INLINE void PWR_WKUPEnable(uint32_t pin) {
	PWR->CR3 |= pin;
}

// Disable wakeup pin (WKUP#)
// input:
//   pin - wakeup pin to disable, one of PWR_WKUP_PIN_x values
__STATIC_INLINE void PWR_WKUPDisable(uint32_t pin) {
	PWR->CR3 &= ~pin;
}

// Enable GPIO pull-up and pull-down configuration in standby and shutdown modes
__STATIC_INLINE void PWR_EnablePUCfg(void) {
	PWR->CR3 |= PWR_CR3_APC;
}

// Disable GPIO pull-up and pull-down configuration in standby and shutdown modes
__STATIC_INLINE void PWR_DisablePUCfg(void) {
	PWR->CR3 &= ~PWR_CR3_APC;
}


// Function prototypes
uint32_t PWR_GetResetSource(void);

void PWR_EnterSLEEPMode(uint32_t entry);
void PWR_EnterSTOPMode(uint32_t entry, uint32_t mode);
void PWR_EnterSTANDBYMode(void);
void PWR_EnterSHUTDOWNMode(void);

void PWR_WKUPPolarity(uint32_t pin, uint32_t polarity);
void PWR_GPIOPUSet(uint32_t GPIO, uint32_t pins, uint32_t state);

#endif // __PWR_H
