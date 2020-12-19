#ifndef __STM32L4xx_CLOCKS
#define __STM32L4xx_CLOCKS

// Oscillator values adaptation

// The value of External High Speed oscillator (HSE)
#if !defined(HSE_VALUE)
  #define HSE_VALUE             ((uint32_t)16000000U) // Hz
#endif // HSE_VALUE

// The default value of Internal Multiple Speed oscillator (MSI)
// This value is the default MSI range value after reset
#if !defined(MSI_VALUE)
  #define MSI_VALUE             ((uint32_t)4000000U) // Hz
#endif // MSI_VALUE

// The value of Internal High Speed oscillator (HSI)
#if !defined(HSI_VALUE)
  #define HSI_VALUE             ((uint32_t)16000000U) // Hz
#endif // HSI_VALUE

// The value of External Low Speed oscillator (LSE)
#if !defined(LSE_VALUE)
  #define LSE_VALUE             ((uint32_t)32768U) // Hz
#endif // LSE_VALUE

// The value of Internal High Speed oscillator for USB FS/SDMMC/RNG (HSI48)
#if defined(RCC_HSI48_SUPPORT)
  #if !defined(HSI48_VALUE)
    #define HSI48_VALUE  ((uint32_t)48000000U) // Hz
  #endif // HSI48_VALUE
#endif // RCC_HSI48_SUPPORT

// Startup timeout for HSE
#if !defined(HSE_STARTUP_TIMEOUT)
  #define HSE_STARTUP_TIMEOUT   ((uint32_t)5000U)
#endif // HSE_STARTUP_TIMEOUT

// Startup timeout for LSE
#if !defined(LSE_STARTUP_TIMEOUT)
  #define LSE_STARTUP_TIMEOUT   ((uint32_t)5000U)
#endif // HSE_STARTUP_TIMEOUT

#endif // __STM32L4xx_CLOCKS