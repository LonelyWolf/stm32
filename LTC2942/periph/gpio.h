#ifndef __GPIO_H
#define __GPIO_H


#include "stm32l1xx.h"


// GPIO pin definitions
#define GPIO_Pin_0                 ((uint16_t)(1 <<  0))
#define GPIO_Pin_1                 ((uint16_t)(1 <<  1))
#define GPIO_Pin_2                 ((uint16_t)(1 <<  2))
#define GPIO_Pin_3                 ((uint16_t)(1 <<  3))
#define GPIO_Pin_4                 ((uint16_t)(1 <<  4))
#define GPIO_Pin_5                 ((uint16_t)(1 <<  5))
#define GPIO_Pin_6                 ((uint16_t)(1 <<  6))
#define GPIO_Pin_7                 ((uint16_t)(1 <<  7))
#define GPIO_Pin_8                 ((uint16_t)(1 <<  8))
#define GPIO_Pin_9                 ((uint16_t)(1 <<  9))
#define GPIO_Pin_10                ((uint16_t)(1 << 10))
#define GPIO_Pin_11                ((uint16_t)(1 << 11))
#define GPIO_Pin_12                ((uint16_t)(1 << 12))
#define GPIO_Pin_13                ((uint16_t)(1 << 13))
#define GPIO_Pin_14                ((uint16_t)(1 << 14))
#define GPIO_Pin_15                ((uint16_t)(1 << 15))

// GPIO pin sources for alternative functions
#define GPIO_PinSource0            ((uint8_t)0x00)
#define GPIO_PinSource1            ((uint8_t)0x01)
#define GPIO_PinSource2            ((uint8_t)0x02)
#define GPIO_PinSource3            ((uint8_t)0x03)
#define GPIO_PinSource4            ((uint8_t)0x04)
#define GPIO_PinSource5            ((uint8_t)0x05)
#define GPIO_PinSource6            ((uint8_t)0x06)
#define GPIO_PinSource7            ((uint8_t)0x07)
#define GPIO_PinSource8            ((uint8_t)0x08)
#define GPIO_PinSource9            ((uint8_t)0x09)
#define GPIO_PinSource10           ((uint8_t)0x0A)
#define GPIO_PinSource11           ((uint8_t)0x0B)
#define GPIO_PinSource12           ((uint8_t)0x0C)
#define GPIO_PinSource13           ((uint8_t)0x0D)
#define GPIO_PinSource14           ((uint8_t)0x0E)
#define GPIO_PinSource15           ((uint8_t)0x0F)

// Alternative function selection
#define GPIO_AFIO0                 ((uint8_t)0x00)
#define GPIO_AFIO1                 ((uint8_t)0x01)
#define GPIO_AFIO2                 ((uint8_t)0x02)
#define GPIO_AFIO3                 ((uint8_t)0x03)
#define GPIO_AFIO4                 ((uint8_t)0x04)
#define GPIO_AFIO5                 ((uint8_t)0x05)
#define GPIO_AFIO6                 ((uint8_t)0x06)
#define GPIO_AFIO7                 ((uint8_t)0x07)
#define GPIO_AFIO8                 ((uint8_t)0x08)
#define GPIO_AFIO11                ((uint8_t)0x0B)
#define GPIO_AFIO12                ((uint8_t)0x0C)
#define GPIO_AFIO14                ((uint8_t)0x0E)
#define GPIO_AFIO15                ((uint8_t)0x0F)


// GPIO macros definitions
#define GPIO_MODE_SET(pin,mode)    (mode << ((pin) << 1)) // set MODER bits for a specified pin
#define GPIO_MODE_MSK(pin)         (0x03 << ((pin) << 1)) // mask MODER bits for a specified pin
#define GPIO_PUPD_SET(pin,pupd)    (pupd << ((pin) << 1)) // set PUPDR bits for a specified pin
#define GPIO_PUPD_MSK(pin)         (0x03 << ((pin) << 1)) // mask PUPDR bits for a specified pin
#define GPIO_SPD_SET(pin,spd)      (spd  << ((pin) << 1)) // set SPEEDR bits for a specified pin
#define GPIO_SPD_MSK(pin)          (0x03 << ((pin) << 1)) // mask SPEEDR bits for a specified pin
#define GPIO_AF_SET(pin,af)        (af   << ((pin) << 2)) // set AFR bits for a specified pin
#define GPIO_AF_MSK(pin)           (0x0f << ((pin) << 2)) // mask AFR bits for a specified pin

// Alias word access to pin input state
// input:
//   PORT - one of GPIOx values
//   pin - pin number
#define GPIO_PIN_ISTATE(PORT,pin)  (*(__I uint32_t *)(PERIPH_BB_BASE + ((((uint32_t)&((PORT)->IDR)) - PERIPH_BASE) << 5) + ((pin) << 2)))

// Alias word access to pin output state
// input:
//   PORT - one of GPIOx values
//   pin - pin number
#define GPIO_PIN_OSTATE(PORT,pin)  (*(__I uint32_t *)(PERIPH_BB_BASE + ((((uint32_t)&((PORT)->ODR)) - PERIPH_BASE) << 5) + ((pin) << 2)))


// GPIO pin configuration mode
typedef enum {
	GPIO_Mode_IN  = 0x00, // input
	GPIO_Mode_OUT = 0x01, // output
	GPIO_Mode_AF  = 0x02, // alternative function
	GPIO_Mode_AN  = 0x03  // analog
} GPIOMode_TypeDef;

// GPIO pin pull-up/pull-down configuration
typedef enum {
	GPIO_PUPD_NONE = 0x00, // no pull-up, pull-down
	GPIO_PUPD_PU   = 0x01, // pull-up
	GPIO_PUPD_PD   = 0x02  // pull-down
} GPIOPUPD_TypeDef;

// GPIO pin output type
typedef enum {
	GPIO_OT_PP = 0x00, // push-pull
	GPIO_OT_OD = 0x01  // open-drain
} GPIOOT_TypeDef;

// GPIO pin output speed
typedef enum {
	GPIO_SPD_VERYLOW = 0x00, // very low
	GPIO_SPD_LOW     = 0x01, // low
	GPIO_SPD_MEDIUM  = 0x02, // medium
	GPIO_SPD_HIGH    = 0x03  // high
} GPIOSPD_TypeDef;


// EXTI lines definitions
#define EXTI_Line0                 ((uint32_t)(1 <<  0))
#define EXTI_Line1                 ((uint32_t)(1 <<  1))
#define EXTI_Line2                 ((uint32_t)(1 <<  2))
#define EXTI_Line3                 ((uint32_t)(1 <<  3))
#define EXTI_Line4                 ((uint32_t)(1 <<  4))
#define EXTI_Line5                 ((uint32_t)(1 <<  5))
#define EXTI_Line6                 ((uint32_t)(1 <<  6))
#define EXTI_Line7                 ((uint32_t)(1 <<  7))
#define EXTI_Line8                 ((uint32_t)(1 <<  8))
#define EXTI_Line9                 ((uint32_t)(1 <<  9))
#define EXTI_Line10                ((uint32_t)(1 << 10))
#define EXTI_Line11                ((uint32_t)(1 << 11))
#define EXTI_Line12                ((uint32_t)(1 << 12))
#define EXTI_Line13                ((uint32_t)(1 << 13))
#define EXTI_Line14                ((uint32_t)(1 << 14))
#define EXTI_Line15                ((uint32_t)(1 << 15))
#define EXTI_Line16                ((uint32_t)(1 << 16))
#define EXTI_Line17                ((uint32_t)(1 << 17))
#define EXTI_Line18                ((uint32_t)(1 << 18))
#define EXTI_Line19                ((uint32_t)(1 << 19))
#define EXTI_Line20                ((uint32_t)(1 << 20))
#define EXTI_Line21                ((uint32_t)(1 << 21))
#define EXTI_Line22                ((uint32_t)(1 << 22))
#define EXTI_Line23                ((uint32_t)(1 << 23))

// EXTI port sources
#define EXTI_PortSourceGPIOA       ((uint8_t)0x00)
#define EXTI_PortSourceGPIOB       ((uint8_t)0x01)
#define EXTI_PortSourceGPIOC       ((uint8_t)0x02)
#define EXTI_PortSourceGPIOD       ((uint8_t)0x03)
#if defined(GPIOE)
#define EXTI_PortSourceGPIOE       ((uint8_t)0x04)
#endif
#if defined(GPIOF)
#define EXTI_PortSourceGPIOF       ((uint8_t)0x06)
#endif
#if defined(GPIOG)
#define EXTI_PortSourceGPIOG       ((uint8_t)0x07)
#endif
#if defined(GPIOH)
#define EXTI_PortSourceGPIOH       ((uint8_t)0x05)
#endif

// EXTI pin sources
#define EXTI_PinSource0            ((uint8_t)0x00)
#define EXTI_PinSource1            ((uint8_t)0x01)
#define EXTI_PinSource2            ((uint8_t)0x02)
#define EXTI_PinSource3            ((uint8_t)0x03)
#define EXTI_PinSource4            ((uint8_t)0x04)
#define EXTI_PinSource5            ((uint8_t)0x05)
#define EXTI_PinSource6            ((uint8_t)0x06)
#define EXTI_PinSource7            ((uint8_t)0x07)
#define EXTI_PinSource8            ((uint8_t)0x08)
#define EXTI_PinSource9            ((uint8_t)0x09)
#define EXTI_PinSource10           ((uint8_t)0x0A)
#define EXTI_PinSource11           ((uint8_t)0x0B)
#define EXTI_PinSource12           ((uint8_t)0x0C)
#define EXTI_PinSource13           ((uint8_t)0x0D)
#define EXTI_PinSource14           ((uint8_t)0x0E)
#define EXTI_PinSource15           ((uint8_t)0x0F)


// EXTI mode enumeration
typedef enum {
	EXTI_MODE_NONE = 0x00, // EXTI line disabled
	EXTI_MODE_IRQ  = 0x01, // EXTI line generates IRQ
	EXTI_MODE_EVT  = 0x02, // EXTI line generates Event
	EXTI_MODE_BOTH = 0x03  // EXTI line generates both IRQ and Event
} EXTIMode_TypeDef;

// EXTI trigger enumeration
typedef enum {
	EXTI_TRG_NONE    = 0x00, // Trigger disabled
	EXTI_TRG_RISING  = 0x01, // Trigger on rising edge
	EXTI_TRG_FALLING = 0x02, // Trigger on falling edge
	EXTI_TRG_BOTH    = 0x03  // Trigger on both falling and rising edges
} EXTITRG_TypeDef;


// GPIO pin handle structure
typedef struct {
	uint32_t      GPIO_AHB;  // AHB bit for GPIO port
	GPIO_TypeDef *GPIO;      // Pointer to the pin GPIO port
	uint16_t      GPIO_PIN;  // GPIO pin
	uint8_t       GPIO_SRC;  // GPIO pin source
} GPIO_HandleTypeDef;


// Function prototypes
void GPIO_set_mode(GPIO_TypeDef *GPIOx, GPIOMode_TypeDef Mode, GPIOPUPD_TypeDef PUPD, uint16_t Pins);
void GPIO_out_cfg(GPIO_TypeDef *GPIOx, GPIOOT_TypeDef OT, GPIOSPD_TypeDef Speed, uint16_t Pins);
void GPIO_af_cfg(GPIO_TypeDef *GPIOx, uint16_t Pin, uint8_t AF);

void GPIO_EXTI_cfg(uint32_t EXTI_Line, EXTIMode_TypeDef mode, EXTITRG_TypeDef trigger);
void GPIO_EXTI_src(uint8_t portsrc, uint8_t pinsrc);

#endif // __GPIO_H
