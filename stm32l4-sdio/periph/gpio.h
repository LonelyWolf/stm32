#ifndef __GPIO_H
#define __GPIO_H


#include <stm32l4xx.h>


// GPIO pin definitions
#define GPIO_PIN_0                 GPIO_BSRR_BS0
#define GPIO_PIN_1                 GPIO_BSRR_BS1
#define GPIO_PIN_2                 GPIO_BSRR_BS2
#define GPIO_PIN_3                 GPIO_BSRR_BS3
#define GPIO_PIN_4                 GPIO_BSRR_BS4
#define GPIO_PIN_5                 GPIO_BSRR_BS5
#define GPIO_PIN_6                 GPIO_BSRR_BS6
#define GPIO_PIN_7                 GPIO_BSRR_BS7
#define GPIO_PIN_8                 GPIO_BSRR_BS8
#define GPIO_PIN_9                 GPIO_BSRR_BS9
#define GPIO_PIN_10                GPIO_BSRR_BS10
#define GPIO_PIN_11                GPIO_BSRR_BS11
#define GPIO_PIN_12                GPIO_BSRR_BS12
#define GPIO_PIN_13                GPIO_BSRR_BS13
#define GPIO_PIN_14                GPIO_BSRR_BS14
#define GPIO_PIN_15                GPIO_BSRR_BS15

// GPIO pin sources for alternate functions
#define GPIO_PinSource0            ((uint32_t)0x00000000U)
#define GPIO_PinSource1            ((uint32_t)0x00000001U)
#define GPIO_PinSource2            ((uint32_t)0x00000002U)
#define GPIO_PinSource3            ((uint32_t)0x00000003U)
#define GPIO_PinSource4            ((uint32_t)0x00000004U)
#define GPIO_PinSource5            ((uint32_t)0x00000005U)
#define GPIO_PinSource6            ((uint32_t)0x00000006U)
#define GPIO_PinSource7            ((uint32_t)0x00000007U)
#define GPIO_PinSource8            ((uint32_t)0x00000008U)
#define GPIO_PinSource9            ((uint32_t)0x00000009U)
#define GPIO_PinSource10           ((uint32_t)0x0000000AU)
#define GPIO_PinSource11           ((uint32_t)0x0000000BU)
#define GPIO_PinSource12           ((uint32_t)0x0000000CU)
#define GPIO_PinSource13           ((uint32_t)0x0000000DU)
#define GPIO_PinSource14           ((uint32_t)0x0000000EU)
#define GPIO_PinSource15           ((uint32_t)0x0000000FU)

// GPIO alternate function
#define GPIO_AF0                   ((uint32_t)0x00000000U)
#define GPIO_AF1                   ((uint32_t)0x00000001U)
#define GPIO_AF2                   ((uint32_t)0x00000002U)
#define GPIO_AF3                   ((uint32_t)0x00000003U)
#define GPIO_AF4                   ((uint32_t)0x00000004U)
#define GPIO_AF5                   ((uint32_t)0x00000005U)
#define GPIO_AF6                   ((uint32_t)0x00000006U)
#define GPIO_AF7                   ((uint32_t)0x00000007U)
#define GPIO_AF8                   ((uint32_t)0x00000008U)
#define GPIO_AF9                   ((uint32_t)0x00000009U)
#define GPIO_AF10                  ((uint32_t)0x0000000AU)
#define GPIO_AF11                  ((uint32_t)0x0000000BU)
#define GPIO_AF12                  ((uint32_t)0x0000000CU)
#define GPIO_AF13                  ((uint32_t)0x0000000DU)
#define GPIO_AF14                  ((uint32_t)0x0000000EU)
#define GPIO_AF15                  ((uint32_t)0x0000000FU)


// GPIO macros definitions
#define GPIO_MODE_SET(pin,mode)    (mode  << ((pin) << 1)) // set MODER bits for a specified pin
#define GPIO_MODE_MSK(pin)         (0x03U << ((pin) << 1)) // mask MODER bits for a specified pin
#define GPIO_PUPD_SET(pin,pupd)    (pupd  << ((pin) << 1)) // set PUPDR bits for a specified pin
#define GPIO_PUPD_MSK(pin)         (0x03U << ((pin) << 1)) // mask PUPDR bits for a specified pin
#define GPIO_SPD_SET(pin,spd)      (spd   << ((pin) << 1)) // set SPEEDR bits for a specified pin
#define GPIO_SPD_MSK(pin)          (0x03U << ((pin) << 1)) // mask SPEEDR bits for a specified pin
#define GPIO_AF_SET(pin,af)        (af    << ((pin) << 2)) // set AFR bits for a specified pin
#define GPIO_AF_MSK(pin)           (0x0FU << ((pin) << 2)) // mask AFR bits for a specified pin


// GPIO pin output speed
typedef enum {
	GPIO_SPD_LOW    = ((uint32_t)0x00000000U), // low
	GPIO_SPD_MEDIUM = GPIO_OSPEEDR_OSPEED0_0,  // medium
	GPIO_SPD_FAST   = GPIO_OSPEEDR_OSPEED0_1,  // fast
	GPIO_SPD_HIGH   = GPIO_OSPEEDR_OSPEED0     // high
} GPIOSPD_TypeDef;

// GPIO pin output type
typedef enum {
	GPIO_OT_PP = ((uint32_t)0x00000000U), // push-pull
	GPIO_OT_OD = GPIO_OTYPER_OT0          // open-drain
} GPIOOT_TypeDef;

// GPIO pin configuration mode
typedef enum {
	GPIO_Mode_IN  = ((uint32_t)0x00000000U), // input
	GPIO_Mode_OUT = GPIO_MODER_MODE0_0,      // output
	GPIO_Mode_AF  = GPIO_MODER_MODE0_1,      // alternate function
	GPIO_Mode_AN  = GPIO_MODER_MODE0         // analog
} GPIOMode_TypeDef;

// GPIO pin pull-up/pull-down configuration
typedef enum {
	GPIO_PUPD_NONE = ((uint32_t)0x00000000U), // no pull
	GPIO_PUPD_PU   = GPIO_PUPDR_PUPD0_0,      // pull-up
	GPIO_PUPD_PD   = GPIO_PUPDR_PUPD0_1       // pull-down
} GPIOPUPD_TypeDef;

// GPIO pin handle structure
typedef struct {
	uint32_t      GPIO_AHB;  // AHB bit for GPIO port
	GPIO_TypeDef *GPIO;      // Pointer to the pin GPIO port
	uint16_t      GPIO_PIN;  // GPIO pin
	uint8_t       GPIO_SRC;  // GPIO pin source
} GPIO_HandleTypeDef;


// Public macros and functions

// Port bit(s) set
// input:
//   GPIOx - pointer to a GPIO peripheral handle
//   pin - combination of GPIO_PIN_X values
__STATIC_INLINE void GPIO_PIN_SET(GPIO_TypeDef* GPIOx, uint32_t pin) {
	GPIOx->BSRR = pin;
}

// Port bit(s) reset
// input:
//   GPIOx - pointer to a GPIO peripheral handle
//   pin - combination of GPIO_PIN_X values
__STATIC_INLINE void GPIO_PIN_RESET(GPIO_TypeDef* GPIOx, uint32_t pin) {
	GPIOx->BRR = pin;
}

// Invert pin(s) output state
// input:
//   GPIOx - pointer to a GPIO peripheral handle
//   pin - combination of GPIO_PIN_X values
//#define GPIO_PIN_INVERT(PORT,pin)  ((PORT)->ODR ^= (uint32_t)pin)
__STATIC_INLINE void GPIO_PIN_INVERT(GPIO_TypeDef* GPIOx, uint32_t pin) {
	GPIOx->ODR ^= pin;
}

// Get pin(s) input state
// input:
//   GPIOx - pointer to a GPIO peripheral handle
//   pin - combination of GPIO_Pin_X values
__STATIC_INLINE uint32_t GPIO_PIN_ISTATE(GPIO_TypeDef* GPIOx, uint32_t pin) {
	return (uint32_t)((GPIOx->IDR & pin) == pin);
}

// Get pin(s) output state
// input:
//   GPIOx - pointer to a GPIO peripheral handle
//   pin - combination of GPIO_Pin_X values
__STATIC_INLINE uint32_t GPIO_PIN_OSTATE(GPIO_TypeDef* GPIOx, uint32_t pin) {
	return (uint32_t)((GPIOx->ODR & pin) == pin);
}


// Function prototypes
void GPIO_set_mode(GPIO_TypeDef *GPIOx, GPIOMode_TypeDef Mode, GPIOPUPD_TypeDef PUPD, uint16_t Pins);
void GPIO_out_cfg(GPIO_TypeDef *GPIOx, GPIOOT_TypeDef OT, GPIOSPD_TypeDef Speed, uint16_t Pins);
void GPIO_af_cfg(GPIO_TypeDef *GPIOx, uint16_t Pin, uint8_t AF);

#endif // __GPIO_H
