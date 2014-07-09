// Define to prevent recursive inclusion -------------------------------------
#ifndef __BEEPER_H
#define __BEEPER_H


// Buzzer on PB5 -> TIM3_CH2
#define BEEPER_PIN               GPIO_Pin_5
#define BEEPER_GPIO              GPIOB
#define BEEPER_PERIPH            RCC_AHBPeriph_GPIOB
#define BEEPER_GPIO_AF           GPIO_AF_TIM3
#define BEEPER_GPIO_PIN_SRC      GPIO_PinSource5

#define BEEPER_TIM               TIM3
#define BEEPER_TIM_PERIPH        RCC_APB1Periph_TIM3
#define BEEPER_TIM_IRQN          TIM3_IRQn


// Function prototypes
void BEEPER_Init(void);
void BEEPER_Enable(uint16_t freq, uint32_t duration);
void BEEPER_Disable(void);

#endif // __BEEPER_H
