// Define to prevent recursive inclusion -------------------------------------
#ifndef __BEEPER_H
#define __BEEPER_H


// Single tone definition
typedef struct {
	uint16_t frequency;
	uint8_t  duration;
} Tone_TypeDef;


static const Tone_TypeDef tones_startup[] = {
		{2000,3},
		{   0,3},
		{3000,3},
		{   0,3},
		{4000,3},
		{   0,3},
		{1200,4},
		{   0,6},
		{4500,6},
		{   0,0}     // <-- tones end
};

static const Tone_TypeDef tones_3beep[] = {
		{4000, 3},
		{   0,10},
		{1000, 6},
		{   0,10},
		{4000, 3},
		{   0, 0}
};

// "Super Mario bros." =)
static const Tone_TypeDef tones_SMB[] = {
		{2637,18}, // E7 x2
		{   0, 9}, // x3
		{2637, 9}, // E7
		{   0, 9}, // x3
		{2093, 9}, // C7
		{2637, 9}, // E7
		{   0, 9}, // x3
		{3136, 9}, // G7
		{   0,27}, // x3
		{1586, 9}, // G6
		{   0,27}, // x3

		{2093, 9}, // C7
		{   0,18}, // x2
		{1586, 9}, // G6
		{   0,18}, // x2
		{1319, 9}, // E6
		{   0,18}, // x2
		{1760, 9}, // A6
		{   0, 9}, // x1
		{1976, 9}, // B6
		{   0, 9}, // x1
		{1865, 9}, // AS6
		{1760, 9}, // A6
		{   0, 9}, // x1

		{1586,12}, // G6
		{2637,12}, // E7
		{3136,12}, // G7
		{3520, 9}, // A7
		{   0, 9}, // x1
		{2794, 9}, // F7
		{3136, 9}, // G7
		{   0, 9}, // x1
		{2637, 9}, // E7
		{   0, 9}, // x1
		{2093, 9}, // C7
		{2349, 9}, // D7
		{1976, 9}, // B6
		{   0,18}, // x2

		{2093, 9}, // C7
		{   0,18}, // x2
		{1586, 9}, // G6
		{   0,18}, // x2
		{1319, 9}, // E6
		{   0,18}, // x2
		{1760, 9}, // A6
		{   0, 9}, // x1
		{1976, 9}, // B6
		{   0, 9}, // x1
		{1865, 9}, // AS6
		{1760, 9}, // A6
		{   0, 9}, // x1

		{1586,12}, // G6
		{2637,12}, // E7
		{3136,12}, // G7
		{3520, 9}, // A7
		{   0, 9}, // x1
		{2794, 9}, // F7
		{3136, 9}, // G7
		{   0, 9}, // x1
		{2637, 9}, // E7
		{   0, 9}, // x1
		{2093, 9}, // C7
		{2349, 9}, // D7
		{1976, 9}, // B6

		{   0, 0}
};


// Buzzer on PB5 -> TIM3_CH2
#define BEEPER_PIN               GPIO_Pin_7
#define BEEPER_GPIO              GPIOA
#define BEEPER_PERIPH            RCC_AHBPeriph_GPIOA
#define BEEPER_GPIO_AF           GPIO_AF_TIM3
#define BEEPER_GPIO_PIN_SRC      GPIO_PinSource7

#define BEEPER_TIM               TIM3
#define BEEPER_TIM_PERIPH        RCC_APB1Periph_TIM3
#define BEEPER_TIM_IRQN          TIM3_IRQn


// Function prototypes
void BEEPER_Init(void);
void BEEPER_Enable(uint16_t freq, uint32_t duration);
void BEEPER_Disable(void);
void BEEPER_PlayTones(const Tone_TypeDef * melody);

#endif // __BEEPER_H
