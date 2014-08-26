#include <stm32l1xx_rcc.h>
#include <stm32l1xx_gpio.h>
#include <misc.h>

#include <wolk.h> // bool typedef
#include <beeper.h>


volatile       uint32_t          _beep_duration;
volatile       bool              _tones_playing;
volatile const Tone_TypeDef     *_tones;

GPIO_InitTypeDef PORT;


// BEEPER TIMx IRQ handler
void TIM3_IRQHandler(void) {
	if (BEEPER_TIM->SR & TIM_SR_UIF) {
		BEEPER_TIM->SR &= ~TIM_SR_UIF; // Clear the TIMx's interrupt pending bit

		_beep_duration--;
		if (_beep_duration == 0) {
			if (_tones_playing) {
				// Currently playing tones, take next tone
				_tones++;
				if (_tones->frequency == 0 && _tones->duration == 0) {
					// Last tone in sequence
					BEEPER_Disable();
					_tones_playing = FALSE;
					_tones = NULL;
				} else {
					if (_tones->frequency == 0) {
						// Silence period
						BEEPER_TIM->ARR = SystemCoreClock / (100 * BEEPER_TIM->PSC) - 1;
						BEEPER_TIM->CCR2 = 0; // 0% duty cycle
						_beep_duration = _tones->duration + 1;
					} else {
						// Play next tone in sequence
						BEEPER_Enable(_tones->frequency,_tones->duration);
					}
				}
			} else {
				BEEPER_Disable();
			}
		}
	}
}

// Initialize buzzer output
void BEEPER_Init(void) {
	NVIC_InitTypeDef NVICInit;

	// Enable PORTB peripheral
	RCC_AHBPeriphClockCmd(BEEPER_PERIPH,ENABLE);

	// Beeper pin (PB5)
	PORT.GPIO_Mode  = GPIO_Mode_AF;
	PORT.GPIO_Speed = GPIO_Speed_40MHz;
	PORT.GPIO_OType = GPIO_OType_PP;
	PORT.GPIO_PuPd  = GPIO_PuPd_UP;
	PORT.GPIO_Pin   = BEEPER_PIN;
	GPIO_Init(BEEPER_GPIO,&PORT);
	GPIO_PinAFConfig(BEEPER_GPIO,BEEPER_GPIO_PIN_SRC,BEEPER_GPIO_AF); // Alternative function of PA7 -> TIM3_CH2

	// Configure timer BEEPER_TIM
	RCC->APB1ENR |= BEEPER_TIM_PERIPH; // Enable TIMx peripheral
	BEEPER_TIM->CR1   |= TIM_CR1_ARPE; // Auto-preload enable
	BEEPER_TIM->CCMR1 |= TIM_CCMR1_OC2PE; // Output compare 2 preload enable
	BEEPER_TIM->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1; // PWM mode 1
//	BEEPER_TIM->PSC    = 8; // prescaler
	BEEPER_TIM->PSC    = SystemCoreClock / 4000000;
	BEEPER_TIM->ARR    = 999; // auto reload value
	BEEPER_TIM->CCR2   = 499; // 50% duty cycle
	BEEPER_TIM->CCER  |= TIM_CCER_CC2P; // Output polarity
	BEEPER_TIM->CCER  |= TIM_CCER_CC2E; // BEEPER_TIM_CH2 output compare enable
	BEEPER_TIM->EGR    = TIM_EGR_UG; // Generate an update event to reload the prescaler value immediately
	BEEPER_TIM->DIER  |= TIM_DIER_UIE; // ÒIMx update interrupt enable

	// BEEPER_TIM IRQ
	NVICInit.NVIC_IRQChannel = BEEPER_TIM_IRQN;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 0x0b;
	NVIC_Init(&NVICInit);
}

// Turn on buzzer with specified frequency
// input:
//   freq - PWM frequency for buzzer (Hz)
//   duration - duration of buzzer work (tens ms: 1 -> 10ms sound duration)
void BEEPER_Enable(uint16_t freq, uint32_t duration) {
	if (freq < 100 || freq > 8000 || duration == 0) {
		BEEPER_Disable();
	} else {
		_beep_duration = (freq / 100) * duration + 1;

		// Configure buzzer pin
		PORT.GPIO_Mode  = GPIO_Mode_AF;
		PORT.GPIO_PuPd  = GPIO_PuPd_UP;
		GPIO_Init(BEEPER_GPIO,&PORT);

		// Configure and enable PWM timer
		RCC->APB1ENR |= BEEPER_TIM_PERIPH; // Enable TIMx peripheral
		BEEPER_TIM->ARR = SystemCoreClock / (freq * BEEPER_TIM->PSC) - 1;
		BEEPER_TIM->CCR2 = BEEPER_TIM->ARR >> 1; // 50% duty cycle
		BEEPER_TIM->CR1 |= TIM_CR1_CEN; // Counter enable
	}
}

// Turn off buzzer
void BEEPER_Disable(void) {
	// Counter disable
	BEEPER_TIM->CR1 &= ~TIM_CR1_CEN;
	// Disable TIMx peripheral to conserve power
	RCC->APB1ENR &= ~BEEPER_TIM_PERIPH;
	// Configure buzzer pin as analog input without pullup to conserve power
	PORT.GPIO_Mode = GPIO_Mode_AN;
	PORT.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(BEEPER_GPIO,&PORT);
}

// Start playing tones sequence
// input:
//   tones - pointer to tones array
void BEEPER_PlayTones(const Tone_TypeDef * tones) {
	_tones = tones;
	_tones_playing = TRUE;
	BEEPER_Enable(_tones->frequency,_tones->duration);
}
