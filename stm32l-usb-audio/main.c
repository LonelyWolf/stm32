///////////////////
// STM32L151RBT6 //
///////////////////


// SPL
#include <stm32l1xx_rcc.h>
#include <stm32l1xx_gpio.h>
#include <stm32l1xx_syscfg.h>
//#include <stm32l1xx_tim.h>
#include <misc.h>

// USB related stuff
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"

// Wolk libs
#include <delay.h>
#include <wolk.h>


GPIO_InitTypeDef PORT;
NVIC_InitTypeDef NVICInit;

// ---> usb_endp.c
extern uint8_t stream_buffer[100];
extern uint16_t In_Data_Offset;
extern uint16_t Out_Data_Offset;
// <--- usb_endp.c

// ---> usb_prop.c
extern uint32_t MUTE_DATA;
// <--- usb_prop.c



/*

//       Sound with DAC


void TIM9_IRQHandler(void) {
	if (TIM9->SR & TIM_SR_UIF) {
		TIM9->SR &= ~TIM_SR_UIF; // Clear the TIMx interrupt pending bit

		if ((Out_Data_Offset < In_Data_Offset) && !MUTE_DATA) {
			// Set DAC channel2 DHR register
			DAC->DHR8R2 = stream_buffer[Out_Data_Offset];
			Out_Data_Offset++;
		}
	}
}


int main(void) {
	Delay_Init(NULL);

	RCC->AHBENR |= RCC_AHBPeriph_GPIOA; // Enable the PORTA peripheral

	// Configure PA5 as DAC_OUT2
	GPIOA->MODER   |=  GPIO_MODER_MODER5; // Analog mode for pin 5
	GPIOA->OSPEEDR |=  GPIO_OSPEEDER_OSPEEDR5; // High speed
	GPIOA->PUPDR   &= ~GPIO_PUPDR_PUPDR5; // No pull-up, pull-down
	GPIOA->OTYPER  &= ~GPIO_OTYPER_OT_5; // Output push-pull (reset state)

	// Configure TIM9 (trigger for DAC)
	RCC->APB2ENR |= RCC_APB2Periph_TIM9; // Enable the TIMx peripheral
	TIM9->ARR   =  SystemCoreClock/44100; // Audio sample rate 44.1kHz
	TIM9->CR2  &= ~TIM_CR2_MMS; // Master mode selection reset
	TIM9->CR2  |=  TIM_CR2_MMS_1; // The update event is selected as trigger output (TRGO)
	TIM9->DIER |=  TIM_DIER_UIE; // TIMx update interrupt enable
	TIM9->EGR   =  TIM_EGR_UG; // Generate an update event to reload the prescaler value immediately

	// Configure DAC channel2
	RCC->APB1ENR |= RCC_APB1ENR_DACEN; // Enable the DAC peripheral
	DAC->CR |= DAC_CR_TEN2; // DAC channel2 trigger enable
//	DAC->CR |= DAC_CR_TSEL2; // DAC channel2 software trigger
	DAC->CR |= DAC_CR_TSEL2_1 | DAC_CR_TSEL2_0; // DAC channel2 TIM9 TRGO event
	DAC->CR |= DAC_CR_BOFF2; // DAC channel2 output buffer disabled

	// Enable the TIM9 Interrupt
	NVICInit.NVIC_IRQChannel = TIM9_IRQn;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 2;
	NVICInit.NVIC_IRQChannelSubPriority = 0;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVICInit);

	// Enable DAC and timer
	DAC->CR |=  DAC_CR_EN2; // DAC channel2 enable
	TIM9->CR1 |= TIM_CR1_CEN; // Enable TIM9


	// Configure USB peripheral
	USB_HWConfig();

	// Initialize USB device
	USB_Init();

    while(1);
}

*/



/*

//       Sound with two timers and SPL

void TIM2_IRQHandler(void) {
	if (TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET) {
		// Clear TIM2 update interrupt
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);

	    if ((Out_Data_Offset < In_Data_Offset) && ((uint8_t)(MUTE_DATA) == 0)) {
	    	TIM_SetCompare3(TIM3,stream_buffer[Out_Data_Offset]);
	    	Out_Data_Offset++;
	    }
	}
}

int main(void) {
	Delay_Init(NULL);

	// Enable PORTB peripheral
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);

	// Beeper pin (PB0 -> TIM3_CH3)
	PORT.GPIO_Mode  = GPIO_Mode_AF;
	PORT.GPIO_Speed = GPIO_Speed_40MHz;
	PORT.GPIO_OType = GPIO_OType_PP;
	PORT.GPIO_PuPd  = GPIO_PuPd_UP;
	PORT.GPIO_Pin   = GPIO_Pin_0;
	GPIO_Init(GPIOB,&PORT);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource0,GPIO_AF_TIM3); // Alternative function of PB0 -> TIM3_CH3

	// Enable the TIM2 and TIM3 peripherals
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3,ENABLE);

	// TIM2 IRQ
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVICInit.NVIC_IRQChannel = TIM2_IRQn;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 0x02;
	NVICInit.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Init(&NVICInit);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	// TIM3 configuration
	TIM_TimeBaseStructure.TIM_Prescaler = 0x00; // TIM3CLK = 32 MHz
	TIM_TimeBaseStructure.TIM_Period = 0xFF;    // PWM frequency : 125.000 Hz
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseStructure);
	// TIM3's Channel3 in PWM1 mode
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_Pulse = 0x7F;  // Duty cycle: 50%
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  // set high polarity
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC3Init(TIM3,&TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3,TIM_OCPreload_Enable);

	// TIM2 configuration
	TIM_TimeBaseStructure.TIM_Period = SystemCoreClock / 44100;
	TIM_TimeBaseStructure.TIM_Prescaler = 0x00;    // TIM2CLK = 32 MHz
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	// Output Compare Inactive Mode configuration: Channel1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_Pulse = 0x0;
	TIM_OC1Init(TIM2,&TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM2,TIM_OCPreload_Disable);

	// Start TIM3
	TIM_Cmd(TIM3,ENABLE);

	// Start TIM2
	TIM_Cmd(TIM2,ENABLE);

	// Enable TIM2 update interrupt
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);

	// Configure USB peripheral
	USB_HWConfig();

	// Initialize USB device
	USB_Init();

    while(1);
}

*/



///*

//               Sound with two timers, no SPL


void TIM2_IRQHandler(void) {
	if (TIM2->SR & TIM_SR_UIF) {
		TIM2->SR &= ~TIM_SR_UIF; // Clear the TIM2 interrupt pending bit

	    if ((Out_Data_Offset < In_Data_Offset) && !MUTE_DATA) {
	    	TIM3->CCR1 = stream_buffer[Out_Data_Offset];
	    	Out_Data_Offset++;
	    }

	}
}


int main(void) {
	Delay_Init(NULL);

	// Enable PORTB peripheral
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA,ENABLE);

	// Beeper pin (PA6 -> TIM3_CH1)
	PORT.GPIO_Mode  = GPIO_Mode_AF;
	PORT.GPIO_Speed = GPIO_Speed_40MHz;
	PORT.GPIO_OType = GPIO_OType_PP;
	PORT.GPIO_PuPd  = GPIO_PuPd_UP;
	PORT.GPIO_Pin   = GPIO_Pin_6;
	GPIO_Init(GPIOA,&PORT);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_TIM3); // Alternative function of PA6 -> TIM3_CH1

	// Configure TIM3 (PWM output on CH1)
	RCC->APB1ENR |= RCC_APB1Periph_TIM3; // Enable the TIM3 peripheral
	TIM3->CR1   |= TIM_CR1_ARPE; // Auto-preload enable
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE; // Output compare 1 preload enable
	TIM3->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; // PWM mode 1
	TIM3->PSC    = 0; // TIM3CLK = 32MHz
	TIM3->ARR    = 0xFF; // PWM frequency = 125kHz
	TIM3->CCR1   = 0x7F; // 50% duty cycle
	TIM3->CCER  |= TIM_CCER_CC1NP; // Output polarity
	TIM3->CCER  |= TIM_CCER_CC1E; // BEEPER TIM3_CH1 output compare enable
	TIM3->EGR    = TIM_EGR_UG; // Generate an update event to reload the prescaler value immediately

	// Configure TIM2 (OCMode_Timing)
	RCC->APB1ENR |= RCC_APB1Periph_TIM2; // Enable the TIM2 peripheral
	TIM2->CR1   |= TIM_CR1_ARPE; // Auto-preload enable
	TIM2->PSC    = 0; // TIM2CLK = 32MHz
	TIM2->ARR    = SystemCoreClock / 44100; // 44.100 kHz
	TIM2->CCER  |= TIM_CCER_CC1P;
	TIM2->CCR1   = 0;
	TIM2->EGR    = TIM_EGR_UG; // Generate an update event to reload the prescaler value immediately
	TIM2->DIER  |= TIM_DIER_UIE; // TIMx update interrupt enable

	// TIM2 IRQ
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVICInit.NVIC_IRQChannel = TIM2_IRQn;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 0x02;
	NVICInit.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Init(&NVICInit);

	TIM3->CR1 |= TIM_CR1_CEN; // Enable TIM3
	TIM2->CR1 |= TIM_CR1_CEN; // Enable TIM2


	// Configure USB peripheral
	USB_HWConfig();

	// Initialize USB device
	USB_Init();

    while(1);
}

//*/



/*

//                  Sound with a single timer


void TIM3_IRQHandler(void) {
	if (TIM3->SR & TIM_SR_UIF) {
		TIM3->SR &= ~TIM_SR_UIF; // Clear the TIM3 interrupt pending bit

	    if ((Out_Data_Offset < In_Data_Offset) && !MUTE_DATA) {
	    	TIM3->CCR3 = stream_buffer[Out_Data_Offset];
	    	Out_Data_Offset++;
	    } else {
	    	TIM3->CCR3 = TIM3->ARR;
	    }

	}
}


int main(void) {
	Delay_Init(NULL);
//	BEEPER_Init();

	// Enable PORTB peripheral
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);

	// Beeper pin (PB5)
	PORT.GPIO_Mode  = GPIO_Mode_AF;
	PORT.GPIO_Speed = GPIO_Speed_40MHz;
	PORT.GPIO_OType = GPIO_OType_PP;
	PORT.GPIO_PuPd  = GPIO_PuPd_UP;
	PORT.GPIO_Pin   = GPIO_Pin_0;
	GPIO_Init(GPIOB,&PORT);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource0,GPIO_AF_TIM3); // Alternative function of PB0 -> TIM3_CH2

	// Configure timer TIM3
	RCC->APB1ENR |= RCC_APB1Periph_TIM3; // Enable TIMx peripheral
	TIM3->CR1   |= TIM_CR1_ARPE; // Auto-preload enable
	TIM3->CCMR2 |= TIM_CCMR2_OC3PE; // Output compare 3 preload enable
	TIM3->CCMR2 |= TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1; // PWM mode 1
	TIM3->PSC    = SystemCoreClock / (44100 * 256); // 32MHz / (44100 * 256) => 44.1kHz
	TIM3->ARR    = 256; // auto reload value
	TIM3->CCR3   = 127; // 50% duty cycle
	TIM3->CCER  |= TIM_CCER_CC3P; // Output polarity
	TIM3->CCER  |= TIM_CCER_CC3E; // BEEPER TIMx_CH3 output compare enable
	TIM3->EGR    = TIM_EGR_UG; // Generate an update event to reload the prescaler value immediately
	TIM3->DIER  |= TIM_DIER_UIE; // TIMx update interrupt enable

	// TIM3 IRQ
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVICInit.NVIC_IRQChannel = TIM3_IRQn;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 0x03; // lowest priority
	NVICInit.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Init(&NVICInit);

	// Play from the beginning
	TIM3->CR1 |= TIM_CR1_CEN; // Enable TIM3


	// Configure USB peripheral
	USB_HWConfig();

	// Initialize USB device
	USB_Init();

    while(1);
}
*/
