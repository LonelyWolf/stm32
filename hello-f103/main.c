#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_rtc.h>
#include <lcd1602.h>
#include <delay.h>
#include <rtc.h>


// Rotary encoder variables
typedef enum { FORWARD, BACKWARD } Direction;
Direction rotary_dir   = FORWARD; // Rotation direction
uint32_t  rotary_cntr  = 0;       // Rotation counter
uint32_t  rotary_ready = 1;       // Is rotation counter updated?

// RTC variables
RTC_Time now; // Last time readings
__IO uint8_t rtc_ready = 1; // Is RTC readings updated?


// Print time with leading zero
void LCD_LZ(uint8_t time) {
	LCD_data_4bit('0' + time / 10);
	LCD_data_4bit('0' + time % 10);
}

// TIMER #3 IRQ
void TIM3_IRQHandler(void) {
	__disable_irq();
	if (TIM_GetITStatus(TIM3,TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
		rotary_dir = (TIM3->CR1 & TIM_CR1_DIR ? FORWARD : BACKWARD);
		(rotary_dir == BACKWARD) ? rotary_cntr-- : rotary_cntr++;
		rotary_ready = 1;
	}
	__enable_irq();
}

// RTC Interrupt handler
void RTC_IRQHandler(void) {
	__disable_irq();
	if (RTC_GetITStatus(RTC_IT_SEC) == SET) {
		RTC_ClearITPendingBit(RTC_IT_SEC);
		RTC_WaitForLastTask();
		GPIOC->ODR ^= GPIO_Pin_8; // Toggle LED
		RTCToTime(RTC_GetCounter(),&now); // Convert epoch to time
		rtc_ready = 1;
	}
	__enable_irq();
}


int main(void) {
	// Initialize peripherials
	GPIO_InitTypeDef PORT;

	// Enable peripheral clocks for PortA
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	// Set Keyboard ROW# pins as input
	PORT.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	PORT.GPIO_Mode = GPIO_Mode_Out_OD; // Output (with Push-Pull?)
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&PORT);
	// Set Keyboard COL# pins as output
	PORT.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
	PORT.GPIO_Mode = GPIO_Mode_IPU; // Input with Pull-Up
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&PORT);
	// Set Encoder pins as input (ENC_A: PA6, ENC_B: PA7)
	PORT.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	PORT.GPIO_Mode = GPIO_Mode_IPU; // Input with Pull-Up
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&PORT);

	// Enable peripheral clocks for PortB
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	// Set LCD pins as output
	// LCD: PC15..PC10
	PORT.GPIO_Pin = pin_E | pin_RS | pin_DB4 | pin_DB5 | pin_DB6 | pin_DB7;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&PORT);

	// Enable peripheral clocks for PortC
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);

	// Set LED pins as output (Green: PC9, Blue:  PC8)
	PORT.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&PORT);

	// Init RTC
	RTC_Init();
	NVIC_SetPriority(RTC_IRQn,(1<<__NVIC_PRIO_BITS) - 1);
	NVIC_EnableIRQ(RTC_IRQn);

	/*
	 *   Initialize timers
	 */
	TIM_TimeBaseInitTypeDef TIM;

	// Enable clock for TIM3
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	// Time base configuration
	TIM_TimeBaseStructInit(&TIM);
	TIM.TIM_Period = 4;
	TIM.TIM_CounterMode = TIM_CounterMode_Up | TIM_CounterMode_Down;
	TIM_TimeBaseInit(TIM3,&TIM);
	TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	// Enable counter for TIM3
	NVIC_EnableIRQ(TIM3_IRQn); // Enable TIM3 IRQ
	TIM_Cmd(TIM3,ENABLE); // Enable counter on TIM3

	// Init LCD
	LCD_Init();
	LCD_Cls();

	while(1) {
		if (rtc_ready) {
			LCD_GotoXY(8,1);
			LCD_LZ(now.hour);
			LCD_data_4bit(':');
			LCD_LZ(now.min);
			LCD_data_4bit(':');
			LCD_LZ(now.sec);
			rtc_ready = 0;
		}
		if (rotary_ready) {
			LCD_GotoXY(0,0);
			LCD_PrintH(rotary_cntr);
			LCD_data_4bit(' ');
			(rotary_dir == BACKWARD) ? LCD_data_4bit('<') : LCD_data_4bit('>');
			LCD_Print("             ");
			rotary_ready = 0;
		}
    }
}
