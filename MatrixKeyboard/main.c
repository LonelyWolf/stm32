#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_rtc.h>
#include <lcd1602.h>
#include <delay.h>
#include <rtc.h>

// -------------------------------------------------- //

// Rotary Encoder direction
typedef enum { FORWARD, BACKWARD } Direction;

// Rotary encoder variables
//volatile uint32_t rotary_capture_is_first = 1
Direction rotary_dir   = FORWARD; // Rotation direction
uint32_t  rotary_cntr  = 0;       // Rotation counter
uint32_t  rotary_ready = 1;       // Is rotation counter updated?

// Keyboard variables
uint32_t keys_debounce = 0; // Debounce counter for keyboard
uint32_t keys_last     = 0; // Previous keys state
uint32_t keys_ready    = 1; // Is keyboard readings updated?
uint16_t keys_pressed  = 0; // Pressed keys
uint32_t keys_lkp      = 0; // For readings update detection

// RTC variables
RTC_Time now; // Last time readings
uint8_t rtc_ready = 1; // Is RTC readings updated?

// -------------------------------------------------- //

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

// TIMER #4 IRQ
void TIM4_IRQHandler(void) {
	__disable_irq();
	if (TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);

		uint16_t keys = 0;

		// COL#1
	    GPIO_WriteBit(GPIOA,GPIO_Pin_0,Bit_RESET);
	    GPIO_WriteBit(GPIOA,GPIO_Pin_1,Bit_SET);
	    keys = (GPIO_ReadInputData(GPIOA) << 1) & 0b111000;
	    // COL#2
	    GPIO_WriteBit(GPIOA,GPIO_Pin_0,Bit_SET);
	    GPIO_WriteBit(GPIOA,GPIO_Pin_1,Bit_RESET);
	    keys |= (GPIO_ReadInputData(GPIOA) >> 2) & 0b000111;

	    if (keys != keys_last) {
	    	keys_debounce = 0;
	    	keys_ready = 0;
	    } else {
	    	keys_debounce++;
	    	if (keys_debounce >= 2) {
	    		keys_debounce = 0;
	    		keys_ready = 1;
	    		keys_pressed = keys;
	    	} else keys_ready = 0;
	    }
	    keys_last = keys;
	}
	__enable_irq();
}

// TIMER #16 IRQ
void TIM1_UP_TIM16_IRQHandler(void) {
	__disable_irq();
	if (TIM_GetITStatus(TIM16,TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM16,TIM_IT_Update);

//		GPIOC->ODR ^= GPIO_Pin_8; // Toggle LED

		// Redraw screen here
		if (rotary_ready) {
			LCD_GotoXY(0,0);
			LCD_PrintH(rotary_cntr);
			LCD_data_4bit(' ');
			(rotary_dir == BACKWARD) ? LCD_data_4bit('<') : LCD_data_4bit('>');
			LCD_Print("             ");
			rotary_ready = 0;
		}
		if (keys_ready && keys_lkp != keys_pressed) {
			LCD_GotoXY(0,1);
			LCD_PrintB8(keys_pressed);
			keys_ready = 0;
			keys_lkp = keys_pressed;
			GPIO_WriteBit(GPIOC,GPIO_Pin_8,(keys_pressed == 0b111111) ? Bit_RESET : Bit_SET);
		}
		if (rtc_ready) {
			LCD_GotoXY(8,1);
			LCD_LZ(now.hour);
			LCD_data_4bit(':');
			LCD_LZ(now.min);
			LCD_data_4bit(':');
			LCD_LZ(now.sec);
			rtc_ready = 0;
		}
	}
	__enable_irq();
}

// RTC Interrupt handler
void RTC_IRQHandler(void) {
	__disable_irq();
	if (RTC_GetITStatus(RTC_IT_SEC) != RESET) {
		RTC_ClearITPendingBit(RTC_IT_SEC);
		RTC_WaitForLastTask();

		GPIOC->ODR ^= GPIO_Pin_9; // Toggle LED

		RTCToTime(RTC_GetCounter(),&now); // Convert epoch to time
		rtc_ready = 1;
	}
	__enable_irq();
}

// -------------------------------------------------- //

volatile int main(void)
{
	/*
	 *    Initialize peripherials
	 */
	GPIO_InitTypeDef PORT;

	// Enable peripheral clocks for PortA
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	// Set Button pin for input (PA0)
	PORT.GPIO_Pin = (GPIO_Pin_0);
	PORT.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	PORT.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA,&PORT);
	// Set Encoder pins as input
	// ENC_A: PA6
	// ENC_B: PA7
	PORT.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	PORT.GPIO_Mode = GPIO_Mode_IPU; // Input with Pull-Up
	PORT.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA,&PORT);
	// Set Keyboard ROW# pins as input
	PORT.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	PORT.GPIO_Mode = GPIO_Mode_Out_OD; // Output (with Push-Pull?)
	PORT.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA,&PORT);
	// Set Keyboard COL# pins as output
	PORT.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
	PORT.GPIO_Mode = GPIO_Mode_IPU; // Input with Pull-Up
	PORT.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA,&PORT);

	// Enable peripheral clocks for PortB
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	// Set LCD pins as output
	// LCD: PC15..PC10
	PORT.GPIO_Pin = pin_E | pin_RS | pin_DB4 | pin_DB5 | pin_DB6 | pin_DB7;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB,&PORT);

	// Enable peripheral clocks for PortC
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	// Set LED pins as output
	// Green LED: PC9
	// Blue LED:  PC8
	PORT.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC,&PORT);

	// Init LCD
	LCD_Init();
	LCD_Cls();

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

	// Enable clock for TIM4
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	// Time base configuration
	TIM_TimeBaseStructInit(&TIM);
	TIM.TIM_Prescaler = (SystemCoreClock/DELAY_TIM_FREQUENCY_MS)-1;
	TIM.TIM_Period = 10; // 10ms interval
	TIM.TIM_ClockDivision = 0;
	TIM.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4,&TIM);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);
	NVIC_EnableIRQ(TIM4_IRQn); // Enable TIM4 IRQ
	TIM_Cmd(TIM4,ENABLE); // Enable counter on TIM4

	// Enable clock for TIM16
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16,ENABLE);
	// Time base configuration
	TIM_TimeBaseStructInit(&TIM);
	TIM.TIM_Prescaler = (SystemCoreClock/DELAY_TIM_FREQUENCY_MS)-1;
	TIM.TIM_Period = 50; // 50ms interval => ~20FPS
	TIM.TIM_ClockDivision = 0;
	TIM.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM16,&TIM);
	TIM_ITConfig(TIM16,TIM_IT_Update,ENABLE);
	NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn); // Enable TIM16 IRQ
	TIM_Cmd(TIM16,ENABLE); // Enable TIMER #16

	// Init RTC
	RTC_Init();
	NVIC_SetPriority(RTC_IRQn,(1<<__NVIC_PRIO_BITS) - 1);
	NVIC_EnableIRQ(RTC_IRQn);

	// Main program loop
	while(1) {
		//__WFI(); // Go to sleep mode (Wait for interrupt)
	}

	return 0; // D'oh?
}
