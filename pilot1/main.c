#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_tim.h>
#include <lcd1602.h>
#include <delay.h>

typedef enum { FORWARD, BACKWARD } Direction;

volatile uint32_t capture_is_first = 1, capture_is_ready = 0;
volatile Direction captured_direction = FORWARD;
volatile uint32_t capture_cntr = 0;

// TIMER #3 IRQ
void TIM3_IRQHandler(void) {
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
		if (!capture_is_first) capture_is_ready = 1;
		capture_is_first = 0;
		/* ¬ бите TIM_CR1_DIR регистра TIM3_CR1 хранитс€ направление вращени€ энкодера, запоминаем его. */
		captured_direction = (TIM3->CR1 & TIM_CR1_DIR ? FORWARD : BACKWARD);
		capture_cntr++;
	}
}

volatile int main(void)
{
	// Let's initialize peripherals
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
	PORT.GPIO_Mode = GPIO_Mode_IPU; // Input with Pull-Up
	PORT.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA,&PORT);
	// Set Keyboard COL# pins as output
	PORT.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP; // Output (with Push-Pull?)
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

	// Enable clock for TIM3
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	// Time base configuration
	TIM_TimeBaseInitTypeDef TIM;
	TIM_TimeBaseStructInit(&TIM);
	TIM.TIM_Period = 4;
	TIM.TIM_CounterMode = TIM_CounterMode_Up | TIM_CounterMode_Down;
	TIM_TimeBaseInit(TIM3,&TIM);
	TIM_EncoderInterfaceConfig(TIM3,TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);
	// Enable counter for TIM3
	TIM_Cmd(TIM3,ENABLE);
	// Enable TIM3 IRQ
	NVIC_EnableIRQ(TIM3_IRQn);

	// Init LCD
	LCD_Init();
	LCD_Cls();

	// Fancy greetings
	LCD_GotoXY(16,0);
	LCD_Print("- Hello STM32! -");
	LCD_GotoXY(16,1);
	LCD_Print("CPU: ");
	LCD_PrintI(SystemCoreClock/1000000);
	LCD_Print("MHz");
	int i;
	for (i=16; i > 0; i--) {
		LCD_cmd_4bit(0b00011000); // shift screen contents to the left
		Delay_ms(62);
	}
	Delay_ms(1500);
	for (i=16; i > 0; i--) {
		LCD_cmd_4bit(0b00011000); // shift screen contents to the left
		Delay_ms(62);
	}
	LCD_Cls();

    // Encoder counter
	uint32_t cntr = 0;

	// Main program loop
	while(1) {
		LCD_GotoXY(0,0);

		// Lit LED
	    GPIO_WriteBit(GPIOC,GPIO_Pin_9,Bit_SET);
	    GPIO_WriteBit(GPIOC,GPIO_Pin_8,Bit_RESET);
	    Delay_ms(500);

	    if (capture_is_ready) {
			NVIC_DisableIRQ(TIM3_IRQn);
			capture_is_ready = 0;
			const Direction direction = captured_direction;
			NVIC_EnableIRQ(TIM3_IRQn);

			/* ... direction ... */
			if (direction == BACKWARD) cntr -= capture_cntr; else cntr += capture_cntr;
			if (direction == FORWARD) {
				LCD_Print(">> ");
			} else {
				LCD_Print("<< ");
			}
			capture_cntr = 0;
	    }

		LCD_PrintI(cntr);
		LCD_Print("                ");

		// Dim LED
		GPIO_WriteBit(GPIOC,GPIO_Pin_9,Bit_RESET);
		GPIO_WriteBit(GPIOC,GPIO_Pin_8,Bit_SET);
	    Delay_ms(500);

	}

    // D'oh?!
	return 0;
}
