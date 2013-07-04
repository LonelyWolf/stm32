#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <delay.h>
#include <ssd1289.h>

int main(void) {
	GPIO_InitTypeDef PORT;

	// Init DATA port
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	PORT.GPIO_Pin = GPIO_Pin_All;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&PORT);

	// Init LCD control pins
	// PB10 - LCD_Reset
	// PB11 - LCD_RS
	// PB12 - LCD_WR
	// PB13 - LCD_RD
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	PORT.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&PORT);

	// Init LED pins
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	PORT.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&PORT);

	LCD_Reset();

	while(1);
}
