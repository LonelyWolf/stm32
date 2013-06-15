#include <string.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <delay.h>
#include <ili9320.h>


#include <facepalm.h>
#include <face565.h>


int main(void)
{
	// Init LED pins
	GPIO_InitTypeDef PORT;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	PORT.GPIO_Pin = (GPIO_Pin_9 | GPIO_Pin_8);
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&PORT);

	LCD_Init();

	LCD_Clear(RGB565(0,0,0)); // BLACK

	LCD_Rect(0,0,320,240,RGB565(255,255,255));

	LCD_PutStr(6,5,"Hello world!",RGB565(255,0,255));
	LCD_PutStr(8,23,"Serial interface really SUXX",RGB565(255,255,0));
	LCD_PutStr(16,42,"... and 65K colors only ...",RGB565(255,128,0));

	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
	LCD_PutStr(24,60,"CPU:",RGB565(255,255,255));
	LCD_PutInt(50,60,RCC_Clocks.SYSCLK_Frequency,RGB565(255,255,192));

	LCD_BMP_Mono(118,118,25,120,&FacePalm[0],RGB565(0,162,232));
	LCD_PutStr(120,134,"1-bit color >",RGB565(255,128,0));

	LCD_Rect(19,89,82,98,RGB565(0,0,255));
	LCD_BMP(20,90,80,96,&Face565[0]);
	LCD_PutStr(106,102,"< 16-bit color",RGB565(255,255,0));

	LCD_PutStr(5,220,"(c) Dimon",RGB565(255,55,55));

	LCD_Ellipse(285,50,24,24,RGB565(255,255,128));
	LCD_Rect(260,25,51,51,RGB565(128,255,128));

	while(1);
}
