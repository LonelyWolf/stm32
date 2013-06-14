#include <string.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <delay.h>
#include <ili9320.h>


//#include <facepalm.h>
//#include <face565.h>


int main(void)
{
	// Init LED pins
	GPIO_InitTypeDef PORT;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	PORT.GPIO_Pin = (GPIO_Pin_9 | GPIO_Pin_8);
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	// PORT.GPIO_Speed = GPIO_Speed_50MHz; // <=== Already set in PORT variable
	GPIO_Init(GPIOC,&PORT);

	LCD_Init();

	LCD_Clear(RGB888to565(0,0,0)); // BLACK

	LCD_Rect(0,0,320,240,RGB888to565(255,255,255));

	LCD_Text(5,5,"Hello world!",RGB888to565(255,0,255),RGB888to565(0,100,0));
	LCD_Text(6,32,"Look at this extremally slow screen :(",RGB888to565(0,255,255),RGB888to565(32,64,32));
	LCD_Text(40,50,"serial interface totally SUXX",RGB888to565(255,255,0),RGB888to565(0,0,0));
	LCD_Text(50,70,"... and 65K colors only ...",RGB888to565(255,128,0),RGB888to565(0,0,0));

/*
	LCD_BMP_Mono(118,118,25,120,&FacePalm[0],RGB888to565(0,162,232));
	LCD_Text(120,134,"1-bit color >",RGB888to565(255,128,0),RGB888to565(0,0,0));

	LCD_Rect(19,99,82,98,RGB888to565(0,0,255));
	LCD_BMP(20,100,80,96,&Face565[0]);
	LCD_Text(106,112,"< 16-bit color",RGB888to565(255,255,0),RGB888to565(0,0,0));
 */

	LCD_Text(5,220,"(c) Dimon",RGB888to565(255,55,55),RGB888to565(32,64,16));

	while(1) {
    }
}
