#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <delay.h>
#include <ssd1289.h>


//#include <facepalm.h>
//#include <face565.h>


int main(void) {
	GPIO_InitTypeDef PORT;

	// Init DATA port
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	PORT.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | \
	         		GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&PORT);

	// Init LCD control pins
	// PB00 - LCD_DB13
	// PB01 - LCD_DB14
	// PB02 - LCD_DB15
	// PB10 - LCD_Reset
	// PB11 - LCD_RS
	// PB12 - LCD_WR
	// PB13 - LCD_RD
	// PB14 - LCD_CS
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	PORT.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | \
			        GPIO_Pin_13 | GPIO_Pin_14;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&PORT);

	// Init LED pins
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	PORT.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&PORT);

	uint16_t devcode = 0;
	devcode = LCD_ReadReg(0x0000);
	if (devcode == 0x8989) GPIOC->BSRR = GPIO_Pin_8;

	LCD_Reset();
	LCD_Init();

	LCD_Clear(0x0000);

	LCD_PutStr(10,10,"Hello fucking 16-bit interface!",RGB565(255,255,255));
	LCD_PutStr(30,30,"It's much faster and that's good!",RGB565(255,128,0));

	LCD_HLine(0,319,120,RGB565(255,0,128));
	LCD_VLine(160,0,239,RGB565(128,128,255));
	LCD_Line(0,0,319,239,RGB565(0,255,0));
	LCD_Line(319,0,0,239,RGB565(0,255,0));
	LCD_Rect(50,50,220,140,RGB565(0,0,255));
	LCD_FillRect(100,100,120,40,RGB565(0,0,255));
	LCD_Ellipse(159,119,60,60,RGB565(255,255,255));
	LCD_FillEllipse(159,119,50,100,RGB565(255,0,0));
	LCD_Ellipse(159,119,159,50,RGB565(255,255,255));
	LCD_Ellipse(159,119,50,119,RGB565(255,255,255));

	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
	LCD_PutStr(24,60,"CPU:",RGB565(255,255,255));
	LCD_PutInt(50,60,RCC_Clocks.SYSCLK_Frequency,RGB565(255,128,128));

	LCD_PutHex(50,80,devcode,RGB565(255,128,192));

//	LCD_BMPMono(118,118,25,120,&FacePalm[0],RGB565(0,162,232));
//	LCD_PutStr(120,134,"1-bit color >",RGB565(255,128,0));

//	LCD_Rect(19,89,82,98,RGB565(0,0,255));
//	LCD_BMP(20,90,80,96,&Face565[0]);
//	LCD_PutStr(106,102,"< 16-bit color",RGB565(255,255,0));

	LCD_PutStr(5,220,"(c) Dimon",RGB565(255,55,55));

	uint8_t i;

	for (i = 0; i<256; i++) {
		LCD_VLine(i,190,199,RGB565(i,0,0));
		LCD_VLine(i,200,209,RGB565(0,i,0));
		LCD_VLine(i,210,219,RGB565(0,0,i));
	}

	while(1);
}
