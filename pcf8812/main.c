#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>

#include <delay.h>
#include <pcf8812.h>


int main(void) {
	// Init PCF8812 pins
	// PA0 - Vdd
	// PA1 - D/C
	// PA2 - CS
	// PA3 - RES
	// PA5 - SCK
	// PA7 - MOSI
	PCF8812_Init();
	PCF8812_PowerOn();
	PCF8812_Reset();

	////////////////////////////////////////////////////////////////////////
 	PCF8812_Write(0x21); // Extended instruction set (H=1)

// 	PCF8812_Write(0x08); // Internal HV-gen = x2
	PCF8812_Write(0x09); // Internal HV-gen = x3
//	PCF8812_Write(0x0a); // Internal HV-gen = x4
//	PCF8812_Write(0x0b); // Internal HV-gen = x5

 	PCF8812_Write(0xe0); // Write Vop to register

	PCF8812_Write(0x15); // Set bias system n = 2 (contrast?)

//	PCF8812_Write(0x04); // Temperature coefficient = 0
//	PCF8812_Write(0x05); // Temperature coefficient = 1
	PCF8812_Write(0x06); // Temperature coefficient = 2
//	PCF8812_Write(0x07); // Temperature coefficient = 3

	PCF8812_Write(0x20); // Normal instruction set (H = 0)

//	PCF8812_Write(0x08); // Display control: display blank
	PCF8812_Write(0x0c); // Display control: normal mode
//	PCF8812_Write(0x09); // Display control: all display segments on
//	PCF8812_Write(0x0d); // Display control: inverse video mode
	////////////////////////////////////////////////////////////////////////

	PCF8812_Fill(0x00);
	PCF8812_Flush();

	PCF8812_PutStr5x7(10,2,"STM32F103RET6",opaque);
	PCF8812_Flush();
	Delay_ms(1000);
	PCF8812_PutStr5x7(10,12,"in cooperation",opaque);
	PCF8812_PutStr5x7(12,22,"with old good",opaque);
	PCF8812_Flush();
	Delay_ms(1000);
	PCF8812_PutStr5x7(15,35,"Siemens LCD",opaque);
	PCF8812_PutStr5x7(25,46,"101 x 64",opaque);
	PCF8812_Flush();
	Delay_ms(1000);

	uint32_t i;

	for (i = 0; i < 5; i++) {
		PCF8812_PutStr5x7(2,56,"#@* PRESENTS *@#",opaque);
		PCF8812_Flush();
		Delay_ms(1000);
		PCF8812_FillRect(0,56,100,64,PReset);
		PCF8812_Flush();
		Delay_ms(450);
	}

	PCF8812_Fill(0x00);

	PCF8812_PutStr5x7(2,2,"5x7 font",opaque);
	PCF8812_PutStr5x7(2,10,"with framebuffer",opaque);
	PCF8812_PutStr5x7(42,19,"animation",opaque);
	PCF8812_PutInt5x7(2,40,0xffffffff,opaque);
	PCF8812_PutHex5x7(2,50,0xdeadbeef,opaque);

	PCF8812_Flush();

	uint8_t x = 65;
	uint8_t y = 30;
	int8_t dx = 1;
	int8_t dy = 1;

	PCF8812_Rect(64,29,97,61,PSet);

	while(1) {
		PCF8812_FillRect(65,30,96,60,PReset);
		PCF8812_PutHex5x7(2,30,i,opaque);
		i += 11;

		PCF8812_HLine(65,96,y,PSet);
		PCF8812_VLine(x,30,60,PSet);

		x += dx;
		if (x > 96 || x < 65) dx = -dx;

		y += dy;
		if (y > 60 || y < 30) dy = -dy;

		PCF8812_Flush();
		Delay_ms(50);
	}
}
