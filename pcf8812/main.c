#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>

#include <delay.h>
#include <pcf8812.h>

#include <slava.h>


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

	uint8_t i;

	for (i = 0; i < 65; i++) {
		PCF8812_SetPixel(i,i);
		PCF8812_SetPixel(i,64 - i);
		PCF8812_SetPixel(100 - i,i);
		PCF8812_SetPixel(100 - i,64 - i);
	}

	uint8_t x,y;

	for (y = 0; y < 3; y++) {
		for (x = 0; x < 55; x++) {
			vRAM[x + 20 + (5 - y) * 102] = slava[(54 - x) + y * 55];
		}
	}

	PCF8812_Flush();

	while(1);
}
