#include "main.h"


///////////////////
// STM32L151RDT6 //
///////////////////


// Variables
uint32_t i,j,k;
uint32_t frames;


int main(void) {
	// Initialize the MCU clock system
	SystemInit();
	SetSysClock();
	SystemCoreClockUpdate();


	// Initialize delay functions
	Delay_Init();


	// Initialize the display SPI port:
	//   ST7528_SPI_PORT = SPI1
	//   1 line TX-only
	//   Clock line: idle high
	//   Clock phase: 2nd edge
	//   8MHz at 32MHz CPU
	SPI1_HandleInit();
	SPIx_Init(&ST7528_SPI_PORT,SPI_CLK_PH_E2,SPI_DIR_TX,SPI_BR_4);


	// Initialize the display GPIO lines
	ST7528_InitGPIO();

	// Initialize the display
	ST7528_Init();
	ST7528_Orientation(SCR_ORIENT_NORMAL);

	j = 0;
	k = 1234567;
	frames = 0;
	while (1) {
		// Clear vRAM
		ST7528_Clear();

		// Depending on frames count draw different stuff...
		if (frames < 48) {
			// Some drawing

			// Draw boxes with all grayscale levels
			i = 0;
			while (i < 16) {
				// Fill box
				LCD_FillRect((i * 8),0,(i * 8) + 7,30,i);
				// Print grayscale level
				if (i > 7) lcd_color = 0;
				if (i < 10) {
					LCD_PutIntU((i * 8) + 1,10,i,dig5x9);
				} else {
					LCD_PutIntU((i * 8) + 1,4,1,dig5x9);
					LCD_PutIntU((i * 8) + 1,14,i - 10,dig5x9);
				}
				lcd_color = 15;
				i++;
			}

			// Print two strings with different fonts
			LCD_PutStr(2,34,"Display: BO128128C",fnt5x7);
			LCD_PutStr(38,46,"IC: ST7528",fnt7x10);

			// Print two numbers, no meaning, just for dynamic picture
			// with gray background
			LCD_FillRect(0,63,63,75,2);
			LCD_PutIntU(10,65,k,dig5x9);
			LCD_FillRect(0,76,127,96,4);
			LCD_PutIntU(10,78,0xFFFFFFFF - k,dig8x16);
			k += 1234321;

			// Draw "moving" bitmap (monochrome)
			LCD_DrawBitmap(j,scr_height - 25,30,25,bmp_bikeman_1bit);
			j++;
			if (j > scr_width - 30) j = 0;
		} else if (frames < 78) {
			// 4-bit bitmap #1
			LCD_DrawBitmapGS(0,0,128,128,bmp_ladybird_4bit);
		} else if (frames < 108) {
			// 4-bit bitmap #2
			LCD_DrawBitmapGS(0,0,128,128,bmp_anime_4bit);
		}

		// Increase frame counter
		frames++;
		if (frames > 108) frames = 0;

		// Send vRAM data to display
		ST7528_Flush();

		// Some delay between frames
		Delay_ms(50);
	}
}
