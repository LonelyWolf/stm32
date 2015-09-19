#include "main.h"




// Draw string with wave effect
void LCD_WavyText(uint8_t X, uint8_t Y, const char *str, uint8_t pos, uint8_t step, const Font_TypeDef *Font) {
	uint8_t pX = X;
	uint8_t eX = scr_width - Font->font_Width - 1;
	uint8_t ps = pos;

	while (*str) {
		pX += LCD_PutChar(pX,Y + sine_LUT[ps],*str++,Font);
		ps += step;
		if (ps > sizeof(sine_LUT) - 1) ps = 0;
		if (pX > eX) break;
	}
}




int main(void) {
	// Initialize the MCU clock system
	SystemInit();
	SystemCoreClockUpdate();

	// Initialize delay functions
	Delay_Init();

	// Initialize display SPI port
	SPI1_HandleInit();
	SPIx_Init(&SSD1306_SPI_PORT,SPI_DIR_TX,SPI_BR_2); // highest SPI speed (16MHz on 32MHz CPU)

	// Initialize display
	SSD1306_InitGPIO();
	SSD1306_Init();

	// Screen orientation normal (FPC cable at the bottom)
	SSD1306_Orientation(LCD_ORIENT_NORMAL);

	// Mid level contrast
	SSD1306_Contrast(127);


	// Now do some drawing

	// Clear video buffer
	SSD1306_Fill(0x00);

	// Drawing mode: set pixels
	LCD_PixelMode = LCD_PSET;

	// Draw a couple of strings
	LCD_PutStr(35,11,"SSD1306",fnt7x10);
	LCD_PutStr(19,43,"OLED 128x64",fnt7x10);

	// Drawing mode: invert pixels
	LCD_PixelMode = LCD_PINV;

	// Draw a filled rectangle on a half of screen
	// Current mode invert pixels, so second text string will be inverted
	LCD_FillRect(0,scr_height >> 1,scr_width - 1,scr_height - 1);

	// Send video buffer to the screen
	SSD1306_Flush();

	// Wait for 5 seconds
	Delay_ms(5000);

	// Drawing mode: set pixels
	LCD_PixelMode = LCD_PSET;

	// Clear video buffer and draw a 128x64 bitmap
	SSD1306_Fill(0x00);
	LCD_DrawBitmap(0,0,128,64,bmp_dangerous_tunnels);

	// Send video buffer to the screen
	SSD1306_Flush();

	// Wait for 3 seconds
	Delay_ms(3000);

	// Some image flashing by inverting all pixels on display
	i = 256;
	while (i) {
		Delay_ms(i << 2);
		SSD1306_SetInvert(LCD_INVERT_ON);
		Delay_ms(i << 2);
		SSD1306_SetInvert(LCD_INVERT_OFF);
		i >>= 1;
	}

	// Fill whole video buffer with pixels (faster than FillRect)
	SSD1306_Fill(0xFF);

	// Drawing mode: reset pixels (pixels will be reset while drawing)
	LCD_PixelMode = LCD_PRES;

	// Draw some sort of banner
	LCD_PutStr(35,5,"SSD1306",fnt7x10);
	LCD_PutStr(19,scr_height - 15,"OLED 128x64",fnt7x10);
	LCD_FillRect(0,18,scr_width - 1,scr_height - 20);

	// Drawing mode: set pixels
	LCD_PixelMode = LCD_PSET;

	// Draw tiled bitmap
	for (i = 0; i < scr_width - 1; i += 16)	LCD_DrawBitmap(i,23,16,17,bmp_tile);

	// Send video buffer to the screen
	SSD1306_Flush();

	// Configure display to scroll horizontally right
	SSD1306_ScrollHSetup(LCD_SCROLL_RIGHT,2,5,LCD_SCROLL_IF25);

	// Start hardware display scrolling
	SSD1306_ScrollStart();

	// Delay for 5 seconds
	Delay_ms(5000);

	// Stop scrolling
	SSD1306_ScrollStop();

	// Configure display to scroll horizontally left with maximum speed
	SSD1306_ScrollHSetup(LCD_SCROLL_LEFT,2,5,LCD_SCROLL_IF2);

	// Start hardware display scrolling
	SSD1306_ScrollStart();

	// Delay for 5 seconds
	Delay_ms(5000);

	// Stop scrolling
	SSD1306_ScrollStop();

	// Weavy letters for infinite
	uint8_t pos = 16;
	while (1) {
		SSD1306_Fill(0x00);
		LCD_WavyText(7,0,"128x64 SSD1306",pos,1,fnt7x10);
		LCD_WavyText(0,30,"Nice OLED screen",pos,3,fnt7x10);
		pos++;
		if (pos > sizeof(sine_LUT) - 1) pos = 0;
		Delay_ms(5);
		SSD1306_Flush();
	}
}
