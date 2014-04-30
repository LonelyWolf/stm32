#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_spi.h>

#include <delay.h>
#include <st7735.h>
#include <garmin-digits.h>

void ST7735_BigDig(uint8_t digit, uint16_t X, uint16_t Y, uint16_t color) {
	uint8_t i,j;
    uint8_t CH = color >> 8;
    uint8_t CL = (uint8_t)color;

	CS_L();
	ST7735_AddrSet(X,Y,X + 15,Y + 43);
	A0_H();
	for (j = 0; j < 44; j++) {
		for (i = 0; i < 16; i++) {
			if ((garmin_big_digits[(digit * 96) + i + (j / 8) * 16] >> (j % 8)) & 0x01) {
    			ST7735_write(CH);
    			ST7735_write(CL);
			} else {
    			ST7735_write(0x00);
    			ST7735_write(0x00);
			}
		}
	}
	CS_H();
}

void ST7735_MidDig(uint8_t digit, uint16_t X, uint16_t Y, uint16_t color) {
	uint8_t i,j;
    uint8_t CH = color >> 8;
    uint8_t CL = (uint8_t)color;

	CS_L();
	ST7735_AddrSet(X,Y,X + 11,Y + 23);
	A0_H();
	for (j = 0; j < 24; j++) {
		for (i = 0; i < 12; i++) {
			if ((garmin_mid_digits[(digit * 36) + i + (j / 8) * 12] >> (j % 8)) & 0x01) {
    			ST7735_write(CH);
    			ST7735_write(CL);
			} else {
    			ST7735_write(0x00);
    			ST7735_write(0x00);
			}
		}
	}
	CS_H();
}

void ST7735_SmallDig(uint8_t digit, uint16_t X, uint16_t Y, uint16_t color) {
	uint8_t i,j;
    uint8_t CH = color >> 8;
    uint8_t CL = (uint8_t)color;

	CS_L();
	ST7735_AddrSet(X,Y,X + 10,Y + 20);
	A0_H();
	for (j = 0; j < 21; j++) {
		for (i = 0; i < 11; i++) {
			if ((garmin_small_digits[(digit * 33) + i + (j / 8) * 11] >> (j % 8)) & 0x01) {
    			ST7735_write(CH);
    			ST7735_write(CL);
			} else {
    			ST7735_write(0x00);
    			ST7735_write(0x00);
			}
		}
	}
	CS_H();
}


uint8_t i;


int main(void)
{
	// Screen connection
	// SCK  -> PB3
	// A0   -> PB4
	// SDA  -> PB5
	// RST  -> PB6
	// CS   -> PB7

	ST7735_Init();
	ST7735_AddrSet(0,0,159,127);
	ST7735_Clear(0x0000);

	ST7735_PutStr5x7(0,0,"Hello fucking screen!",RGB565(255,0,0));
	ST7735_PutStr5x7(0,10,"This is 5x7 font",RGB565(0,255,0));
	ST7735_PutStr5x7(0,20,"Screen 128x160 pixels",RGB565(0,0,255));

	ST7735_Orientation(scr_normal);

	while(1) {
		ST7735_BigDig(2,0,35,RGB565(212,246,190));
		ST7735_BigDig(3,16,35,RGB565(212,246,190));
		ST7735_FillRect(33,42,35,44,RGB565(177,211,190));
		ST7735_FillRect(33,67,35,69,RGB565(177,211,190));
		ST7735_BigDig(4,37,35,RGB565(212,246,190));
		ST7735_BigDig(8,53,35,RGB565(212,246,190));
		ST7735_FillRect(70,42,72,44,RGB565(177,211,190));
		ST7735_FillRect(70,67,72,69,RGB565(177,211,190));
		ST7735_BigDig(5,75,35,RGB565(212,246,190));
		ST7735_BigDig(6,91,35,RGB565(212,246,190));
		ST7735_PutStr5x7(0,80,"Time:",RGB565(255,255,255));

		Delay_ms(500);
	}

    while(1) {
    }
}
