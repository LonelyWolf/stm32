#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_spi.h>

#include <delay.h>
#include <uc1701.h>
#include <garmin-digits.h>


void UC1701_BigDig(uint8_t digit, uint8_t X, uint8_t Y) {
	uint8_t i,j;

	// one digit = 16 x 6 bytes (16x44 pixels)
	for (i = 0; i < 16; i++) {
		for (j = 0; j < 44; j++) {
			if ((garmin_big_digits[(digit * 96) + i + (j / 8) * 16] >> (j % 8)) & 0x01) UC1701_SetPixel(X + i, Y + j); else UC1701_ResetPixel(X + i, Y + j);
		}
	}
}

void UC1701_MidDig(uint8_t digit, uint8_t X, uint8_t Y) {
	uint8_t i,j;

	// one digit = 12 x 3 bytes (12x24 pixels)
	for (i = 0; i < 12; i++) {
		for (j = 0; j < 24; j++) {
			if ((garmin_mid_digits[(digit * 36) + i + (j / 8) * 12] >> (j % 8)) & 0x01) UC1701_SetPixel(X + i, Y + j); else UC1701_ResetPixel(X + i, Y + j);
		}
	}
}

void UC1701_SmallDig(uint8_t digit, uint8_t X, uint8_t Y) {
	uint8_t i,j;

	// one digit = 11 x 3 bytes (11x21 pixels)
	for (i = 0; i < 11; i++) {
		for (j = 0; j < 21; j++) {
			if ((garmin_small_digits[(digit * 33) + i + (j / 8) * 11] >> (j % 8)) & 0x01) UC1701_SetPixel(X + i, Y + j); else UC1701_ResetPixel(X + i, Y + j);
		}
	}
}



int main(void) {
	UC1701_Init();
	UC1701_Contrast(4,24);

	UC1701_Orientation(scr_normal);
	UC1701_Fill(0x00);

/*
	// Speedometer sample
	UC1701_BigDig(2,0,0);
	UC1701_BigDig(3,16,0);
	UC1701_FillRect(33,38,35,43,PSet);
	UC1701_BigDig(5,37,0);
	UC1701_SmallDig(11,53,12);
*/

	// Time sample
	UC1701_BigDig(2,0,0);
	UC1701_BigDig(3,16,0);
	UC1701_FillRect(33,8,35,10,PSet);
	UC1701_FillRect(33,32,35,34,PSet);
	UC1701_BigDig(4,37,0);
	UC1701_BigDig(7,53,0);
	UC1701_FillRect(70,8,72,10,PSet);
	UC1701_FillRect(70,32,72,34,PSet);
	UC1701_BigDig(5,74,0);
	UC1701_BigDig(8,90,0);
	UC1701_SmallDig(9,107,0);
	UC1701_SmallDig(6,118,0);

	UC1701_Flush();

	while(1) {
	}
}
