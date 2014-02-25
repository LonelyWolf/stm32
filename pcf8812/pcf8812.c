#include <string.h> // For memcpy
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_spi.h>

#include <pcf8812.h>
#include <delay.h>
#include <font5x7.h>

uint8_t vRAM[917]; // Display buffer


void PCF8812_Init() {
	// Configure pins as output with Push-Pull
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef PORT;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;

	PORT.GPIO_Pin = PCF8812_MOSI_PIN;
	GPIO_Init(PCF8812_MOSI_PORT,&PORT);

	PORT.GPIO_Pin = PCF8812_SCK_PIN;
	GPIO_Init(PCF8812_SCK_PORT,&PORT);

	PORT.GPIO_Pin = PCF8812_CS_PIN;
	GPIO_Init(PCF8812_CS_PORT,&PORT);

	PORT.GPIO_Pin = PCF8812_PWR_PIN;
	GPIO_Init(PCF8812_PWR_PORT,&PORT);

	PORT.GPIO_Pin = PCF8812_DC_PIN;
	GPIO_Init(PCF8812_DC_PORT,&PORT);

	PORT.GPIO_Pin = PCF8812_RES_PIN;
	GPIO_Init(PCF8812_RES_PORT,&PORT);

	CS_H();
	RES_H();
	DC_L();
	PWR_L();
}

// PCF8812 power on
void PCF8812_PowerOn(void) {
	CS_L();
	RES_L();
	Delay_ms(20);
	PWR_H();
	Delay_ms(20);
	RES_H();
}

// Hardware reset of PCF8812
void PCF8812_Reset(void) {
	RES_L();
	RES_H();
}

// Software SPI send byte
void PCF8812_Write(uint8_t data) {
	uint8_t i;

	for (i = 0; i < 8; i++) {
		if (data & 0x80) MOSI_H(); else MOSI_L();
		data <<= 1;
		SCK_L();
		SCK_H();
	}
}

// Set RAM address (Y - bank number, X - position in bank)
void PCF8812_SetXY(uint8_t X, uint8_t Y) {
	DC_L();
	PCF8812_Write(0x40 | Y); // Select display RAM bank (0..8)
	PCF8812_Write(0x80 | X); // Set X address (0..101)
}

// Send vRAM buffer into display
void PCF8812_Flush(void) {
	uint32_t i;

	DC_L();
	PCF8812_Write(0x40); // Select display RAM bank 0
	PCF8812_Write(0x80); // Set column 0
	DC_H();
	for (i = 0; i < 816; i++) PCF8812_Write(vRAM[i]);
}

// Fill vRAM with byte pattern
void PCF8812_Fill(uint8_t pattern) {
	uint32_t i;

	for (i = 0; i < 816; i++) vRAM[i] = pattern;
}

// Set pixel in vRAM buffer
void PCF8812_SetPixel(uint8_t X, uint8_t Y) {
	vRAM[((Y / 8) * 102) + X] |= 1 << (Y % 8);
}

// Clear pixel in vRAM buffer
void PCF8812_ResetPixel(uint8_t X, uint8_t Y) {
	vRAM[((Y / 8) * 102) + X] &= ~(1 << (Y % 8));
}

void PCF8812_HLine(uint8_t X1, uint8_t X2, uint8_t Y, PSetReset_TypeDef SR) {
	uint8_t x;

	if (SR == PSet) {
		for (x = X1; x <= X2; x++) PCF8812_SetPixel(x,Y);
	} else {
		for (x = X1; x <= X2; x++) PCF8812_ResetPixel(x,Y);
	}
}

void PCF8812_VLine(uint8_t X, uint8_t Y1, uint8_t Y2, PSetReset_TypeDef SR) {
	uint8_t y;

	if (SR == PSet) {
		for (y = Y1; y <= Y2; y++) PCF8812_SetPixel(X,y);
	} else {
		for (y = Y1; y <= Y2; y++) PCF8812_ResetPixel(X,y);
	}
}

void PCF8812_Rect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2, PSetReset_TypeDef SR) {
	PCF8812_HLine(X1,X2,Y1,SR);
	PCF8812_HLine(X1,X2,Y2,SR);
	PCF8812_VLine(X1,Y1 + 1,Y2 - 1,SR);
	PCF8812_VLine(X2,Y1 + 1,Y2 - 1,SR);
}

void PCF8812_FillRect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2, PSetReset_TypeDef SR) {
	uint8_t y;

	for (y = Y1; y <= Y2; y++) PCF8812_HLine(X1,X2,y,SR);
}

void PCF8812_Line(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2) {
	int16_t dX = X2-X1;
	int16_t dY = Y2-Y1;
	int16_t dXsym = (dX > 0) ? 1 : -1;
	int16_t dYsym = (dY > 0) ? 1 : -1;

	if (dX == 0) {
		if (Y2 > Y1) PCF8812_VLine(X1,Y1,Y2,PSet); else PCF8812_VLine(X1,Y2,Y1,PSet);
		return;
	}
	if (dY == 0) {
		if (X2 > X1) PCF8812_HLine(X1,X2,Y1,PSet); else PCF8812_HLine(X2,X1,Y1,PSet);
		return;
	}

	dX *= dXsym;
	dY *= dYsym;
	int16_t dX2 = dX << 1;
	int16_t dY2 = dY << 1;
	int16_t di;

	if (dX >= dY) {
		di = dY2 - dX;
		while (X1 != X2) {
			PCF8812_SetPixel(X1,Y1);
			X1 += dXsym;
			if (di < 0) {
				di += dY2;
			} else {
				di += dY2 - dX2;
				Y1 += dYsym;
			}
		}
	} else {
		di = dX2 - dY;
		while (Y1 != Y2) {
			PCF8812_SetPixel(X1,Y1);
			Y1 += dYsym;
			if (di < 0) {
				di += dX2;
			} else {
				di += dX2 - dY2;
				X1 += dXsym;
			}
		}
	}
	PCF8812_SetPixel(X1,Y1);
}

void PCF8812_Ellipse(uint16_t X, uint16_t Y, uint16_t A, uint16_t B) {
	int16_t Xc = 0, Yc = B;

	long A2 = (long)A*A, B2 = (long)B*B;
	long C1 = -(A2/4 + A % 2 + B2);
	long C2 = -(B2/4 + B % 2 + A2);
	long C3 = -(B2/4 + B % 2);
	long t = -A2 * Yc;
	long dXt = B2*Xc*2, dYt = -A2*Yc*2;
	long dXt2 = B2*2, dYt2 = A2*2;
	while (Yc >= 0 && Xc <= A) {
		PCF8812_SetPixel(X + Xc,Y + Yc);
		if (Xc != 0 || Yc != 0) PCF8812_SetPixel(X - Xc,Y - Yc);
		if (Xc != 0 && Yc != 0) {
			PCF8812_SetPixel(X + Xc,Y - Yc);
			PCF8812_SetPixel(X - Xc,Y + Yc);
		}
		if (t + Xc*B2 <= C1 || t + Yc*A2 <= C3) {
			Xc++;
			dXt += dXt2;
			t   += dXt;
		} else if (t - Yc*A2 > C2) {
			Yc--;
			dYt += dYt2;
			t   += dYt;
		} else {
			Xc++;
			Yc--;
			dXt += dXt2;
			dYt += dYt2;
			t   += dXt;
			t   += dYt;
		}
	}
}

void PCF8812_PutChar5x7(uint8_t X, uint8_t Y, uint8_t Char, Opaque_TypeDef bckgnd) {
	uint16_t i,j;
	uint8_t buffer[5],tmpCh;

	memcpy(buffer,&Font5x7[(Char - 32) * 5],5);

	if (bckgnd == opaque) {
	    for (i = 0; i < 5; i++) {
	    	tmpCh = buffer[i];
	    	for (j = 0; j < 8; j++) {
	    		if ((tmpCh >> j) & 0x01) PCF8812_SetPixel(X + i,Y + j); else PCF8812_ResetPixel(X + i,Y + j);
	    	}
	    }
	} else {
		for (i = 0; i < 5; i++) {
			tmpCh = buffer[i];
			for (j = 0; j < 8; j++) {
				if ((tmpCh >> j) & 0x01) PCF8812_SetPixel(X + i,Y + j);
			}
		}
	}
}

void PCF8812_PutStr5x7(uint8_t X, uint8_t Y, char *str, Opaque_TypeDef bckgnd) {
    while (*str) {
        PCF8812_PutChar5x7(X,Y,*str++,bckgnd);
        if (X < 101 - 6) { X += 6; } else if (Y < 64 - 8) { X = 0; Y += 8; } else { X = 0; Y = 0; }
    };
}

void PCF8812_PutInt5x7(uint8_t X, uint8_t Y, uint32_t num, Opaque_TypeDef bckgnd) {
	char str[11]; // 10 chars max for UINT32_MAX
	int i = 0;

	do { str[i++] = num % 10 + '0'; } while ((num /= 10) > 0);
	int strLen = i;
	for (i--; i >= 0; i--) PCF8812_PutChar5x7(X + (strLen * 6) - ((i + 1) * 6),Y,str[i],bckgnd);
}

void PCF8812_PutHex5x7(uint8_t X, uint8_t Y, uint32_t num, Opaque_TypeDef bckgnd) {
	char str[11]; // 10 chars max for UINT32_MAX
	int i = 0;

	do { str[i++] = "0123456789ABCDEF"[num % 0x10]; } while ((num /= 0x10) > 0);
	str[i++] = 'x';
	str[i++] = '0';

	int strLen = i;

	for (i--; i >= 0; i--) PCF8812_PutChar5x7(X + (strLen * 6) - ((i + 1) * 6),Y,str[i],bckgnd);
}
