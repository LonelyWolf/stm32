#include <stm32f10x_gpio.h>
#include <delay.h>
#include <ssd1289.h>
#include <string.h>
#include <math.h>


uint16_t RGB565(uint8_t R,uint8_t G,uint8_t B) {
	return ((R >> 3) << 11) | ((G >> 2) << 5) | (B >> 3);
}

void LCD_write_command(uint16_t cmd) {
	GPIOB->BRR = LCD_CS;       // LCD_CS low (chip select pull)
	GPIOB->BRR = LCD_RS;       // LCD_RS low (register select = instruction)
	//GPIOA->ODR = cmd;         // put cmd to PortA (full length)
    // put cmd [0..12] bits to PortA (actual LCD_DB00..LCD_DB12)
    // put cmd [13..15] bits to PortB (actual LCD_DB13..LCD_DB15)
    GPIOA->ODR = cmd & 0x1fff;
    GPIOB->ODR = (GPIOB->ODR & 0xfff8) | (cmd >> 13);
    GPIOB->BRR = LCD_WR;       // pull LCD_WR to low (write strobe start)
    // Write strobe 66ns long by datasheet. GPIO speed on STM32F103 at 72MHz slower -> delay is unnecessary
    // asm volatile ("nop");
	GPIOB->BSRR = LCD_WR;      // pull LCD_WR to high (write strobe end)
	GPIOB->BSRR = LCD_CS;      // LCD_CS high (chip select release)
}

void LCD_write_data(uint16_t data) {
	GPIOB->BRR = LCD_CS;        // LCD_CS low (chip select pull)
	GPIOB->BSRR = LCD_RS;       // LCD_RS high (register select = data)
	//GPIOA->ODR = data;         // put data to PortA
    // put data [0..12] bits to PortA (actual LCD_DB00..LCD_DB12)
    // put data [13..15] bits to PortB (actual LCD_DB13..LCD_DB15)
    GPIOA->ODR =  data & 0x1fff;
    GPIOB->ODR = (GPIOB->ODR & 0xfff8) | (data >> 13);
	GPIOB->BRR = LCD_WR;        // pull LCD_WR to low (write strobe start)
    // Write strobe 66ns long by datasheet. GPIO speed on STM32F103 at 72MHz slower -> delay is unnecessary
    // asm volatile ("nop");
	GPIOB->BSRR = LCD_WR;       // pull LCD_WR to high (write strobe end)
	GPIOB->BSRR = LCD_CS;       // LCD_CS high (chip select release)
}

uint16_t LCD_read_data(void) {
	volatile uint16_t data = 0;

	// Set data pins as input
	GPIO_InitTypeDef PORT;
	PORT.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2  | GPIO_Pin_3  | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | \
					GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	PORT.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&PORT);
	PORT.GPIO_Pin  = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_Init(GPIOB,&PORT);

	// Read 16-bits from LCD
	GPIOB->BRR = LCD_CS;                     // LCD_CS low (chip select pull)
	GPIOB->BSRR = LCD_RS;                    // LCD_RS high (register select = data)
	GPIOB->BRR = LCD_RD;                     // pull LCD_RD to low (read strobe start)
	Delay_us(2);
	//data = (uint16_t)GPIOA->IDR;             // get data from PortA (full length)
	data  = (uint16_t)GPIOA->IDR & 0x1fff;   // get data [0..12] bits from PortA (actual LCD_DB00..LCD_DB12)
	data |= (uint16_t)GPIOB->IDR << 13;      // get data [13..15] bits from PortA (actual LCD_DB13..LCD_DB15)
	GPIOB->BSRR = LCD_RD;                    // pull LCD_RD to high (read strobe end)
	GPIOB->BSRR = LCD_CS;                    // LCD_CS high (chip select release)

	// Set data pins as output
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB,&PORT);
	PORT.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2  | GPIO_Pin_3  | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | \
	                GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_Init(GPIOA,&PORT);

	return data;
}

void LCD_WriteReg(uint16_t reg, uint16_t data) {
	LCD_write_command(reg);
	LCD_write_data(data);
}

uint16_t LCD_ReadReg(uint16_t reg) {
	LCD_write_command(reg);
	return LCD_read_data();
}

void LCD_Reset(void) {
	GPIO_WriteBit(GPIOB,GPIO_Pin_10,Bit_SET);
	Delay_ms(3); // 1ms by datasheet
	GPIO_WriteBit(GPIOB,GPIO_Pin_10,Bit_RESET);
	Delay_ms(3); // 1ms by datasheet
	GPIO_WriteBit(GPIOB,GPIO_Pin_10,Bit_SET);
	Delay_ms(3); // 1ms by datasheet
}

void LCD_Init(void) {
	LCD_Reset();
	GPIOB->BSRR = LCD_CS;  // pull LCD_CS to high
	GPIOB->BSRR = LCD_RD;  // pull LCD_WR to high
	GPIOB->BSRR = LCD_WR;  // pull LCD_WR to high
	Delay_ms(20);

	// power supply setting
    // set R07h at 0021h (GON=1,DTE=0,D[1:0]=01)
    LCD_WriteReg(0x0007,0x0021);
    // set R00h at 0001h (OSCEN=1)
    LCD_WriteReg(0x0000,0x0001);
    // set R07h at 0023h (GON=1,DTE=0,D[1:0]=11)
    LCD_WriteReg(0x0007,0x0023);
    // set R10h at 0000h (Exit sleep mode)
    LCD_WriteReg(0x0010,0x0000);
    // Wait 30ms
    Delay_ms(30);
    // set R07h at 0033h (GON=1,DTE=1,D[1:0]=11)
    LCD_WriteReg(0x0007,0x0033);
    // Entry mode setting (R11h)
    // R11H Entry mode
    // vsmode DFM1 DFM0 TRANS OEDef WMode DMode1 DMode0 TY1 TY0 ID1 ID0 AM LG2 LG2 LG0
    //   0     1    1     0     0     0     0      0     0   1   1   1  *   0   0   0
    // 14..13.DFM[1,0] = (11 => 65k Color; 10 => 256k Color)
    // 05..04.ID[1,0] = (00 = hdec,vdec; 01 = hinc,vdec; 10 = hdec,vinc; 11 = hinc,vinc
    // 03.AM = 0 for horizontal; AM = 1 for vertical
    LCD_WriteReg(0x0011,(1<<14)|(1<<13)|(1<<6)|(1<<5)|(1<<4)|(1<<3));
    // LCD driver AC setting (R02h)
    LCD_WriteReg(0x0002,0x0600);
    // power control 1
    // DCT3 DCT2 DCT1 DCT0 BT2 BT1 BT0 0 DC3 DC2 DC1 DC0 AP2 AP1 AP0 0
    // 1     0    1    0    1   0   0  0  1   0   1   0   0   1   0  0
    // DCT[3:0] fosc/4 BT[2:0]  DC{3:0] fosc/4
    LCD_WriteReg(0x0003,0x0804);//0xA8A4
    LCD_WriteReg(0x000C,0x0000);//
    LCD_WriteReg(0x000D,0x0808);// 0x080C --> 0x0808
    // power control 4
    // 0 0 VCOMG VDV4 VDV3 VDV2 VDV1 VDV0 0 0 0 0 0 0 0 0
    // 0 0   1    0    1    0    1    1   0 0 0 0 0 0 0 0
    LCD_WriteReg(0x000E,0x2900);
    LCD_WriteReg(0x001E,0x00B8);
    // Driver output control
    // 0 RL REV CAD BGR SM TB MUX8 MUX7 MUX6 MUX5 MUX4 MUX3 MUX2 MUX1 MUX0
    // 0 0  1    0   1  0  1  1     0    0    1    1    1    1    1    1
    // 14.RL  = Output shift direction of Source driver (1 = S0->S719; 0 = S719->S0)
    // 13.REV = Grayscale mode (1 = normal; 0 = inverted)
    // 11.BGR = Components order(1 = BGR; 0 = RGB)
    // 09.TB  = Output shift direction of Gate driver (1 = G0->G319; 0 = G319->G0)
    // 08..00.MUX = Number of lines (0..319)
    LCD_WriteReg(0x0001,(0<<14)|(1<<13)|(1<<11)|(0<<9)|(1<<8)|(1<<5)|(1<<4)|(1<<3)|(1<<2)|(1<<1)|(1<<0));
    LCD_WriteReg(0x0010,0x0000);
    LCD_WriteReg(0x0005,0x0000);
    LCD_WriteReg(0x0006,0x0000);
    LCD_WriteReg(0x0016,0xEF1C);
    LCD_WriteReg(0x0017,0x0003);
    LCD_WriteReg(0x0007,0x0233);
    LCD_WriteReg(0x000B,0x0000|(3<<6));
    LCD_WriteReg(0x000F,0x0000);
    LCD_WriteReg(0x0041,0x0000);
    LCD_WriteReg(0x0042,0x0000);
    LCD_WriteReg(0x0048,0x0000);
    LCD_WriteReg(0x0049,0x013F);
    LCD_WriteReg(0x004A,0x0000);
    LCD_WriteReg(0x004B,0x0000);
    LCD_WriteReg(0x0044,0xEF00);
    LCD_WriteReg(0x0045,0x0000);
    LCD_WriteReg(0x0046,0x013F);
    // Gamma control
    LCD_WriteReg(0x0030,0x0707);
    LCD_WriteReg(0x0031,0x0204);
    LCD_WriteReg(0x0032,0x0204);
    LCD_WriteReg(0x0033,0x0502);
    LCD_WriteReg(0x0034,0x0507);
    LCD_WriteReg(0x0035,0x0204);
    LCD_WriteReg(0x0036,0x0204);
    LCD_WriteReg(0x0037,0x0502);
    LCD_WriteReg(0x003A,0x0302);
    LCD_WriteReg(0x003B,0x0302);
    // Gamma control end
    LCD_WriteReg(0x0023,0x0000);
    LCD_WriteReg(0x0024,0x0000);
    LCD_WriteReg(0x0025,0x8000);   // 65hz
    LCD_WriteReg(0x004f,0);
    LCD_WriteReg(0x004e,0);
}

void LCD_SetCursor(uint16_t X, uint16_t Y) {
	LCD_WriteReg(0x004e,Y);
	LCD_WriteReg(0x004f,X);
}

void LCD_Pixel(uint16_t X, uint16_t Y, uint16_t C) {
	LCD_SetCursor(X,Y);
	LCD_WriteReg(0x0022,C);
}

void LCD_SetWindow(uint16_t X, uint16_t Y, uint16_t W, uint16_t H) {
	uint16_t XW = X + W - 1;
	uint16_t YH = Y + H - 1;
	LCD_WriteReg(0x0045,X);           // Top RAM address pos
	LCD_WriteReg(0x0046,XW);          // Bottom RAM address pos
	LCD_WriteReg(0x0044,(YH << 8) + (Y & 0x00ff)); // Left and Right RAM address pos
	LCD_SetCursor(X,Y);
}

void LCD_Clear(uint16_t C) {
	uint32_t i = 0;
	LCD_SetWindow(0,0,320,240);
	LCD_write_command(0x0022);
	for (i = 0; i < 320*240; i++) { LCD_write_data(C); }
}

void LCD_HLine(uint16_t X0, uint16_t X1, uint16_t Y, uint16_t Color) {
	uint16_t W = X1 - X0 + 1;
    uint16_t i;

    LCD_SetWindow(X0,Y,W,1);
	LCD_write_command(0x0022);
	for (i = 0; i < W; i++) { LCD_write_data(Color); }
}

void LCD_VLine(uint16_t X, uint16_t Y0, uint16_t Y1, uint16_t Color) {
	uint16_t H = Y1 - Y0 + 1;
	uint16_t i;

	LCD_SetWindow(X,Y0,1,H);
	LCD_write_command(0x0022);
	for (i = 0; i < H; i++) { LCD_write_data(Color); }
}

void LCD_Line(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, uint16_t Color) {
	int16_t dX = X2 - X1;
	int16_t dY = Y2 - Y1;
	int16_t dXsym = (dX > 0) ? 1 : -1;
	int16_t dYsym = (dY > 0) ? 1 : -1;

	if (dX == 0) {
		if (Y2>Y1) LCD_VLine(X1,Y1,Y2,Color); else LCD_VLine(X1,Y2,Y1,Color);
		return;
	}
	if (dY == 0) {
		if (X2>X1) LCD_HLine(X1,X2,Y1,Color); else LCD_HLine(X2,X1,Y1,Color);
		return;
	}

	LCD_SetWindow(0,0,320,240);

	dX *= dXsym;
	dY *= dYsym;
	int16_t dX2 = dX << 1;
	int16_t dY2 = dY << 1;
	int16_t di;

	if (dX >= dY) {
		di = dY2 - dX;
		while (X1 != X2) {
			LCD_Pixel(X1,Y1,Color);
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
			LCD_Pixel(X1,Y1,Color);
			Y1 += dYsym;
			if (di < 0) {
				di += dX2;
			} else {
				di += dX2 - dY2;
				X1 += dXsym;
			}
		}
	}
	LCD_Pixel(X1,Y1,Color);
}

#define ipart_(X) ((int)(X))
#define round_(X) ((int)(((float)(X))+0.5))
#define fpart_(X) (((float)(X))-(float)ipart_(X))
#define rfpart_(X) (1.0-fpart_(X))
#define swap_(a, b) do{ __typeof__(a) tmp;  tmp = a; a = b; b = tmp; }while(0)

static void dla_plot(int x, int y, uint8_t r, uint8_t g, uint8_t b, float br) {
	r = (uint8_t)round_(br*r);
	g = (uint8_t)round_(br*g);
	b = (uint8_t)round_(br*b);

	LCD_Pixel(x,y,RGB565(r,g,b));
}

void LCD_LineAA(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, uint16_t Color) {
	float dX = (float)X2 - (float)X1;
	float dY = (float)Y2 - (float)Y1;

	uint8_t R,G,B;

	R = (uint8_t)(Color >> 11) << 3;
	G = (uint8_t)((Color >> 5) & 0x3f) << 2;
	B = (uint8_t)(Color & 0x1f) << 3;

	if (dX == 0) { if (Y2>Y1) LCD_VLine(X1,Y1,Y2,Color); else LCD_VLine(X1,Y2,Y1,Color); return; }
	if (dY == 0) { if (X2>X1) LCD_HLine(X1,X2,Y1,Color); else LCD_HLine(X2,X1,Y1,Color); return; }

	LCD_SetWindow(0,0,320,240);

	float dx = (float)X2 - (float)X1;
	float dy = (float)Y2 - (float)Y1;
	if (fabs(dx) > fabs(dy)) {
		if (X2 < X1) { swap_(X1,X2); swap_(Y1,Y2); }
		float gradient = dy / dx;
		float xend = round_(X1);
		float yend = Y1 + gradient*(xend - X1);
		float xgap = rfpart_(X1 + 0.5f);
		int xpxl1 = xend;
		int ypxl1 = ipart_(yend);
		dla_plot(xpxl1,ypxl1,R,G,B,rfpart_(yend)*xgap);
		dla_plot(xpxl1,ypxl1+1,R,G,B,fpart_(yend)*xgap);
		float intery = yend + gradient;

		xend = round_(X2);
		yend = Y2 + gradient*(xend - X2);
		xgap = fpart_(X2+0.5f);
		int xpxl2 = xend;
		int ypxl2 = ipart_(yend);
		dla_plot(xpxl2,ypxl2,R,G,B,rfpart_(yend)*xgap);
		dla_plot(xpxl2,ypxl2 + 1,R,G,B,fpart_(yend)*xgap);

		int x;
		for (x = xpxl1+1; x <= (xpxl2-1); x++) {
			dla_plot(x,ipart_(intery),R,G,B,rfpart_(intery));
			dla_plot(x,ipart_(intery)+1,R,G,B,fpart_(intery));
			intery += gradient;
		}
	} else {
		if ( Y2 < Y1 ) { swap_(X1,X2); swap_(Y1,Y2); }
		float gradient = dx / dy;
		float yend = round_(Y1);
		float xend = X1 + gradient*(yend - Y1);
		float ygap = rfpart_(Y1+0.5f);
		int ypxl1 = yend;
		int xpxl1 = ipart_(xend);
		dla_plot(xpxl1,ypxl1,R,G,B,rfpart_(xend)*ygap);
		dla_plot(xpxl1,ypxl1+1,R,G,B,fpart_(xend)*ygap);
		float interx = xend + gradient;

		yend = round_(Y2);
		xend = X2 + gradient*(yend - Y2);
		ygap = fpart_(Y2+0.5f);
		int ypxl2 = yend;
		int xpxl2 = ipart_(xend);
		dla_plot(xpxl2,ypxl2,R,G,B,rfpart_(xend)*ygap);
		dla_plot(xpxl2,ypxl2+1,R,G,B,fpart_(xend)*ygap);

		int y;
		for(y=ypxl1+1; y <= (ypxl2-1); y++) {
			dla_plot(ipart_(interx),y,R,G,B,rfpart_(interx));
			dla_plot(ipart_(interx)+1,y,R,G,B,fpart_(interx));
			interx += gradient;
		}
	}
}

void LCD_Rect(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, uint16_t Color) {
	LCD_HLine(X,X+W-1,Y,Color);
	LCD_HLine(X,X+W-1,Y+H-1,Color);
	LCD_VLine(X,Y,Y+H-1,Color);
	LCD_VLine(X+W-1,Y,Y+H-1,Color);
}

void LCD_FillRect(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, uint16_t Color) {
	uint32_t i = 0;
	LCD_SetWindow(X,Y,W,H);
	LCD_write_command(0x0022);
	for (i = 0; i < W*H; i++) { LCD_write_data(Color); }
}

void LCD_Ellipse(uint16_t X, uint16_t Y, uint16_t A, uint16_t B, uint16_t Color) {
	LCD_SetWindow(0,0,320,240);

	int16_t Xc = 0, Yc = B;
	long A2 = (long)A*A, B2 = (long)B*B;
	long C1 = -(A2/4 + A % 2 + B2);
	long C2 = -(B2/4 + B % 2 + A2);
	long C3 = -(B2/4 + B % 2);
	long t = -A2*Yc;
	long dXt = B2*Xc*2, dYt = -A2*Yc*2;
	long dXt2 = B2*2, dYt2 = A2*2;
	while (Yc >= 0 && Xc <= A) {
		LCD_Pixel(X+Xc,Y+Yc,Color);
		LCD_Pixel(X-Xc,Y+Yc,Color);
		LCD_Pixel(X+Xc,Y-Yc,Color);
		LCD_Pixel(X-Xc,Y-Yc,Color);
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

void LCD_FillEllipse(uint16_t X, uint16_t Y, uint16_t A, uint16_t B, uint16_t Color) {
	LCD_SetWindow(0,0,320,240);

	int16_t Xc = 0, Yc = B;
	long A2 = (long)A*A, B2 = (long)B*B;
	long C1 = -(A2/4 + A % 2 + B2);
	long C2 = -(B2/4 + B % 2 + A2);
	long C3 = -(B2/4 + B % 2);
	long t = -A2*Yc;
	long dXt = B2*Xc*2, dYt = -A2*Yc*2;
	long dXt2 = B2*2, dYt2 = A2*2;
	while (Yc >= 0 && Xc <= A) {
		LCD_HLine(X-Xc,X+Xc,Y+Yc,Color);
		LCD_HLine(X-Xc,X+Xc,Y-Yc,Color);
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

void LCD_PutChar(uint16_t X, uint16_t Y, uint8_t Char, uint16_t Color) {
	uint16_t i,j;
	uint8_t buffer[16],tmpCh;

    LCD_SetWindow(X,Y,8,16);
    memcpy(buffer,AsciiLib[Char-32],16);

    for (i = 0; i < 16; i++) {
    	tmpCh = buffer[i];
    	for (j = 0; j < 8; j++) {
    		if (((tmpCh >> (7-j)) & 0x01) == 0x01) LCD_Pixel(X+j,Y+i,Color);
    	}
    }
}

void LCD_PutCharO(uint16_t X, uint16_t Y, uint8_t Char, uint16_t Color, uint16_t bgColor) {
	uint16_t i,j;
	uint8_t buffer[16],tmpCh;

    LCD_SetWindow(X,Y,8,16);
    memcpy(buffer,AsciiLib[Char-32],16);

    for (i = 0; i < 16; i++) {
    	tmpCh = buffer[i];
    	for (j = 0; j < 8; j++) LCD_Pixel(X+j,Y+i,(((tmpCh >> (7-j)) & 0x01) == 0x01) ? Color : bgColor);
    }
}

void LCD_PutStr(uint16_t X, uint16_t Y, char *str, uint16_t Color) {
    while (*str) {
        LCD_PutChar(X,Y,*str++,Color);
        if (X < 320-8) { X += 8; } else if (Y < 240-16) { X = 0; Y += 16; } else { X = 0; Y = 0; }
    };
}

void LCD_PutStrO(uint16_t X, uint16_t Y, char *str, uint16_t Color, uint16_t bgColor) {
    while (*str) {
        LCD_PutCharO(X,Y,*str++,Color,bgColor);
        if (X < 320-8) { X += 8; } else if (Y < 240-16) { X = 0; Y += 16; } else { X = 0; Y = 0; }
    }
}

void LCD_PutInt(uint16_t X, uint16_t Y, uint32_t num, uint16_t Color) {
	char str[11]; // 10 chars max for UINT32_MAX
	int i = 0;
	do { str[i++] = num % 10 + '0'; } while ((num /= 10) > 0);
	int strLen = i;
	for (i--; i>=0; i--) LCD_PutChar(X+(strLen << 3)-(i << 3),Y,str[i],Color);
}

void LCD_PutIntO(uint16_t X, uint16_t Y, uint32_t num, uint16_t Color, uint16_t bgColor) {
	char str[11]; // 10 chars max for UINT32_MAX
	int i = 0;
	do { str[i++] = num % 10 + '0'; } while ((num /= 10) > 0);
	int strLen = i;
	for (i--; i>=0; i--) LCD_PutCharO(X+(strLen << 3)-(i << 3),Y,str[i],Color,bgColor);
}

void LCD_PutHex(uint16_t X, uint16_t Y, uint32_t num, uint16_t Color) {
	char str[11]; // 10 chars max for UINT32_MAX
	int i = 0;
	do { str[i++] = "0123456789ABCDEF"[num % 0x10]; } while ((num /= 0x10) > 0);
	str[i++] = 'x';
	str[i++] = '0';
	int strLen = i;
	for (i--; i>=0; i--) LCD_PutChar(X+(strLen << 3)-(i << 3),Y,str[i],Color);
}

void LCD_PutHexO(uint16_t X, uint16_t Y, uint32_t num, uint16_t Color, uint16_t bgColor) {
	char str[11]; // 10 chars max for UINT32_MAX
	int i = 0;
	do { str[i++] = "0123456789ABCDEF"[num % 0x10]; } while ((num /= 0x10) > 0);
	str[i++] = 'x';
	str[i++] = '0';
	int strLen = i;
	for (i--; i>=0; i--) LCD_PutCharO(X+(strLen << 3)-(i << 3),Y,str[i],Color,bgColor);
}

void LCD_BMPMono(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, const uint8_t* pBMP, uint16_t Color) {
	uint16_t i,j,k;
	uint8_t buffer[W];

	LCD_SetWindow(X,Y,W*8,H);
	for (i = 0; i < H; i++) {
		memcpy(buffer,&pBMP[i*W],W);
		for (j = 0; j < W; j++) {
			for (k = 0; k < 8; k++) if ((buffer[j] >> (7-k)) & 0x01) LCD_Pixel(X+(j << 3)+k,Y+i,Color);
		}
	}
}

void LCD_BMPMonoO(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, const uint8_t* pBMP, uint16_t Color, uint16_t bgColor) {
	uint16_t i,j,k;
	uint8_t buffer[W];

	LCD_SetWindow(X,Y,W*8,H);
	for (i = 0; i < H; i++) {
		memcpy(buffer,&pBMP[i*W],W);
		for (j = 0; j < W; j++) {
			for (k = 0; k < 8; k++) LCD_Pixel(X+(j << 3)+k,Y+i,((buffer[j] >> (7-k)) & 0x01) ? Color : bgColor);
		}
	}
}

void LCD_BMP(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, const uint16_t* pBMP) {
	uint16_t i,j;
	uint16_t buffer[W];

	LCD_SetWindow(X,Y,W,H);
	LCD_write_command(0x0022);
	for (i = 0; i < H; i++) {
		memcpy(buffer,&pBMP[i*W],W*2);
		for (j = 0; j < W; j++) LCD_write_data(buffer[j]);
	}
}
