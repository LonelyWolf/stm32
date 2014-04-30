#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_spi.h>
#include <string.h> // For memcpy

#include <delay.h>
#include <uc1701.h>
#include <font5x7.h>


uint16_t                  scr_width       = SCR_W;
uint16_t                  scr_height      = SCR_H;
ScrOrientation_TypeDef    scr_orientation = scr_normal;

uint8_t vRAM[SCR_W * SCR_H / 8]; // Display buffer


void UC1701_write(uint8_t data) {
#ifdef SOFT_SPI
	uint8_t i;

	for(i = 0; i < 8; i++) {
		if (data & 0x80) SDA_H(); else SDA_L();
		data = data << 1;
		SCK_L();
		SCK_H();
	}
#else
	while (SPI_I2S_GetFlagStatus(SPI_PORT,SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI_PORT,data);
#endif
}

void UC1701_cmd(uint8_t cmd) {
	RS_L();
	UC1701_write(cmd);
#ifndef SOFT_SPI
	while (SPI_I2S_GetFlagStatus(SPI_PORT,SPI_I2S_FLAG_BSY) == SET);
#endif
}

void UC1701_cmd_double(uint8_t cmd1, uint8_t cmd2) {
	RS_L();
	UC1701_write(cmd1);
#ifndef SOFT_SPI
	while (SPI_I2S_GetFlagStatus(SPI_PORT,SPI_I2S_FLAG_BSY) == SET);
#endif
	UC1701_write(cmd2);
#ifndef SOFT_SPI
	while (SPI_I2S_GetFlagStatus(SPI_PORT,SPI_I2S_FLAG_BSY) == SET);
#endif
}

void UC1701_data(uint8_t data) {
	RS_H();
	UC1701_write(data);
#ifndef SOFT_SPI
	while (SPI_I2S_GetFlagStatus(SPI_PORT,SPI_I2S_FLAG_BSY) == SET);
#endif
}

uint16_t RGB565(uint8_t R,uint8_t G,uint8_t B) {
	return ((R >> 3) << 11) | ((G >> 2) << 5) | (B >> 3);
}

void UC1701_Init(void) {
#ifdef SOFT_SPI
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); // Disable JTAG for use PB3
#else
	#if _SPI_PORT == 1
		// SPI1
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOA,ENABLE);
	#elif _SPI_PORT == 2
		// SPI2
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
	#elif _SPI_PORT == 3
		// SPI3
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,ENABLE);
		GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); // Disable JTAG for use PB3
	#endif

	// Configure and enable SPI
	SPI_InitTypeDef SPI;
	SPI.SPI_Mode = SPI_Mode_Master;
	SPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI.SPI_Direction = SPI_Direction_1Line_Tx;
	SPI.SPI_CPOL = SPI_CPOL_Low;
	SPI.SPI_CPHA = SPI_CPHA_1Edge;
	SPI.SPI_CRCPolynomial = 7;
	SPI.SPI_DataSize = SPI_DataSize_8b;
	SPI.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI.SPI_NSS = SPI_NSS_Soft;
	SPI_Init(SPI_PORT,&SPI);
	// NSS must be set to '1' due to NSS_Soft settings (otherwise it will be Multimaster mode).
	SPI_NSSInternalSoftwareConfig(SPI_PORT,SPI_NSSInternalSoft_Set);
	SPI_Cmd(SPI_PORT,ENABLE);
#endif

	GPIO_InitTypeDef PORT;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
#ifdef SOFT_SPI
	PORT.GPIO_Pin = UC1701_SDA_PIN;
	GPIO_Init(UC1701_SDA_PORT,&PORT);
	PORT.GPIO_Pin = UC1701_SCK_PIN;
	GPIO_Init(UC1701_SCK_PORT,&PORT);
#else
	// Configure SPI pins
	PORT.GPIO_Pin = SPI_SCK_PIN | SPI_MOSI_PIN;
	PORT.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SPI_GPIO_PORT,&PORT);
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
#endif

	PORT.GPIO_Pin = UC1701_CS_PIN;
	GPIO_Init(UC1701_CS_PORT,&PORT);
	PORT.GPIO_Pin = UC1701_RST_PIN;
	GPIO_Init(UC1701_RST_PORT,&PORT);
	PORT.GPIO_Pin = UC1701_RS_PIN;
	GPIO_Init(UC1701_RS_PORT,&PORT);

	// Reset display
	CS_H();
	RST_L();
	Delay_ms(1); // Must hold RST low at least 1ms
	RST_H();
	Delay_ms(5); // Wait at least 5ms
	CS_L();

	UC1701_cmd(0xe2); // Software system reset
	UC1701_cmd(0x2f); // Power control: Boost ON,  V.Regular ON,  V.Follower ON
	UC1701_cmd(0xa2); // Set LCD bias ratio (BR = 0)
	UC1701_cmd(0xaf); // Display enable

	UC1701_cmd_double(0xfa,0x93); // Advanced program control 0:
								  //   Temperature compensation -0.11%/C
								  //   PA wrap around enabled, CA wrap around enabled

	CS_H();
}

// Reset display registers
// Doesn't affect the display memory
void UC1701_Reset(void) {
	CS_L();
	UC1701_cmd(0xe2);
	CS_H();
}

// Set LCD contrast
// Input:
//   res_ratio: internal resistor ratio [0..7], power on value is 4
//   el_vol: electronic volume [0..63], power on value is 32
void UC1701_Contrast(uint8_t res_ratio, uint8_t el_vol) {
	CS_L();
	UC1701_cmd(0x20 | (res_ratio & 0x07));
	UC1701_cmd_double(0x81,el_vol & 0x3f);
	CS_H();
}

// Set all LCD pixels on or off
// Input:
//   state: ON or OFF
// Doesn't affect the display memory
void UC1701_SetAllPixelOn(OnOffStatus state) {
	CS_L();
	UC1701_cmd(state == ON ? 0xa5 : 0xa4);
	CS_H();
}

// Set inverse display pixels
// Input:
//   state: NORMAL or INVERT
void UC1701_SetInvert(InvertStatus state) {
	CS_L();
	UC1701_cmd(state == NORMAL ? 0xa6 : 0xa7);
	CS_H();
}

// Toggle display on/off
// Input:
//   state: ENABLED or DISABLED
// Doesn't affect the display memory
void UC1701_SetDisplayState(DisplayState state) {
	CS_L();
	UC1701_cmd(state == ENABLED ? 0xaf : 0xae);
	CS_H();
}

// Set X coordinate mapping (normal or mirrored)
// Input:
//   state: NORMAL or INVERT
// Doesn't affect the display memory
void UC1701_SetXDir(InvertStatus MX) {
	CS_L();
	UC1701_cmd(MX == NORMAL ? 0xa0 : 0xa1);
	CS_H();
}

// Set Y coordinate mapping (normal or mirrored)
// Input:
//   state: NORMAL or INVERT
// Doesn't affect the display memory
void UC1701_SetYDir(InvertStatus MY) {
	CS_L();
	UC1701_cmd(MY == NORMAL ? 0xc8 : 0xc0);
	CS_H();
}

// Set display column:page according to pixel coordinates
// Input:
//   X, Y - pixel coordinates
void UC1701_SetAddr(uint8_t X, uint8_t Y) {
	CS_L();
	UC1701_cmd(X & 0x0f); // Column address LSB
	UC1701_cmd((X > 4) | 0x10); // Column address MSB
	UC1701_cmd(((Y / 8) & 0x0f) | 0xb0); // Page
	CS_H();
}

// Set scroll line number
// Input:
//   line - start line number (0..63)
void UC1701_SetScrollLine(uint8_t line) {
	CS_L();
	UC1701_cmd(0x40 | (line & 0x3f));
	CS_H();
}

// Set display orientation
void UC1701_Orientation(uint8_t orientation) {
	CS_L();
	switch(orientation) {
	case scr_CW:
		scr_width  = SCR_H;
		scr_height = SCR_W;
		UC1701_SetXDir(NORMAL);
		UC1701_SetYDir(INVERT);
		break;
	case scr_CCW:
		scr_width  = SCR_H;
		scr_height = SCR_W;
		UC1701_SetXDir(INVERT);
		UC1701_SetYDir(NORMAL);
		break;
	case scr_180:
		scr_width  = SCR_W;
		scr_height = SCR_H;
		UC1701_SetXDir(INVERT);
		UC1701_SetYDir(INVERT);
		break;
	default:
		scr_width  = SCR_W;
		scr_height = SCR_H;
		UC1701_SetXDir(NORMAL);
		UC1701_SetYDir(NORMAL);
		break;
	}
	scr_orientation = orientation;
	CS_H();
}

// Send vRAM buffer content into display
void UC1701_Flush(void) {
	uint16_t i,j;

	CS_L();
	for (j = 0; j < 8; j++) {
		 // Column 0 address LSB
		if (scr_orientation == scr_180 || scr_orientation == scr_CCW) UC1701_cmd(0x04); else UC1701_cmd(0x00);
		UC1701_cmd(0x10); // Column 0 address MSB
		UC1701_cmd(0xb0 | j); // Page address
		for (i = 0; i < 128; i++) {
			UC1701_data(vRAM[(j * SCR_W) + i]);
		}
	}
	CS_H();
}

// Fill VRam memory with specified pattern
void UC1701_Fill(uint8_t pattern) {
	memset(vRAM,pattern,sizeof(vRAM));
//	for (i = 0; i < sizeof(vRAM); i++) vRAM[i] = pattern;
}

// Set pixel in vRAM buffer
void UC1701_SetPixel(uint8_t X, uint8_t Y) {
	uint8_t XX = X;
	uint8_t YY = Y;

	if (scr_orientation == scr_CW || scr_orientation == scr_CCW) {
		XX = Y; YY = X;
	}
	vRAM[((YY >> 3) * SCR_W) + XX] |= 1 << (YY % 8);
//	vRAM[((Y >> 3) * SCR_W) + X] |= 1 << (Y % 8);
}

// Clear pixel in vRAM buffer
void UC1701_ResetPixel(uint8_t X, uint8_t Y) {
	uint8_t XX = X;
	uint8_t YY = Y;

	if (scr_orientation == scr_CW || scr_orientation == scr_CCW) {
		XX = Y; YY = X;
	}
	vRAM[((YY >> 3) * SCR_W) + XX] &= ~(1 << (YY % 8));
//	vRAM[((Y >> 3) * SCR_W) + X] &= ~(1 << (Y % 8));
}

void UC1701_HLine(uint8_t X1, uint8_t X2, uint8_t Y, PSetReset_TypeDef SR) {
	uint8_t x;

	if (SR == PSet) {
		for (x = X1; x <= X2; x++) UC1701_SetPixel(x,Y);
	} else {
		for (x = X1; x <= X2; x++) UC1701_ResetPixel(x,Y);
	}
}

void UC1701_VLine(uint8_t X, uint8_t Y1, uint8_t Y2, PSetReset_TypeDef SR) {
	uint8_t y;

	if (SR == PSet) {
		for (y = Y1; y <= Y2; y++) UC1701_SetPixel(X,y);
	} else {
		for (y = Y1; y <= Y2; y++) UC1701_ResetPixel(X,y);
	}
}

void UC1701_Rect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2, PSetReset_TypeDef SR) {
	UC1701_HLine(X1,X2,Y1,SR);
	UC1701_HLine(X1,X2,Y2,SR);
	UC1701_VLine(X1,Y1 + 1,Y2 - 1,SR);
	UC1701_VLine(X2,Y1 + 1,Y2 - 1,SR);
}

void UC1701_FillRect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2, PSetReset_TypeDef SR) {
	uint8_t y;

	for (y = Y1; y <= Y2; y++) UC1701_HLine(X1,X2,y,SR);
}

void UC1701_Line(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2) {
	int16_t dX = X2-X1;
	int16_t dY = Y2-Y1;
	int16_t dXsym = (dX > 0) ? 1 : -1;
	int16_t dYsym = (dY > 0) ? 1 : -1;

	if (dX == 0) {
		if (Y2 > Y1) UC1701_VLine(X1,Y1,Y2,PSet); else UC1701_VLine(X1,Y2,Y1,PSet);
		return;
	}
	if (dY == 0) {
		if (X2 > X1) UC1701_HLine(X1,X2,Y1,PSet); else UC1701_HLine(X2,X1,Y1,PSet);
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
			UC1701_SetPixel(X1,Y1);
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
			UC1701_SetPixel(X1,Y1);
			Y1 += dYsym;
			if (di < 0) {
				di += dX2;
			} else {
				di += dX2 - dY2;
				X1 += dXsym;
			}
		}
	}
	UC1701_SetPixel(X1,Y1);
}

void UC1701_Ellipse(uint16_t X, uint16_t Y, uint16_t A, uint16_t B) {
	int16_t Xc = 0, Yc = B;

	long A2 = (long)A*A, B2 = (long)B*B;
	long C1 = -(A2/4 + A % 2 + B2);
	long C2 = -(B2/4 + B % 2 + A2);
	long C3 = -(B2/4 + B % 2);
	long t = -A2 * Yc;
	long dXt = B2*Xc*2, dYt = -A2*Yc*2;
	long dXt2 = B2*2, dYt2 = A2*2;
	while (Yc >= 0 && Xc <= A) {
		UC1701_SetPixel(X + Xc,Y + Yc);
		if (Xc != 0 || Yc != 0) UC1701_SetPixel(X - Xc,Y - Yc);
		if (Xc != 0 && Yc != 0) {
			UC1701_SetPixel(X + Xc,Y - Yc);
			UC1701_SetPixel(X - Xc,Y + Yc);
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

void UC1701_PutChar5x7(uint8_t X, uint8_t Y, uint8_t Char, Opaque_TypeDef bckgnd) {
	uint16_t i,j;
	uint8_t buffer[5],tmpCh;

	memcpy(buffer,&Font5x7[(Char - 32) * 5],5);

	if (bckgnd == opaque) {
	    for (i = 0; i < 5; i++) {
	    	tmpCh = buffer[i];
	    	for (j = 0; j < 8; j++) {
	    		if ((tmpCh >> j) & 0x01) UC1701_SetPixel(X + i,Y + j); else UC1701_ResetPixel(X + i,Y + j);
	    	}
	    }
	} else {
		for (i = 0; i < 5; i++) {
			tmpCh = buffer[i];
			for (j = 0; j < 8; j++) {
				if ((tmpCh >> j) & 0x01) UC1701_SetPixel(X + i,Y + j);
			}
		}
	}
}

void UC1701_PutStr5x7(uint8_t X, uint8_t Y, char *str, Opaque_TypeDef bckgnd) {
    while (*str) {
        UC1701_PutChar5x7(X,Y,*str++,bckgnd);
        if (X < scr_width - 6) { X += 6; } else if (Y < scr_height - 8) { X = 0; Y += 8; } else { X = 0; Y = 0; }
    };
}

void UC1701_PutInt5x7(uint8_t X, uint8_t Y, uint32_t num, Opaque_TypeDef bckgnd) {
	char str[11]; // 10 chars max for UINT32_MAX
	int i = 0;

	do { str[i++] = num % 10 + '0'; } while ((num /= 10) > 0);
	int strLen = i;
	for (i--; i >= 0; i--) UC1701_PutChar5x7(X + (strLen * 6) - ((i + 1) * 6),Y,str[i],bckgnd);
}

void UC1701_PutHex5x7(uint8_t X, uint8_t Y, uint32_t num, Opaque_TypeDef bckgnd) {
	char str[11]; // 10 chars max for UINT32_MAX
	int i = 0;

	do { str[i++] = "0123456789ABCDEF"[num % 0x10]; } while ((num /= 0x10) > 0);
	str[i++] = 'x';
	str[i++] = '0';

	int strLen = i;

	for (i--; i >= 0; i--) UC1701_PutChar5x7(X + (strLen * 6) - ((i + 1) * 6),Y,str[i],bckgnd);
}
