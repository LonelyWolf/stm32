#include <stm32l1xx_rcc.h>
#include <stm32l1xx_gpio.h>

#include "delay.h"
#include "spi.h"

#include "ST7541.h"


GrayScale_TypeDef         lcd_color       = gs_black; // Foreground color
uint16_t                  scr_width       = SCR_W; // Screen width
uint16_t                  scr_height      = SCR_H; // Screen height
ScrOrientation_TypeDef    scr_orientation = scr_normal; // Screen orientation

// Display buffer
uint8_t vRAM[(SCR_W * SCR_H) >> 2] __attribute__((aligned(4)));

// Grayscale palette (PWM values for white, light gray, dark gray, black)
uint8_t const GrayPalette[] = {0x00,0x00,0x99,0x99,0xcc,0xcc,0xff,0xff}; // 15PWM
//uint8_t const GrayPalette[] = {0x00,0x00,0xaa,0xaa,0xdd,0xdd,0xff,0xff}; // 15PWM
//uint8_t const GrayPalette[] = {0x00,0x00,0xdd,0xdd,0xee,0xee,0xff,0xff}; // 15PWM
//uint8_t const GrayPalette[] = {0x00,0x00,0x55,0x55,0xaa,0xaa,0xcc,0xcc}; // 12PWM
//uint8_t const GrayPalette[] = {0x00,0x00,0x77,0x77,0x88,0x88,0x99,0x99}; // 9PWM


// Send single byte command to display
// input:
//   cmd - display command
void ST7541_cmd(uint8_t cmd) {
	ST7541_RS_L();
	SPIx_Send(&ST7541_SPI_PORT,cmd);
}

// Send double byte command to display
// input:
//   cmd1 - first byte of display command
//   cmd2 - first byte of display command
void ST7541_cmd_double(uint8_t cmd1, uint8_t cmd2) {
	ST7541_RS_L();
	SPIx_Send(&ST7541_SPI_PORT,cmd1);
	SPIx_Send(&ST7541_SPI_PORT,cmd2);
}

// Send data byte to display
// input:
//   data - data byte
void ST7541_data(uint8_t data) {
	ST7541_RS_H();
	SPIx_Send(&ST7541_SPI_PORT,data);
}

// Initialize the display control GPIO pins
void ST7541_InitGPIO(void) {
	GPIO_InitTypeDef PORT;

	// Enable the GPIO peripheral(s) clock
	RCC->AHBENR |= ST7541_GPIO_PERIPH;

	// Configure the GPIO pins
	PORT.GPIO_Mode = GPIO_Mode_OUT;
	PORT.GPIO_OType = GPIO_OType_PP;
	PORT.GPIO_PuPd = GPIO_PuPd_UP;
	PORT.GPIO_Speed = GPIO_Speed_40MHz;

	ST7541_CS_H();
	PORT.GPIO_Pin = ST7541_CS_PIN;
	GPIO_Init(ST7541_CS_PORT,&PORT);

	ST7541_RS_L();
	PORT.GPIO_Pin = ST7541_RS_PIN;
	GPIO_Init(ST7541_RS_PORT,&PORT);

	ST7541_RST_H();
	PORT.GPIO_Pin = ST7541_RST_PIN;
	GPIO_Init(ST7541_RST_PORT,&PORT);
}

// Initialize SPI peripheral and ST7541 display
// note: SPI peripheral must be initialized before
//       Delay must be initialized before
void ST7541_Init(void) {
	uint8_t i;

	// Reset display
	ST7541_CS_H();
	ST7541_RST_L();
	Delay_ms(10);
	ST7541_RST_H();
	Delay_ms(10);
	ST7541_CS_L();

	ST7541_cmd(0xc8); // COM reverse direction
	ST7541_cmd(0xa1); // ADC reverse direction
	ST7541_cmd_double(0x40,0); // Initial display line 0
	ST7541_cmd_double(0x44,0); // Initial COM0 0
	ST7541_cmd_double(0x48,128); // Duty cycle 128

	ST7541_cmd(0xab); // Enable built-in oscillator circuit

//	ST7541_cmd(0x20); // Regulator resistor: 2.3
//	ST7541_cmd(0x21); // Regulator resistor: 3.0
//	ST7541_cmd(0x22); // Regulator resistor: 3.7
//	ST7541_cmd(0x23); // Regulator resistor: 4.4
//	ST7541_cmd(0x24); // Regulator resistor: 5.1
//	ST7541_cmd(0x25); // Regulator resistor: 5.8
//	ST7541_cmd(0x26); // Regulator resistor: 6.5
	ST7541_cmd(0x27); // Regulator resistor: 7.2

//	ST7541_cmd_double(0x81,26); // Contrast
	ST7541_cmd_double(0x81,32); // Contrast
//	ST7541_cmd_double(0x81,41); // Contrast

//	ST7541_cmd(0x50); // LCD bias: 1/5
//	ST7541_cmd(0x51); // LCD bias: 1/6
//	ST7541_cmd(0x52); // LCD bias: 1/7
//	ST7541_cmd(0x53); // LCD bias: 1/8
//	ST7541_cmd(0x54); // LCD bias: 1/9
//	ST7541_cmd(0x55); // LCD bias: 1/10
//	ST7541_cmd(0x56); // LCD bias: 1/11
	ST7541_cmd(0x57); // LCD bias: 1/12

//	ST7541_cmd_double(0x38,0xf4); // Frame rate 123Hz, Booster efficiency level 2
//	ST7541_cmd_double(0x38,0xf0); // Frame rate 123Hz, Booster efficiency level 0
//	ST7541_cmd_double(0x38,0x54); // Frame rate 67Hz, Booster efficiency level 2
	ST7541_cmd_double(0x38,0x04); // Frame rate 77Hz, Booster efficiency level 2
//	ST7541_cmd_double(0x38,0x00); // Frame rate 77Hz, Booster efficiency level 0
//	ST7541_cmd_double(0x38,0x14); // Frame rate 51Hz, Booster efficiency level 2
//	ST7541_cmd_double(0x38,0x74); // Frame rate 70Hz, Booster efficiency level 2

	ST7541_cmd(0x2e); // Power control: VC,VR,VF = 1,1,0 (internal voltage booster)
//	ST7541_cmd(0x2a); // Power control: VC,VR,VF = 0,1,0 (external LCD bias supply)
	Delay_ms(10);
	ST7541_cmd(0x2f); // Power control: VC,VR,VF = 1,1,1 (internal voltage booster)
//	ST7541_cmd(0x2b); // Power control: VC,VR,VF = 0,1,1 (external LCD bias supply)

//	ST7541_cmd(0x64); // DC-DC converter: 3 times boosting circuit
//	ST7541_cmd(0x65); // DC-DC converter: 4 times boosting circuit
	ST7541_cmd(0x66); // DC-DC converter: 5 times boosting circuit
//	ST7541_cmd(0x67); // DC-DC converter: 6 times boosting circuit

//	ST7541_cmd(0x90); // FRC/PWM mode: 4FRC, 9PWM
//	ST7541_cmd(0x92); // FRC/PWM mode: 4FRC, 12PWM
//	ST7541_cmd(0x93); // FRC/PWM mode: 4FRC, 15PWM
//	ST7541_cmd(0x94); // FRC/PWM mode: 3FRC, 9PWM
//	ST7541_cmd(0x96); // FRC/PWM mode: 3FRC, 12PWM
	ST7541_cmd(0x97); // FRC/PWM mode: 3FRC, 15PWM

	// Enable high power mode
//	ST7541_cmd_double(0xf7,0x1a); // High power mode enable
//	ST7541_cmd_double(0xf3,0x0d); // High power mode control

	// Load grayscale palette
	for (i = 0; i < 8; i++)	ST7541_cmd_double(0x88 + i,GrayPalette[i]);

	ST7541_cmd(0xaf); // Display ON

	ST7541_Orientation(scr_normal);

	ST7541_CS_H();
}

// Reset display registers
// Doesn't affect the display memory and some registers
void ST7541_Reset(void) {
	ST7541_CS_L();
	ST7541_cmd(0xe2);
	ST7541_CS_H();
}

// Set LCD contrast
// input:
//   res_ratio: internal regulator resistor ratio [0..7], power on value is 0 => 2.3
//   lcd_bias: LCD bias [0..7], power on value is 7 => 1/12
//   el_vol: electronic volume [0..63], power on value is 32
void ST7541_Contrast(uint8_t res_ratio, uint8_t lcd_bias, uint8_t el_vol) {
	ST7541_CS_L();
	ST7541_cmd(0x20 | (res_ratio & 0x07)); // Regulator resistor ratio
	ST7541_cmd(0x50 | (lcd_bias & 0x07)); // LCD bias
	ST7541_cmd_double(0x81,el_vol & 0x3f); // Electronic volume
	ST7541_CS_H();
}

// Set all LCD pixels on or off
// Input:
//   state: ON or OFF
// Doesn't affect the display memory
void ST7541_SetAllPixelOn(OnOffStatus state) {
	ST7541_CS_L();
	ST7541_cmd(state == ON ? 0xa5 : 0xa4);
	ST7541_CS_H();
}

// Set inverse display pixels
// Input:
//   state - NORMAL or INVERT
void ST7541_SetInvert(InvertStatus state) {
	ST7541_CS_L();
	ST7541_cmd(state == NORMAL ? 0xa6 : 0xa7);
	ST7541_CS_H();
}

// Toggle display on/off
// Input:
//   state - ENABLED or DISABLED
// Doesn't affect the display memory
void ST7541_SetDisplayState(DisplayState state) {
	ST7541_CS_L();
	ST7541_cmd(state == ENABLED ? 0xaf : 0xae);
	ST7541_CS_H();
}

// Configure partial display on LCD
// input:
//   COM - initial display COM
//   Line - initial display line
//   Duty - partial display duty
void ST7541_SetDisplayPartial(uint8_t COM, uint8_t Line, uint8_t Duty) {
	ST7541_CS_L();
	ST7541_cmd_double(0x40,Line & 0x7f); // Line
	ST7541_cmd_double(0x44,COM & 0x7f);  // COM
	ST7541_cmd_double(0x48,Duty);        // Duty
	ST7541_CS_H();
}

// Enter display power save mode
// input:
//   state - ON puts display to sleep mode, OFF - returns to normal mode
void ST7541_PowerSave(OnOffStatus state) {
	ST7541_CS_L();
	ST7541_cmd(state == ON ? 0xa9 : 0xa8);
	ST7541_CS_H();
}

// Set X coordinate mapping (normal or mirrored)
// Input:
//   state: NORMAL or INVERT
// Doesn't affect the display memory
void ST7541_SetXDir(InvertStatus MX) {
	ST7541_CS_L();
	ST7541_cmd(MX == NORMAL ? 0xa0 : 0xa1); // SEG direction select
	ST7541_CS_H();
}

// Set Y coordinate mapping (normal or mirrored)
// Input:
//   state: NORMAL or INVERT
// Doesn't affect the display memory
void ST7541_SetYDir(InvertStatus MY) {
	ST7541_CS_L();
	ST7541_cmd(MY == NORMAL ? 0xc8 : 0xc0); // COM direction select
	ST7541_CS_H();
}

// Set display column:page according to pixel coordinates
// Input:
//   X, Y - pixel coordinates
void ST7541_SetAddr(uint8_t X, uint8_t Y) {
	ST7541_CS_L();
	ST7541_cmd(X & 0x0f); // Column address LSB
	ST7541_cmd(0x10 | ((X > 4) & 0x07)); // Column address MSB
	ST7541_cmd(0xb0 | ((Y / 8) & 0x0f)); // Page address
	ST7541_CS_H();
}

// Set scroll line number
// Input:
//   line - start line number (0..127)
void ST7541_SetScrollLine(uint8_t line) {
	ST7541_CS_L();
	ST7541_cmd_double(0x40,line & 0x7f); // Set initial display line
	ST7541_CS_H();
}

// Set display orientation
void ST7541_Orientation(uint8_t orientation) {
	ST7541_CS_L();
	switch(orientation) {
	case scr_CW:
		scr_width  = SCR_H;
		scr_height = SCR_W;
		ST7541_SetXDir(NORMAL);
		ST7541_SetYDir(NORMAL);
		break;
	case scr_CCW:
		scr_width  = SCR_W;
		scr_height = SCR_H;
		ST7541_SetXDir(INVERT);
		ST7541_SetYDir(INVERT);
		break;
	case scr_180:
		scr_width  = SCR_W;
		scr_height = SCR_H;
		ST7541_SetXDir(NORMAL);
		ST7541_SetYDir(INVERT);
		break;
	default:
		scr_width  = SCR_H;
		scr_height = SCR_W;
		ST7541_SetXDir(INVERT);
		ST7541_SetYDir(NORMAL);
		break;
	}
	scr_orientation = orientation;
	ST7541_CS_H();
}

// Send vRAM buffer into display
void ST7541_Flush(void) {
	// Send video buffer with SPI 16-bit frame
	ST7541_SetAddr(0,0);
	ST7541_CS_L();
	ST7541_RS_H();
	// Disable the SPI peripheral, set 16-bit data frame format and then enable the SPI back
	ST7541_SPI_PORT.Instance->CR1 &= ~SPI_CR1_SPE;
	ST7541_SPI_PORT.Instance->CR1 |= SPI_CR1_DFF | SPI_CR1_SPE;
	// Send buffer
	SPIx_SendBuf16(&ST7541_SPI_PORT,(uint16_t *)&vRAM[0],(SCR_W * SCR_H) >> 3);
	ST7541_CS_H();
	// Disable the SPI peripheral, set 8-bit data frame format and then enable the SPI back
	ST7541_SPI_PORT.Instance->CR1 &= ~(SPI_CR1_DFF | SPI_CR1_SPE);
	ST7541_SPI_PORT.Instance->CR1 |= SPI_CR1_SPE;

/*
	// Send video buffer with SPI 8-bit frame
	ST7541_SetAddr(0,0);
	ST7541_CS_L();
	ST7541_RS_H();
	SPIx_SendBuf(ST7541_SPI_PORT,vRAM,(SCR_W * SCR_H) >> 2);
	ST7541_CS_H();
*/
}

// Send vRAM buffer into display using DMA
// input:
//   blocking: blocking or non-blocking operation (BLOCK/NOBLOCK)
// note: in case of blocking operation this function waits for the end of transmit
//       in case of non-blocking operation the application must deassert the CS pin
//       after end of transmit (or call the ST7541_Wait_Flush())
void ST7541_Flush_DMA(BlockingState blocking) {
	ST7541_SetAddr(0,0);
	ST7541_CS_L();
	ST7541_RS_H();
	// Configure the DMA transfer
	SPIx_Configure_DMA_TX(&ST7541_SPI_PORT,vRAM,(SCR_W * SCR_H) >> 2);
	// Enable the DMA channel
	SPIx_SetDMA(&ST7541_SPI_PORT,SPI_DMA_TX,ENABLE);
	if (blocking == BLOCK) {
		// Wait while DMA transaction ongoing
		while (ST7541_SPI_PORT.DMA_TX.State == DMA_STATE_BUSY);
	}
}

// Fill vRAM memory with specified pattern
// input:
//   pattern - byte to fill vRAM buffer
void ST7541_Fill(uint16_t pattern) {
	register uint16_t i = 0;
	register uint8_t b0 = (uint8_t)pattern;
	register uint8_t b1 = (uint8_t)(pattern >> 8);

	do {
		vRAM[i++] = b1;
		vRAM[i++] = b0;
	} while (i < ((SCR_W * SCR_H) >> 2));
}

// Set pixel in vRAM buffer
// input:
//   X, Y - pixel coordinates
//   GS - grayscale pixel color (gs_[white,ltgray,dkgray,black])
// note: defining this function as an 'inline' increases the code size but also improves performance
__attribute__((always_inline)) void Pixel(uint8_t X, uint8_t Y, GrayScale_TypeDef GS) {
	uint32_t *pvRAM_BB;

	// Offset of pixel in the vRAM array must be computed by formula ((Y >> 3) * (SCR_W << 1)) + (X << 1)
	// But ST7541 screen width 128, therefore formula can be simplified to ((Y >> 3) << 8) + (X << 1)

	// Compute RAM address for bit-banding
	if (scr_orientation == scr_CW || scr_orientation == scr_CCW) {
		// Swap X and Y coordinates if screen rotated for 90 degrees (clockwise or counter-clockwise)
		pvRAM_BB = (uint32_t*)(SRAM_BB_BASE + (((uint32_t)((void*)(&vRAM[((X >> 3) << 8) + (Y << 1)])) - SRAM_BASE) << 5) + (((uint32_t)(X % 8)) << 2));
	} else {
		pvRAM_BB = (uint32_t*)(SRAM_BB_BASE + (((uint32_t)((void*)(&vRAM[((Y >> 3) << 8) + (X << 1)])) - SRAM_BASE) << 5) + (((uint32_t)(Y % 8)) << 2));
	}

	// Set bits in vRAM according to specified color
	if (GS == gs_ltgray) {
		*pvRAM_BB = 0; *(pvRAM_BB + 8) = 1;
	} else if (GS == gs_black){
		*pvRAM_BB = 1; *(pvRAM_BB + 8) = 1;
	} else if (GS == gs_dkgray) {
		*pvRAM_BB = 1; *(pvRAM_BB + 8) = 0;
	} else {
		// gs_white (in case of invalid GS value pixel will be cleared)
		*pvRAM_BB = 0; *(pvRAM_BB + 8) = 0;
	}

/*
	// Draw pixel without bit-banding (add 88 bytes of code compared to bit-banding)
	uint8_t  XX;
	uint8_t  YY;
	uint8_t  bit;
	uint16_t *pvRAM;

	if (scr_orientation == scr_CW || scr_orientation == scr_CCW) {
		// Swap coordinates if screen rotated to 90 degrees
		XX = Y;
		YY = X;
	} else {
		XX = X;
		YY = Y;
	}

	// Vertical shift in byte of video buffer
	bit = 1 << (YY % 8);

	// Calculate offset in video buffer
	pvRAM = (uint16_t *)&vRAM[((YY >> 3) << 8) + (XX << 1)]; // screen width 128, therefore this a bit faster

	if (GS == gs_white) {
		*pvRAM &= ~(bit | (bit << 8));
	} else if (GS == gs_black) {
		*pvRAM |=  (bit | (bit << 8));
	} else if (GS == gs_dkgray) {
		*pvRAM &= ~(bit | (bit << 8));
		*pvRAM |=   bit;
	} else {
		// gs_ltgray
		*pvRAM &= ~(bit | (bit << 8));
		*pvRAM |=  (bit << 8);
	}
*/
}

// Draw horizontal line
// input:
//   X1, X2 - left and right horizontal coordinates (X1 must be less than X2)
//   Y - vertical coordinate
//   GS - grayscale pixel color
void HLine(uint8_t X1, uint8_t X2, uint8_t Y, GrayScale_TypeDef GS) {
	register uint8_t X,eX;

	if (X1 > X2) {
		X = X1; eX = X2;
	} else {
		X = X2; eX = X1;
	}
	do {
		Pixel(X,Y,GS);
	} while (X-- > eX);
}

// Draw vertical line
// input:
//   X - horizontal coordinate
//   Y1,Y2 - top and bottom vertical coordinates (Y1 must be less than Y2)
//   GS - grayscale pixel color
void VLine(uint8_t X, uint8_t Y1, uint8_t Y2, GrayScale_TypeDef GS) {
	register uint8_t Y,eY;

	if (Y1 > Y2) {
		Y = Y1; eY = Y2;
	} else {
		Y = Y2; eY = Y1;
	}
	do {
		Pixel(X,Y,GS);
	} while (Y-- > eY);
}

// Draw rectangle
// input:
//   X1,Y1 - top left coordinates
//   X2,Y2 - bottom right coordinates
//   GS - grayscale pixel color
void Rect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2, GrayScale_TypeDef GS) {
	HLine(X1,X2,Y1,GS);
	HLine(X1,X2,Y2,GS);
	VLine(X1,Y1 + 1,Y2 - 1,GS);
	VLine(X2,Y1 + 1,Y2 - 1,GS);
}

// Draw filled rectangle
// input:
//   X1,Y1 - top left coordinates
//   X2,Y2 - bottom right coordinates
//   GS - grayscale pixel color
void FillRect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2, GrayScale_TypeDef GS) {
	uint8_t Y;

	for (Y = Y1; Y <= Y2; Y++) HLine(X1,X2,Y,GS);
}

// Draw line
// input:
//    X1,Y1 - top left coordinates
//    X2,Y2 - bottom right coordinates
//    GS - grayscale pixel color
void Line(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, GrayScale_TypeDef GS) {
	int16_t dX = X2-X1;
	int16_t dY = Y2-Y1;
	int16_t dXsym = (dX > 0) ? 1 : -1;
	int16_t dYsym = (dY > 0) ? 1 : -1;

	if (dX == 0) {
		VLine(X1,Y1,Y2,GS);

		return;
	}
	if (dY == 0) {
		HLine(X1,X2,Y1,GS);

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
			Pixel(X1,Y1,GS);
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
			Pixel(X1,Y1,GS);
			Y1 += dYsym;
			if (di < 0) {
				di += dX2;
			} else {
				di += dX2 - dY2;
				X1 += dXsym;
			}
		}
	}
	Pixel(X1,Y1,GS);
}

// Draw ellipse
// input:
//   X,Y - coordinates of center of the ellipse
//   A,B - horizontal and vertical radiuses
//    GS - grayscale pixel color
void Ellipse(uint16_t X, uint16_t Y, uint16_t A, uint16_t B, GrayScale_TypeDef GS) {
	int16_t Xc = 0, Yc = B;

	long A2 = (long)A*A, B2 = (long)B*B;
	long C1 = -((A2 >> 2) + A % 2 + B2);
	long C2 = -((B2 >> 2) + B % 2 + A2);
	long C3 = -((B2 >> 2) + B % 2);
	long t = -A2 * Yc;
	long dXt = B2*Xc*2, dYt = -A2*Yc*2;
	long dXt2 = B2*2, dYt2 = A2*2;

	while (Yc >= 0 && Xc <= A) {
		Pixel(X + Xc,Y + Yc,GS);
		if (Xc != 0 || Yc != 0) Pixel(X - Xc,Y - Yc,GS);
		if (Xc != 0 && Yc != 0) {
			Pixel(X + Xc,Y - Yc,GS);
			Pixel(X - Xc,Y + Yc,GS);
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

// Draw single character
// input:
//   X,Y - character top left corner coordinates
//   Char - character to be drawn
//   Font - pointer to font
// return: character width in pixels
uint8_t DrawChar(uint8_t X, uint8_t Y, uint8_t Char, const Font_TypeDef *Font) {
	register uint8_t pX;
	register uint8_t pY;
	register uint8_t tmpCh;
	register uint8_t bL;
	const uint8_t *pCh;

	// If the specified character code is out of bounds should substitute the code of the "unknown" character
	if (Char < Font->font_MinChar || Char > Font->font_MaxChar) Char = Font->font_UnknownChar;

	// Pointer to the first byte of character in font data array
	pCh = &Font->font_Data[(Char - Font->font_MinChar) * Font->font_BPC];

	// Draw character
	if (Font->font_Scan == font_V) {
		// Vertical pixels order
		if (Font->font_Height < 9) {
			// Height is 8 pixels or less (one byte per column)
			pX = X;
			while (pX < X + Font->font_Width) {
				pY = Y;
				tmpCh = *pCh++;
				while (tmpCh) {
					if (tmpCh & 0x01) Pixel(pX,pY,lcd_color);
					tmpCh >>= 1;
					pY++;
				}
				pX++;
			}
		} else {
			// Height is more than 8 pixels (several bytes per column)
			pX = X;
			while (pX < X + Font->font_Width) {
				pY = Y;
				while (pY < Y + Font->font_Height) {
					bL = 8;
					tmpCh = *pCh++;
					if (tmpCh) {
						while (bL) {
							if (tmpCh & 0x01) Pixel(pX,pY,lcd_color);
							tmpCh >>= 1;
							if (tmpCh) {
								pY++;
								bL--;
							} else {
								pY += bL;
								break;
							}
						}
					} else {
						pY += bL;
					}
				}
				pX++;
			}
		}
	} else {
		// Horizontal pixels order
		if (Font->font_Width < 9) {
			// Width is 8 pixels or less (one byte per row)
			pY = Y;
			while (pY < Y + Font->font_Height) {
				pX = X;
				tmpCh = *pCh++;
				while (tmpCh) {
					if (tmpCh & 0x01) Pixel(pX,pY,lcd_color);
					tmpCh >>= 1;
					pX++;
				}
				pY++;
			}
		} else {
			// Width is more than 8 pixels (several bytes per row)
			pY = Y;
			while (pY < Y + Font->font_Height) {
				pX = X;
				while (pX < X + Font->font_Width) {
					bL = 8;
					tmpCh = *pCh++;
					if (tmpCh) {
						while (bL) {
							if (tmpCh & 0x01) Pixel(pX,pY,lcd_color);
							tmpCh >>= 1;
							if (tmpCh) {
								pX++;
								bL--;
							} else {
								pX += bL;
								break;
							}
						}
					} else {
						pX += bL;
					}
				}
				pY++;
			}
		}
	}

	return Font->font_Width + 1;
}

// Draw string
// input:
//   X,Y - top left coordinates of first character
//   str - pointer to zero-terminated string
//   Font - pointer to font
// return: string width in pixels
uint16_t PutStr(uint8_t X, uint8_t Y, const char *str, const Font_TypeDef *Font) {
	register uint8_t pX = X;
	register uint8_t eX = scr_width - Font->font_Width - 1;

	while (*str) {
		pX += DrawChar(pX,Y,*str++,Font);
		if (pX > eX) break;
	}

	return (pX - X);
}

// Draw string with line feed (by screen edge)
// input:
//   X,Y - top left coordinates of first character
//   str - pointer to zero-terminated string
//   Font - pointer to font
// return: string width in pixels
uint16_t PutStrLF(uint8_t X, uint8_t Y, const char *str, const Font_TypeDef *Font) {
	uint8_t strLen = 0;

    while (*str) {
        DrawChar(X,Y,*str++,Font);
        if (X < scr_width - Font->font_Width - 1) {
        	X += Font->font_Width + 1;
        } else if (Y < scr_height - Font->font_Height - 1) {
        	X = 0; Y += Font->font_Height;
        } else {
        	X = 0; Y = 0;
        }
        strLen++;
    };

    return strLen * (Font->font_Width + 1);
}

// Draw signed integer value
// input:
//   X,Y - top left coordinates of first symbol
//   num - signed integer value
//   Font - pointer to font
// return: number width in pixels
uint8_t PutInt(uint8_t X, uint8_t Y, int32_t num, const Font_TypeDef *Font) {
	uint8_t str[11]; // 10 chars max for INT32_MIN..INT32_MAX (without sign)
	register uint8_t *pStr = str;
	register uint8_t pX = X;
	register uint8_t neg = 0;

	// Convert number to characters
	*pStr++ = 0; // String termination character
	if (num < 0) {
		neg = 1;
		num *= -1;
	}
	do { *pStr++ = (num % 10) + '0'; } while (num /= 10);

	// Draw a number
	if (neg) pX += DrawChar(pX,Y,'-',Font);
	while (*--pStr) pX += DrawChar(pX,Y,*pStr,Font);

	return (pX - X);
}

// Draw unsigned integer value
// input:
//   X,Y - top left coordinates of first symbol
//   num - unsigned integer value
//   Font - pointer to font
// return: number width in pixels
uint8_t PutIntU(uint8_t X, uint8_t Y, uint32_t num, const Font_TypeDef *Font) {
	uint8_t str[11]; // 10 chars max for UINT32_MAX
	register uint8_t *pStr = str;
	register uint8_t pX = X;

	// Convert number to characters
	*pStr++ = 0; // String termination character
	do { *pStr++ = (num % 10) + '0'; } while (num /= 10);

	// Draw a number
	while (*--pStr) pX += DrawChar(pX,Y,*pStr,Font);

	return (pX - X);
}

// Draw signed integer value with decimal point
// input:
//   X,Y - top left coordinates of first symbol
//   num - unsigned integer value
//   decimals - number of digits after decimal point
//   Font - pointer to font
// return: number width in pixels
uint8_t PutIntF(uint8_t X, uint8_t Y, int32_t num, uint8_t decimals, const Font_TypeDef *Font) {
	uint8_t str[11]; // 10 chars max for INT32_MIN..INT32_MAX (without sign)
	register uint8_t *pStr = str;
	register uint8_t pX = X;
	register uint8_t neg = 0;
	register uint8_t strLen = 0;

	// Convert number to characters
	*pStr++ = 0; // String termination character
	if (num < 0) {
		neg = 1;
		num *= -1;
	}
	do {
		*pStr++ = (num % 10) + '0';
		strLen++;
	} while (num /= 10);

	// Add zeroes after the decimal point
	if (strLen <= decimals) while (strLen++ <= decimals) *pStr++ = '0';

	// Draw a number
	if (neg) pX += DrawChar(pX,Y,'-',Font);
	while (*--pStr) {
		pX += DrawChar(pX,Y,*pStr,Font);
		if ((--strLen == decimals) && (decimals)) {
			// Draw decimal point
			Rect(pX,Y + Font->font_Height - 2,pX + 1,Y + Font->font_Height - 1,lcd_color);
			pX += 3;
		}
	}

	return (pX - X);
}

// Draw signed integer value with leading zeros
// input:
//   X,Y - top left coordinates of first symbol
//   num - unsigned integer value
//   digits - minimal number of length (e.g. num=35, digits=5 --> 00035)
//   Font - pointer to font
// return: number width in pixels
uint8_t PutIntLZ(uint8_t X, uint8_t Y, int32_t num, uint8_t digits, const Font_TypeDef *Font) {
	uint8_t str[11]; // 10 chars max for INT32_MIN..INT32_MAX (without sign)
	register uint8_t *pStr = str;
	register uint8_t pX = X;
	register uint8_t neg = 0;
	register uint8_t strLen = 0;

	// Convert number to characters
	*pStr++ = 0; // String termination character
	if (num < 0) {
		neg = 1;
		num *= -1;
	}
	do {
		*pStr++ = (num % 10) + '0';
		strLen++;
	} while (num /= 10);

	// Add leading zeroes
	if (strLen < digits) while (strLen++ < digits) *pStr++ = '0';

	// Draw a number
	if (neg) pX += DrawChar(pX,Y,'-',Font);
	while (*--pStr) pX += DrawChar(pX,Y,*pStr,Font);

	return (pX - X);
}

// Draw integer as hexadecimal
// input:
//   X,Y - top left coordinates of first symbol
//   num - unsigned integer value
//   Font - pointer to font
// return: number width in pixels
uint8_t PutHex(uint8_t X, uint8_t Y, uint32_t num, const Font_TypeDef *Font) {
	uint8_t str[11]; // 10 chars max for UINT32_MAX
	register uint8_t *pStr = str;
	register uint8_t pX = X;

	// Convert number to characters
	*pStr++ = 0; // String termination character
	do { *pStr++ = (num % 0x10) + '0'; } while (num /= 0x10);

	// Draw a number
	while (*--pStr) {
		if (*pStr > '9') {
			pX += DrawChar(pX,Y,*pStr + 7,Font);
		} else {
			pX += DrawChar(pX,Y,*pStr,Font);
		}
	}

	return (pX - X);
}

// Draw monochrome bitmap with lcd_color
// input:
//   X, Y - top left corner coordinates of bitmap
//   W, H - width and height of bitmap in pixels
//   pBMP - pointer to array containing bitmap
// note: each '1' bit in the bitmap will be drawn as a pixel with a lcd_color color
//       each '0' bit in the will not be drawn (transparent bitmap)
// bitmap: one byte per 8 vertical pixels, LSB top, truncate bottom bits
void DrawBitmap(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, const uint8_t* pBMP) {
	register uint8_t pX;
	register uint8_t pY;
	register uint8_t tmpCh;
	register uint8_t bL;

	pY = Y;
	while (pY < Y + H) {
		pX = X;
		while (pX < X + W) {
			bL = 0;
			tmpCh = *pBMP++;
			if (tmpCh) {
				while (bL < 8) {
					if (tmpCh & 0x01) Pixel(pX,pY + bL,lcd_color);
					tmpCh >>= 1;
					if (tmpCh) {
						bL++;
					} else {
						pX++;
						break;
					}
				}
			} else {
				pX++;
			}
		}
		pY += 8;
	}
}

// Draw grayscale bitmap
// input:
//   X, Y - top left corner coordinates of bitmap
//   W, H - width and height of bitmap in pixels
//   pBMP - pointer to array containing bitmap
// bitmap: one byte per 8 vertical pixels, LSB top, truncate bottom bits
// note: white pixels will not be drawn (transparent bitmap)
// note: ImageMagick command line: 'convert -depth 2 image.bmp image.gray'
void DrawBitmapGS(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, const uint8_t* pBMP) {
	uint8_t pX;
	uint8_t pY;
	uint8_t tmpCh;
	uint8_t bL;
	uint8_t GS;

	pY = Y;
	while (pY < Y + H) {
		pX = X;
		while (pX < X + W) {
			bL = 0;
			tmpCh = *pBMP++;
			if (tmpCh) {
				while (bL < 8) {
					GS = tmpCh & 0x03;
					if (GS) Pixel(pX,pY + bL,GS);
					tmpCh >>= 2;
					if (tmpCh) {
						bL += 2;
					} else {
						pX++;
						break;
					}
				}
			} else {
				pX++;
			}
		}
		pY += 8;
	}
}
