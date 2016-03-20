#include "st7528.h"


// Foreground color
uint8_t lcd_color = 15;

// Screen dimensions
uint16_t scr_width  = SCR_W;
uint16_t scr_height = SCR_H;

// Display image orientation
static uint8_t scr_orientation = SCR_ORIENT_NORMAL;

// Video RAM buffer (128x128x4bit = 8192 bytes)
static uint8_t vRAM[(SCR_W * SCR_H) >> 1] __attribute__((aligned(4)));

// Look-up table of pixel grayscale level
static const uint32_t GS_LUT[] = {
		0x00000000, // 0 (white)
		0x01000000, // 1
		0x00010000, // 2
		0x01010000, // 3
		0x00000100, // 4
		0x01000100, // 5
		0x00010100, // 6
		0x01010100, // 7
		0x00000001, // 8
		0x01000001, // 9
		0x00010001, // 10
		0x01010001, // 11
		0x00000101, // 12
		0x01000101, // 13
		0x00010101, // 14
		0x01010101  // 15 (black)
};

// Look-up table of masks to clear pixel
static const uint32_t POC_LUT[] = {
		0xFEFEFEFE, // 0
		0xFDFDFDFD, // 1
		0xFBFBFBFB, // 2
		0xF7F7F7F7, // 3
		0xEFEFEFEF, // 4
		0xDFDFDFDF, // 5
		0xBFBFBFBF, // 6
		0x7F7F7F7F  // 7
};

// Look-up table of mask for partial page filing
static const uint32_t LUT_PPM[] = {
		0x00000000, // 0
		0x80808080, // 1
		0xC0C0C0C0, // 2
		0xE0E0E0E0, // 3
		0xF0F0F0F0, // 4
		0xF8F8F8F8, // 5
		0xFCFCFCFC, // 6
		0xFEFEFEFE  // 7
};

// Look-up table of full page by grayscale level
static const uint32_t LUT_SBC[] = {
		0x00000000, // 0 (white)
		0xFF000000, // 1
		0x00FF0000, // 2
		0xFFFF0000, // 3
		0x0000FF00, // 4
		0xFF00FF00, // 5
		0x00FFFF00, // 6
		0xFFFFFF00, // 7
		0x000000FF, // 8
		0xFF0000FF, // 9
		0x00FF00FF, // 10
		0xFFFF00FF, // 11
		0x0000FFFF, // 12
		0xFF00FFFF, // 13
		0x00FFFFFF, // 14
		0xFFFFFFFF  // 15 (black)
};

// Grayscale palette (4 bytes for each level of gray, 4 * 14 bytes total)
static const uint8_t GrayPalette[] = {
		0x06,0x06,0x06,0x06, // level 1
		0x0b,0x0b,0x0b,0x0b, // level 2
		0x10,0x10,0x10,0x10, // level 3
		0x15,0x15,0x15,0x15, // level 4
		0x1a,0x1a,0x1a,0x1a, // level 5
		0x1e,0x1e,0x1e,0x1e, // level 6
		0x23,0x23,0x23,0x23, // level 7
		0x27,0x27,0x27,0x27, // level 8
		0x2b,0x2b,0x2b,0x2b, // level 9
		0x2f,0x2f,0x2f,0x2f, // level 10
		0x32,0x32,0x32,0x32, // level 11
		0x35,0x35,0x35,0x35, // level 12
		0x38,0x38,0x38,0x38, // level 13
		0x3a,0x3a,0x3a,0x3a  // level 14
};


// Send single byte command to display
// input:
//   cmd - display command
static void ST7528_cmd(uint8_t cmd) {
	ST7528_A0_L(); // A0 pin LOW -> command transmit
	ST7528_CS_L();
	SPIx_SendRecv(&ST7528_SPI_PORT,cmd);
	ST7528_CS_H();
}

// Send double byte command to display
// input:
//   cmd1 - first byte of double-byte command
//   cmd2 - second byte of double-byte command
static void ST7528_cmd_double(uint8_t cmd1, uint8_t cmd2) {
	ST7528_A0_L(); // A0 pin LOW -> command transmit
	ST7528_CS_L();
	SPIx_SendRecv(&ST7528_SPI_PORT,cmd1);
	SPIx_SendRecv(&ST7528_SPI_PORT,cmd2);
	ST7528_CS_H();
}

// Initialize the display control GPIO pins
void ST7528_InitGPIO(void) {
	// Enable the GPIO peripheral(s) clock
	RCC->AHBENR |= ST7528_GPIO_PERIPH;

	// Configure CS pin as push-pull output with pull-up
	GPIO_set_mode(ST7528_CS_PORT,GPIO_Mode_OUT,GPIO_PUPD_PU,ST7528_CS_PIN);
	GPIO_out_cfg(ST7528_CS_PORT,GPIO_OT_PP,GPIO_SPD_VERYLOW,ST7528_CS_PIN);
	ST7528_CS_H();

	// Configure A0 pin as push-pull output with pull-up
	GPIO_set_mode(ST7528_A0_PORT,GPIO_Mode_OUT,GPIO_PUPD_PU,ST7528_A0_PIN);
	GPIO_out_cfg(ST7528_A0_PORT,GPIO_OT_PP,GPIO_SPD_VERYLOW,ST7528_A0_PIN);
	ST7528_A0_L();

	// Configure RST pin as push-pull output with pull-up
	GPIO_set_mode(ST7528_RST_PORT,GPIO_Mode_OUT,GPIO_PUPD_PU,ST7528_RST_PIN);
	GPIO_out_cfg(ST7528_RST_PORT,GPIO_OT_PP,GPIO_SPD_VERYLOW,ST7528_RST_PIN);
	ST7528_RST_H();
}

// Initialize SPI peripheral and ST7528 display
// note: SPI peripheral must be initialized before
void ST7528_Init(void) {
	uint8_t i;

	ST7528_CS_H();
	ST7528_A0_L();

	// Hardware display reset
	ST7528_RST_L();
	Delay_ms(1); // The minimum reset "L" pulse width (tRW) is 1us at VDD=3.3V and 2us at VDD=1.8V
	ST7528_RST_H();
	Delay_ms(1); // The maximum reset duration (tR) is 1us at VDD=3.3V and 2us at VDD=1.8V

	// Initial display configuration

	// ICON disable
	ST7528_cmd(ST7528_CMD_ICON_OFF);

	// Display OFF
	ST7528_cmd(ST7528_CMD_DISPOFF);

	// Set duty cycle: 128
	ST7528_cmd_double(ST7528_CMD_PD_DUTY,128);

	// Set initial COM0: 0
	ST7528_cmd_double(ST7528_CMD_COM0,0);

	// Set initial display line: 0
	ST7528_cmd_double(ST7528_CMD_LINE,0);

	// Set COM scan direction (SHL bit): normal
	ST7528_cmd(ST7528_CMD_SHL_OFF);

	// Set SEG scan direction (ADC bit): normal
	ST7528_cmd(ST7528_CMD_ADC_OFF);

	// Enable built-in oscillator circuit
	ST7528_cmd(ST7528_CMD_OSCON);

	// Set internal resistance ratio of the regulator resistor
	ST7528_cmd(ST7528_CMD_RREG | ST7528_RREG_72);

	// Set LCD bias
	ST7528_cmd(ST7528_CMD_BIAS | ST7528_BIAS_11);

	// Electronic volume
	ST7528_cmd_double(ST7528_CMD_ELVOL,32);

	// Configure FRC/PWM mode
	ST7528_cmd(ST7528_CMD_FRC_PWM | ST7528_FRC_4 | ST7528_PWM_60);

	// DC-DC DC[1:0]=00 booster 3X
	ST7528_cmd(ST7528_CMD_DCDC | ST7528_BOOST_3X);
	// Delay 200ms
	Delay_ms(200);

	// Power control (VC=1,VR=0,VF=0): turn on internal voltage converter circuit
	ST7528_cmd(ST7528_CMD_PWR | ST7528_PWR_VC);

	// DC-DC DC[1:0]=11 booster 6X
	ST7528_cmd(ST7528_CMD_DCDC | ST7528_BOOST_6X);
	// Delay 200ms
	Delay_ms(200);

	// Power control (VC=1,VR=1,VF=0): turn on internal voltage regulator circuit
	ST7528_cmd(ST7528_CMD_PWR | ST7528_PWR_VC | ST7528_PWR_VR);
	// Delay 10ms
	Delay_ms(10);

	// Power control (VC=1,VR=1,VF=1): turn on internal voltage follower circuit
	ST7528_cmd(ST7528_CMD_PWR | ST7528_PWR_VC | ST7528_PWR_VR | ST7528_PWR_VF);

	// Set EXT=1 mode
	ST7528_cmd_double(ST7528_CMD_MODE,ST7528_MODE_EXT1);

	// Set white mode and 1st frame
	ST7528_cmd_double(ST7528_CMD1_W1F,0x00);
	// Set white mode and 2nd frame
	ST7528_cmd_double(ST7528_CMD1_W2F,0x00);
	// Set white mode and 3rd frame
	ST7528_cmd_double(ST7528_CMD1_W3F,0x00);
	// Set white mode and 4th frame (not used in 3FRC mode)
	ST7528_cmd_double(ST7528_CMD1_W4F,0x00);

	// Load grayscale palette
	for (i = 0; i < sizeof(GrayPalette); i++) {
		ST7528_cmd_double(ST7528_CMD1_GRAYPAL + i,GrayPalette[i]);
	}

	// Set dark mode and 1st frame
	ST7528_cmd_double(ST7528_CMD1_D1F,0x3c);
	// Set dark mode and 2nd frame
	ST7528_cmd_double(ST7528_CMD1_D2F,0x3c);
	// Set dark mode and 3rd frame
	ST7528_cmd_double(ST7528_CMD1_D3F,0x3c);
	// Set dark mode and 4th frame (not used in 3FRC mode)
	ST7528_cmd_double(ST7528_CMD1_D4F,0x3c);

	// Set EXT=0 mode, booster efficiency level and frame frequency
	ST7528_cmd_double(ST7528_CMD_MODE,ST7528_MODE_EXT0 | ST7528_MODE_BE1 | ST7528_FF_77);

	// Configure N-line inversion to reduce display pixels crosstalk
	// Recommended to used the odd number from range of 3..33
	//ST7528_cmd_double(ST7528_CMD_NLINE_INV,15);

	// Display ON
	ST7528_cmd(ST7528_CMD_DISPON);
}

// Do a software reset of display
// note: doesn't affect on display memory contents and some registers
void ST7528_Reset(void) {
	ST7528_cmd(ST7528_CMD_RESET);
}

// Send vRAM buffer into display
void ST7528_Flush(void) {
	uint8_t *ptr = vRAM;
	uint8_t buf[3];

	// Column LSB
	if (scr_orientation & (SCR_ORIENT_180 | SCR_ORIENT_CW)) {
		// The display controller actually have 132 columns but the display
		// itself have only 128, therefore must shift for 4 nonexistent columns
		buf[0] = ST7528_CMD_COLL + 4;
	} else {
		buf[0] = ST7528_CMD_COLL;
	}
	// Column MSB always zero
	buf[1] = ST7528_CMD_COLM;
	// Start from page 0
	buf[2] = ST7528_CMD_PAGE;

	// Send vRAM to display by pages
	while (buf[2] < ST7528_CMD_PAGE + 16) {
		// Write page/column address
		ST7528_A0_L(); // Command transmit
		ST7528_CS_L();
		SPIx_SendBuf(&ST7528_SPI_PORT,buf,sizeof(buf));
		ST7528_CS_H();

		// Transmit one page
		ST7528_A0_H(); // Data transmit
		ST7528_CS_L();
		SPIx_SendBuf(&ST7528_SPI_PORT,ptr,SCR_PAGE_WIDTH * 4);
		ST7528_CS_H();

		// Move vRAM pointer to the next page and increase display page number
		ptr += SCR_PAGE_WIDTH * 4;
		buf[2]++;
	}
}

// Clears the vRAM memory (fill with zeros)
// note: memset() here will be faster, but needs "string.h" include
void ST7528_Clear(void) {
	register uint32_t *ptr = (uint32_t *)vRAM;
	register uint32_t i = sizeof(vRAM) >> 2;

	while (i--) {
		*ptr++ = 0x00000000;
	}
}

// Set LCD contrast
// input:
//   res_ratio - internal regulator resistor ratio (one of ST7528_RREG_XX values)
//   lcd_bias - LCD bias (one of ST7528_BIAS_XX values)
//   el_vol - electronic volume [0..63]
void ST7528_Contrast(uint8_t res_ratio, uint8_t lcd_bias, uint8_t el_vol) {
	uint8_t buf[4];

	// Prepare a buffer with commands sequence:
	// Select internal resistance ratio of the regulator resistor
	buf[0] = ST7528_CMD_RREG | (res_ratio & 0x07);
	// Select LCD bias
	buf[1] = ST7528_CMD_BIAS | (lcd_bias & 0x07);
	// Select electronic volume (double byte command)
	buf[2] = ST7528_CMD_ELVOL;
	buf[3] = el_vol & 0x3f;

	// Transmit sequence to display
	ST7528_A0_L(); // Command transfer
	ST7528_CS_L();
	SPIx_SendBuf(&ST7528_SPI_PORT,buf,sizeof(buf));
	ST7528_CS_H();
}

// Set all LCD pixels on or off
// input:
//   eon_state - new pixels state (one of SCR_ALL_PIXELS_XXX values)
// note: SCR_ALL_PIXELS_ON means what all pixels on display will be on
//       without regard of display memory contents
void ST7528_SetAllPixelsOn(uint8_t eon_state) {
	ST7528_cmd(eon_state ? ST7528_CMD_EDON : ST7528_CMD_EDOFF);
}

// Set display pixels inversion
// input:
//   inv_state - new state of display inversion (one of SCR_INVERT_XXX values)
// note: SCR_INVERT_ON means what all pixels on display will be inverted
void ST7528_SetInvert(uint8_t inv_state) {
	ST7528_cmd(inv_state ? ST7528_CMD_REVON : ST7528_CMD_REVOFF);
}

// Toggle display on/off
// input:
//   disp_state - new display state (SCR_ON or SCR_OFF)
// note: doesn't affect the display memory
void ST7528_SetDisplayState(uint8_t disp_state) {
	ST7528_cmd(disp_state ? ST7528_CMD_DISPON : ST7528_CMD_DISPOFF);
}

// Configure LCD partial display
// input:
//   phy_line - partial display physical starting line (COM0) [0..127]
//   log_line - partial display logical starting line [0 .. 127]
//   lines_num - number of lines for partial display [0 .. 128]
void ST7528_SetPartialDisplay(uint8_t phy_line, uint8_t log_line, uint8_t lines_num) {
	// Set initial display line
	ST7528_cmd_double(ST7528_CMD_LINE,log_line & 0x7f);
	// Set initial COM0
	ST7528_cmd_double(ST7528_CMD_COM0,phy_line & 0x7f);
	// Set partial display duty
	ST7528_cmd_double(ST7528_CMD_PD_DUTY,lines_num);
}

// Control display power save mode
// input:
//   pm_state - set new power save display mode (SCR_ON or SCR_OFF)
// note: SCR_OFF puts display to sleep mode, SCR_ON returns to normal mode
void ST7528_PowerSave(uint8_t pm_state) {
	ST7528_cmd(pm_state ? ST7528_CMD_PM_OFF : ST7528_CMD_PM_ON);
}

// Set X coordinate mapping (normal or mirrored)
// input:
//   x_map - new mapping of X coordinate (one of SCR_INVERT_XXX values)
// note: SCR_INVERT_OFF means normal COM scan direction
// note: doesn't affect on display memory contents
void ST7528_SetXDir(uint8_t x_map) {
	// Configure SEG scan direction
	ST7528_cmd(x_map ? ST7528_CMD_ADC_OFF : ST7528_CMD_ADC_ON);
}

// Set Y coordinate mapping (normal or mirrored)
// input:
//   y_map - new mapping of Y coordinate (one of SCR_INVERT_XXX values)
// note: SCR_INVERT_OFF means normal SEG scan direction
// note: it is necessary to rewrite the display data RAM after calling this function
void ST7528_SetYDir(uint8_t y_map) {
	// Configure COM scan direction
	ST7528_cmd(y_map ? ST7528_CMD_SHL_OFF : ST7528_CMD_SHL_ON);
}

// Set display column:page according to a specified coordinates of pixel
// input:
//   X, Y - pixel coordinates
void ST7528_SetAddr(uint8_t X, uint8_t Y) {
	// Column address LSB
	ST7528_cmd(ST7528_CMD_COLL | (X & 0x0f));
	// Column address MSB
	ST7528_cmd(ST7528_CMD_COLM | (X >> 4));
	// Page address
	ST7528_cmd(ST7528_CMD_PAGE | (Y >> 4));
}

// Set scroll line
// input:
//   line - start line number [0..127]
void ST7528_SetScrollLine(uint8_t line) {
	// Set initial display line
	ST7528_cmd_double(ST7528_CMD_LINE,line & 0x7f);
}

// Set display orientation
// input:
//   orientation - display image rotation (one of SCR_ORIENT_XXX values)
void ST7528_Orientation(uint8_t orientation) {
	// Configure display SEG/COM scan direction
	switch(orientation) {
		case SCR_ORIENT_CW:
			// Clockwise rotation
			scr_width  = SCR_H;
			scr_height = SCR_W;
			ST7528_SetXDir(SCR_INVERT_OFF);
			ST7528_SetYDir(SCR_INVERT_OFF);
			break;
		case SCR_ORIENT_CCW:
			// Counter-clockwise rotation
			scr_width  = SCR_H;
			scr_height = SCR_W;
			ST7528_SetXDir(SCR_INVERT_ON);
			ST7528_SetYDir(SCR_INVERT_ON);
			break;
		case SCR_ORIENT_180:
			// 180 degree rotation
			scr_width  = SCR_W;
			scr_height = SCR_H;
			ST7528_SetXDir(SCR_INVERT_OFF);
			ST7528_SetYDir(SCR_INVERT_ON);
			break;
		default:
			// Normal orientation
			scr_width  = SCR_W;
			scr_height = SCR_H;
			ST7528_SetXDir(SCR_INVERT_ON);
			ST7528_SetYDir(SCR_INVERT_OFF);
			break;
	}

	// Store orientation
	scr_orientation = orientation;
}

// Set pixel color in vRAM buffer
// input:
//   X, Y - pixel coordinates
//   GS - grayscale pixel color [0..15]
void LCD_Pixel(uint8_t X, uint8_t Y, uint8_t GS) {
	// Pointer to the pixel byte in video buffer computed by
	// formula ((Y / 8) * (SCR_PAGE_WIDTH * 4)) + (X * 4), since screen
	// width is 128 the formula can be simplified
	register uint32_t *ptr;
	// Vertical offset of pixel in display page
	register uint32_t voffs;
	// Bitmap for one pixel (4 consecutive bytes), get it from
	// the look-up table (it is better in terms of performance)
	register uint32_t bmap = GS_LUT[GS];

	// X and Y coordinate values must be swapped if screen rotated either clockwise or counter-clockwise
	if (scr_orientation & (SCR_ORIENT_CW | SCR_ORIENT_CCW)) {
		voffs = X & 0x07;
		ptr = (uint32_t *)&vRAM[((X >> 3) << 9) + (Y << 2)];
	} else {
		voffs = Y & 0x07;
		ptr = (uint32_t *)&vRAM[((Y >> 3) << 9) + (X << 2)];
	}

	// Vertical shift of the pixel bitmap
	bmap <<= voffs;

	// Clear pixel in vRAM (take mask from look-up table)
	*ptr &= POC_LUT[voffs];

	// Finally write new pixel color in vRAM
	*ptr |= bmap;
}

// Optimized draw horizontal line
// input:
//   X - horizontal coordinate of the left pixel
//   Y - vertical coordinate of line
//   W - line width (in pixels)
//   GS - grayscale pixel color
static void LCD_HLineInt(uint8_t X, uint8_t Y, uint8_t W, uint8_t GS) {
	register uint32_t bmap = GS_LUT[GS] << (Y & 0x07);
	register uint32_t bmsk = POC_LUT[Y & 0x07];
	register uint32_t *ptr = (uint32_t *)&vRAM[((Y >> 3) << 9) + (X << 2)];

	do {
		*ptr   &= bmsk;
		*ptr++ |= bmap;
	} while (W--);
}

// Optimized draw vertical line
// input:
//   X - horizontal coordinate of the line
//   Y - vertical coordinate of the top pixel
//   H - line height (in pixels)
//   GS - grayscale pixel color
static void LCD_VLineInt(uint8_t X, uint8_t Y, uint8_t H, uint8_t GS) {
	register uint32_t bmap = LUT_SBC[GS];
	register uint32_t vofs = (Y & 0x07);
	register uint32_t *ptr = (uint32_t *)&vRAM[((Y >> 3) << 9) + (X << 2)];

	// If the vertical line is not aligned to the screen page, the drawing function must
	// mask a bits in the bitmap corresponding to the pixels above the top of the line
	// Another PITA is what the end of the line can be at the same page, thus the drawing
	// function must also mask a bits in the bitmap corresponding to the pixels below the line
	if (vofs) {
		vofs = 8 - vofs;

		if (vofs > H) {
			// The line ends at the same page where it begins
			// Mask pixels in the bitmap, apply them to the vRAM and bail out
			*ptr &= ~LUT_PPM[vofs] |  LUT_PPM[vofs - H];
			*ptr |=  LUT_PPM[vofs] & ~LUT_PPM[vofs - H] & bmap;

			return;
		}

		// The line continued at the next page
		*ptr &= ~LUT_PPM[vofs];
		*ptr |=  LUT_PPM[vofs] & bmap;
		ptr  += SCR_PAGE_WIDTH;
		H    -= vofs;
	}

	// Draw part of the line which aligned to the screen pages (8 pixels at once)
	if (H > 7) {
		do {
			*ptr = bmap;
			ptr += SCR_PAGE_WIDTH;
			H -= 8;
		} while (H > 7);
	}

	// If end of the line is not aligned to the screen page mask a bits
	// corresponding to the pixels below the line
	if (H) {
		vofs = LUT_PPM[8 - H];
		*ptr &=  vofs;
		*ptr |= ~vofs & bmap;
	}
}

// Draw horizontal line
// input:
//   X1 - horizontal coordinate of line begin
//   X2 - horizontal coordinate of line end
//   Y - vertical coordinate of line
//   GS - grayscale pixel color
void LCD_HLine(uint8_t X1, uint8_t X2, uint8_t Y, uint8_t GS) {
	uint8_t X,W;

	if (X1 > X2) {
		X = X2; W = X1 - X2;
	} else {
		X = X1; W = X2 - X1;
	}

	if (scr_orientation & (SCR_ORIENT_CW | SCR_ORIENT_CCW)) {
		LCD_VLineInt(Y,X,W + 1,GS);
	} else {
		LCD_HLineInt(X,Y,W,GS);
	}
}

// Draw vertical line
// input:
//   X - horizontal coordinate of line
//   Y1 - vertical coordinate of line begin
//   Y2 - vertical coordinate of line end
//   GS - grayscale pixel color
void LCD_VLine(uint8_t X, uint8_t Y1, uint8_t Y2, uint8_t GS) {
	uint8_t Y,H;

	if (Y1 > Y2) {
		Y = Y2; H = Y1 - Y2;
	} else {
		Y = Y1; H = Y2 - Y1;
	}

	if (scr_orientation & (SCR_ORIENT_CW | SCR_ORIENT_CCW)) {
		LCD_HLineInt(Y,X,H,GS);
	} else {
		LCD_VLineInt(X,Y,H + 1,GS);
	}
}

// Draw rectangle
// input:
//   X1,X2 - horizontal coordinates of the opposite rectangle corners
//   Y1,Y2 - vertical coordinates of the opposite rectangle corners
//   GS - grayscale pixel color
void LCD_Rect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2, uint8_t GS) {
	LCD_HLine(X1,X2,Y1,GS);
	LCD_HLine(X1,X2,Y2,GS);
	LCD_VLine(X1,Y1,Y2,GS);
	LCD_VLine(X2,Y1,Y2,GS);
}

// Draw filled rectangle
// input:
//   X1,Y1 - top left coordinates
//   X2,Y2 - bottom right coordinates
void LCD_FillRect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2, uint8_t GS) {
	uint8_t Z,E,T,L;

	// Most optimal way to draw a filled rectangle draw it by optimized
	// vertical line function

	// Calculate coordinates based on screen rotation
	if (scr_orientation & (SCR_ORIENT_CW | SCR_ORIENT_CCW)) {
		if (X1 > X2) {
			T = X2; L = X1 - X2;
		} else {
			T = X1; L = X2 - X1;
		}
		if (Y1 > Y2) {
			Z = Y1; E = Y2;
		} else {
			Z = Y2; E = Y1;
		}
	} else {
		if (Y1 > Y2) {
			T = Y2; L = Y1 - Y2;
		} else {
			T = Y1; L = Y2 - Y1;
		}
		if (X1 > X2) {
			Z = X1; E = X2;
		} else {
			Z = X2; E = X1;
		}
	}
	L++;

	// Fill a rectangle
	do {
		LCD_VLineInt(Z,T,L,GS);
	} while (Z-- > E);
}

// Draw line
// input:
//   X1,Y1 - coordinates of line start
//   X2,Y2 - coordinates of line end
//   GS - grayscale pixel color
void LCD_Line(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, uint8_t GS) {
	int16_t dX = X2 - X1;
	int16_t dY = Y2 - Y1;
	int16_t dXsym = (dX > 0) ? 1 : -1;
	int16_t dYsym = (dY > 0) ? 1 : -1;

	// Absolute values of dX/dY
	dX *= dXsym;
	dY *= dYsym;

	// Draw either horizontal or vertical line by optimized functions
	if (dX == 0) {
		if (scr_orientation & (SCR_ORIENT_CW | SCR_ORIENT_CCW)) {
			if (dYsym < 0) {
				LCD_HLineInt(Y2,X1,dY,GS);
			} else {
				LCD_HLineInt(Y1,X1,dY,GS);
			}
		} else {
			if (dYsym < 0) {
				LCD_VLineInt(X1,Y2,dY + 1,GS);
			} else {
				LCD_VLineInt(X1,Y1,dY + 1,GS);
			}
		}

		return;
	}
	if (dY == 0) {
		if (scr_orientation & (SCR_ORIENT_CW | SCR_ORIENT_CCW)) {
			if (dXsym < 0) {
				LCD_VLineInt(Y1,X2,dX + 1,GS);
			} else {
				LCD_VLineInt(Y1,X1,dX + 1,GS);
			}
		} else {
			if (dXsym < 0) {
				LCD_HLineInt(X2,Y1,dX,GS);
			} else {
				LCD_HLineInt(X1,Y1,dX,GS);
			}
		}

		return;
	}

	// Precalculations for speed-up
	int16_t dX2 = dX << 1;
	int16_t dY2 = dY << 1;
	int16_t di;

	if (dX >= dY) {
		di = dY2 - dX;
		while (X1 != X2) {
			LCD_Pixel(X1,Y1,GS);
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
			LCD_Pixel(X1,Y1,GS);
			Y1 += dYsym;
			if (di < 0) {
				di += dX2;
			} else {
				di += dX2 - dY2;
				X1 += dXsym;
			}
		}
	}
	LCD_Pixel(X1,Y1,GS);
}

// Draw circle
// input:
//   Xc,Yc - circle center coordinates
//   R - circle radius
//   GS - grayscale pixel color
void LCD_Circle(int16_t Xc, int16_t Yc, uint8_t R, uint8_t GS) {
	int16_t err = 1 - R;
	int16_t dx  = 0;
	int16_t dy  = -2 * R;
	int16_t x   = 0;
	int16_t y   = R;
	// Screen width and height for less calculations
	int16_t sh  = scr_height - 1;
	int16_t sw  = scr_width  - 1;

	// Draw pixels with respect of screen boundaries
	while (x < y) {
		if (err >= 0) {
			dy  += 2;
			err += dy;
			y--;
		}
		dx  += 2;
		err += dx + 1;
		x++;

		// Draw eight pixels of each octant
		if (Xc + x < sw) {
			if (Yc + y < sh) LCD_Pixel(Xc + x,Yc + y,GS);
			if (Yc - y > -1) LCD_Pixel(Xc + x,Yc - y,GS);
		}
		if (Xc - x > -1) {
			if (Yc + y < sh) LCD_Pixel(Xc - x,Yc + y,GS);
			if (Yc - y > -1) LCD_Pixel(Xc - x,Yc - y,GS);
		}
		if (Xc + y < sw) {
			if (Yc + x < sh) LCD_Pixel(Xc + y,Yc + x,GS);
			if (Yc - x > -1) LCD_Pixel(Xc + y,Yc - x,GS);
		}
		if (Xc - y > -1) {
			if (Yc + x < sh) LCD_Pixel(Xc - y,Yc + x,GS);
			if (Yc - x > -1) LCD_Pixel(Xc - y,Yc - x,GS);
		}
	}

	// Vertical and horizontal points
	if (Xc + R < sw) LCD_Pixel(Xc + R,Yc,GS);
	if (Xc - R > -1) LCD_Pixel(Xc - R,Yc,GS);
	if (Yc + R < sh) LCD_Pixel(Xc,Yc + R,GS);
	if (Yc - R > -1) LCD_Pixel(Xc,Yc - R,GS);
}

// Draw ellipse
// input:
//   Xc,Yc - ellipse center coordinates
//   Ra,Rb - horizontal and vertical radiuses
//   GS - grayscale pixel color
void LCD_Ellipse(uint16_t Xc, uint16_t Yc, uint16_t Ra, uint16_t Rb, uint8_t GS) {
	int16_t x  = 0;
	int16_t y  = Rb;
	int32_t A2 = Ra * Ra;
	int32_t B2 = Rb * Rb;
	int32_t C1 = -((A2 >> 2) + (Ra & 0x01) + B2);
	int32_t C2 = -((B2 >> 2) + (Rb & 0x01) + A2);
	int32_t C3 = -((B2 >> 2) + (Rb & 0x01));
	int32_t t  = -A2 * y;
	int32_t dX = B2 * x * 2;
	int32_t dY = -A2 * y * 2;
	int32_t dXt2 = B2 * 2;
	int32_t dYt2 = A2 * 2;
	// Screen width and height for less calculations
	int16_t sh = scr_height - 1;
	int16_t sw = scr_width  - 1;

	// Draw pixels with respect of screen boundaries
	while ((y >= 0) && (x <= Ra)) {
		if ((Xc + x < sw) && (Yc + y < sh)) {
			LCD_Pixel(Xc + x,Yc + y,GS);
		}
		if (x || y) {
			if ((Xc - x > -1) && (Yc - y > -1)) {
				LCD_Pixel(Xc - x,Yc - y,GS);
			}
		}
		if (x && y) {
			if ((Xc + x < sw) && (Yc - y > - 1)) {
				LCD_Pixel(Xc + x,Yc - y,GS);
			}
			if ((Xc - x > -1) && (Yc + y < sh)) {
				LCD_Pixel(Xc - x,Yc + y,GS);
			}
		}

		if ((t + x*B2 <= C1) || (t + y*A2 <= C3)) {
			dX += dXt2;
			t  += dX;
			x++;
		} else if (t - y*A2 > C2) {
			dY += dYt2;
			t  += dY;
			y--;
		} else {
			dX += dXt2;
			dY += dYt2;
			t  += dX;
			t  += dY;
			x++;
			y--;
		}
	}
}

// Draw a single character
// input:
//   X,Y - character top left corner coordinates
//   Char - character to be drawn
//   Font - pointer to font
// return: character width in pixels
uint8_t LCD_PutChar(uint8_t X, uint8_t Y, uint8_t Char, const Font_TypeDef *Font) {
	uint8_t pX;
	uint8_t pY;
	uint8_t tmpCh;
	uint8_t bL;
	const uint8_t *pCh;

	// If the specified character code is out of bounds should substitute the code of the "unknown" character
	if ((Char < Font->font_MinChar) || (Char > Font->font_MaxChar)) Char = Font->font_UnknownChar;

	// Pointer to the first byte of character in font data array
	pCh = &Font->font_Data[(Char - Font->font_MinChar) * Font->font_BPC];

	// Draw character
	if (Font->font_Scan == FONT_V) {
		// Vertical pixels order
		if (Font->font_Height < 9) {
			// Height is 8 pixels or less (one byte per column)
			pX = X;
			while (pX < X + Font->font_Width) {
				pY = Y;
				tmpCh = *pCh++;
				while (tmpCh) {
					if (tmpCh & 0x01) LCD_Pixel(pX,pY,lcd_color);
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
							if (tmpCh & 0x01) LCD_Pixel(pX,pY,lcd_color);
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
					if (tmpCh & 0x01) LCD_Pixel(pX,pY,lcd_color);
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
							if (tmpCh & 0x01) LCD_Pixel(pX,pY,lcd_color);
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
uint16_t LCD_PutStr(uint8_t X, uint8_t Y, const char *str, const Font_TypeDef *Font) {
	uint8_t pX = X;
	uint8_t eX = scr_width - Font->font_Width - 1;

	while (*str) {
		pX += LCD_PutChar(pX,Y,*str++,Font);
		if (pX > eX) break;
	}

	return (pX - X);
}

// Draw string with line feed by screen edge
// input:
//   X,Y - top left coordinates of first character
//   str - pointer to zero-terminated string
//   Font - pointer to font
// return: string width in pixels
uint16_t LCD_PutStrLF(uint8_t X, uint8_t Y, const char *str, const Font_TypeDef *Font) {
	uint8_t strLen = 0;

    while (*str) {
        LCD_PutChar(X,Y,*str++,Font);
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
uint8_t LCD_PutInt(uint8_t X, uint8_t Y, int32_t num, const Font_TypeDef *Font) {
	uint8_t str[11]; // 10 chars max for INT32_MIN..INT32_MAX (without sign)
	uint8_t *pStr = str;
	uint8_t pX = X;
	uint8_t neg = 0;

	// String termination character
	*pStr++ = '\0';

	// Convert number to characters
	if (num < 0) {
		neg = 1;
		num *= -1;
	}
	do { *pStr++ = (num % 10) + '0'; } while (num /= 10);
	if (neg) *pStr++ = '-';

	// Draw a number
	while (*--pStr) pX += LCD_PutChar(pX,Y,*pStr,Font);

	return (pX - X);
}

// Draw unsigned integer value
// input:
//   X,Y - top left coordinates of first symbol
//   num - unsigned integer value
//   Font - pointer to font
// return: number width in pixels
uint8_t LCD_PutIntU(uint8_t X, uint8_t Y, uint32_t num, const Font_TypeDef *Font) {
	uint8_t str[11]; // 10 chars max for UINT32_MAX
	uint8_t *pStr = str;
	uint8_t pX = X;

	// Convert number to characters
	*pStr++ = 0; // String termination character
	do { *pStr++ = (num % 10) + '0'; } while (num /= 10);

	// Draw a number
	while (*--pStr) pX += LCD_PutChar(pX,Y,*pStr,Font);

	return (pX - X);
}

// Draw signed integer value with decimal point
// input:
//   X,Y - top left coordinates of first symbol
//   num - unsigned integer value
//   decimals - number of digits after decimal point
//   Font - pointer to font
// return: number width in pixels
uint8_t LCD_PutIntF(uint8_t X, uint8_t Y, int32_t num, uint8_t decimals, const Font_TypeDef *Font) {
	uint8_t str[11]; // 10 chars max for INT32_MIN..INT32_MAX (without sign)
	uint8_t *pStr = str;
	uint8_t pX = X;
	uint8_t neg = 0;
	uint8_t strLen = 0;

	// Convert number to characters
	*pStr++ = '\0'; // String termination character
	if (num < 0) {
		neg = 1;
		num *= -1;
	}
	do {
		*pStr++ = (num % 10) + '0';
		strLen++;
	} while (num /= 10);

	// Add leading zeroes
	if (strLen <= decimals) {
		while (strLen <= decimals) {
			*pStr++ = '0';
			strLen++;
		}
	}

	// Minus sign?
	if (neg) {
		*pStr++ = '-';
		strLen++;
	}

	// Draw a number
	while (*--pStr) {
		pX += LCD_PutChar(pX,Y,*pStr,Font);
		if (decimals && (--strLen == decimals)) {
			// Draw decimal point
			LCD_Rect(pX,Y + Font->font_Height - 2,pX + 1,Y + Font->font_Height - 1,lcd_color);
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
uint8_t LCD_PutIntLZ(uint8_t X, uint8_t Y, int32_t num, uint8_t digits, const Font_TypeDef *Font) {
	uint8_t str[11]; // 10 chars max for INT32_MIN..INT32_MAX (without sign)
	uint8_t *pStr = str;
	uint8_t pX = X;
	uint8_t neg = 0;
	uint8_t strLen = 0;

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

	// Minus sign?
	if (neg) *pStr++ = '-';

	// Draw a number
	while (*--pStr) pX += LCD_PutChar(pX,Y,*pStr,Font);

	return (pX - X);
}

// Draw integer as hexadecimal
// input:
//   X,Y - top left coordinates of first symbol
//   num - unsigned integer value
//   Font - pointer to font
// return: number width in pixels
uint8_t LCD_PutHex(uint8_t X, uint8_t Y, uint32_t num, const Font_TypeDef *Font) {
	uint8_t str[11]; // 10 chars max for UINT32_MAX
	uint8_t *pStr = str;
	uint8_t pX = X;

	// Convert number to characters
	*pStr++ = 0; // String termination character
	do {
		*pStr = (num % 0x10) + '0';
		if (*pStr > '9') *pStr += 7;
		pStr++;
	} while (num /= 0x10);

	// Draw a number
	while (*--pStr) pX += LCD_PutChar(pX,Y,*pStr,Font);

	return (pX - X);
}

// Draw monochrome bitmap with lcd_color
// input:
//   X, Y - top left corner coordinates of bitmap
//   W, H - width and height of bitmap in pixels
//   pBMP - pointer to array containing bitmap
// note:
//   each SET bit in the bitmap means a pixel with lcd_color color
//   each RESET bit in the bitmap means no drawing (transparent)
// bitmap coding:
//   1 byte represents 8 vertical pixels
//   scan top->bottom, left->right
//   LSB top
//   bottom bits truncated
void LCD_DrawBitmap(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, const uint8_t* pBMP) {
	uint8_t pX;
	uint8_t pY;
	uint8_t tmpCh;
	uint8_t bL;

	pY = Y;
	while (pY < Y + H) {
		pX = X;
		while (pX < X + W) {
			bL = 0;
			tmpCh = *pBMP++;
			if (tmpCh) {
				while (bL < 8) {
					if (tmpCh & 0x01) LCD_Pixel(pX,pY + bL,lcd_color);
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

// Draw grayscale bitmap (4-bit)
// input:
//   X, Y - top left corner coordinates of bitmap
//   W, H - width and height of bitmap in pixels
//   pBMP - pointer to array containing bitmap
// note:
//   this function uses LCD_Pixel for drawing, in terms of performance it is a bad idea
//   but such way respects screen rotation
// bitmap coding:
//   1 byte represents 2 horizontal pixels (color coded in byte nibbles)
//   scan left->right, top->bottom
//   LSB left
void LCD_DrawBitmapGS(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, const uint8_t* pBMP) {
	uint8_t pX;
	uint8_t pY;
	uint8_t tmpCh;

	pY = Y;
	while (pY < Y + H) {
		pX = X;
		while (pX < X + W) {
			tmpCh = *pBMP++;

			// First nibble
			LCD_Pixel(pX,pY,tmpCh >> 4);
			pX++;

			// Seconds nibble
			if (pX < X + W) {
				LCD_Pixel(pX,pY,tmpCh & 0x0F);
				pX++;
			}
		}
		pY++;
	}
}

// Inverts area of image in video buffer
// input:
//   X,Y - coordinates of top left area corner
//   W - area width (pixels)
//   H - area height (pixels)
void LCD_Invert(uint8_t X, uint8_t Y, uint8_t W, uint8_t H) {
	register uint32_t vofs;
	register uint32_t *ptr;
	register uint32_t mask;
	register uint32_t dX;
	uint8_t dH;
	uint8_t dW;
	uint8_t rW;

	// X and Y coordinate values must be swapped if screen rotated either clockwise or counter-clockwise
	if (scr_orientation & (SCR_ORIENT_CW | SCR_ORIENT_CCW)) {
		dH   = W;
		dW   = H;
		ptr  = (uint32_t *)&vRAM[((X >> 3) << 9) + (Y << 2)];
		rW   = SCR_PAGE_WIDTH - H;
		vofs = X & 0x07;
	} else {
		dH   = H;
		dW   = W;
		ptr  = (uint32_t *)&vRAM[((Y >> 3) << 9) + (X << 2)];
		rW   = SCR_PAGE_WIDTH - W;
		vofs = Y & 0x07;
	}

	// If the vertical coordinate of area is not aligned to the screen page, the bits
	// in the mask corresponding to the pixels above the area must be reset
	// Also if the area ends at the same page, the bits in the mask corresponding to
	// the pixels below the area must be reset
	if (vofs) {
		// Get bit mask from the look-up table
		vofs = 8 - vofs;
		mask = LUT_PPM[vofs];

		// Trim mask if he area ends at the same page
		if (vofs > dH) mask &= ~LUT_PPM[vofs - dH];

		// Invert row
		dX = dW;
		while (dX--) { *ptr++ ^= mask; }

		// Bail out in case of area end
		if (vofs > dH) return;

		// Jump to the next page
		dH  -= vofs;
		ptr += rW;
	}

	// Invert pixels on whole page (8 rows at once)
	if (dH > 7) {
		do {
			dX   = dW;
			while (dX--) { *ptr++ ^= 0xFFFFFFFF; }
			ptr += rW;
			dH  -= 8;
		} while (dH > 7);
	}

	// If end of the area is not aligned to the screen page the bits corresponding
	// to the pixels below the area must be masked
	if (dH) {
		// Get a bit mask from the look-up table
		mask = ~LUT_PPM[8 - dH];

		// Invert row
		dX = dW;
		while (dX--) { *ptr++ ^= mask; }
	}
}
