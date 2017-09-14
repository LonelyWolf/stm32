#include "ls027b7dh01.h"
#include "string.h"


// Screen dimensions
uint16_t scr_width;
uint16_t scr_height;

// Pixel drawing mode
// Whereas in most drawing operations pixels are set, use global variable to select drawing mode
// instead of passing set/reset/invert mode in each call of drawing functions
uint8_t LCD_PixelMode;

// Display image orientation
static uint8_t lcd_orientation = LCD_ORIENT_NORMAL;

// Video RAM buffer
static uint8_t vRAM[(SCR_W * SCR_H) >> 3];

#if (SMLCD_VCOM_SOFT)
// VCOM state
static uint8_t SMLCD_VCOM = 0x00;
#endif // SMLCD_VCOM_SOFT

// Look-up tables for single pixel set/reset
static const uint8_t LUT_PSET[8] = { 0x7F, 0xBF, 0xDF, 0xEF, 0xF7, 0xFB, 0xFD, 0xFE };
static const uint8_t LUT_PRST[8] = { 0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01 };


// Swap values of variables (XOR algorithm)
// notes:
//   variables must be same data type
//   doesn't work when A and B are the same object - result will be 0 in that case
#define SWAP_VARS(A, B) do { (A) ^= (B); (B) ^= (A); (A) ^= (B); } while (0)


#if (!SMLCD_FLUSH_LUT)
// Reverse bits order in byte
__STATIC_INLINE uint8_t __reverse8bit(uint8_t byte) {
#if 1
	// Using ARM RBIT instruction
	// Since it operates with 32-bit values only, result must be shifted by 24 bits to the right
	return (uint8_t)(__RBIT(byte) >> 24);
#else
	// One of the 'standard' methods
	// And yes, the look-up table will be faster, but it will eat as much as 256 bytes of flash...
	byte = (byte & 0xF0) >> 4 | (byte & 0x0F) << 4;
	byte = (byte & 0xCC) >> 2 | (byte & 0x33) << 2;
	byte = (byte & 0xAA) >> 1 | (byte & 0x55) << 1;

	return byte;
#endif
}
#endif // SMLCD_FLUSH_LUT


// Initialize display control GPIO pins
void SMLCD_InitGPIO(void) {
	// Enable GPIO peripherals
	RCC->AHB2ENR |= SMLCD_GPIO_PERIPH;

	// Configure DISP pin
	GPIO_set_mode(SMLCD_DISP_PORT, GPIO_Mode_OUT, GPIO_PUPD_PU, SMLCD_DISP_PIN);
	GPIO_out_cfg(SMLCD_DISP_PORT, GPIO_OT_PP, GPIO_SPD_LOW, SMLCD_DISP_PIN);
	SMLCD_DISP_L;

	// Configure SCS pin
	GPIO_set_mode(SMLCD_SCS_PORT, GPIO_Mode_OUT, GPIO_PUPD_PU, SMLCD_SCS_PIN);
	GPIO_out_cfg(SMLCD_SCS_PORT, GPIO_OT_PP, GPIO_SPD_LOW, SMLCD_SCS_PIN);
	SMLCD_SCS_L;
}

// Initialize display
// note: SPI and GPIO must be initialized before calling this function
void SMLCD_Init(void) {
	// Set default drawing mode
	LCD_PixelMode = LCD_PSET;

	// Set initial image orientation
	SMLCD_Orientation(LCD_ORIENT_NORMAL);
}

// Clear display memory (clear screen)
void SMLCD_Clear(void) {
	// Send "Clear Screen" command
	SMLCD_SCS_H;
	SPI_SendRecv(&SMLCD_SPI_PORT, SMLCD_CMD_CLS);
	SPI_SendRecv(&SMLCD_SPI_PORT, SMLCD_CMD_NOP);
	SMLCD_SCS_L;
}

#if (SMLCD_VCOM_SOFT)
// Toggle VCOM bit
void SMLCD_ToggleVCOM(void) {
	SMLCD_VCOM ^= SMLCD_CMD_VCOM;
	SMLCD_SCS_H;
	SPI_SendRecv(&SMLCD_SPI_PORT, SMLCD_VCOM);
	SPI_SendRecv(&SMLCD_SPI_PORT, SMLCD_CMD_NOP);
	SMLCD_SCS_L;
}
#endif // SMLCD_VCOM_SOFT

// Send vRAM buffer into display
void SMLCD_Flush(void) {
#if 0

	register uint8_t *ptr = vRAM;
	register uint8_t line = 0;
	register uint32_t idx;

	SMLCD_SCS_H;
	// Send "Write Line" command
	SPI_SendRecv(&SMLCD_SPI_PORT, SMLCD_CMD_WRITE);
	switch (lcd_orientation) {
		case LCD_ORIENT_CW:
			line = SCR_H + 1;
			while (--line > 0) {
				SPI_SendRecv(&SMLCD_SPI_PORT, __reverse8bit(line));
				SPI_SendBuf(&SMLCD_SPI_PORT, ptr, SCR_W >> 3);
				SPI_SendRecv(&SMLCD_SPI_PORT, SMLCD_CMD_NOP);
				ptr += SCR_W >> 3;
			}
			break;
		case LCD_ORIENT_CCW:
			line = SCR_H + 1;
			ptr = &vRAM[((SCR_W * SCR_H) >> 3) - 1];
			while (--line > 0) {
				SPI_SendRecv(&SMLCD_SPI_PORT, __reverse8bit(line));
				for (idx = 0; idx < SCR_W >> 3; idx++) {
					SPI_SendRecv(&SMLCD_SPI_PORT, *ptr--);
				}
				SPI_SendRecv(&SMLCD_SPI_PORT, SMLCD_CMD_NOP);
			}
			break;
		case LCD_ORIENT_180:
			line = 0;
			ptr = &vRAM[((SCR_W * SCR_H) >> 3) - 1];
			while (line++ < SCR_H + 1) {
				SPI_SendRecv(&SMLCD_SPI_PORT, __reverse8bit(line));
				for (idx = 0; idx < SCR_W >> 3; idx++) {
					SPI_SendRecv(&SMLCD_SPI_PORT, *ptr--);
				}
				SPI_SendRecv(&SMLCD_SPI_PORT, SMLCD_CMD_NOP);
			}
			break;
		case LCD_ORIENT_NORMAL:
		default:
			line = 0;
			while (line++ < SCR_H + 1) {
				SPI_SendRecv(&SMLCD_SPI_PORT, __reverse8bit(line));
				SPI_SendBuf(&SMLCD_SPI_PORT, ptr, SCR_W >> 3);
				SPI_SendRecv(&SMLCD_SPI_PORT, SMLCD_CMD_NOP);
				ptr += SCR_W >> 3;
			}
			break;
	}
	// One more trailer after last string has been transmitted
	SPI_SendRecv(&SMLCD_SPI_PORT, SMLCD_CMD_NOP);
	SMLCD_SCS_L;

#else

	register uint8_t *ptr = vRAM;

	SMLCD_SCS_H;
	// Send "Write Line" command
	SPI_SendRecv(&SMLCD_SPI_PORT, SMLCD_CMD_WRITE);

#if (SMLCD_FLUSH_LUT)
	// Use look-up table for line number

	// Look-up table for line numbers with reversed bit order
	static const uint8_t LUT_LINE[240] = {
			0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0, 0x08,
			0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8, 0x04,
			0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4, 0x0C,
			0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC, 0x02,
			0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2, 0x0A,
			0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA, 0x06,
			0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6, 0x0E,
			0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE, 0x01,
			0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1, 0x09,
			0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9, 0x05,
			0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5, 0x0D,
			0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD, 0x03,
			0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3, 0x0B,
			0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB, 0x07,
			0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7, 0x0F
	};

	// This variable declared as 32-bit to improve performance on on 32-bit MCU
	// it can be changed to 8 or 16 bit
	register uint32_t line;

	if (lcd_orientation & (LCD_ORIENT_180 | LCD_ORIENT_CW)) {
		// Inverted lines order (e.g. 240th line from vRAM will come as 1st to the screen)
		line = SCR_H;
		while (line-- > 0) {
			// Send: line number -> line data -> trailer byte
			SPI_SendRecv(&SMLCD_SPI_PORT, LUT_LINE[line]);
			SPI_SendBuf(&SMLCD_SPI_PORT, ptr, SCR_W >> 3);
			SPI_SendRecv(&SMLCD_SPI_PORT, SMLCD_CMD_NOP);
			ptr += SCR_W >> 3;
		}
	} else {
		// Normal lines order
		line = 0;
		do {
			// Send: line number -> line data -> trailer byte
			SPI_SendRecv(&SMLCD_SPI_PORT, LUT_LINE[line]);
			SPI_SendBuf(&SMLCD_SPI_PORT, ptr, SCR_W >> 3);
			SPI_SendRecv(&SMLCD_SPI_PORT, SMLCD_CMD_NOP);
			ptr += SCR_W >> 3;
		} while (++line < SCR_H);
	}
#else
	// Use real-time bit order reversing in line number

	register uint8_t line;

	if (lcd_orientation & (LCD_ORIENT_180 | LCD_ORIENT_CW)) {
		// Inverted lines order (e.g. 240th line from vRAM will come as 1st in the screen)
		line = SCR_H;
		do {
			// Send: line number -> line data -> trailer byte
			SPI_SendRecv(&SMLCD_SPI_PORT, __reverse8bit(line));
			SPI_SendBuf(&SMLCD_SPI_PORT, ptr, SCR_W >> 3);
			SPI_SendRecv(&SMLCD_SPI_PORT, SMLCD_CMD_NOP);
			ptr += SCR_W >> 3;
		} while (--line > 0);
	} else {
		// Normal lines order
		line = 1;
		do {
			// Send: line number -> line data -> trailer byte
			SPI_SendRecv(&SMLCD_SPI_PORT, __reverse8bit(line));
			SPI_SendBuf(&SMLCD_SPI_PORT, ptr, SCR_W >> 3);
			SPI_SendRecv(&SMLCD_SPI_PORT, SMLCD_CMD_NOP);
			ptr += SCR_W >> 3;
		} while (line++ < SCR_H);
	}
#endif // SMLCD_FLUSH_LUT

	// One more trailer after last string has been transmitted
	SPI_SendRecv(&SMLCD_SPI_PORT, SMLCD_CMD_NOP);
	SMLCD_SCS_L;

#endif
}

// Set screen orientation
// input:
//   orientation - one of LCD_ORIENT_xx values
void SMLCD_Orientation(uint8_t orientation) {
	if (orientation & (LCD_ORIENT_CW | LCD_ORIENT_CCW)) {
		scr_width  = SCR_H;
		scr_height = SCR_W;
	} else {
		scr_width  = SCR_W;
		scr_height = SCR_H;
	}
	lcd_orientation = orientation;
}

// Clear the vRAM memory
// note: size of video buffer must be a multiple of 4
void LCD_Clear(void) {
#if 0
	// This variant can be faster, speed depends on libraries used
	// But also needs include of "string.h"
	memset(vRAM, 0xFFFFFFFF, sizeof(vRAM) >> 2);
#else
	register uint32_t *ptr = (uint32_t *)vRAM;
	register uint32_t i = sizeof(vRAM) >> 2;

	while (i--) {
		*ptr++ = 0xFFFFFFFF;
	}
#endif
}

// Draw a single pixel
// input:
//   X, Y - coordinates of pixel
// note: value of LCD_PixelMode will be used as a drawing mode of a pixel
// note: X and Y coordinates are declared as "register uint32_t" for performance
//       for other compilers/CPUs this can/or should be changed to other type (16-bit)
void LCD_Pixel(register uint32_t X, register uint32_t Y) {
	register uint32_t offset;
	register uint8_t bpos;

#if (SMLCD_PIXEL_METHOD)
	switch (lcd_orientation) {
		case LCD_ORIENT_180:
			X = SCR_W - 1 - X;
			break;
		case LCD_ORIENT_CCW:
			SWAP_VARS(X, Y);
			X = SCR_W - 1 - X;
			break;
		case LCD_ORIENT_CW:
			SWAP_VARS(X, Y);
			break;
		case LCD_ORIENT_NORMAL:
		default:
			break;
	}
#else
	if (lcd_orientation & (LCD_ORIENT_CCW | LCD_ORIENT_CW)) {
		SWAP_VARS(X, Y);
	}
	if (lcd_orientation & (LCD_ORIENT_180 | LCD_ORIENT_CCW)) {
		X = SCR_W - 1 - X;
	}
#endif

	// Offset in video buffer
	offset = ((Y * SCR_W) + X) >> 3;

#if (SMLCD_PIXEL_SAFE)
	// Ensure offset is inside of video buffer
	if (offset > ((SCR_W * SCR_H) >> 3) - 1) {
		return;
	}
#endif // SMLCD_PIXEL_SAFE

	// Bit position in byte
	bpos = X & 0x07;

	// Update pixel in vRAM
#if (SMLCD_USE_BITBAND)
	// Using bit banding

	// Look-up table for calculation bit banding address
	// note: yeah, looks scary, but it's static
	static const uint32_t LUT_BB[8] = {
			SRAM_BB_BASE + (SRAM_BASE << 5) + 0x1C,
			SRAM_BB_BASE + (SRAM_BASE << 5) + 0x18,
			SRAM_BB_BASE + (SRAM_BASE << 5) + 0x14,
			SRAM_BB_BASE + (SRAM_BASE << 5) + 0x10,
			SRAM_BB_BASE + (SRAM_BASE << 5) + 0x0C,
			SRAM_BB_BASE + (SRAM_BASE << 5) + 0x08,
			SRAM_BB_BASE + (SRAM_BASE << 5) + 0x04,
			SRAM_BB_BASE + (SRAM_BASE << 5) + 0x00
	};

	// Pointer to bit banding address corresponding to pixel with given coordinates
	register uint32_t *BB = (uint32_t *)(LUT_BB[bpos] + ((uint32_t)((void *)(&vRAM[offset])) << 5));

	// Update pixel
	switch (LCD_PixelMode) {
		case LCD_PRES:
			*BB  = 1;
			break;
		case LCD_PINV:
			*BB ^= 1;
			break;
		case LCD_PSET:
		default:
			*BB  = 0;
			break;
	}
#else // SMLCD_USE_BITBAND
	// Using access via array

	switch (LCD_PixelMode) {
		case LCD_PRES:
			vRAM[offset] |= LUT_PRST[bpos];
			break;
		case LCD_PINV:
			vRAM[offset] ^= LUT_PRST[bpos];
			break;
		case LCD_PSET:
		default:
			vRAM[offset] &= LUT_PSET[bpos];
			break;
	}
#endif // SMLCD_USE_BITBAND
}

// Optimized vertical line drawing (without regard of screen rotation)
// input:
//   X - horizontal coordinate
//   Y - vertical coordinate
//   H - line height
static void LCD_VLineInt(uint16_t X, uint16_t Y, uint16_t H) {
	register uint8_t *ptr = &vRAM[((Y * SCR_W) + X) >> 3];
	register uint8_t mask;

	// Draw line
	X &= 0x07;
	switch (LCD_PixelMode) {
		case LCD_PRES:
			mask = LUT_PRST[X];
			while (H--) {
				*ptr |= mask;
				ptr += SCR_W >> 3;
			}
			break;
		case LCD_PINV:
			mask = LUT_PRST[X];
			while (H--) {
				*ptr ^= mask;
				ptr += SCR_W >> 3;
			}
			break;
		case LCD_PSET:
		default:
			mask = LUT_PSET[X];
			while (H--) {
				*ptr &= mask;
				ptr += SCR_W >> 3;
			}
			break;
	}
}

// Optimized draw horizontal line (without regard of screen rotation)
// input:
//   X - horizontal coordinate of line start
//   Y - vertical coordinate
//   W - line width
static void LCD_HLineInt(uint16_t X, uint16_t Y, uint16_t W) {
	register uint8_t *ptr = &vRAM[((Y * SCR_W) + X) >> 3];
	register uint8_t modulo = X & 0x07;
	register uint8_t mask;

	// Look-up tables
	static const uint8_t LUT_B1[] = { 0xFF, 0xFE, 0xFC, 0xF8, 0xF0, 0xE0, 0xC0, 0x80 };
	static const uint8_t LUT_B2[] = { 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F };

	// First partial byte
	if (modulo) {
		// Get bit mask for first partial byte
		modulo = 8 - modulo;
		mask = LUT_B1[modulo];

		if (modulo > W) {
			// Trim bit mask if line will not go out from a current byte
			mask |= LUT_B2[modulo - W];
		}

		// Update first partial byte
		switch (LCD_PixelMode) {
			case LCD_PRES:
				*ptr |= ~mask;
				break;
			case LCD_PINV:
				*ptr ^= ~mask;
				break;
			case LCD_PSET:
			default:
				*ptr &=  mask;
				break;
		}

		// Line is over?
		if (modulo > W) {
			return;
		}

		// Shift pointer to the next byte in line and decrease line height counter
		ptr++;
		W -= modulo;
	}

	// Fill solid bytes
	if (W > 32) {
		// Modify 32 pixels at once
		register uint32_t *ptr32 = (uint32_t *)ptr;
		switch (LCD_PixelMode) {
			case LCD_PRES:
				do {
					*ptr32++ = 0xFFFFFFFF;
					W -= 32;
				} while (W > 31);
				break;
			case LCD_PINV:
				do {
					*ptr32++ ^= 0xFFFFFFFF;
					W -= 32;
				} while (W > 31);
				break;
			case LCD_PSET:
			default:
				do {
					*ptr32++ = 0x00000000;
					W -= 32;
				} while (W > 31);
				break;
		}
		ptr = (uint8_t *)ptr32;
	}
	if (W > 7) {
		// Modify full bytes (8 pixels at once)
		switch (LCD_PixelMode) {
			case LCD_PRES:
				do {
					*ptr++ = 0xFF;
					W -= 8;
				} while (W > 7);
				break;
			case LCD_PINV:
				do {
					*ptr++ ^= 0xFF;
					W -= 8;
				} while (W > 7);
				break;
			case LCD_PSET:
			default:
				do {
					*ptr++ = 0x00;
					W -= 8;
				} while (W > 7);
				break;
		}
	}

	// Last partial byte?
	if (W) {
		mask = LUT_B2[8 - W];
		switch (LCD_PixelMode) {
			case LCD_PRES:
				*ptr |= ~mask;
				break;
			case LCD_PINV:
				*ptr ^= ~mask;
				break;
			case LCD_PSET:
			default:
				*ptr &= mask;
				break;
		}
	}
}

// Draw horizontal line
// input:
//   X1, X2 - left and right horizontal coordinates
//   Y - vertical coordinate
void LCD_HLine(uint16_t X1, uint16_t X2, uint16_t Y) {
	register uint16_t X;
	register uint16_t L;

	if (X1 > X2) {
		X = X2; L = X1 - X2;
	} else {
		X = X1; L = X2 - X1;
	}
	L++;

	switch (lcd_orientation) {
		case LCD_ORIENT_CW:
			LCD_VLineInt(Y, X, L);
			break;
		case LCD_ORIENT_CCW:
			LCD_VLineInt(SCR_W - 1 - Y, X, L);
			break;
		case LCD_ORIENT_180:
			LCD_HLineInt(SCR_W - X - L, Y, L);
			break;
		case LCD_ORIENT_NORMAL:
		default:
			LCD_HLineInt(X, Y, L);
			break;
	}
}

// Draw vertical line
// input:
//   X - horizontal coordinate
//   Y1,Y2 - top and bottom vertical coordinates
void LCD_VLine(uint16_t X, uint16_t Y1, uint16_t Y2) {
	register uint16_t Y;
	register uint16_t L;

	if (Y1 > Y2) {
		Y = Y2; L = Y1 - Y2;
	} else {
		Y = Y1; L = Y2 - Y1;
	}
	L++;

	switch (lcd_orientation) {
		case LCD_ORIENT_CW:
			LCD_HLineInt(Y, X, L);
			break;
		case LCD_ORIENT_CCW:
			LCD_HLineInt(SCR_W - Y - L, X, L);
			break;
		case LCD_ORIENT_180:
			LCD_VLineInt(SCR_W - 1 - X, Y, L);
			break;
		case LCD_ORIENT_NORMAL:
		default:
			LCD_VLineInt(X, Y, L);
			break;
	}
}

// Draw rectangle
// input:
//   X1,Y1 - top left coordinates
//   X2,Y2 - bottom right coordinates
void LCD_Rect(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2) {
	LCD_HLine(X1, X2, Y1);
	LCD_HLine(X1, X2, Y2);
	if (Y1 > Y2) {
		SWAP_VARS(Y1, Y2);
	}
	Y1++;
	Y2--;
	LCD_VLine(X1, Y1, Y2);
	LCD_VLine(X2, Y1, Y2);
}

// Draw filled rectangle
// input:
//   X1,Y1 - top left coordinates
//   X2,Y2 - bottom right coordinates
// note: doesn't check vRAM boundaries, so caller must respect
//       screen width and height while specifying X and Y coordinates
void LCD_FillRect(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2) {
	static const uint8_t LUT_B1[] = { 0xFF, 0x7F, 0x3F, 0x1F, 0x0F, 0x07, 0x03, 0x01 };
	static const uint8_t LUT_B2[] = { 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE, 0xFF };

	if (X1 > X2) {
		SWAP_VARS(X1, X2);
	}

	if (Y1 > Y2) {
		SWAP_VARS(Y1, Y2);
	}

	uint16_t dW;

	switch (lcd_orientation) {
		case LCD_ORIENT_180:
			dW = SCR_W - 1 - X2;
			X2 = SCR_W - 1 - X1;
			X1 = dW;
			break;
		case LCD_ORIENT_CCW:
			SWAP_VARS(X1, Y1);
			SWAP_VARS(X2, Y2);
			dW = SCR_W - 1 - X2;
			X2 = SCR_W - 1 - X1;
			X1 = dW;
			break;
		case LCD_ORIENT_CW:
			SWAP_VARS(X1, Y1);
			SWAP_VARS(X2, Y2);
			break;
		case LCD_ORIENT_NORMAL:
		default:
			break;
	}

	// Mask for first and last byte
	register uint8_t mask_fb = LUT_B1[X1 & 0x07];
	register uint8_t mask_lb = LUT_B2[X2 & 0x07];

	// Offset in vRAM
	uint8_t *ptr_base = &vRAM[(((Y1 * SCR_W) + X1) >> 3)];

	// Line width in bytes
	dW = (X2 >> 3) - (X1 >> 3);

	if (dW) {
		// Multiple bytes
		register uint16_t cntr;
		register uint8_t *ptr;

		switch (LCD_PixelMode) {
			case LCD_PRES:
				do {
					cntr = dW;
					ptr = ptr_base;
					*ptr++ |= mask_fb;
					while (--cntr) {
						*ptr++ = 0xFF;
					};
					*ptr |= mask_lb;
					ptr_base += SCR_W >> 3;
				} while (Y1++ < Y2);
				break;
			case LCD_PINV:
				do {
					cntr = dW;
					ptr = ptr_base;
					*ptr++ ^= mask_fb;
					while (--cntr) {
						*ptr++ ^= 0xFF;
					};
					*ptr ^= mask_lb;
					ptr_base += SCR_W >> 3;
				} while (Y1++ < Y2);
				break;
			case LCD_PSET:
			default:
				mask_fb = ~mask_fb;
				mask_lb = ~mask_lb;
				do {
					cntr = dW;
					ptr = ptr_base;
					*ptr++ &= mask_fb;
					while (--cntr) {
						*ptr++ = 0x00;
					};
					*ptr &= mask_lb;
					ptr_base += SCR_W >> 3;
				} while (Y1++ < Y2);
				break;
		}
	} else {
		// Single byte
		mask_fb &= mask_lb;

		switch (LCD_PixelMode) {
			case LCD_PRES:
				do {
					*ptr_base |= mask_fb;
					ptr_base += SCR_W >> 3;
				} while (Y1++ < Y2);
				break;
			case LCD_PINV:
				do {
					*ptr_base ^= mask_fb;
					ptr_base += SCR_W >> 3;
				} while (Y1++ < Y2);
				break;
			case LCD_PSET:
			default:
				mask_fb = ~mask_fb;
				do {
					*ptr_base &= mask_fb;
					ptr_base += SCR_W >> 3;
				} while (Y1++ < Y2);
				break;
		}
	}
}

// Draw line
// input:
//   X1,Y1 - top left coordinates
//   X2,Y2 - bottom right coordinates
void LCD_Line(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2) {
	int16_t dX = X2 - X1;
	int16_t dY = Y2 - Y1;
	int16_t dXsym = (dX > 0) ? 1 : -1;
	int16_t dYsym = (dY > 0) ? 1 : -1;

	if (dX == 0) {
		LCD_VLine(X1, Y1, Y2);
		return;
	}
	if (dY == 0) {
		LCD_HLine(X1, X2, Y1);
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
			LCD_Pixel(X1, Y1);
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
			LCD_Pixel(X1, Y1);
			Y1 += dYsym;
			if (di < 0) {
				di += dX2;
			} else {
				di += dX2 - dY2;
				X1 += dXsym;
			}
		}
	}
	LCD_Pixel(X1, Y1);
}

// Draw circle
// input:
//   Xc, Yc - coordinates of the center of the circle
//   R - circle radius
void LCD_Circle(int16_t Xc, int16_t Yc, uint16_t R) {
	int16_t err = 1 - R;
	int16_t dx  = 1;
	int16_t dy  = -2 * R;
	int16_t x   = 0;
	int16_t y   = R;

	register int16_t sh = scr_height;
	register int16_t sw = scr_width;
	register int16_t tt;

	// Vertical and horizontal points
	if (Xc + R < sw) LCD_Pixel(Xc + R, Yc);
	if (Xc - R > -1) LCD_Pixel(Xc - R, Yc);
	if (Yc + R < sh) LCD_Pixel(Xc, Yc + R);
	if (Yc - R > -1) LCD_Pixel(Xc, Yc - R);

	while (x < y) {
		if (err >= 0) {
			dy  += 2;
			err += dy;
			y--;
		}
		dx  += 2;
		err += dx + 1;
		x++;

		// Draw pixels of eight octants
		tt = Xc + x;
		if (tt < sw) {
			if (Yc + y < sh) LCD_Pixel(tt, Yc + y);
			if (Yc - y > -1) LCD_Pixel(tt, Yc - y);
		}
		tt = Xc - x;
		if (tt > -1) {
			if (Yc + y < sh) LCD_Pixel(tt, Yc + y);
			if (Yc - y > -1) LCD_Pixel(tt, Yc - y);
		}
		tt = Xc + y;
		if (tt < sw) {
			if (Yc + x < sh) LCD_Pixel(tt, Yc + x);
			if (Yc - x > -1) LCD_Pixel(tt, Yc - x);
		}
		tt = Xc - y;
		if (tt > -1) {
			if (Yc + x < sh) LCD_Pixel(tt, Yc + x);
			if (Yc - x > -1) LCD_Pixel(tt, Yc - x);
		}
	}
}

// Draw ellipse
// input:
//   Xc, Yc - coordinates of the center of the ellipse
//   Ra, Rb - horizontal and vertical radiuses
void LCD_Ellipse(int16_t Xc, int16_t Yc, uint16_t Ra, uint16_t Rb) {
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

	register int16_t sh = scr_height;
	register int16_t sw = scr_width;

	while ((y >= 0) && (x <= Ra)) {
		if ((Xc + x < sw) && (Yc + y < sh)) {
			LCD_Pixel(Xc + x, Yc + y);
		}
		if (x || y) {
			if ((Xc - x > -1) && (Yc - y > -1)) {
				LCD_Pixel(Xc - x, Yc - y);
			}
		}
		if (x && y) {
			if ((Xc + x < sw) && (Yc - y > - 1)) {
				LCD_Pixel(Xc + x, Yc - y);
			}
			if ((Xc - x > -1) && (Yc + y < sh)) {
				LCD_Pixel(Xc - x, Yc + y);
			}
		}

		if ((t + (x * B2) <= C1) || (t + (y * A2) <= C3)) {
			dX += dXt2;
			t  += dX;
			x++;
		} else if (t - (y * A2) > C2) {
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

// Text functions

// Draw a single character
// input:
//   X,Y - character top left corner coordinates
//   chr - character to be drawn
//   font - pointer to font
// return: character width in pixels
uint8_t LCD_PutChar(uint16_t X, uint16_t Y, uint8_t chr, const Font_TypeDef *font) {
	uint16_t pX;
	uint16_t pY;
	uint8_t tmpCh;
	uint8_t bL;
	const uint8_t *pCh;

	// If the specified character code is out of bounds should substitute the code of the "unknown" character
	if ((chr < font->font_MinChar) || (chr > font->font_MaxChar)) {
		chr = font->font_UnknownChar;
	}

	// Pointer to the first byte of character in font data array
	pCh = &font->font_Data[(chr - font->font_MinChar) * font->font_BPC];

	// Draw character
	if (font->font_Scan == FONT_V) {
		// Vertical pixels order
		if (font->font_Height < 9) {
			// Height is 8 pixels or less (one byte per column)
			pX = X;
			while (pX < X + font->font_Width) {
				pY = Y;
				tmpCh = *pCh++;
				while (tmpCh) {
					if (tmpCh & 0x01) {
						LCD_Pixel(pX, pY);
					}
					tmpCh >>= 1;
					pY++;
				}
				pX++;
			}
		} else {
			// Height is more than 8 pixels (several bytes per column)
			pX = X;
			while (pX < X + font->font_Width) {
				pY = Y;
				while (pY < Y + font->font_Height) {
					bL = 8;
					tmpCh = *pCh++;
					if (tmpCh) {
						while (bL) {
							if (tmpCh & 0x01) {
								LCD_Pixel(pX, pY);
							}
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
		if (font->font_Width < 9) {
			// Width is 8 pixels or less (one byte per row)
			pY = Y;
			while (pY < Y + font->font_Height) {
				pX = X;
				tmpCh = *pCh++;
				while (tmpCh) {
					if (tmpCh & 0x01) {
						LCD_Pixel(pX, pY);
					}
					tmpCh >>= 1;
					pX++;
				}
				pY++;
			}
		} else {
			// Width is more than 8 pixels (several bytes per row)
			pY = Y;
			while (pY < Y + font->font_Height) {
				pX = X;
				while (pX < X + font->font_Width) {
					bL = 8;
					tmpCh = *pCh++;
					if (tmpCh) {
						while (bL) {
							if (tmpCh & 0x01) {
								LCD_Pixel(pX, pY);
							}
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

	return font->font_Width + 1;
}

// Draw string
// input:
//   X,Y - top left coordinates of first character
//   str - pointer to zero-terminated string
//   font - pointer to font
// return: string width in pixels
uint16_t LCD_PutStr(uint16_t X, uint16_t Y, const char *str, const Font_TypeDef *font) {
	uint16_t pX = X;
	uint16_t eX = scr_width - font->font_Width - 1;

	while (*str) {
		pX += LCD_PutChar(pX, Y, *str++, font);
		if (pX > eX) break;
	}

	return (pX - X);
}

// Draw string with line feed by screen edge
// input:
//   X,Y - top left coordinates of first character
//   str - pointer to zero-terminated string
//   font - pointer to font
// return: string width in pixels
uint16_t LCD_PutStrLF(uint16_t X, uint16_t Y, const char *str, const Font_TypeDef *font) {
	uint32_t strLen = 0;

	while (*str) {
		LCD_PutChar(X, Y, *str++, font);
		if (X < scr_width - font->font_Width - 1) {
			X += font->font_Width + 1;
		} else if (Y < scr_height - font->font_Height - 1) {
			X = 0;
			Y += font->font_Height;
		} else {
			X = 0;
			Y = 0;
		}
		strLen++;
	};

	return strLen * (font->font_Width + 1);
}

// Draw signed integer value
// input:
//   X,Y - top left coordinates of first symbol
//   num - signed integer value
//   font - pointer to font
// return: number width in pixels
uint8_t LCD_PutInt(uint16_t X, uint16_t Y, int32_t num, const Font_TypeDef *font) {
	uint8_t str[11]; // 10 chars max for INT32_MIN..INT32_MAX (without sign)
	uint8_t *pStr = str;
	uint8_t neg = 0;
	uint16_t pX = X;

	// String termination character
	*pStr++ = '\0';

	// Convert number to characters
	if (num < 0) {
		neg = 1;
		num *= -1;
	}
	do { *pStr++ = (num % 10) + '0'; } while (num /= 10);
	if (neg) {
		*pStr++ = '-';
	}

	// Draw a number
	while (*--pStr) {
		pX += LCD_PutChar(pX, Y, *pStr, font);
	}

	return (pX - X);
}

// Draw unsigned integer value
// input:
//   X,Y - top left coordinates of first symbol
//   num - unsigned integer value
//   font - pointer to font
// return: number width in pixels
uint8_t LCD_PutIntU(uint16_t X, uint16_t Y, uint32_t num, const Font_TypeDef *font) {
	uint8_t str[11]; // 10 chars max for UINT32_MAX
	uint8_t *pStr = str;
	uint16_t pX = X;

	// Convert number to characters
	*pStr++ = 0; // String termination character
	do { *pStr++ = (num % 10) + '0'; } while (num /= 10);

	// Draw a number
	while (*--pStr) {
		pX += LCD_PutChar(pX, Y, *pStr, font);
	}

	return (pX - X);
}

// Draw signed integer value with decimal point
// input:
//   X,Y - top left coordinates of first symbol
//   num - unsigned integer value
//   decimals - number of digits after decimal point
//   font - pointer to font
// return: number width in pixels
uint8_t LCD_PutIntF(uint16_t X, uint16_t Y, int32_t num, uint8_t decimals, const Font_TypeDef *font) {
	uint8_t str[11]; // 10 chars max for INT32_MIN..INT32_MAX (without sign)
	uint8_t *pStr = str;
	uint8_t neg = 0;
	uint8_t strLen = 0;
	uint16_t pX = X;

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
		pX += LCD_PutChar(pX, Y, *pStr, font);
		if (decimals && (--strLen == decimals)) {
			// Draw decimal point
			LCD_Rect(pX, Y + font->font_Height - 2, pX + 1, Y + font->font_Height - 1);
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
//   font - pointer to font
// return: number width in pixels
uint8_t LCD_PutIntLZ(uint16_t X, uint16_t Y, int32_t num, uint8_t digits, const Font_TypeDef *font) {
	uint8_t str[11]; // 10 chars max for INT32_MIN..INT32_MAX (without sign)
	uint8_t *pStr = str;
	uint8_t neg = 0;
	uint8_t strLen = 0;
	uint16_t pX = X;

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
	if (strLen < digits) {
		while (strLen++ < digits) {
			*pStr++ = '0';
		}
	}

	// Minus sign?
	if (neg) *pStr++ = '-';

	// Draw a number
	while (*--pStr) {
		pX += LCD_PutChar(pX, Y, *pStr, font);
	}

	return (pX - X);
}

// Draw integer as hexadecimal
// input:
//   X,Y - top left coordinates of first symbol
//   num - unsigned integer value
//   font - pointer to font
// return: number width in pixels
uint8_t LCD_PutHex(uint16_t X, uint16_t Y, uint32_t num, const Font_TypeDef *font) {
	uint8_t str[11]; // 10 chars max for UINT32_MAX
	uint8_t *pStr = str;
	uint16_t pX = X;

	// Convert number to characters
	*pStr++ = 0; // String termination character
	do {
		*pStr = (num % 0x10) + '0';
		if (*pStr > '9') {
			*pStr += 7;
		}
		pStr++;
	} while (num /= 0x10);

	// Draw a number
	while (*--pStr) {
		pX += LCD_PutChar(pX, Y, *pStr, font);
	}

	return (pX - X);
}

// Draw monochrome bitmap
// input:
//   X, Y - top left corner coordinates of bitmap
//   W, H - width and height of bitmap in pixels
//   pBMP - pointer to array containing bitmap
// note: each '1' bit in the bitmap will be drawn as a pixel
//       each '0' bit in the will not be drawn (transparent bitmap)
// bitmap: one byte per 8 vertical pixels, LSB top, truncate bottom bits
void LCD_DrawBitmap(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, const uint8_t* pBMP) {
	uint16_t pX;
	uint16_t pY;
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
					if (tmpCh & 0x01) {
						LCD_Pixel(pX, pY + bL);
					}
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

// Invert area of image in video buffer
// input:
//   X,Y - coordinates of top left area corner
//   W - area width (pixels)
//   H - area height (pixels)
void LCD_Invert(uint16_t X, uint16_t Y, uint16_t W, uint16_t H) {
	uint8_t tmp;

	// Inverting part of image with support of screen rotation functionality
	// is a non-trivial task (look at LCD_FillRect...)
	// Therefore do that in the most simple and blunt way...

	// Save value of PixelMode variable
	tmp = LCD_PixelMode;
	// Change drawing mode to INVERT
	LCD_PixelMode = LCD_PINV;
	// Draw a filled rectangle
	LCD_FillRect(X, Y, X + W - 1, Y + H - 1);
	// Restore previous value of PixelMode
	LCD_PixelMode = tmp;
}

// Invert a whole video buffer
// note: size of video buffer must be a multiple of 4
void LCD_InvertFull(void) {
	register uint32_t *ptr = (uint32_t *)vRAM;
	register uint32_t i = sizeof(vRAM) >> 2;

	while (i--) {
		*ptr++ ^= 0xFFFFFFFF;
	}
}
