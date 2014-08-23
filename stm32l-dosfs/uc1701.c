#include <stm32l1xx_gpio.h>
#include <stm32l1xx_rcc.h>
#include <string.h> // For memcpy

#include <spi.h>
#include <delay.h>
#include <uc1701.h>
#include <resources.h>


uint16_t                  scr_width        = SCR_W;
uint16_t                  scr_height       = SCR_H;

ScrOrientation_TypeDef    scr_orientation  = scr_normal;

uint16_t                  UC1701_SPI_state;

uint8_t vRAM[SCR_W * SCR_H / 8]; // Display buffer


// Send command to display controller
// input:
//   cmd - 8-bit command to send
void UC1701_cmd(uint8_t cmd) {
	UC1701_RS_L();
	SPIx_SendRecv(UC1701_SPI_PORT,cmd);
}

// Send double byte command to display controller
// input:
//   cmd1 - first byte of command to send
//   cmd2 - second byte of command to send
void UC1701_cmd_double(uint8_t cmd1, uint8_t cmd2) {
	UC1701_RS_L();
	SPIx_SendRecv(UC1701_SPI_PORT,cmd1);
	SPIx_SendRecv(UC1701_SPI_PORT,cmd2);
}

// Send data byte to display controller
// input:
//   data - date byte to send
void UC1701_data(uint8_t data) {
	UC1701_RS_H();
	SPIx_SendRecv(UC1701_SPI_PORT,data);
}

// Save CS pin state and disable it
void UC1701_PauseSPI(void) {
	UC1701_SPI_state = UC1701_CS_PORT->IDR & UC1701_CS_PIN;
	UC1701_CS_H();
}

// Restore previously saved CS pin state
void UC1701_ResumeSPI(void) {
	UC1701_CS_PORT->BSRRL = UC1701_SPI_state;
}

// Set backlight brightness
// input:
//   brightness - backlight brightness level (0..100); 0 means PWM off
void UC1701_SetBacklight(uint8_t brightness) {
	GPIO_InitTypeDef PORT;

	PORT.GPIO_Speed = GPIO_Speed_40MHz;
	PORT.GPIO_OType = GPIO_OType_PP;
	PORT.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	PORT.GPIO_Pin   = UC1701_LEDA_PIN;

	if (brightness == 0 || brightness == 100) {
		if (RCC->APB1ENR & RCC_APB1Periph_TIM2) {
			// Turn off TIM2 peripheral to conserve some power
			TIM2->CCR2 = brightness;
			TIM2->CR1 &= ~TIM_CR1_CEN; // Disable TIM2 counter
			RCC->APB1ENR &= ~RCC_APB1Periph_TIM2; // Disable the TIM2 peripheral

			// Configure LEDA control pin as push-pull output
			PORT.GPIO_Mode  = GPIO_Mode_OUT;
			GPIO_Init(UC1701_LEDA_PORT,&PORT);
		}
		if (brightness == 0) UC1701_LEDA_H(); else UC1701_LEDA_L();

		return;
	}

	if (!(RCC->APB1ENR & RCC_APB1Periph_TIM2)) {
		// PWM disabled, enable it first
		PORT.GPIO_Mode  = GPIO_Mode_AF;
		GPIO_Init(UC1701_LEDA_PORT,&PORT);
		GPIO_PinAFConfig(UC1701_LEDA_PORT,GPIO_PinSource1,GPIO_AF_TIM2); // Alternative function PA1 -> TIM2_CH2
		RCC->APB1ENR |= RCC_APB1Periph_TIM2; // Enable the TIM2 peripheral
		TIM2->CR1 |= TIM_CR1_CEN; // Counter enable
	}
	TIM2->CCR2 = brightness; // Set PWM duty cycle
}

// Display initialization
void UC1701_Init(void) {
	GPIO_InitTypeDef PORT;

	//////////////////////////////////////////////////
	// ****** SPI must be initialized before ****** //
	//////////////////////////////////////////////////

	RCC_AHBPeriphClockCmd(UC1701_PORT_PERIPH,ENABLE); // Enable PORTA and PORTB peripherals

	// Configure display control lines as push-pull output with pullup
	PORT.GPIO_Mode  = GPIO_Mode_OUT;
	PORT.GPIO_Speed = GPIO_Speed_40MHz;
	PORT.GPIO_OType = GPIO_OType_PP;
	PORT.GPIO_PuPd  = GPIO_PuPd_UP;
/*
	PORT.GPIO_Pin = UC1701_CS_PIN;
	GPIO_Init(UC1701_CS_PORT,&PORT);
	PORT.GPIO_Pin = UC1701_RST_PIN;
	GPIO_Init(UC1701_RST_PORT,&PORT);
	PORT.GPIO_Pin = UC1701_RS_PIN;
	GPIO_Init(UC1701_RS_PORT,&PORT);
*/
	// Since all control pins in same port, do this in once command
	PORT.GPIO_Pin = UC1701_CS_PIN | UC1701_RST_PIN | UC1701_RS_PIN;
	GPIO_Init(UC1701_CS_PORT,&PORT);

	// Backlight LED control line
	PORT.GPIO_Mode  = GPIO_Mode_AF;
	PORT.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	PORT.GPIO_Pin   = UC1701_LEDA_PIN;
	GPIO_Init(UC1701_LEDA_PORT,&PORT);
	GPIO_PinAFConfig(UC1701_LEDA_PORT,GPIO_PinSource1,GPIO_AF_TIM2); // Alternative function PA1 -> TIM2_CH2
	// Configure TIM2_CH2 as PWM output
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE); // Enable TIM2 peripheral
	TIM2->CR1   |= TIM_CR1_ARPE; // Auto-preload enable
	TIM2->CCMR1 |= TIM_CCMR1_OC2PE; // Output compare 2 preload enable
	TIM2->CCMR1 |= TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1; // PWM mode 1
//	TIM2->PSC    = 0x0318; // TIM2 prescaler (32MHz AHB -> 400Hz PWM) [ PSC = APB1clock / (PWMfreq * PWMlevels) ]
	TIM2->PSC    = SystemCoreClock / 40000; // PWM frequency 400Hz
	TIM2->ARR    = 99; // TIM2 auto reload value
	TIM2->CCR2   = 50; // 50% duty cycle
	TIM2->CCER  |= TIM_CCER_CC2P; // Output polarity
	TIM2->CCER  |= TIM_CCER_CC2E; // TIM2_CH2 output compare enable
	TIM2->EGR    = 1; // Generate an update event to reload the prescaler value immediately
	TIM2->CR1   |= TIM_CR1_CEN; // Counter enable

	// Reset display
	UC1701_CS_H();
	UC1701_RST_L();
	Delay_ms(1); // Must hold RST low at least 1ms
	UC1701_RST_H();
	Delay_ms(5); // Wait at least 5ms
	UC1701_CS_L();

	UC1701_cmd(0xe2); // Software display reset
	UC1701_cmd(0x2f); // Power control: Boost ON,  V.Regular ON,  V.Follower ON
	UC1701_cmd(0xa2); // Set LCD bias ratio (BR = 0)
	UC1701_cmd(0xaf); // Display enable

	UC1701_cmd_double(0xfa,0x93); // Advanced program control 0:
								  //   Temperature compensation -0.11%/C
								  //   PA wrap around enabled, CA wrap around enabled

	UC1701_CS_H();
}

// Reset display registers (software display reset)
// Doesn't affect the display memory
void UC1701_Reset(void) {
	UC1701_CS_L();
	UC1701_cmd(0xe2); // Software display reset
	UC1701_CS_H();
}

// Set LCD contrast
// input:
//   res_ratio: internal resistor ratio [0..7], power on value is 4
//   el_vol: electronic volume [0..63], power on value is 32
void UC1701_Contrast(uint8_t res_ratio, uint8_t el_vol) {
	UC1701_CS_L();
	UC1701_cmd(0x20 | (res_ratio & 0x07));
	UC1701_cmd_double(0x81,el_vol & 0x3f);
	UC1701_CS_H();
}

// Set all LCD pixels on or off
// input:
//   state: ON or OFF
// note: doesn't affect the display memory
void UC1701_SetAllPixelOn(OnOffStatus state) {
	UC1701_CS_L();
	UC1701_cmd(state == ON ? 0xa5 : 0xa4);
	UC1701_CS_H();
}

// Set inverse display pixels
// Input:
//   state: NORMAL or INVERT
void UC1701_SetInvert(InvertStatus state) {
	UC1701_CS_L();
	UC1701_cmd(state == NORMAL ? 0xa6 : 0xa7);
	UC1701_CS_H();
}

// Toggle display on/off
// input:
//   state: ENABLED or DISABLED
// note: doesn't affect the display memory
void UC1701_SetDisplayState(DisplayState state) {
	UC1701_CS_L();
	UC1701_cmd(state == ENABLED ? 0xaf : 0xae);
	UC1701_CS_H();
}

// Set X coordinate mapping (normal or mirrored)
// input:
//   state: NORMAL or INVERT
// note: Doesn't affect the display memory
void UC1701_SetXDir(InvertStatus MX) {
	UC1701_CS_L();
	UC1701_cmd(MX == NORMAL ? 0xa0 : 0xa1);
	UC1701_CS_H();
}

// Set Y coordinate mapping (normal or mirrored)
// input:
//   state: NORMAL or INVERT
// note: doesn't affect the display memory
void UC1701_SetYDir(InvertStatus MY) {
	UC1701_CS_L();
	UC1701_cmd(MY == NORMAL ? 0xc8 : 0xc0);
	UC1701_CS_H();
}

// Set display column:page according to pixel coordinates
// input:
//   X, Y - pixel coordinates
void UC1701_SetAddr(uint8_t X, uint8_t Y) {
	UC1701_CS_L();
	UC1701_cmd(X & 0x0f); // Column address LSB
	UC1701_cmd((X > 4) | 0x10); // Column address MSB
	UC1701_cmd(((Y / 8) & 0x0f) | 0xb0); // Page
	UC1701_CS_H();
}

// Set scroll line number
// input:
//   line - start line number (0..63)
void UC1701_SetScrollLine(uint8_t line) {
	UC1701_CS_L();
	UC1701_cmd(0x40 | (line & 0x3f));
	UC1701_CS_H();
}

// Set display orientation
// input:
//   orientation - screen orientation (scr_*)
// note: doesn't affect on current display memory
void UC1701_Orientation(uint8_t orientation) {
	UC1701_CS_L();
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
	UC1701_CS_H();
}

// Send vRAM buffer content into display
void UC1701_Flush(void) {
	uint16_t i,j;
	uint16_t offset;

	offset = 0;
	UC1701_CS_L();
	for (j = 0; j < 8; j++) {
		 // Column 0 address LSB
		if (scr_orientation == scr_180 || scr_orientation == scr_CCW) UC1701_cmd(0x04); else UC1701_cmd(0x00);
		UC1701_cmd(0x10); // Column 0 address MSB
		UC1701_cmd(0xb0 | j); // Page address
		UC1701_RS_H(); // Send data
		for (i = 0; i < 128; i++) SPIx_SendRecv(UC1701_SPI_PORT,vRAM[offset + i]);
		offset += SCR_W; // Don't use multiplication
	}
	UC1701_CS_H();
}

// Fill vRAM memory with specified pattern
// input:
//   pattern - byte pattern to fill vRAM memory
void UC1701_Fill(uint8_t pattern) {
	memset(vRAM,pattern,sizeof(vRAM));
}

// Set pixel in vRAM buffer
// input:
//   X,Y - coordinates of pixel
void SetPixel(uint8_t X, uint8_t Y) {
	uint8_t XX = X;
	uint8_t YY = Y;

	if (scr_orientation == scr_CW || scr_orientation == scr_CCW) {
		XX = Y; YY = X;
	}
	vRAM[((YY >> 3) * SCR_W) + XX] |= 1 << (YY % 8);
}

// Clear pixel in vRAM buffer
// input:
//   X,Y - coordinates of pixel
void ResetPixel(uint8_t X, uint8_t Y) {
	uint8_t XX = X;
	uint8_t YY = Y;

	if (scr_orientation == scr_CW || scr_orientation == scr_CCW) {
		XX = Y; YY = X;
	}
	vRAM[((YY >> 3) * SCR_W) + XX] &= ~(1 << (YY % 8));
}

// Invert rectangle in vRAM buffer
// input:
//   X,Y - top left coordinates of rectangle
//   W - rectangle width
//   H - rectangle height
void InvertRect(uint8_t X, uint8_t Y, uint8_t W, uint8_t H) {
	uint8_t i,j,pX;

	for (j = 0; j < H; j++) {
		pX = X;
		for (i = 0; i < W; i++) {
		 	vRAM[((Y >> 3) * SCR_W) + pX] ^= (1 << (Y % 8));
		 	pX++;
		}
		Y++;
	}
}

// Draw horizontal line
// input:
//   X1 - left horizontal coordinate of the line
//   X2 - right horizontal coordinate of the line
//   Y - vertical coordinate of the line
//   SR - Set or reset line pixels (PSet or PReset)
// note: X2 must be greater than X1
void HLine(uint8_t X1, uint8_t X2, uint8_t Y, PSetReset_TypeDef SR) {
	uint8_t x;

	if (SR == PSet) {
		for (x = X1; x <= X2; x++) SetPixel(x,Y);
	} else {
		for (x = X1; x <= X2; x++) ResetPixel(x,Y);
	}
}

// Draw vertical line
// input:
//   X - horizontal coordinate of the line
//   Y1 - top vertical coordinate of the line
//   Y2 - bottom vertical coordinate of the line
//   SR - Set or reset line pixels (PSet or PReset)
// note: Y2 must be greater than Y1
void VLine(uint8_t X, uint8_t Y1, uint8_t Y2, PSetReset_TypeDef SR) {
	uint8_t y;

	if (SR == PSet) {
		for (y = Y1; y <= Y2; y++) SetPixel(X,y);
	} else {
		for (y = Y1; y <= Y2; y++) ResetPixel(X,y);
	}
}

// Draw rectangle
// input:
//   X1 - left horizontal coordinate of the line
//   X2 - right horizontal coordinate of the line
//   Y1 - top vertical coordinate of the line
//   Y2 - bottom vertical coordinate of the line
//   SR - Set or reset rectangle pixels (PSet or PReset)
// note: must be (X2 > X1) and (Y2 > Y1)
void Rect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2, PSetReset_TypeDef SR) {
	HLine(X1,X2,Y1,SR);
	HLine(X1,X2,Y2,SR);
	VLine(X1,Y1 + 1,Y2 - 1,SR);
	VLine(X2,Y1 + 1,Y2 - 1,SR);
}

// Draw filled rectangle
// input:
//   X1 - left horizontal coordinate of the line
//   X2 - right horizontal coordinate of the line
//   Y1 - top vertical coordinate of the line
//   Y2 - bottom vertical coordinate of the line
//   SR - Set or reset rectangle pixels (PSet or PReset)
// note: must be (X2 > X1) and (Y2 > Y1)
void FillRect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2, PSetReset_TypeDef SR) {
	uint8_t y;

	for (y = Y1; y <= Y2; y++) HLine(X1,X2,y,SR);
}

// Draw line
// input:
//   X1 - left horizontal coordinate of the line
//   X2 - right horizontal coordinate of the line
//   Y1 - top vertical coordinate of the line
//   Y2 - bottom vertical coordinate of the line
void Line(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2) {
	int16_t dX = X2-X1;
	int16_t dY = Y2-Y1;
	int16_t dXsym = (dX > 0) ? 1 : -1;
	int16_t dYsym = (dY > 0) ? 1 : -1;

	if (dX == 0) {
		if (Y2 > Y1) VLine(X1,Y1,Y2,PSet); else VLine(X1,Y2,Y1,PSet);
		return;
	}
	if (dY == 0) {
		if (X2 > X1) HLine(X1,X2,Y1,PSet); else HLine(X2,X1,Y1,PSet);
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
			SetPixel(X1,Y1);
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
			SetPixel(X1,Y1);
			Y1 += dYsym;
			if (di < 0) {
				di += dX2;
			} else {
				di += dX2 - dY2;
				X1 += dXsym;
			}
		}
	}
	SetPixel(X1,Y1);
}

// Draw ellipse
// input:
//   X,Y - ellipse center coordinates
//   A,B - horizontal and vertical radius of the ellipse
void Ellipse(uint16_t X, uint16_t Y, uint16_t A, uint16_t B) {
	int16_t Xc = 0, Yc = B;

	long A2 = (long)A*A, B2 = (long)B*B;
	long C1 = -(A2/4 + A % 2 + B2);
	long C2 = -(B2/4 + B % 2 + A2);
	long C3 = -(B2/4 + B % 2);
	long t = -A2 * Yc;
	long dXt = B2*Xc*2, dYt = -A2*Yc*2;
	long dXt2 = B2*2, dYt2 = A2*2;
	while (Yc >= 0 && Xc <= A) {
		SetPixel(X + Xc,Y + Yc);
		if (Xc != 0 || Yc != 0) SetPixel(X - Xc,Y - Yc);
		if (Xc != 0 && Yc != 0) {
			SetPixel(X + Xc,Y - Yc);
			SetPixel(X - Xc,Y + Yc);
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
uint8_t PutChar(uint8_t X, uint8_t Y, uint8_t Char, const Font_TypeDef *Font) {
	uint8_t chW,chH;
	uint8_t pX = X;
	uint8_t pY = Y;
	uint16_t i,j,k;
	uint8_t buffer[32];
	uint8_t tmpCh;

	chW = Font->font_Width; // Character width
	chH = Font->font_Height; // Character height
	if (Char < 32 || Char > 0x7e) Char = 0x7e;
	memcpy(buffer,&Font->font_Data[(Char - 32) * Font->font_BPC],Font->font_BPC);

	if (Font->font_Scan == font_V) {
		if (chH < 8) {
			// Small font, one byte height
			for (i = 0; i < chW; i++) {
				tmpCh = buffer[i];
				for (j = 0; j < chH; j++) {
					if ((tmpCh >> j) & 0x01) SetPixel(pX,pY);
					pY++;
				}
				pY = Y;
				pX++;
			}
		} else {
			// Big font, more than one byte height
			for (k = 0; k <= (chH / 8); k++) {
				for (i = 0; i < chW; i++) {
					tmpCh = buffer[(i << 1) + k];
					for (j = 0; j < 8; j++) {
						if ((tmpCh >> (7 - j)) & 0x01) SetPixel(pX,pY);
						pY++;
						if (pY > Y + chH) break;
					}
					pX++;
				}
				pY = Y + (k << 3);
				pX = X;
			}
		}
	} else {
		for (j = 0; j < chH; j++) {
			tmpCh = buffer[j];
			for (i = 0; i < chW; i++) {
				if ((tmpCh >> i) & 0x01) SetPixel(pX,pY);
				pX++;
			}
			pX = X;
			pY++;
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
uint16_t PutStr(uint8_t X, uint8_t Y, char *str, const Font_TypeDef *Font) {
	uint8_t strLen = 0;

    while (*str) {
        X += PutChar(X,Y,*str++,Font);
        if (X > scr_width - Font->font_Width - 1) break;
        strLen++;
    };

    return strLen * (Font->font_Width + 1);
}

// Draw string with line feed (by screen edge)
// input:
//   X,Y - top left coordinates of first character
//   str - pointer to zero-terminated string
//   Font - pointer to font
// return: string width in pixels
uint16_t PutStrLF(uint8_t X, uint8_t Y, char *str, const Font_TypeDef *Font) {
	uint8_t strLen = 0;

    while (*str) {
        PutChar(X,Y,*str++,Font);
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
	uint8_t str[11]; // 10 chars max for UINT32_MAX
	int8_t i = 0;
	uint8_t neg = 0;
	int8_t intLen;
	uint8_t pX;

	if (num < 0) {
		neg = 1;
		num *= -1;
	}
	do { str[i++] = num % 10 + '0'; } while ((num /= 10) > 0);
	if (neg) str[i++] = '-';
	intLen = i;
	pX = X + (intLen - 1) * (Font->font_Width + 1);
	for (i = 0; i < intLen; i++) {
		pX -= PutChar(pX,Y,str[i],Font);
	}

    return intLen * (Font->font_Width + 1);
}

// Draw unsigned integer value
// input:
//   X,Y - top left coordinates of first symbol
//   num - unsigned integer value
//   Font - pointer to font
// return: number width in pixels
uint8_t PutIntU(uint8_t X, uint8_t Y, uint32_t num, const Font_TypeDef *Font) {
	uint8_t str[11]; // 10 chars max for UINT32_MAX
	int8_t i = 0;
	int8_t intLen;
	uint8_t pX;

	do { str[i++] = num % 10 + '0'; } while ((num /= 10) > 0);
	intLen = i;
	pX = X + (intLen - 1) * (Font->font_Width + 1);
	for (i = 0; i < intLen; i++) {
		pX -= PutChar(pX,Y,str[i],Font);
	}

    return intLen * (Font->font_Width + 1);
}

// Draw signed integer value with decimal point
// input:
//   X,Y - top left coordinates of first symbol
//   num - unsigned integer value
//   decimals - number of digits after decimal point
//   Font - pointer to font
// return: number width in pixels
uint8_t PutIntF(uint8_t X, uint8_t Y, int32_t num, uint8_t decimals, const Font_TypeDef *Font) {
	uint8_t str[12];
	int8_t i;
	uint8_t neg;
	int8_t strLen;

	if (num < 0) {
		neg  = 1;
		num *= -1;
	} else neg = 0;

	i = 0;
	do { str[i++] = num % 10 + '0'; } while ((num /= 10) > 0);
	strLen = i;
	if (strLen <= decimals) {
		for (i = strLen; i <= decimals; i++) str[i] = '0';
		strLen = decimals + 1;
	}
	if (neg) str[strLen++] = '-';

	neg = X;
	for (i = 0; i < strLen; i++) {
		PutChar(neg,Y,str[strLen - i - 1],Font);
		neg += Font->font_Width + 1;
		if (strLen - i - 1 == decimals && decimals != 0) {
			Rect(neg,Y + Font->font_Height - 2,neg + 1,Y + Font->font_Height - 1,PSet);
			neg += 3;
		}
	}

	return (neg - X);
}

// Draw signed integer value with leading zeros
// input:
//   X,Y - top left coordinates of first symbol
//   num - unsigned integer value
//   digits - minimal number of length (e.g. num=35, digits=5 --> 00035)
//   Font - pointer to font
// return: number width in pixels
uint8_t PutIntLZ(uint8_t X, uint8_t Y, int32_t num, uint8_t digits, const Font_TypeDef *Font) {
	uint8_t i;
	uint8_t neg;
	int32_t tmp = num;
	uint8_t len = 0;
	uint8_t pX = X;

	if (tmp < 0) {
		neg  = 1;
		tmp *= -1;
	} else neg = 0;

	do { len++; } while ((tmp /= 10) > 0); // Calculate number length in symbols

	if (len > digits) {
		X += PutInt(X,Y,num,Font);
		return X;
	}

	if (neg) {
		PutChar(X,Y,'-',Font);
		X += Font->font_Width + 1;
		num *= -1;
	}
	for (i = 0; i < digits - len; i++) {
		PutChar(X,Y,'0',Font);
		X += Font->font_Width + 1;
	}
	X += PutInt(X,Y,num,Font);

	return X - pX;
}

// Draw integer as hexadecimal
// input:
//   X,Y - top left coordinates of first symbol
//   num - unsigned integer value
//   Font - pointer to font
// return: number width in pixels
uint8_t PutHex(uint8_t X, uint8_t Y, uint32_t num, const Font_TypeDef *Font) {
	uint8_t str[11]; // 10 chars max for UINT32_MAX
	int8_t i = 0;
	uint32_t onum = num;
	int8_t intLen;
	uint8_t pX;

	do { str[i++] = "0123456789ABCDEF"[num % 0x10]; } while ((num /= 0x10) > 0);
	if (onum < 0x10) str[i++] = '0';

	intLen = i;

	pX = X + (intLen - 1) * (Font->font_Width + 1);
	for (i = 0; i < intLen; i++) {
		pX -= PutChar(pX,Y,str[i],Font);
	}

    return intLen * (Font->font_Width + 1);
}

// Draw small digit (3x5)
// input:
//   X,Y - digit top left corner coordinate
//   digit - digit to draw
void PutDigit3x5(uint8_t X, uint8_t Y, uint8_t digit) {
	uint8_t i,j;
	uint8_t ch;

    for (i = 0; i < 3; i++) {
    	ch = digits_3x5[i + digit * 3];
    	for (j = 0; j < 5; j++) {
    		if ((ch >> j) & 0x01) SetPixel(X + i,Y + j); else ResetPixel(X + i,Y + j);
    	}
    }
}

// Draw unsigned integer with leading zeros in small 3x5 font
// input:
//   X,Y - number top left corner coordinate
//   num - number to draw
//   digits - number of leading zeros (0 for none)
// return:
//   number width in pixels
uint8_t PutIntULZ3x5(uint8_t X, uint8_t Y, uint32_t num, uint8_t digits) {
	uint8_t i = 0;
	uint8_t strLen;
	uint8_t str[11];

	do { str[i++] = num % 10; } while ((num /= 10) > 0);
	strLen = i;

	if (strLen < digits) for (i = 0; i < digits - strLen; i++) {
		PutDigit3x5(X,Y,0);
		X += 4;
	}

	for (i = strLen; i > 0; i--) {
		PutDigit3x5(X,Y,str[i - 1]);
		X += 4;
	}

	if (strLen < digits) return digits * 4; else return strLen * 4;
}
