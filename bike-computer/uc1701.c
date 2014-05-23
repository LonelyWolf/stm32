#include <stm32l1xx_gpio.h>
#include <stm32l1xx_rcc.h>
#include <stm32l1xx_tim.h>
#include <string.h> // For memcpy

#include <spi.h>
#include <delay.h>
#include <uc1701.h>

#include <font5x7.h>


uint16_t                  scr_width        = SCR_W;
uint16_t                  scr_height       = SCR_H;

ScrOrientation_TypeDef    scr_orientation  = scr_normal;

uint16_t                  UC1701_SPI_state;

uint8_t vRAM[SCR_W * SCR_H / 8]; // Display buffer


// Send command to display controller
void UC1701_cmd(uint8_t cmd) {
	UC1701_RS_L();
	SPI2_Send(cmd);
}

// Send double byte command to display controller
void UC1701_cmd_double(uint8_t cmd1, uint8_t cmd2) {
	UC1701_RS_L();
	SPI2_Send(cmd1);
	SPI2_Send(cmd2);
}

// Send data byte to display controller
void UC1701_data(uint8_t data) {
	UC1701_RS_H();
	SPI2_Send(data);
}

// Convert 3-byte RGB color into 2-byte RGB565
uint16_t RGB565(uint8_t R,uint8_t G,uint8_t B) {
	return ((R >> 3) << 11) | ((G >> 2) << 5) | (B >> 3);
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

	if (brightness == 0 || brightness == 100) {
		// Turn off TIM2 peripheral to conserve some power
		TIM2->CCR2 = brightness;
		TIM2->CR1 &= ~TIM_CR1_CEN; // Disable TIM2 counter
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,DISABLE); // Disable TIM2 peripheral

		// Configure LEDA control pin as push-pull output
		PORT.GPIO_Mode  = GPIO_Mode_OUT;
		PORT.GPIO_Speed = GPIO_Speed_40MHz;
		PORT.GPIO_OType = GPIO_OType_PP;
		PORT.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		PORT.GPIO_Pin   = UC1701_LEDA_PIN;
		GPIO_Init(UC1701_LEDA_PORT,&PORT);
		if (brightness == 0) UC1701_LEDA_H(); else UC1701_LEDA_L();

		return;
	}

	if (!(TIM2->CR1 & TIM_CR1_CEN)) {
		// PWM disabled, enable it first
		PORT.GPIO_Mode  = GPIO_Mode_AF;
		PORT.GPIO_Speed = GPIO_Speed_40MHz;
		PORT.GPIO_OType = GPIO_OType_PP;
		PORT.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		PORT.GPIO_Pin   = UC1701_LEDA_PIN;
		GPIO_Init(UC1701_LEDA_PORT,&PORT);
		GPIO_PinAFConfig(UC1701_LEDA_PORT,GPIO_PinSource1,GPIO_AF_TIM2); // Alternative function PA1 -> TIM2_CH2
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE); // Enable TIM2 peripheral
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
	TIM2->PSC    = 0x0318; // TIM2 prescaler (32MHz AHB -> 400Hz PWM) [ PSC = APB1clock / (PWMfreq * PWMlevels) ]
	TIM2->ARR    = 100; // TIM2 auto reload value
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

	UC1701_cmd(0xe2); // Software system reset
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
	UC1701_cmd(0xe2);
	UC1701_CS_H();
}

// Set LCD contrast
// Input:
//   res_ratio: internal resistor ratio [0..7], power on value is 4
//   el_vol: electronic volume [0..63], power on value is 32
void UC1701_Contrast(uint8_t res_ratio, uint8_t el_vol) {
	UC1701_CS_L();
	UC1701_cmd(0x20 | (res_ratio & 0x07));
	UC1701_cmd_double(0x81,el_vol & 0x3f);
	UC1701_CS_H();
}

// Set all LCD pixels on or off
// Input:
//   state: ON or OFF
// Doesn't affect the display memory
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
// Input:
//   state: ENABLED or DISABLED
// Doesn't affect the display memory
void UC1701_SetDisplayState(DisplayState state) {
	UC1701_CS_L();
	UC1701_cmd(state == ENABLED ? 0xaf : 0xae);
	UC1701_CS_H();
}

// Set X coordinate mapping (normal or mirrored)
// Input:
//   state: NORMAL or INVERT
// Doesn't affect the display memory
void UC1701_SetXDir(InvertStatus MX) {
	UC1701_CS_L();
	UC1701_cmd(MX == NORMAL ? 0xa0 : 0xa1);
	UC1701_CS_H();
}

// Set Y coordinate mapping (normal or mirrored)
// Input:
//   state: NORMAL or INVERT
// Doesn't affect the display memory
void UC1701_SetYDir(InvertStatus MY) {
	UC1701_CS_L();
	UC1701_cmd(MY == NORMAL ? 0xc8 : 0xc0);
	UC1701_CS_H();
}

// Set display column:page according to pixel coordinates
// Input:
//   X, Y - pixel coordinates
void UC1701_SetAddr(uint8_t X, uint8_t Y) {
	UC1701_CS_L();
	UC1701_cmd(X & 0x0f); // Column address LSB
	UC1701_cmd((X > 4) | 0x10); // Column address MSB
	UC1701_cmd(((Y / 8) & 0x0f) | 0xb0); // Page
	UC1701_CS_H();
}

// Set scroll line number
// Input:
//   line - start line number (0..63)
void UC1701_SetScrollLine(uint8_t line) {
	UC1701_CS_L();
	UC1701_cmd(0x40 | (line & 0x3f));
	UC1701_CS_H();
}

// Set display orientation
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

	UC1701_CS_L();
	for (j = 0; j < 8; j++) {
		 // Column 0 address LSB
		if (scr_orientation == scr_180 || scr_orientation == scr_CCW) UC1701_cmd(0x04); else UC1701_cmd(0x00);
		UC1701_cmd(0x10); // Column 0 address MSB
		UC1701_cmd(0xb0 | j); // Page address
		for (i = 0; i < 128; i++) {
			UC1701_data(vRAM[(j * SCR_W) + i]);
		}
	}
	UC1701_CS_H();
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
}

// Clear pixel in vRAM buffer
void UC1701_ResetPixel(uint8_t X, uint8_t Y) {
	uint8_t XX = X;
	uint8_t YY = Y;

	if (scr_orientation == scr_CW || scr_orientation == scr_CCW) {
		XX = Y; YY = X;
	}
	vRAM[((YY >> 3) * SCR_W) + XX] &= ~(1 << (YY % 8));
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

void UC1701_PutChar5x7(uint8_t X, uint8_t Y, uint8_t Char, CharType_TypeDef CharType) {
	uint16_t i,j;
	uint8_t buffer[5],tmpCh;

	if (Char < 32 || Char > 0x7e) Char = 0x7e;
	memcpy(buffer,&Font5x7[(Char - 32) * 5],5);

	// code bigger, but much faster
	switch(CharType) {
		case CT_opaque_inv:
			// Clear pixels, black background
		    for (i = 0; i < 5; i++) {
		    	tmpCh = buffer[i];
		    	for (j = 0; j < 7; j++) {
		    		if ((tmpCh >> j) & 0x01) UC1701_ResetPixel(X + i,Y + j); else UC1701_SetPixel(X + i,Y + j);
		    	}
		    }
			break;
		case CT_transp:
			// Black pixels, untouched background
			for (i = 0; i < 5; i++) {
				tmpCh = buffer[i];
				for (j = 0; j < 7; j++) {
					if ((tmpCh >> j) & 0x01) UC1701_SetPixel(X + i,Y + j);
				}
			}
			break;
		case CT_transp_inv:
			// Clear pixels, untouched background
			for (i = 0; i < 5; i++) {
				tmpCh = buffer[i];
				for (j = 0; j < 7; j++) {
					if ((tmpCh >> j) & 0x01) UC1701_ResetPixel(X + i,Y + j);
				}
			}
			break;
		default:
			// Black pixels with clear background
		    for (i = 0; i < 5; i++) {
		    	tmpCh = buffer[i];
		    	for (j = 0; j < 7; j++) {
		    		if ((tmpCh >> j) & 0x01) UC1701_SetPixel(X + i,Y + j); else UC1701_ResetPixel(X + i,Y + j);
		    	}
		    }
			break;
	}
}

uint16_t UC1701_PutStr5x7(uint8_t X, uint8_t Y, char *str, CharType_TypeDef CharType) {
	uint8_t strLen;

	strLen = 0;
    while (*str) {
        UC1701_PutChar5x7(X,Y,*str++,CharType);
        if (CharType == CT_opaque_inv || CharType == CT_transp_inv) UC1701_VLine(X + 5,Y,Y + 6,PSet);
        if (X < scr_width - 7) { X += 6; } else if (Y < scr_height - 8) { X = 0; Y += 7; } else { X = 0; Y = 0; }
        strLen++;
    };

    return strLen * 6;
}

uint8_t UC1701_PutInt5x7(uint8_t X, uint8_t Y, int32_t num, CharType_TypeDef CharType) {
	char str[11]; // 10 chars max for UINT32_MAX
	int i = 0;
	uint8_t neg = 0;

	if (num < 0) {
		neg = 1;
		num *= -1;
	}
	do { str[i++] = num % 10 + '0'; } while ((num /= 10) > 0);
	if (neg) str[i++] = '-';

	int strLen = i;
	for (i--; i >= 0; i--) UC1701_PutChar5x7(X + (strLen * 6) - ((i + 1) * 6),Y,str[i],CharType);

	return strLen * 6;
}

uint8_t UC1701_PutIntF5x7(uint8_t X, uint8_t Y, int32_t num, uint8_t decimals, CharType_TypeDef CharType) {
	char str[12];
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
		UC1701_PutChar5x7(neg,Y,str[strLen - i - 1],CharType);
		neg += 6;
		if (strLen - i - 1 == decimals && decimals != 0) {
			UC1701_Rect(neg,Y + 5,neg + 1,Y + 6,PSet);
			neg += 3;
		}
	}

	return (neg - X);
}

uint8_t UC1701_PutIntLZ5x7(uint8_t X, uint8_t Y, int32_t num, uint8_t digits, CharType_TypeDef CharType) {
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
		X += UC1701_PutInt5x7(X,Y,num,CharType);
		return X;
	}

	if (neg) {
		UC1701_PutChar5x7(X,Y,'-',CharType);
		X += 6;
		num *= -1;
	}
	for (i = 0; i < digits - len; i++) {
		UC1701_PutChar5x7(X,Y,'0',CharType);
		X += 6;
	}
	X += UC1701_PutInt5x7(X,Y,num,CharType);

	return X - pX;
}

uint8_t UC1701_PutHex5x7(uint8_t X, uint8_t Y, uint32_t num, CharType_TypeDef CharType) {
	char str[11]; // 10 chars max for UINT32_MAX
	int i = 0;
	uint32_t onum = num;

	do { str[i++] = "0123456789ABCDEF"[num % 0x10]; } while ((num /= 0x10) > 0);
	if (onum < 0x10) str[i++] = '0';
//	str[i++] = 'x';
//	str[i++] = '0';

	int strLen = i;

	for (i--; i >= 0; i--) UC1701_PutChar5x7(X + (strLen * 6) - ((i + 1) * 6),Y,str[i],CharType);

	return strLen * 6;
}

// Draw time with standard 5x7 font
// input:
//   X,Y - top left coordinates of text
//   time - time in seconds (max value = 3599999 seconds or 999:59:59)
//   CharType - character drawing style
uint8_t UC1701_PutTimeSec5x7(uint8_t X, uint8_t Y, uint32_t time, CharType_TypeDef CharType) {
	uint8_t pX = X;
	uint16_t hours;
	uint8_t minutes,seconds;

	hours   = time / 3600;
	minutes = (time / 60) % 60;
	seconds = time % 60;
	if (hours > 999) {
		hours   = 999;
		minutes = 59;
		seconds = 59;
	}

	if (hours < 10) {
		UC1701_PutChar5x7(X,Y,'0',CharType);
		X += 6;
	}
	UC1701_PutInt5x7(X,Y,hours,CharType);
	if (hours > 99) X += 6;
	if (hours > 9)  X += 11; else X += 5;
	UC1701_PutChar5x7(X,Y,':',CharType);
	X += 4;

	if (minutes < 10) {
		UC1701_PutChar5x7(X,Y,'0',CharType);
		X += 6;
	}
	UC1701_PutInt5x7(X,Y,minutes,CharType);
	if (minutes > 9) X += 11; else X += 5;
	UC1701_PutChar5x7(X,Y,':',CharType);
	X += 4;

	if (seconds < 10) {
		UC1701_PutChar5x7(X,Y,'0',CharType);
		X += 6;
	}
	X += UC1701_PutInt5x7(X,Y,seconds,CharType);

	return X - pX;
}

// Draw date with standard 5x7 font
// input:
//   X,Y - top left coordinates of text
//   date - date in format DDMMYYYY
//   CharType - character drawing style
uint8_t UC1701_PutDate5x7(uint8_t X, uint8_t Y, uint32_t date, CharType_TypeDef CharType) {
	uint8_t pX = X;
	uint16_t dig;

	// Day
	dig = date / 1000000;
	X += UC1701_PutIntLZ5x7(X,Y,dig,2,CharType);
	UC1701_PutChar5x7(X,Y,'.',CharType);
	X += 6;

	// Month
	dig = (date - (dig * 1000000)) / 10000;
	X += UC1701_PutIntLZ5x7(X,Y,dig,2,CharType);
	UC1701_PutChar5x7(X,Y,'.',CharType);
	X += 6;

	// Year
	dig = date % 10000;
	X += UC1701_PutIntLZ5x7(X,Y,dig,4,CharType);

	return X - pX;
}

// Print pressure with 'mmHg'
// input:
//   X,Y - top left coordinates of text
//   pressure - pressure value in Pa
//   PressureType - what units pressure should be displayed (PT_hPa, PT_mmHg)
//   CharType - character drawing style
uint8_t UC1701_PutPressure5x7(uint8_t X, uint8_t Y, int32_t pressure, PressureType_TypeDef PressureType,
		CharType_TypeDef CharType) {
	uint8_t pX = X;

	if (PressureType == PT_mmHg) {
		pressure = pressure * 75 / 1000;
		X += UC1701_PutIntF5x7(X,Y,pressure,1,CharType);
		X += UC1701_PutStr5x7(X,Y,"mmHg",CharType);
	} else {
		X += UC1701_PutIntF5x7(X,Y,pressure,2,CharType);
		X += UC1701_PutStr5x7(X,Y,"hPa",CharType);
	}

	return X - pX;
}

// Print temperature value with Celsius sign
// input:
//   X,Y - top left coordinates of text
//   temperature - temperature value in Celsius degree
//   CharType - character drawing style
void UC1701_PutTemperature5x7(uint8_t X, uint8_t Y, int32_t temperature, CharType_TypeDef CharType) {
	if (temperature < 0) {
		UC1701_HLine(X,X + 2,Y + 3,PSet);
		temperature *= -1;
		X += 4;
	}
	X += UC1701_PutInt5x7(X,Y,temperature / 10,CharType);

	// Decimal point
	UC1701_Rect(X,Y + 5,X + 1,Y + 6,PSet);
	X += 3;

	// Temperature fractional
	X += UC1701_PutInt5x7(X,Y,temperature % 10,CharType);

	// Celsius degree sign
	UC1701_HLine(X + 1,X + 2,Y,PSet);
	UC1701_HLine(X + 1,X + 2,Y + 3,PSet);
	UC1701_VLine(X,Y + 1,Y + 2,PSet);
	UC1701_VLine(X + 3,Y + 1,Y + 2,PSet);
	X += 5;
	UC1701_PutStr5x7(X,Y,"C",CharType);
}
