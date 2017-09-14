#ifndef __LS027B7DH01_H
#define __LS027B7DH01_H


#include "gpio.h"
#include "spi.h"


// Note: works with SPI frequency up to 16MHz


// Compilation settings for library

// Choose a usage of a bit-banding in function for drawing a pixel
//   0 - do not use
//   1 - use bit banding (CPU must support this, by the way)
// In some circumstances can speed up pixels drawing
#define SMLCD_USE_BITBAND          1

// Choose calculation method for pixel offset in function of drawing a pixel
//   0 - use switch method, in tests it showed better results when compilation was made without optimizations (O0)
//   1 - use if..else method, it showed better results with compiler optimizations (O2/O3/Os)
// note: in fact better method depends on the compiler used, its optimization settings, bit-banding usage and so on...
//       is better to perform a tests in each case and choose which method suits better
#define SMLCD_PIXEL_METHOD         0

// Choose whether vRAM boundary will be checked for drawing each pixel
//   0 - do not use boundary checking (faster)
//       that behavior is not safe because if specified coordinates of the pixel will outside
//       of screen, it will cause corruption of memory contents outside of vRAM buffer
//   1 - check every pixel (slower, but safe)
#define SMLCD_PIXEL_SAFE           1

// Choose whether video buffer will be sent using look-up table or not
//   0 - use real-time bit reversing for line numbers (less code, a bit slower)
//   1 - use look-up table for line numbers (bigger code, a bit faster)
#define SMLCD_FLUSH_LUT            1

// Choose VCOM generation method
//   0 - hardware VCOM generation, set when [EXTMODE = H]
//       clock should be supplied to EXTCOMIN pin somehow
//   1 - software VCOM generation, set when [EXTMODE = L]
//       SMLCD_ToggleVCOM() function must be used to toggle VCOM state
//       State of VCOM should alternate in equal periods of time
//       It depends on type of screen used
# define SMLCD_VCOM_SOFT           0


// HAL

// SPI port
#define SMLCD_SPI_PORT             hSPI2

// GPIO peripherals
#define SMLCD_GPIO_PERIPH          (RCC_AHB2ENR_GPIOBEN)

// DISP pin (display on/off) - PB2
#define SMLCD_DISP_PORT            GPIOB
#define SMLCD_DISP_PIN             GPIO_PIN_2
#define SMLCD_DISP_L               GPIO_PIN_RESET(SMLCD_DISP_PORT, SMLCD_DISP_PIN)
#define SMLCD_DISP_H               GPIO_PIN_SET(SMLCD_DISP_PORT, SMLCD_DISP_PIN)

// SCS pin (chip select) - PB14
#define SMLCD_SCS_PORT             GPIOB
#define SMLCD_SCS_PIN              GPIO_PIN_14
#define SMLCD_SCS_L                GPIO_PIN_RESET(SMLCD_SCS_PORT, SMLCD_SCS_PIN)
#define SMLCD_SCS_H                GPIO_PIN_SET(SMLCD_SCS_PORT, SMLCD_SCS_PIN)


// Screen resolution (in pixels)
#define SCR_W                      400 // width
#define SCR_H                      240 // height


// Display commands
#define SMLCD_CMD_WRITE            ((uint8_t)0x80) // Write line
#define SMLCD_CMD_VCOM             ((uint8_t)0x40) // VCOM bit (not a command in fact)
#define SMLCD_CMD_CLS              ((uint8_t)0x20) // Clear the screen to all white
#define SMLCD_CMD_NOP              ((uint8_t)0x00) // No command


// Pixel draw mode
enum {
	LCD_PSET = 0, // Set pixel
	LCD_PRES = 1, // Reset pixel
	LCD_PINV = 2  // Invert pixel
};

// Screen orientation enumeration
enum {
	LCD_ORIENT_NORMAL = 1, // No rotation
	LCD_ORIENT_CW     = 2, // Clockwise rotation
	LCD_ORIENT_CCW    = 4, // Counter-clockwise rotation
	LCD_ORIENT_180    = 8  // 180 degrees rotation
};

// Font scan lines enumeration
enum {
	FONT_V = (uint8_t)0,        // Vertical font scan lines
	FONT_H = (uint8_t)(!FONT_V) // Horizontal font scan lines
};


// Structure describing a font
typedef struct {
	uint8_t font_Width;       // Width of character
	uint8_t font_Height;      // Height of character
	uint8_t font_BPC;         // Bytes for one character
	uint8_t font_Scan;        // Font scan lines behavior
	uint8_t font_MinChar;     // Code of the first known symbol
	uint8_t font_MaxChar;     // Code of the last known symbol
	uint8_t font_UnknownChar; // Code of the unknown symbol
	uint8_t font_Data[];      // Font data
} Font_TypeDef;


// Public variables
extern uint16_t scr_width;
extern uint16_t scr_height;
extern uint8_t LCD_PixelMode;


// Public macros and functions

// Enable the display (using DISP pin)
__STATIC_INLINE void SMLCD_Enable(void) {
	SMLCD_DISP_H;
}

// Disable the display (using DISP pin)
__STATIC_INLINE void SMLCD_Disable(void) {
	SMLCD_DISP_L;
}


// Function prototypes
void SMLCD_InitGPIO(void);
void SMLCD_Init(void);
void SMLCD_Clear(void);
#if (SMLCD_VCOM_SOFT)
void SMLCD_ToggleVCOM(void);
#endif // SMLCD_VCOM_SOFT
void SMLCD_Flush(void);
void SMLCD_Orientation(uint8_t orientation);

void LCD_Clear(void);

//void LCD_Pixel(uint16_t X, uint16_t Y);
void LCD_Pixel(register uint32_t X, register uint32_t Y);

void LCD_HLine(uint16_t X1, uint16_t X2, uint16_t Y);
void LCD_VLine(uint16_t X, uint16_t Y1, uint16_t Y2);
void LCD_Rect(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2);
void LCD_FillRect(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2);
void LCD_Line(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2);
void LCD_Circle(int16_t Xc, int16_t Yc, uint16_t R);
void LCD_Ellipse(int16_t Xc, int16_t Yc, uint16_t Ra, uint16_t Rb);

uint8_t LCD_PutChar(uint16_t X, uint16_t Y, uint8_t chr, const Font_TypeDef *font);
uint16_t LCD_PutStr(uint16_t X, uint16_t Y, const char *str, const Font_TypeDef *font);
uint16_t LCD_PutStrLF(uint16_t X, uint16_t Y, const char *str, const Font_TypeDef *font);
uint8_t LCD_PutInt(uint16_t X, uint16_t Y, int32_t num, const Font_TypeDef *font);
uint8_t LCD_PutIntU(uint16_t X, uint16_t Y, uint32_t num, const Font_TypeDef *font);
uint8_t LCD_PutIntF(uint16_t X, uint16_t Y, int32_t num, uint8_t decimals, const Font_TypeDef *font);
uint8_t LCD_PutIntLZ(uint16_t X, uint16_t Y, int32_t num, uint8_t digits, const Font_TypeDef *font);
uint8_t LCD_PutHex(uint16_t X, uint16_t Y, uint32_t num, const Font_TypeDef *font);

void LCD_DrawBitmap(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, const uint8_t* pBMP);

void LCD_Invert(uint16_t X, uint16_t Y, uint16_t W, uint16_t H);
void LCD_InvertFull(void);

#endif // __LS027B7DH01_H
