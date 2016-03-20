#ifndef __ST7528_H
#define __ST7528_H


#include "gpio.h"
#include "spi.h"
#include "delay.h"


// ST7528 display connection:
//   PB8 --> CS
//   PB9 --> RST
//   PC0 --> RS/A0
//   PA5 --> SCK
//   PA7 --> MOSI


// ST7528 HAL

// SPI port
#define ST7528_SPI_PORT      hSPI1

// GPIO peripherals
#define ST7528_GPIO_PERIPH   (RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN)

// ST7528 A0 (Data/Command select) pin (PC0)
#define ST7528_A0_PORT       GPIOC
#define ST7528_A0_PIN        GPIO_Pin_0
#define ST7528_A0_L()        GPIO_PIN_RESET(ST7528_A0_PORT,ST7528_A0_PIN)
#define ST7528_A0_H()        GPIO_PIN_SET(ST7528_A0_PORT,ST7528_A0_PIN)

// ST7528 RST (Reset) pin (PB9)
#define ST7528_RST_PORT      GPIOB
#define ST7528_RST_PIN       GPIO_Pin_9
#define ST7528_RST_H()       GPIO_PIN_SET(ST7528_RST_PORT,ST7528_RST_PIN)
#define ST7528_RST_L()       GPIO_PIN_RESET(ST7528_RST_PORT,ST7528_RST_PIN)

// ST7528 CS (Chip Select) pin (PB8)
#define ST7528_CS_PORT       GPIOB
#define ST7528_CS_PIN        GPIO_Pin_8
#define ST7528_CS_H()        GPIO_PIN_SET(ST7528_CS_PORT,ST7528_CS_PIN)
#define ST7528_CS_L()        GPIO_PIN_RESET(ST7528_CS_PORT,ST7528_CS_PIN)


// ST7528 commands
// EXT=0/1 modes
#define ST7528_CMD_MODE      (uint8_t)0x38 // Mode set (double byte command)
// EXT=0 mode
#define ST7528_CMD_PAGE      (uint8_t)0xB0 // Set page address
#define ST7528_CMD_COLM      (uint8_t)0x10 // Set column address MSB
#define ST7528_CMD_COLL      (uint8_t)0x00 // Set column address LSB
#define ST7528_CMD_MR_ON     (uint8_t)0xE0 // Enable modify-read
#define ST7528_CMD_MR_OFF    (uint8_t)0xEE // Disable modify-read
#define ST7528_CMD_DISPON    (uint8_t)0xAF // Display ON
#define ST7528_CMD_DISPOFF   (uint8_t)0xAE // Display OFF
#define ST7528_CMD_LINE      (uint8_t)0x40 // Set partial display initial line (double byte command)
#define ST7528_CMD_COM0      (uint8_t)0x44 // Set partial display initial COM0 (double byte command)
#define ST7528_CMD_PD_DUTY   (uint8_t)0x48 // Set partial display number of lines (double byte command)
#define ST7528_CMD_NLINE_INV (uint8_t)0x4C // Set N-line inversion (double byte command)
#define ST7528_CMD_NLINE_REL (uint8_t)0xE4 // Release N-line inversion
#define ST7528_CMD_REVON     (uint8_t)0xA7 // REV bit on: inverted display pixels
#define ST7528_CMD_REVOFF    (uint8_t)0xA6 // REV bit off: normal display
#define ST7528_CMD_EDON      (uint8_t)0xA5 // EON bit on: entire display on
#define ST7528_CMD_EDOFF     (uint8_t)0xA4 // EON bit off: normal display
#define ST7528_CMD_PWR       (uint8_t)0x28 // Control power circuit
#define ST7528_CMD_DCDC      (uint8_t)0x64 // Select the step-up of internal DC-DC converter
#define ST7528_CMD_RREG      (uint8_t)0x20 // Select the internal resistance ratio of the regulator resistor
#define ST7528_CMD_ELVOL     (uint8_t)0x81 // Select electronic volume (double byte command)
#define ST7528_CMD_BIAS      (uint8_t)0x50 // Select LCD bias
#define ST7528_CMD_SHL_OFF   (uint8_t)0xC0 // SHL bit off: normal COM scan direction
#define ST7528_CMD_SHL_ON    (uint8_t)0xC8 // SHL bit on: reverse COM scan direction
#define ST7528_CMD_ADC_OFF   (uint8_t)0xA0 // ADC bit off: normal SEG scan direction
#define ST7528_CMD_ADC_ON    (uint8_t)0xA1 // ADC bit on: reverse SEG scan direction
#define ST7528_CMD_OSCON     (uint8_t)0xAB // Start the built-in oscillator
#define ST7528_CMD_PM_ON     (uint8_t)0xA9 // Power save mode ON
#define ST7528_CMD_PM_OFF    (uint8_t)0xA8 // Power save mode OFF
#define ST7528_CMD_PM_REL    (uint8_t)0xE1 // Release power save mode
#define ST7528_CMD_RESET     (uint8_t)0xE2 // Software reset
#define ST7528_CMD_FRC_PWM   (uint8_t)0x90 // Select FRC and PWM mode
#define ST7528_CMD_ICON_OFF  (uint8_t)0xA2 // ICON disable
#define ST7528_CMD_NOP       (uint8_t)0xE3 // NOP
// EXT=1 mode
#define ST7528_CMD1_W1F      (uint8_t)0x80 // Set white mode and 1st frame
#define ST7528_CMD1_W2F      (uint8_t)0x81 // Set white mode and 2nd frame
#define ST7528_CMD1_W3F      (uint8_t)0x82 // Set white mode and 3rd frame
#define ST7528_CMD1_W4F      (uint8_t)0x83 // Set white mode and 4th frame
#define ST7528_CMD1_D1F      (uint8_t)0xBC // Set dark mode and 1st frame
#define ST7528_CMD1_D2F      (uint8_t)0xBD // Set dark mode and 2nd frame
#define ST7528_CMD1_D3F      (uint8_t)0xBE // Set dark mode and 3rd frame
#define ST7528_CMD1_D4F      (uint8_t)0xBF // Set dark mode and 4th frame
#define ST7528_CMD1_GRAYPAL  (uint8_t)0x84 // Gray levels array begin (4 bytes for each level from 1 to 14)

// ST7528 control bits definitions
#define ST7528_PWR_VC        (uint8_t)0x04 // Internal voltage converter circuit ON
#define ST7528_PWR_VR        (uint8_t)0x02 // Internal voltage regulator circuit ON
#define ST7528_PWR_VF        (uint8_t)0x01 // Internal voltage follower circuit ON
#define ST7528_MODE_EXT0     (uint8_t)0x00 // Instruction EXT=0 mode
#define ST7528_MODE_EXT1     (uint8_t)0x01 // Instruction EXT=1 mode
#define ST7528_MODE_BE1      (uint8_t)0x00 // Booster efficiency level 1
#define ST7528_MODE_BE2      (uint8_t)0x04 // Booster efficiency level 2

// Screen dimensions
#define SCR_W                (uint8_t)128 // width
#define SCR_H                (uint8_t)128 // height

// Screen page width
#define SCR_PAGE_WIDTH       (uint32_t)128 // In pixels


// Frame frequency enumeration
enum {
	ST7528_FF_77  = (uint8_t)0x00, // 77Hz (+/- 5%)
	ST7528_FF_51  = (uint8_t)0x10, // 51Hz (+/- 20%)
	ST7528_FF_55  = (uint8_t)0x20, // 55Hz (+/- 20%)
	ST7528_FF_58  = (uint8_t)0x30, // 58Hz (+/- 20%)
	ST7528_FF_63  = (uint8_t)0x40, // 63Hz (+/- 20%)
	ST7528_FF_67  = (uint8_t)0x50, // 67Hz (+/- 20%)
	ST7528_FF_68  = (uint8_t)0x60, // 68Hz (+/- 20%)
	ST7528_FF_70  = (uint8_t)0x70, // 70Hz (+/- 20%)
	ST7528_FF_73  = (uint8_t)0x80, // 73Hz (+/- 20%)
	ST7528_FF_75  = (uint8_t)0x90, // 75Hz (+/- 20%)
	ST7528_FF_80  = (uint8_t)0xA0, // 80Hz (+/- 20%)
	ST7528_FF_85  = (uint8_t)0xB0, // 85Hz (+/- 20%)
	ST7528_FF_91  = (uint8_t)0xC0, // 91Hz (+/- 20%)
	ST7528_FF_102 = (uint8_t)0xD0, // 102Hz (+/- 20%)
	ST7528_FF_113 = (uint8_t)0xE0, // 113Hz (+/- 20%)
	ST7528_FF_123 = (uint8_t)0xF0  // 123Hz (+/- 20%)
};

// Internal resistance ratio enumeration
enum {
	ST7528_RREG_23 = (uint8_t)0x00, // 2.3
	ST7528_RREG_30 = (uint8_t)0x01, // 3.0
	ST7528_RREG_37 = (uint8_t)0x02, // 3.7
	ST7528_RREG_44 = (uint8_t)0x03, // 4.4
	ST7528_RREG_51 = (uint8_t)0x04, // 5.1
	ST7528_RREG_58 = (uint8_t)0x05, // 5.8
	ST7528_RREG_65 = (uint8_t)0x06, // 6.5
	ST7528_RREG_72 = (uint8_t)0x07  // 7.2
};

// LCD bias ratio enumeration
enum {
	ST7528_BIAS_5  = (uint8_t)0x00, // 1/5
	ST7528_BIAS_6  = (uint8_t)0x01, // 1/6
	ST7528_BIAS_7  = (uint8_t)0x02, // 1/7
	ST7528_BIAS_8  = (uint8_t)0x03, // 1/8
	ST7528_BIAS_9  = (uint8_t)0x04, // 1/9
	ST7528_BIAS_10 = (uint8_t)0x05, // 1/10
	ST7528_BIAS_11 = (uint8_t)0x06, // 1/11
	ST7528_BIAS_12 = (uint8_t)0x07, // 1/12
};

// FRC mode enumeration
enum {
	ST7528_FRC_3 = (uint8_t)0x04, // 3FRC
	ST7528_FRC_4 = (uint8_t)0x00  // 4FRC
};

// PWM mode enumeration
enum {
	ST7528_PWM_45 = (uint8_t)0x00, // 45PWM
	ST7528_PWM_60 = (uint8_t)0x02  // 60PWM
};

// DC-DC converter circuit enumeration
enum {
	ST7528_BOOST_3X = (uint8_t)0x00, // 3 times boosting
	ST7528_BOOST_4X = (uint8_t)0x01, // 4 times boosting
	ST7528_BOOST_5X = (uint8_t)0x02, // 5 times boosting
	ST7528_BOOST_6X = (uint8_t)0x03  // 6 times boosting
};

// Entire display on/off enumeration
enum {
	SCR_ALL_PIXELS_OFF = 0,
	SCR_ALL_PIXELS_ON  = !SCR_ALL_PIXELS_OFF
};

// Display pixels inversion enumeration
enum {
	SCR_INVERT_OFF = 0,
	SCR_INVERT_ON  = !SCR_INVERT_OFF
};

// Display ON/OFF enumeration
enum {
	SCR_OFF = 0,
	SCR_ON  = !SCR_OFF
};

// Screen orientation enumeration
enum {
	SCR_ORIENT_NORMAL = 0x01, // No rotation
	SCR_ORIENT_180    = 0x02, // 180 degrees rotation
	SCR_ORIENT_CW     = 0x04, // Clockwise rotation
	SCR_ORIENT_CCW    = 0x08  // Counter-clockwise rotation
};

// Font structure scan lines enumeration
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
extern uint8_t lcd_color;
extern uint16_t scr_width;
extern uint16_t scr_height;


// Function prototypes
void ST7528_InitGPIO(void);
void ST7528_Init(void);
void ST7528_Reset(void);

void ST7528_Flush(void);
void ST7528_Clear(void);

void ST7528_Contrast(uint8_t res_ratio, uint8_t lcd_bias, uint8_t el_vol);
void ST7528_SetAllPixelsOn(uint8_t eon_state);
void ST7528_SetInvert(uint8_t inv_state);
void ST7528_SetDisplayState(uint8_t disp_state);
void ST7528_SetPartialDisplay(uint8_t phy_line, uint8_t log_line, uint8_t lines_num);
void ST7528_PowerSave(uint8_t pm_state);
void ST7528_SetXDir(uint8_t x_map);
void ST7528_SetYDir(uint8_t y_map);
void ST7528_SetAddr(uint8_t X, uint8_t Y);
void ST7528_SetScrollLine(uint8_t line);
void ST7528_Orientation(uint8_t orientation);

void LCD_Pixel(uint8_t X, uint8_t Y, uint8_t GS);
void LCD_HLine(uint8_t X1, uint8_t X2, uint8_t Y, uint8_t GS);
void LCD_VLine(uint8_t X, uint8_t Y1, uint8_t Y2, uint8_t GS);
void LCD_Rect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2, uint8_t GS);
void LCD_FillRect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2, uint8_t GS);
void LCD_Line(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, uint8_t GS);
void LCD_Circle(int16_t Xc, int16_t Yc, uint8_t R, uint8_t GS);
void LCD_Ellipse(uint16_t Xc, uint16_t Yc, uint16_t Ra, uint16_t Rb, uint8_t GS);

uint8_t LCD_PutChar(uint8_t X, uint8_t Y, uint8_t Char, const Font_TypeDef *Font);
uint16_t LCD_PutStr(uint8_t X, uint8_t Y, const char *str, const Font_TypeDef *Font);
uint16_t LCD_PutStrLF(uint8_t X, uint8_t Y, const char *str, const Font_TypeDef *Font);
uint8_t LCD_PutInt(uint8_t X, uint8_t Y, int32_t num, const Font_TypeDef *Font);
uint8_t LCD_PutIntU(uint8_t X, uint8_t Y, uint32_t num, const Font_TypeDef *Font);
uint8_t LCD_PutIntF(uint8_t X, uint8_t Y, int32_t num, uint8_t decimals, const Font_TypeDef *Font);
uint8_t LCD_PutIntLZ(uint8_t X, uint8_t Y, int32_t num, uint8_t digits, const Font_TypeDef *Font);
uint8_t LCD_PutHex(uint8_t X, uint8_t Y, uint32_t num, const Font_TypeDef *Font);

void LCD_DrawBitmap(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, const uint8_t* pBMP);
void LCD_DrawBitmapGS(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, const uint8_t* pBMP);

void LCD_Invert(uint8_t X, uint8_t Y, uint8_t W, uint8_t H);

#endif // __ST7528_H
