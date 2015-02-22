// Define to prevent recursive inclusion -------------------------------------
#ifndef __ST7541_H
#define __ST7541_H


// ST7541 display connection:
//   PB8 --> CS
//   PB9 --> RST
//   PC0 --> RS/A0
//   PA5 --> SCK
//   PA7 --> MOSI


// ST7541 HAL

// SPI port
#define ST7541_SPI_PORT      SPI1

// GPIO peripherals
#define ST7541_GPIO_PERIPH   RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;

// ST7541 RS/A0 (Data/Command select) pin
#define ST7541_RS_PORT       GPIOC
#define ST7541_RS_PIN        GPIO_Pin_0    // PC0

// ST7541 RST (Reset) pin
#define ST7541_RST_PORT      GPIOB
#define ST7541_RST_PIN       GPIO_Pin_9    // PB9

// ST7541 CS (Chip Select) pin
#define ST7541_CS_PORT       GPIOB
#define ST7541_CS_PIN        GPIO_Pin_8    // PB8

// CS pin macros
#define ST7541_CS_H() ST7541_CS_PORT->BSRRL = ST7541_CS_PIN
#define ST7541_CS_L() ST7541_CS_PORT->BSRRH = ST7541_CS_PIN

// RS pin macros
#define ST7541_RS_H() ST7541_RS_PORT->BSRRL = ST7541_RS_PIN
#define ST7541_RS_L() ST7541_RS_PORT->BSRRH = ST7541_RS_PIN

// RESET pin macros
#define ST7541_RST_H() ST7541_RST_PORT->BSRRL = ST7541_RST_PIN
#define ST7541_RST_L() ST7541_RST_PORT->BSRRH = ST7541_RST_PIN


// Public structures

// Screen dimensions
#define  SCR_W 128
#define  SCR_H 128

typedef enum {ON = 0, OFF = !ON} OnOffStatus;
typedef enum {NORMAL = 0, INVERT = !NORMAL} InvertStatus;
typedef enum {ENABLED = 0, DISABLED = !ENABLED} DisplayState;

// Screen orientation
typedef enum {
	scr_normal = 0,
	scr_CW     = 1,
	scr_CCW    = 2,
	scr_180    = 3
} ScrOrientation_TypeDef;

// Grayscale pixel color
typedef enum {
	gs_white  = 0,
	gs_ltgray = 1,
	gs_dkgray = 2,
	gs_black  = 3
} GrayScale_TypeDef;


// Public variables
extern uint16_t scr_width;
extern uint16_t scr_height;


// Function prototypes
void ST7541_data(uint8_t data);

void ST7541_Init(void);
void ST7541_Reset(void);

void ST7541_Contrast(uint8_t res_ratio, uint8_t lcd_bias, uint8_t el_vol);
void ST7541_SetAllPixelOn(OnOffStatus state);
void ST7541_SetInvert(InvertStatus state);
void ST7541_SetDisplayState(DisplayState state);
void ST7541_SetXDir(InvertStatus MX);
void ST7541_SetYDir(InvertStatus MY);
void ST7541_SetAddr(uint8_t X, uint8_t Y);
void ST7541_SetScrollLine(uint8_t line);
void ST7541_Orientation(uint8_t orientation);

void ST7541_Flush(void);
void ST7541_Fill(uint16_t pattern);

void Pixel(uint8_t X, uint8_t Y, GrayScale_TypeDef GS);
void HLine(uint8_t X1, uint8_t X2, uint8_t Y, GrayScale_TypeDef GS);
void VLine(uint8_t X, uint8_t Y1, uint8_t Y2, GrayScale_TypeDef GS);
void Rect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2, GrayScale_TypeDef GS);
void FillRect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2, GrayScale_TypeDef GS);
void Line(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, GrayScale_TypeDef GS);
void Ellipse(uint16_t X, uint16_t Y, uint16_t A, uint16_t B, GrayScale_TypeDef GS);

void PutChar5x7(uint8_t X, uint8_t Y, uint8_t Char, GrayScale_TypeDef GS);
uint16_t PutStr5x7(uint8_t X, uint8_t Y, char *str, GrayScale_TypeDef GS);
uint8_t PutInt5x7(uint8_t X, uint8_t Y, int32_t num, GrayScale_TypeDef GS);
uint8_t PutIntU5x7(uint8_t X, uint8_t Y, uint32_t num, GrayScale_TypeDef GS);
uint8_t PutIntF5x7(uint8_t X, uint8_t Y, int32_t num, uint8_t decimals, GrayScale_TypeDef GS);
uint8_t PutIntLZ5x7(uint8_t X, uint8_t Y, int32_t num, uint8_t digits, GrayScale_TypeDef GS);
uint8_t PutHex5x7(uint8_t X, uint8_t Y, uint32_t num, GrayScale_TypeDef GS);

#endif // __ST7541_H
