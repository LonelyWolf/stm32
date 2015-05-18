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
#define ST7541_SPI_PORT      hSPI1

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
typedef enum {BLOCK = 0, NOBLOCK = !BLOCK} BlockingState;
typedef enum {INTERNAL = 0, EXTERNAL = !INTERNAL} BoosterState;

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

typedef enum {
	font_V     = 0,        // Vertical font scan lines
	font_H     = 1         // Horizontal font scan lines
} FontScan_TypeDef;

typedef struct {
	uint8_t font_Width;                // Width of character
	uint8_t font_Height;               // Height of character
	uint8_t font_BPC;                  // Bytes for one character
	FontScan_TypeDef font_Scan;        // Font scan lines behavior
	uint8_t font_MinChar;              // Code of the first known symbol
	uint8_t font_MaxChar;              // Code of the last known symbol
	uint8_t font_UnknownChar;          // Code of the unknown symbol
	uint8_t font_Data[];               // Font data
} Font_TypeDef;


// Public variables
extern GrayScale_TypeDef lcd_color;
extern uint16_t scr_width;
extern uint16_t scr_height;


// Function prototypes
void ST7541_data(uint8_t data);

void ST7541_InitGPIO(void);
void ST7541_Init(void);
void ST7541_Reset(void);

void ST7541_Contrast(uint8_t res_ratio, uint8_t lcd_bias, uint8_t el_vol);
void ST7541_SetAllPixelOn(OnOffStatus state);
void ST7541_SetInvert(InvertStatus state);
void ST7541_SetDisplayState(DisplayState state);
void ST7541_SetDisplayPartial(uint8_t COM, uint8_t Line, uint8_t Duty);
void ST7541_PowerSave(OnOffStatus state);
void ST7541_SetXDir(InvertStatus MX);
void ST7541_SetYDir(InvertStatus MY);
void ST7541_SetAddr(uint8_t X, uint8_t Y);
void ST7541_SetScrollLine(uint8_t line);
void ST7541_Orientation(uint8_t orientation);

void ST7541_Flush(void);
void ST7541_Flush_DMA(BlockingState blocking);

void ST7541_Fill(uint16_t pattern);

inline void Pixel(uint8_t X, uint8_t Y, GrayScale_TypeDef GS);
void HLine(uint8_t X1, uint8_t X2, uint8_t Y, GrayScale_TypeDef GS);
void VLine(uint8_t X, uint8_t Y1, uint8_t Y2, GrayScale_TypeDef GS);
void Rect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2, GrayScale_TypeDef GS);
void FillRect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2, GrayScale_TypeDef GS);
void Line(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, GrayScale_TypeDef GS);
void Ellipse(uint16_t X, uint16_t Y, uint16_t A, uint16_t B, GrayScale_TypeDef GS);

uint8_t DrawChar(uint8_t X, uint8_t Y, uint8_t Char, const Font_TypeDef *Font);
uint16_t PutStr(uint8_t X, uint8_t Y, const char *str, const Font_TypeDef *Font);
uint16_t PutStrLF(uint8_t X, uint8_t Y, const char *str, const Font_TypeDef *Font);
uint8_t PutInt(uint8_t X, uint8_t Y, int32_t num, const Font_TypeDef *Font);
uint8_t PutIntU(uint8_t X, uint8_t Y, uint32_t num, const Font_TypeDef *Font);
uint8_t PutIntF(uint8_t X, uint8_t Y, int32_t num, uint8_t decimals, const Font_TypeDef *Font);
uint8_t PutIntLZ(uint8_t X, uint8_t Y, int32_t num, uint8_t digits, const Font_TypeDef *Font);
uint8_t PutHex(uint8_t X, uint8_t Y, uint32_t num, const Font_TypeDef *Font);
void DrawBitmap(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, const uint8_t* pBMP);
void DrawBitmapGS(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, const uint8_t* pBMP);

#endif // __ST7541_H
