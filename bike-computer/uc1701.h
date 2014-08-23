// Define to prevent recursive inclusion -------------------------------------
#ifndef __UC1701_H
#define __UC1701_H

// Display SPI port
#define UC1701_SPI_PORT    SPI2

// Display GPIO peripherals
#define UC1701_PORT_PERIPH RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB

// UC1701 CS (Chip Select) pin
#define UC1701_CS_PORT     GPIOB
#define UC1701_CS_PIN      GPIO_Pin_7    // PB7

// UC1701 RST (Reset) pin
#define UC1701_RST_PORT    GPIOB
#define UC1701_RST_PIN     GPIO_Pin_8    // PB8

// UC1701 RS (Data/Command select) pin
#define UC1701_RS_PORT     GPIOB
#define UC1701_RS_PIN      GPIO_Pin_9    // PB9

// UC1701 LEDA (LED on/off) pin
#define UC1701_LEDA_PORT   GPIOA
#define UC1701_LEDA_PIN    GPIO_Pin_1    // PA1

// CS pin macros
#define UC1701_CS_L()      UC1701_CS_PORT->BSRRH = UC1701_CS_PIN
#define UC1701_CS_H()      UC1701_CS_PORT->BSRRL = UC1701_CS_PIN

// RS pin macros
#define UC1701_RS_L()      UC1701_RS_PORT->BSRRH = UC1701_RS_PIN
#define UC1701_RS_H()      UC1701_RS_PORT->BSRRL = UC1701_RS_PIN

// RESET pin macros
#define UC1701_RST_L()     UC1701_RST_PORT->BSRRH = UC1701_RST_PIN
#define UC1701_RST_H()     UC1701_RST_PORT->BSRRL = UC1701_RST_PIN

// LEDA pin macros
#define UC1701_LEDA_L()    UC1701_LEDA_PORT->BSRRH = UC1701_LEDA_PIN
#define UC1701_LEDA_H()    UC1701_LEDA_PORT->BSRRL = UC1701_LEDA_PIN


// Screen dimensions
#define  SCR_W 128
#define  SCR_H 64


// Public structures
typedef enum {ON = 0, OFF = !ON} OnOffStatus;

typedef enum {NORMAL = 0, INVERT = !NORMAL} InvertStatus;

typedef enum {ENABLED = 0, DISABLED = !ENABLED} DisplayState;

typedef enum {
	PSet = 1,
	PReset = 0
} PSetReset_TypeDef;

typedef enum {
	scr_normal = 0,
	scr_CW     = 1,
	scr_CCW    = 2,
	scr_180    = 3
} ScrOrientation_TypeDef;

typedef enum {
	font_V     = 0,        // Vertical font scan lines
	font_H     = 1         // Horizontal font scan lines
} FontScan_TypeDef;

typedef struct {
	uint8_t font_Width;                // Width of character
	uint8_t font_Height;               // Height of character
	uint8_t font_BPC;                  // Bytes for one character
	FontScan_TypeDef font_Scan;        // Font scan lines behavior
	uint8_t font_Data[];               // Font data
} Font_TypeDef;


// Public variables
extern uint16_t scr_width;
extern uint16_t scr_height;


// Function prototypes
void UC1701_PauseSPI(void);
void UC1701_ResumeSPI(void);
void UC1701_SetBacklight(uint8_t brightness);
void UC1701_Init(void);
void UC1701_Contrast(uint8_t res_ratio, uint8_t el_vol);
void UC1701_SetAllPixelOn(OnOffStatus state);
void UC1701_SetInvert(InvertStatus state);
void UC1701_SetDisplayState(DisplayState state);
void UC1701_SetXDir(InvertStatus MX);
void UC1701_SetYDir(InvertStatus MY);
void UC1701_SetAddr(uint8_t X, uint8_t Y);
void UC1701_SetScrollLine(uint8_t line);
void UC1701_Orientation(uint8_t orientation);

void UC1701_Flush(void);
void UC1701_Fill(uint8_t pattern);

void SetPixel(uint8_t X, uint8_t Y);
void ResetPixel(uint8_t X, uint8_t Y);
void InvertRect(uint8_t X, uint8_t Y, uint8_t W, uint8_t H);

void HLine(uint8_t X1, uint8_t X2, uint8_t Y, PSetReset_TypeDef SR);
void VLine(uint8_t X, uint8_t Y1, uint8_t Y2, PSetReset_TypeDef SR);
void Rect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2, PSetReset_TypeDef SR);
void FillRect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2, PSetReset_TypeDef SR);
void Line(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2);
void Ellipse(uint16_t X, uint16_t Y, uint16_t A, uint16_t B);

uint8_t PutChar(uint8_t X, uint8_t Y, uint8_t Char, const Font_TypeDef *Font);
uint16_t PutStr(uint8_t X, uint8_t Y, char *str, const Font_TypeDef *Font);
uint16_t PutStrLF(uint8_t X, uint8_t Y, char *str, const Font_TypeDef *Font);
uint8_t PutInt(uint8_t X, uint8_t Y, int32_t num, const Font_TypeDef *Font);
uint8_t PutIntU(uint8_t X, uint8_t Y, uint32_t num, const Font_TypeDef *Font);
uint8_t PutIntF(uint8_t X, uint8_t Y, int32_t num, uint8_t decimals, const Font_TypeDef *Font);
uint8_t PutIntLZ(uint8_t X, uint8_t Y, int32_t num, uint8_t digits, const Font_TypeDef *Font);
uint8_t PutHex(uint8_t X, uint8_t Y, uint32_t num, const Font_TypeDef *Font);

void PutDigit3x5(uint8_t X, uint8_t Y, uint8_t digit);
uint8_t PutIntULZ3x5(uint8_t X, uint8_t Y, uint32_t num, uint8_t digits);

#endif // __UC1701_H
