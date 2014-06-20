// Define to prevent recursive inclusion -------------------------------------
#ifndef __UC1701_H
#define __UC1701_H

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
#define UC1701_CS_L() GPIO_ResetBits(UC1701_CS_PORT,UC1701_CS_PIN)
#define UC1701_CS_H() GPIO_SetBits(UC1701_CS_PORT,UC1701_CS_PIN)

// RS pin macros
#define UC1701_RS_L() GPIO_ResetBits(UC1701_RS_PORT,UC1701_RS_PIN)
#define UC1701_RS_H() GPIO_SetBits(UC1701_RS_PORT,UC1701_RS_PIN)

// RESET pin macros
#define UC1701_RST_L() GPIO_ResetBits(UC1701_RST_PORT,UC1701_RST_PIN)
#define UC1701_RST_H() GPIO_SetBits(UC1701_RST_PORT,UC1701_RST_PIN)

// LEDA pin macros
#define UC1701_LEDA_L() GPIO_ResetBits(UC1701_LEDA_PORT,UC1701_LEDA_PIN)
#define UC1701_LEDA_H() GPIO_SetBits(UC1701_LEDA_PORT,UC1701_LEDA_PIN)

// Screen dimensions
#define  SCR_W 128
#define  SCR_H 64


// Public structures
typedef enum {ON = 0, OFF = !ON} OnOffStatus;

typedef enum {NORMAL = 0, INVERT = !NORMAL} InvertStatus;

typedef enum {ENABLED = 0, DISABLED = !ENABLED} DisplayState;

typedef enum {
	CT_opaque      = 0,   // Normal character with opaque background
	CT_transp      = 1,   // Normal character with transparent background
	CT_opaque_inv  = 2,   // Inverted character with opaque background
	CT_transp_inv  = 3    // Inverted character with transparent background
} CharType_TypeDef;

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

void HLine(uint8_t X1, uint8_t X2, uint8_t Y, PSetReset_TypeDef SR);
void VLine(uint8_t X, uint8_t Y1, uint8_t Y2, PSetReset_TypeDef SR);
void Rect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2, PSetReset_TypeDef SR);
void FillRect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2, PSetReset_TypeDef SR);
void Line(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2);
void Ellipse(uint16_t X, uint16_t Y, uint16_t A, uint16_t B);

void PutChar5x7(uint8_t X, uint8_t Y, uint8_t Char, CharType_TypeDef CharType);
uint16_t PutStr5x7(uint8_t X, uint8_t Y, char *str, CharType_TypeDef CharType);
uint8_t PutInt5x7(uint8_t X, uint8_t Y, int32_t num, CharType_TypeDef CharType);
uint8_t PutIntU5x7(uint8_t X, uint8_t Y, uint32_t num, CharType_TypeDef CharType);
uint8_t PutIntF5x7(uint8_t X, uint8_t Y, int32_t num, uint8_t decimals, CharType_TypeDef CharType);
uint8_t PutIntLZ5x7(uint8_t X, uint8_t Y, int32_t num, uint8_t digits, CharType_TypeDef CharType);
uint8_t PutHex5x7(uint8_t X, uint8_t Y, uint32_t num, CharType_TypeDef CharType);

#endif // __UC1701_H
