// Define to prevent recursive inclusion -------------------------------------
#ifndef __GUI_H
#define __GUI_H


typedef enum {
	DS_Big   = 0,
	DS_Mid   = 1,
	DS_Small = 2
} DigitSize_TypeDef;

typedef enum {
	TT_Full  = 0,       // HH:MM:SS
	TT_Short = 1        // HH:MM
} TimeType_TypeDef;

typedef enum {
	GT_dot   = 0,       // Dot graph
	GT_line  = 1,       // Line graph
	GT_fill  = 2        // Filled graph
} GraphType_TypeDef;

typedef enum {
	PT_mmHg = 0,
	PT_hPa  = !PT_mmHg
} PressureType_TypeDef;

typedef enum {
	MA_left   = 0,
	MA_center = 1,
	MA_right  = 2
} MenuAlign_TypeDef;

typedef enum {
	MS_normal = 0,
	MS_over   = 1
} MenuScroll_TypeDef;

typedef struct {
	char                   *ItemName;
} MenuItem_TypeDef;

typedef struct {
	uint8_t                 NumItems;
	MenuAlign_TypeDef       MenuAlign;
	MenuScroll_TypeDef      MenuScroll;
	MenuItem_TypeDef        Items[];
} Menu_TypeDef;

#define MenuItemHeight    9  // Menu item height

typedef void (*funcPtr_TypeDef)(int32_t param);


// Function prototypes
void GUI_DrawBitmap(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, const uint8_t* pBMP);
void GUI_DrawNumber(int8_t X, int8_t Y, int32_t Number, uint8_t Decimals,	DigitSize_TypeDef DigitSize);
void GUI_DrawTime(uint8_t X, uint8_t Y, RTC_TimeTypeDef *RTC_Time, TimeType_TypeDef TimeType, DigitSize_TypeDef DigitSize);

void GUI_Screen_SensorRAW(void);
void GUI_Screen_CurVal1(void);
void GUI_Screen_CurVal2(void);
void GUI_Screen_CurVal3(void);

void GUI_DrawSpeed(int8_t X, int8_t Y, uint32_t speed, uint32_t avg);
void GUI_DrawRideTime(uint8_t X, uint8_t Y, uint32_t time);
void GUI_DrawGraph(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, const int16_t* data, GraphType_TypeDef GraphType);
void GUI_DrawGPSInfo(void);
void GUI_DrawGPSSatsView(void);

uint8_t GUI_PutCoord5x7(uint8_t X, uint8_t Y, uint8_t degree, uint32_t seconds, char ch, CharType_TypeDef CharType);
uint8_t GUI_PutTimeSec5x7(uint8_t X, uint8_t Y, uint32_t time, CharType_TypeDef CharType);
uint8_t GUI_PutDate5x7(uint8_t X, uint8_t Y, uint32_t date, CharType_TypeDef CharType);
uint8_t GUI_PutPressure5x7(uint8_t X, uint8_t Y, int32_t pressure, PressureType_TypeDef PressureType,
		CharType_TypeDef CharType);
uint8_t GUI_PutTemperature5x7(uint8_t X, uint8_t Y, int32_t temperature, CharType_TypeDef CharType);
uint8_t GUI_Menu(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, const Menu_TypeDef *Menu,
		uint8_t StartPos);
void GUI_NumericScroll(int8_t X, int8_t Y, uint8_t W, uint8_t H, int32_t *Value,
		int32_t Min, int32_t Max, int32_t Step, char *unit, funcPtr_TypeDef CallBack);
#endif // __GUI_H
