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

typedef struct {
	char const       *MenuTitle;
	uint8_t  const    NumItems;
} Menu_TypeDef;


// Function prototypes
void GUI_DrawBitmap(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, const uint8_t* pBMP);
void GUI_DrawNumber(uint8_t X, uint8_t Y, uint32_t number, uint8_t digits, uint8_t decimals, DigitSize_TypeDef DigitSize);
void GUI_DrawTime(uint8_t X, uint8_t Y, RTC_TimeTypeDef *RTC_Time, TimeType_TypeDef TimeType, DigitSize_TypeDef DigitSize);
void GUI_Screen_SensorRAW(void);
void GUI_Screen_CurVal1(void);
void GUI_Screen_CurVal2(void);
void GUI_Screen_CurVal3(void);
void GUI_DrawSpeed(uint8_t X, uint8_t Y, uint32_t speed, uint32_t avg);
void GUI_DrawRideTime(uint8_t X, uint8_t Y, uint32_t time);
void GUI_DrawGraph(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, const int32_t* data, GraphType_TypeDef GraphType);

void GUI_Menu(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, char * Title);
void GUI_NumericSet(int32_t Value, int32_t Min, int32_t Max, int32_t Step, char * Title);

void GUI_PutCoord5x7(uint8_t X, uint8_t Y, uint8_t degree, uint32_t seconds, char ch, CharType_TypeDef CharType);
void GUI_DrawGPSInfo(void);

#endif // __GUI_H
