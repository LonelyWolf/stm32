// Define to prevent recursive inclusion -------------------------------------
#ifndef __GUI_H
#define __GUI_H


#define GUI_SCREENSAVER_TIMEOUT       1800  // Timeout for screensaver activation (seconds)
#define GUI_SCREENSAVER_UPDATE          60  // Screensaver update interval (seconds)
#define GUI_TIMEOUT                    300  // Timeout for GUI screens (seconds)
#define GUI_MENU_TIMEOUT               120  // Timeout for menu (seconds)


typedef void (*funcPtrParam_TypeDef)(int32_t param);
typedef void (*funcPtrVoid_TypeDef)(void);
typedef void (*funcPtrKeyPress_TypeDef)(bool Sleep, bool *WaitFlag, uint32_t Timeout);

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

typedef enum {
	MF_none = 0,
	MF_rect = 1,
	MF_top_bottom = 2
} MenuFrame_TypeDef;

typedef struct {
	char                   *subst_str;
	int32_t                 subst_val;
} SubstItem_TypeDef;

typedef struct {
	uint8_t                 NumItems;
	SubstItem_TypeDef       Items[];
} Subst_TypeDef;

// Menus
static const Menu_TypeDef mnuMain = {
		5,
		MA_center,
		MS_over,
		{
				{"Statistics"},
				{"GPS"},
				{"Settings"},
				{"Logging"},
				{"Debug"}
		}
};

static const Menu_TypeDef mnuSettings = {
		4,
		MA_center,
		MS_over,
		{
				{"Display"},
				{"Wheel"},
				{"Time"},
				{"Home alt."}
		}
};

static const Menu_TypeDef mnuDisplay = {
		3,
		MA_center,
		MS_over,
		{
				{"Brightness"},
				{"Timeout"},
				{"Contrast"}
		}
};

static const Menu_TypeDef mnuGPS = {
		3,
		MA_center,
		MS_over,
		{
				{"Satellites"},
				{"GPS RAW"},
				{"NMEA RAW"}
		}
};

static const Menu_TypeDef mnuStatistics = {
		4,
		MA_center,
		MS_over,
		{
				{"Trip values"},
				{"Sensor RAW"},
				{"BMP180 values"},
				{"GPS values"}
		}
};

static const Menu_TypeDef mnuLogging = {
		3,
		MA_center,
		MS_over,
		{
				{"New log"},
				{"Sync log"},
				{"Stop log"}
		}
};

static const Menu_TypeDef mnuDebug = {
		10,
		MA_left,
		MS_over,
		{
				{"Reinit LCD"},
				{"Screensaver"},
				{"Play SMB"},
				{"GPS hot start"},
				{"GPS EASY on"},
				{"GPS EASY off"},
				{"- debug -"},
				{"- debug -"},
				{"- debug -"},
				{"Reboot"}
		}
};

// Display brightness substitute value
static const Subst_TypeDef substDisplayBrightness = {
		6,
		{
				{"Off",   0},
				{  "1",   5},
				{  "2",  25},
				{  "3",  50},
				{  "4",  75},
				{  "5", 100}
		}
};

// Display timeout substitute values
static const Subst_TypeDef substDisplayTimeout = {
		5,
		{
				{   "Stay on",   0},
				{"15 Seconds",  15},
				{"30 Seconds",  30},
				{  "1 Minute",  60},
				{ "2 Minutes", 120}
		}
};


// Public variables
extern bool GUI_refresh;                     // Flag to refresh GUI
extern bool GUI_new_BMP180;                  // BMP180 data updated


// Function prototypes
void GUI_DrawBitmap(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, const uint8_t* pBMP);
void GUI_DrawNumber(int8_t X, int8_t Y, int32_t Number, uint8_t Decimals,
		DigitSize_TypeDef DigitSize);
void GUI_DrawTime(uint8_t X, uint8_t Y, RTC_TimeTypeDef *RTC_Time, TimeType_TypeDef TimeType,
		DigitSize_TypeDef DigitSize);

void GUI_Screen_SensorRAW(funcPtrKeyPress_TypeDef WaitForKey);
void GUI_Screen_CurVal1(funcPtrKeyPress_TypeDef WaitForKey);
void GUI_Screen_CurVal2(funcPtrKeyPress_TypeDef WaitForKey);
void GUI_Screen_CurVal3(funcPtrKeyPress_TypeDef WaitForKey);
void GUI_Screen_GPSSatsView(funcPtrKeyPress_TypeDef WaitForKey);
void GUI_Screen_GPSInfo(funcPtrKeyPress_TypeDef WaitForKey);
void GUI_Screen_Buffer(uint8_t *pBuf, uint16_t BufSize, bool *UpdateFlag,
		funcPtrKeyPress_TypeDef WaitForKey);

void GUI_DrawSpeed(int8_t X, int8_t Y, uint32_t speed, uint32_t avg);
void GUI_DrawRideTime(uint8_t X, uint8_t Y, uint32_t time);
void GUI_DrawGraph(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, const int16_t* data, GraphType_TypeDef GraphType);

uint8_t GUI_PutCoord(uint8_t X, uint8_t Y, uint32_t coord, char ch, const Font_TypeDef *Font);
uint8_t GUI_PutTimeSec(uint8_t X, uint8_t Y, uint32_t time, const Font_TypeDef *Font);
uint8_t GUI_PutDate(uint8_t X, uint8_t Y, uint32_t date, const Font_TypeDef *Font);
uint8_t GUI_PutPressure(uint8_t X, uint8_t Y, int32_t pressure, PressureType_TypeDef PressureType,
		const Font_TypeDef *Font);
uint8_t GUI_PutTemperature(uint8_t X, uint8_t Y, int32_t temperature, const Font_TypeDef *Font);

uint8_t GUI_Menu(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, const Font_TypeDef *Font,
		MenuFrame_TypeDef MenuFrame, const Menu_TypeDef *Menu, uint8_t StartPos,
		funcPtrKeyPress_TypeDef WaitForKey);
void GUI_NumericScroll(int8_t X, int8_t Y, uint8_t W, uint8_t H, const Font_TypeDef *Font,
		int32_t *Value, int32_t Min, int32_t Max, int32_t Step,	char *unit,
		const Subst_TypeDef *Subst, funcPtrParam_TypeDef CallBack, funcPtrKeyPress_TypeDef WaitForKey);

void GUI_MainMenu(void);
void GUI_ScreenSaver(void);

#endif // __GUI_H
