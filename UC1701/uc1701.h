//#define SOFT_SPI

#ifdef SOFT_SPI
	#define	UC1701_SDA_PIN     GPIO_Pin_5    // PB5
	#define UC1701_SDA_PORT    GPIOB
	#define	UC1701_SCK_PIN     GPIO_Pin_3    // PB3
	#define UC1701_SCK_PORT    GPIOB
#else
	/* Which SPI use */
	#define _SPI_PORT 3

	#if _SPI_PORT == 1
		#define SPI_PORT      SPI1
		#define SPI_SCK_PIN   GPIO_Pin_5     // PA5
		#define SPI_MOSI_PIN  GPIO_Pin_7     // PA7
		#define SPI_GPIO_PORT GPIOA
	#elif _SPI_PORT == 2
		#define SPI_PORT      SPI2
		#define SPI_SCK_PIN   GPIO_Pin_13    // PB13
		#define SPI_MOSI_PIN  GPIO_Pin_15    // PB15
		#define SPI_GPIO_PORT GPIOB
	#elif _SPI_PORT == 3
		#define SPI_PORT      SPI3
		#define SPI_SCK_PIN   GPIO_Pin_3     // PB3  (JTDO)
		#define SPI_MOSI_PIN  GPIO_Pin_5     // PB5
		#define SPI_GPIO_PORT GPIOB
	#endif
#endif


// UC1701 RS (Data/Command select) pin
#define UC1701_RS_PORT     GPIOB
#define UC1701_RS_PIN      GPIO_Pin_4    // PB4

// UC1701 RST (Reset) pin
#define UC1701_RST_PORT    GPIOB
#define UC1701_RST_PIN     GPIO_Pin_6    // PB6

// UC1701 CS (Chip Select) pin
#define UC1701_CS_PORT     GPIOB
#define UC1701_CS_PIN      GPIO_Pin_7    // PB7

// CS pin macros
#define CS_L() GPIO_ResetBits(UC1701_CS_PORT,UC1701_CS_PIN)
#define CS_H() GPIO_SetBits(UC1701_CS_PORT,UC1701_CS_PIN)

// RS pin macros
#define RS_L() GPIO_ResetBits(UC1701_RS_PORT,UC1701_RS_PIN)
#define RS_H() GPIO_SetBits(UC1701_RS_PORT,UC1701_RS_PIN)

// RESET pin macros
#define RST_L() GPIO_ResetBits(UC1701_RST_PORT,UC1701_RST_PIN)
#define RST_H() GPIO_SetBits(UC1701_RST_PORT,UC1701_RST_PIN)

#ifdef SOFT_SPI
	// SDA pin macros
	#define SDA_L() GPIO_ResetBits(UC1701_SDA_PORT,UC1701_SDA_PIN)
	#define SDA_H() GPIO_SetBits(UC1701_SDA_PORT,UC1701_SDA_PIN)

	// SCK pin macros
	#define SCK_L() GPIO_ResetBits(UC1701_SCK_PORT,UC1701_SCK_PIN)
	#define SCK_H() GPIO_SetBits(UC1701_SCK_PORT,UC1701_SCK_PIN)
#endif

typedef enum {ON = 0, OFF = !ON} OnOffStatus;
typedef enum {NORMAL = 0, INVERT = !NORMAL} InvertStatus;
typedef enum {ENABLED = 0, DISABLED = !ENABLED} DisplayState;

// Screen dimensions
#define  SCR_W 128
#define  SCR_H 64

typedef enum {
	transparent  = 0,
	opaque       = 1
} Opaque_TypeDef;

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


extern uint16_t scr_width;
extern uint16_t scr_height;


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
void UC1701_SetPixel(uint8_t X, uint8_t Y);
void UC1701_ResetPixel(uint8_t X, uint8_t Y);

void UC1701_HLine(uint8_t X1, uint8_t X2, uint8_t Y, PSetReset_TypeDef SR);
void UC1701_VLine(uint8_t X, uint8_t Y1, uint8_t Y2, PSetReset_TypeDef SR);
void UC1701_Rect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2, PSetReset_TypeDef SR);
void UC1701_FillRect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2, PSetReset_TypeDef SR);
void UC1701_Line(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2);
void UC1701_Ellipse(uint16_t X, uint16_t Y, uint16_t A, uint16_t B);

void UC1701_PutChar5x7(uint8_t X, uint8_t Y, uint8_t Char, Opaque_TypeDef bckgnd);
void UC1701_PutStr5x7(uint8_t X, uint8_t Y, char *str, Opaque_TypeDef bckgnd);
void UC1701_PutInt5x7(uint8_t X, uint8_t Y, uint32_t num, Opaque_TypeDef bckgnd);
void UC1701_PutHex5x7(uint8_t X, uint8_t Y, uint32_t num, Opaque_TypeDef bckgnd);
