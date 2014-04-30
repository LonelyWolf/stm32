//#define SOFT_SPI

#ifdef SOFT_SPI
	#define	ST7735_SDA_PIN     GPIO_Pin_5    // PB5
	#define ST7735_SDA_PORT    GPIOB
	#define	ST7735_SCK_PIN     GPIO_Pin_3    // PB3
	#define ST7735_SCK_PORT    GPIOB
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

// Screen resolution in normal orientation
#define scr_w         128
#define scr_h         160

// ST7735 A0 (Data/Command select) pin
#define ST7735_A0_PORT     GPIOB
#define ST7735_A0_PIN      GPIO_Pin_4    // PB4

// ST7735 RST (Reset) pin
#define ST7735_RST_PORT    GPIOB
#define ST7735_RST_PIN     GPIO_Pin_6    // PB6

// ST7735 CS (Chip Select) pin
#define ST7735_CS_PORT     GPIOB
#define ST7735_CS_PIN      GPIO_Pin_7    // PB7

// CS pin macros
#define CS_L() GPIO_ResetBits(ST7735_CS_PORT,ST7735_CS_PIN)
#define CS_H() GPIO_SetBits(ST7735_CS_PORT,ST7735_CS_PIN)

// A0 pin macros
#define A0_L() GPIO_ResetBits(ST7735_A0_PORT,ST7735_A0_PIN)
#define A0_H() GPIO_SetBits(ST7735_A0_PORT,ST7735_A0_PIN)

// RESET pin macros
#define RST_L() GPIO_ResetBits(ST7735_RST_PORT,ST7735_RST_PIN)
#define RST_H() GPIO_SetBits(ST7735_RST_PORT,ST7735_RST_PIN)

#ifdef SOFT_SPI
	// SDA pin macros
	#define SDA_L() GPIO_ResetBits(ST7735_SDA_PORT,ST7735_SDA_PIN)
	#define SDA_H() GPIO_SetBits(ST7735_SDA_PORT,ST7735_SDA_PIN)

	// SCK pin macros
	#define SCK_L() GPIO_ResetBits(ST7735_SCK_PORT,ST7735_SCK_PIN)
	#define SCK_H() GPIO_SetBits(ST7735_SCK_PORT,ST7735_SCK_PIN)
#endif


typedef enum {
	scr_normal = 0,
	scr_CW     = 1,
	scr_CCW    = 2,
	scr_180    = 3
} ScrOrientation_TypeDef;


extern uint16_t scr_width;
extern uint16_t scr_height;


void ST7735_write(uint8_t data);

uint16_t RGB565(uint8_t R,uint8_t G,uint8_t B);

void ST7735_Init(void);
void ST7735_AddrSet(uint16_t XS, uint16_t YS, uint16_t XE, uint16_t YE);
void ST7735_Orientation(uint8_t orientation);
void ST7735_Clear(uint16_t color);

void ST7735_Pixel(uint16_t X, uint16_t Y, uint16_t color);
void ST7735_HLine(uint16_t X1, uint16_t X2, uint16_t Y, uint16_t color);
void ST7735_VLine(uint16_t X, uint16_t Y1, uint16_t Y2, uint16_t color);
void ST7735_Line(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, uint16_t color);
void ST7735_Rect(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2, uint16_t color);
void ST7735_FillRect(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2, uint16_t color);

void ST7735_PutChar5x7(uint16_t X, uint16_t Y, uint8_t chr, uint16_t color);
void ST7735_PutStr5x7(uint8_t X, uint8_t Y, char *str, uint16_t color);
