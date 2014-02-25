// PCF8812 pins
#define PCF8812_SCK_PIN   GPIO_Pin_5     // PA5
#define PCF8812_SCK_PORT  GPIOA
#define PCF8812_MOSI_PIN  GPIO_Pin_7     // PA7
#define PCF8812_MOSI_PORT GPIOA
#define PCF8812_PWR_PIN   GPIO_Pin_0     // PA0
#define PCF8812_PWR_PORT  GPIOA
#define PCF8812_DC_PIN    GPIO_Pin_1     // PA1
#define PCF8812_DC_PORT   GPIOA
#define PCF8812_CS_PIN    GPIO_Pin_2     // PA2
#define PCF8812_CS_PORT   GPIOA
#define PCF8812_RES_PIN   GPIO_Pin_3     // PA3
#define PCF8812_RES_PORT  GPIOA

// SCK
#define SCK_L() GPIO_ResetBits(PCF8812_SCK_PORT,PCF8812_SCK_PIN)
#define SCK_H() GPIO_SetBits(PCF8812_SCK_PORT,PCF8812_SCK_PIN)

// MOSI
#define MOSI_L() GPIO_ResetBits(PCF8812_MOSI_PORT,PCF8812_MOSI_PIN)
#define MOSI_H() GPIO_SetBits(PCF8812_MOSI_PORT,PCF8812_MOSI_PIN)

// CS
#define CS_L() GPIO_ResetBits(PCF8812_CS_PORT,PCF8812_CS_PIN)
#define CS_H() GPIO_SetBits(PCF8812_CS_PORT,PCF8812_CS_PIN)

// D/C
#define DC_L() GPIO_ResetBits(PCF8812_DC_PORT,PCF8812_DC_PIN)
#define DC_H() GPIO_SetBits(PCF8812_DC_PORT,PCF8812_DC_PIN)

// PWR
#define PWR_L() GPIO_ResetBits(PCF8812_PWR_PORT,PCF8812_PWR_PIN)
#define PWR_H() GPIO_SetBits(PCF8812_PWR_PORT,PCF8812_PWR_PIN)

// RES
#define RES_L() GPIO_ResetBits(PCF8812_RES_PORT,PCF8812_RES_PIN)
#define RES_H() GPIO_SetBits(PCF8812_RES_PORT,PCF8812_RES_PIN)


typedef enum {
	transparent  = 0,
	opaque       = 1
} Opaque_TypeDef;

typedef enum {
	PSet = 1,
	PReset = 0
} PSetReset_TypeDef;


extern uint8_t vRAM[917]; // Display buffer


void PCF8812_Init();

void PCF8812_PowerOn(void);
void PCF8812_Reset(void);
void PCF8812_Write(uint8_t data);

void PCF8812_SetXY(uint8_t X, uint8_t Y);
void PCF8812_Flush(void);

void PCF8812_Fill(uint8_t pattern);
void PCF8812_SetPixel(uint8_t X, uint8_t Y);
void PCF8812_ResetPixel(uint8_t X, uint8_t Y);
void PCF8812_HLine(uint8_t X1, uint8_t X2, uint8_t Y, PSetReset_TypeDef SR);
void PCF8812_VLine(uint8_t X, uint8_t Y1, uint8_t Y2, PSetReset_TypeDef SR);
void PCF8812_Rect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2, PSetReset_TypeDef SR);
void PCF8812_FillRect(uint8_t X1, uint8_t Y1, uint8_t X2, uint8_t Y2, PSetReset_TypeDef SR);
void PCF8812_Line(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2);
void PCF8812_Ellipse(uint16_t X, uint16_t Y, uint16_t A, uint16_t B);
void PCF8812_PutChar5x7(uint8_t X, uint8_t Y, uint8_t Char, Opaque_TypeDef bckgnd);
void PCF8812_PutStr5x7(uint8_t X, uint8_t Y, char *str, Opaque_TypeDef bckgnd);
void PCF8812_PutInt5x7(uint8_t X, uint8_t Y, uint32_t num, Opaque_TypeDef bckgnd);
void PCF8812_PutHex5x7(uint8_t X, uint8_t Y, uint32_t num, Opaque_TypeDef bckgnd);
