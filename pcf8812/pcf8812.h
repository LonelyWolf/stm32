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

