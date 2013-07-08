#define LCD_RESET  GPIO_Pin_10;
#define LCD_RS     GPIO_Pin_11;
#define LCD_WR     GPIO_Pin_12;
#define LCD_RD     GPIO_Pin_13;
#define LCD_CS     GPIO_Pin_14;


//#define  ASCII_8X16_MS_Gothic
#define  ASCII_8X16_System
#include <fonts.h>


uint16_t RGB565(uint8_t R,uint8_t G,uint8_t B);
void LCD_Reset(void);
void LCD_Init(void);
void LCD_write_command(uint16_t cmd);
void LCD_write_data(uint16_t data);
void LCD_WriteReg(uint16_t reg, uint16_t data);
uint16_t LCD_ReadReg(uint16_t reg);
void LCD_SetCursor(uint16_t X, uint16_t Y);
void LCD_Pixel(uint16_t X, uint16_t Y, uint16_t C);
void LCD_SetWindow(uint16_t X, uint16_t Y, uint16_t W, uint16_t H);
void LCD_Clear(uint16_t C);

// Drawing
void LCD_HLine(uint16_t X0, uint16_t X1, uint16_t Y, uint16_t Color);
void LCD_VLine(uint16_t X, uint16_t Y0, uint16_t Y1, uint16_t Color);
void LCD_Line(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, uint16_t Color);
void LCD_LineAA(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, uint16_t Color);
void LCD_Rect(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, uint16_t Color);
void LCD_FillRect(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, uint16_t Color);
void LCD_Ellipse(uint16_t X, uint16_t Y, uint16_t A, uint16_t B, uint16_t Color);
void LCD_FillEllipse(uint16_t X, uint16_t Y, uint16_t A, uint16_t B, uint16_t Color);

// Text
void LCD_PutChar(uint16_t X, uint16_t Y, uint8_t Char, uint16_t Color);
void LCD_PutCharO(uint16_t X, uint16_t Y, uint8_t Char, uint16_t Color, uint16_t bgColor);
void LCD_PutStr(uint16_t X, uint16_t Y, char *str, uint16_t Color);
void LCD_PutStrO(uint16_t X, uint16_t Y, char *str, uint16_t Color, uint16_t bgColor);
void LCD_PutInt(uint16_t X, uint16_t Y, uint32_t num, uint16_t Color);
void LCD_PutIntO(uint16_t X, uint16_t Y, uint32_t num, uint16_t Color, uint16_t bgColor);
void LCD_PutHex(uint16_t X, uint16_t Y, uint32_t num, uint16_t Color);
void LCD_PutHexO(uint16_t X, uint16_t Y, uint32_t num, uint16_t Color, uint16_t bgColor);

// Pictures
void LCD_BMPMono(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, const uint8_t* pBMP, uint16_t Color);
void LCD_BMPMonoO(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, const uint8_t* pBMP, uint16_t Color, uint16_t bgColor);
void LCD_BMP(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, const uint16_t* pBMP);
