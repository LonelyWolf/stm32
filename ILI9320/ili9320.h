//#define SOFT_SPI

#define SPI_START 0x70   /* Start byte for SPI transfer */
#define SPI_RD    0x01   /* WR bit 1 within start */
#define SPI_WR    0x00   /* WR bit 0 within start */
#define SPI_DATA  0x02   /* RS bit 1 within start byte */
#define SPI_INDEX 0x00   /* RS bit 0 within start byte */


//#define  ASCII_8X16_MS_Gothic
#define  ASCII_8X16_System
#include <fonts.h>


void LCD_WriteIndex(uint16_t _index);
uint16_t LCD_ReadData();
void LCD_WriteData(uint16_t _data);
uint16_t LCD_ReadReg(uint16_t _reg);
void LCD_WriteReg(uint16_t _reg, uint16_t _data);
void LCD_WriteDataOnly(uint16_t data);
void LCD_Init();
void LCD_SetCursor(uint16_t X, uint16_t Y);
void LCD_Pixel(uint16_t X, uint16_t Y, uint16_t C);
void LCD_SetWindow(uint16_t X, uint16_t Y, uint16_t W, uint16_t H);
void LCD_Clear(uint16_t color);
uint16_t RGB565(uint8_t R,uint8_t G,uint8_t B);
void LCD_HLine(uint16_t X0, uint16_t X1, uint16_t Y, uint16_t Color);
void LCD_VLine(uint16_t X, uint16_t Y0, uint16_t Y1, uint16_t Color);
void LCD_Line(int16_t X1, int16_t Y1, int16_t X2, int16_t Y2, uint16_t Color);void LCD_Rect(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, uint16_t Color);
void LCD_FillRect(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, uint16_t Color);
void LCD_Ellipse(uint16_t X, uint16_t Y, uint16_t A, uint16_t B, uint16_t Color);void LCD_PutChar(uint16_t X, uint16_t Y, uint8_t Char, uint16_t Color);
void LCD_PutCharO(uint16_t X, uint16_t Y, uint8_t Char, uint16_t Color, uint16_t bgColor);
void LCD_PutStr(uint16_t X, uint16_t Y, char *str, uint16_t Color);
void LCD_PutStrO(uint16_t X, uint16_t Y, char *str, uint16_t Color, uint16_t bgColor);
void LCD_PutInt(uint16_t X, uint16_t Y, uint32_t num, uint16_t Color);
void LCD_PutIntO(uint16_t X, uint16_t Y, uint32_t num, uint16_t Color, uint16_t bgColor);
void LCD_PutHex(uint16_t X, uint16_t Y, uint32_t num, uint16_t Color);
void LCD_PutHexO(uint16_t X, uint16_t Y, uint32_t num, uint16_t Color, uint16_t bgColor);
void LCD_BMP_Mono(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, const uint8_t* pBMP, uint16_t Color);
void LCD_BMP_MonoO(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, const uint8_t* pBMP, uint16_t Color, uint16_t bgColor);
void LCD_BMP(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, const uint16_t* pBMP);
