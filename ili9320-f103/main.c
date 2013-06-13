#include <string.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_spi.h>
#include <delay.h>


#define SPI_START 0x70   /* Start byte for SPI transfer */
#define SPI_RD    0x01   /* WR bit 1 within start */
#define SPI_WR    0x00   /* WR bit 0 within start */
#define SPI_DATA  0x02   /* RS bit 1 within start byte */
#define SPI_INDEX 0x00   /* RS bit 0 within start byte */


#define  ASCII_8X16_MS_Gothic
//#define  ASCII_8X16_System
#include <fonts.h>
#include <facepalm.h>
#include <face565.h>


uint8_t SPI_SendRecv(uint8_t byte) {
	while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE) == RESET);
	SPI_I2S_SendData(SPI1,byte);
	while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE) == RESET);
	uint8_t data = SPI_I2S_ReceiveData(SPI1);
	return data;
}

void LCD_WriteIndex(uint16_t _index) {
	GPIO_WriteBit(GPIOA,GPIO_Pin_2,Bit_RESET); // SPI_CS_LOW
	SPI_SendRecv(SPI_START | SPI_WR | SPI_INDEX);
	SPI_SendRecv(_index >> 8);   // Send Hi byte
	SPI_SendRecv(_index & 0xff); // Send Lo byte
	GPIO_WriteBit(GPIOA,GPIO_Pin_2,Bit_SET); // SPI_CS_HIGH
}

uint16_t LCD_ReadData() {
	GPIO_WriteBit(GPIOA,GPIO_Pin_2,Bit_RESET); // SPI_CS_LOW
	SPI_SendRecv(SPI_START | SPI_RD | SPI_DATA);
	SPI_SendRecv(0x00); // Dummy read
	uint16_t value = SPI_SendRecv(0x00); // Read Hi byte
	value <<= 8;
	value |= SPI_SendRecv(0x00); // Read Lo byte
	GPIO_WriteBit(GPIOA,GPIO_Pin_2,Bit_SET);   // SPI_CS_HIGH
	return value;
}

void LCD_WriteData(uint16_t _data) {
	GPIO_WriteBit(GPIOA,GPIO_Pin_2,Bit_RESET); // SPI_CS_LOW
	SPI_SendRecv(SPI_START | SPI_WR | SPI_DATA);
	SPI_SendRecv(_data >> 8);   // Send Hi byte
	SPI_SendRecv(_data & 0xff); //Send Lo byte
	GPIO_WriteBit(GPIOA,GPIO_Pin_2,Bit_SET);   // SPI_CS_HIGH
}

uint16_t LCD_ReadReg(uint16_t _reg) {
	LCD_WriteIndex(_reg);
	uint16_t LCD_RAM = LCD_ReadData();
	return LCD_RAM;
}

void LCD_WriteReg(uint16_t _reg, uint16_t _data) {
	LCD_WriteIndex(_reg);
	LCD_WriteData(_data);
}

void LCD_WriteDataOnly(uint16_t data) {
	SPI_SendRecv(data >> 8);
	SPI_SendRecv(data & 0xff);
}

void LCD_SetCursor(uint16_t X, uint16_t Y) {
	LCD_WriteReg(0x0020,Y);
	LCD_WriteReg(0x0021,X);
}

void LCD_Pixel(uint16_t X, uint16_t Y, uint16_t C) {
	LCD_SetCursor(X,Y);
	LCD_WriteReg(0x0022,C);
}

void LCD_SetWindow(uint16_t X, uint16_t Y, uint16_t W, uint16_t H) {
	uint16_t XW = X + W - 1;
	uint16_t YH = Y + H - 1;
	LCD_SetCursor(X,Y);
	LCD_WriteReg(0x0050,Y);  // Top RAM address pos
	LCD_WriteReg(0x0051,YH); // Bottom RAM address pos
	LCD_WriteReg(0x0052,X);  // Left RAM address pos
	LCD_WriteReg(0x0053,XW); // Right RAM address pos
}

void LCD_Clear(uint16_t color) {
	uint32_t i = 0;
	LCD_SetWindow(0,0,320,240);
	LCD_WriteIndex(0x0022);
	GPIO_WriteBit(GPIOA,GPIO_Pin_2,Bit_RESET); // SPI_CS_LOW
	SPI_SendRecv(SPI_START | SPI_WR | SPI_DATA);
	for (i = 0; i < 320*240; i++) { LCD_WriteDataOnly(color); }
	GPIO_WriteBit(GPIOA,GPIO_Pin_2,Bit_SET);   // SPI_CS_HIGH
}

uint16_t RGB888to565(uint8_t R,uint8_t G,uint8_t B) {
	return ((R >> 3) << 11) | ((G >> 2) << 5) | (B >> 3);
}

void LCD_HLine(uint16_t X0, uint16_t X1, uint16_t Y, uint16_t Color) {
	uint16_t W = X1 - X0 + 1;
    uint16_t i;

    LCD_SetWindow(X0,Y,W,1);
    LCD_WriteIndex(0x0022);
	GPIO_WriteBit(GPIOA,GPIO_Pin_2,Bit_RESET); // SPI_CS_LOW
	SPI_SendRecv(SPI_START | SPI_WR | SPI_DATA);
	for (i = 0; i < W; i++) { LCD_WriteDataOnly(Color); }
	GPIO_WriteBit(GPIOA,GPIO_Pin_2,Bit_SET);   // SPI_CS_HIGH
}

void LCD_VLine(uint16_t X, uint16_t Y0, uint16_t Y1, uint16_t Color) {
	uint16_t H = Y1 - Y0 + 1;
	uint16_t i;

	LCD_SetWindow(X,Y0,1,H);
	LCD_WriteIndex(0x0022);
	GPIO_WriteBit(GPIOA,GPIO_Pin_2,Bit_RESET); // SPI_CS_LOW
	SPI_SendRecv(SPI_START | SPI_WR | SPI_DATA);
	for (i = 0; i < H; i++) { LCD_WriteDataOnly(Color); }
	GPIO_WriteBit(GPIOA,GPIO_Pin_2,Bit_SET);   // SPI_CS_HIGH
}

void LCD_Rect(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, uint16_t Color) {
	LCD_HLine(X,X+W-1,Y,Color);
	LCD_HLine(X,X+W-1,Y+H-1,Color);
	LCD_VLine(X,Y,Y+H-1,Color);
	LCD_VLine(X+W-1,Y,Y+H-1,Color);
}

void LCD_FillRect(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, uint16_t Color) {
	uint32_t i = 0;
	LCD_SetWindow(X,Y,W,H);
	LCD_WriteIndex(0x0022);
	GPIO_WriteBit(GPIOA,GPIO_Pin_2,Bit_RESET); // SPI_CS_LOW
	SPI_SendRecv(SPI_START | SPI_WR | SPI_DATA);
	for (i = 0; i < W*H; i++) { LCD_WriteDataOnly(Color); }
	GPIO_WriteBit(GPIOA,GPIO_Pin_2,Bit_SET);   // SPI_CS_HIGH
}

void LCD_PutChar(uint16_t X, uint16_t Y, uint8_t Char, uint16_t chColor, uint16_t bgColor) {
	uint16_t i,j;
	uint8_t buffer[16],tmpCh;

    LCD_SetWindow(X,Y,8,16);
    memcpy(buffer,AsciiLib[Char-32],16);

    for (i = 0; i < 16; i++) {
    	tmpCh = buffer[i];
    	for (j = 0; j < 8; j++) {
    		if (((tmpCh >> (7-j)) & 0x01) == 0x01) {
        		LCD_Pixel(X+j,Y+i,chColor);
  			}
    	}
    }
}

void LCD_Text(uint16_t X, uint16_t Y, char *str, uint16_t chColor, uint16_t bgColor) {
    unsigned short tmpCh;

    do {
        tmpCh = *str++;
        LCD_PutChar(X,Y,tmpCh,chColor,bgColor);
        if (X < 320-8) { X += 8; } else if (Y < 240-16) { X = 0; Y += 16; } else { X = 0; Y = 0; }
    } while ( *str != 0 );
}

void LCD_BMP_Mono(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, const uint8_t *pBMP, uint16_t Color) {
	uint16_t i,j,k;
	uint8_t buffer[W];

	LCD_SetWindow(X,Y,W*8,H);
	for (i = 0; i < H; i++) {
		memcpy(buffer,FacePalm[i],W);
		for (j = 0; j < W; j++) {
			for (k = 0; k < 8; k++) {
				if ((buffer[j] >> (7-k)) & 0x01) {
					LCD_Pixel(X+(j << 3)+k,Y+i,Color);
				}
			}
		}
	}
}

void LCD_BMP(uint16_t X, uint16_t Y, uint16_t W, uint16_t H, const uint16_t *pBMP) {
	uint16_t i,j;

	LCD_SetWindow(X,Y,W,H);
	LCD_WriteIndex(0x0022);
	GPIO_WriteBit(GPIOA,GPIO_Pin_2,Bit_RESET); // SPI_CS_LOW
	SPI_SendRecv(SPI_START | SPI_WR | SPI_DATA);
	for (i = 0; i < H; i++) {
		for (j = 0; j < W; j++) {
			LCD_WriteDataOnly(Face565[i*80+j]);
		}
	}
	GPIO_WriteBit(GPIOA,GPIO_Pin_2,Bit_SET);   // SPI_CS_HIGH
}



int main(void)
{
	// Turn on SPI1 clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);

	// Configure SPI1 pins (PA4 = NSS, PA5 = SCK, PA6 = MISO, PA7 = MOSI)
	GPIO_InitTypeDef PORT;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	PORT.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7; // NSS,SCK,MISO output with PP
	PORT.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA,&PORT);
	PORT.GPIO_Pin = GPIO_Pin_6; // MISO input floating
	PORT.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA,&PORT);
	// Configure LCD_nRESET (PA3) pin for output with Push-Pull
	PORT.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA,&PORT);

	// Init LED pins
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	PORT.GPIO_Pin = (GPIO_Pin_9 | GPIO_Pin_8);
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	// PORT.GPIO_Speed = GPIO_Speed_50MHz; // <=== Already set in PORT variable
	GPIO_Init(GPIOC,&PORT);

	// Init SPI1
	SPI_InitTypeDef SPI;
	SPI.SPI_Mode = SPI_Mode_Master;
	SPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI.SPI_CPOL = SPI_CPOL_High;
	SPI.SPI_CPHA = SPI_CPHA_1Edge;
	SPI.SPI_CRCPolynomial = 7;
	SPI.SPI_DataSize = SPI_DataSize_8b;
	SPI.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI.SPI_NSS = SPI_NSS_Soft;
	SPI_Init(SPI1,&SPI);
	// NSS must be set to '1' due to NSS_Soft settings (otherwise it will be Multimaster mode).
	SPI_NSSInternalSoftwareConfig(SPI1,SPI_NSSInternalSoft_Set);

	// Enable SPI1
	SPI_Cmd(SPI1,ENABLE);

	// Reset LCD
	GPIO_WriteBit(GPIOA,GPIO_Pin_3,Bit_SET);   // LCD_nRESET = 1
	Delay_ms(1);                               // Wait 1ms
	GPIO_WriteBit(GPIOA,GPIO_Pin_3,Bit_RESET); // LCD_nRESET = 0
	Delay_ms(10);                              // Wait 10ms
	GPIO_WriteBit(GPIOA,GPIO_Pin_3,Bit_SET);   // LCD_nRESET = 1
	Delay_ms(50);                              // Wait 50ms -- necessary!

	// Init LCD
	LCD_WriteReg(0x0000,0x0000); // Stop oscillator
	LCD_WriteReg(0x0001,(0<<8)); // Driver output ctrl: shift S720 -> S1
	LCD_WriteReg(0x0002,(1<<10)|(1<<9)|(1<<8)); // LCD driving wave ctrl: Line inversion
	LCD_WriteReg(0x0003,(1<<12)|(1<<9)|(1<<5)|(1<<4)|(1<<3)); // Entry mode: Vert; H:dec; V:dec; RGB->BGR swap on
	LCD_WriteReg(0x0004,0x0000); // Scaling: off
	LCD_WriteReg(0x0008,(1<<9) | (1<<1)); // Display ctrl 2: 2 lines for back and front porch
	LCD_WriteReg(0x0009,0x0000); // Display ctrl 3: Scan mode: normal; Scan cycle: 0frame
	LCD_WriteReg(0x000a,0x0000); // Display ctrl 4: FMARK: disabled
	LCD_WriteReg(0x000c,(1<<0)); // RGB interface ctrl 1: 6-bit RGB; Internal sysclock/VSYNC;
	LCD_WriteReg(0x000d,0x0000); // Frame marker position: FMARK: 0th line
	LCD_WriteReg(0x000f,0x0000); // RGB interface ctrl 2
	Delay_ms(50);

	LCD_WriteReg(0x0007,(1<<8) | (1<<0)); // Display control: VCOM to GND; Internal operations On
	Delay_ms(50);

	LCD_WriteReg(0x0010,(1<<12)|(1<<7)|(1<<6)); // Power control 1: SAP: on; Power supply: on; Current Amplifiers = 1.0
	LCD_WriteReg(0x0011,(1<<2)|(1<<1)|(1<<0)); // Power control 2: disable stepup frequency
	LCD_WriteReg(0x0012,(1<<8)|(1<<4)); // Power control 3: VGL:on; internal VcomH
	LCD_WriteReg(0x0013,(1<<11)|(1<<9)|(1<<8)); // Power control 4: VREG1OUTx0.91
	LCD_WriteReg(0x0029,0x0000); // Power control 7
	LCD_WriteReg(0x002b,(1<<5)|(0<<4)); // internal resistor for oscillator; 120Hz frame rate
	LCD_WriteReg(0x0050,0); // Left RAM address pos
	LCD_WriteReg(0x0051,239); // Right RAM address pos
	LCD_WriteReg(0x0052,0); // Top RAM address pos
	LCD_WriteReg(0x0053,319); // Bottom RAM address pos
	Delay_ms(50);

	LCD_WriteReg(0x0060,(0<<15)|(1<<13)|(1<<10)|(1<<9)|(1<<8)); // Gate scan ctrl: 320 lines
	LCD_WriteReg(0x0061,(1<<0)); // Gate scan ctrl: grayscale inversion: on; scan: G1->G320; scrolling: off
	LCD_WriteReg(0x006a,0x0000); // Gate scan ctrl: scroll 0 lines

	LCD_WriteReg(0x0080,0x0000); // Partial image 1 display position
	LCD_WriteReg(0x0081,0x0000); // Partial image 1 RAM start address
	LCD_WriteReg(0x0082,0x0000); // Partial image 1 RAM end address
	LCD_WriteReg(0x0083,0x0000); // Partial image 2 display position
	LCD_WriteReg(0x0084,0x0000); // Partial image 2 RAM start address
	LCD_WriteReg(0x0085,0x0000); // Partial image 2 RAM end address

	LCD_WriteReg(0x0090,0x0000); // Panel interface ctrl 1: Clocks/Line: off; Clck=Fosc/1
	LCD_WriteReg(0x0092,0x0000); // Panel interface ctrl 2: Gate non-overlap period: 0 clocks
	LCD_WriteReg(0x0093,(1<<1)); //	Panel interface ctrl 3: Source output position: 1 clock
	LCD_WriteReg(0x0095,(1<<8)|(1<<4)); // Panel interface ctrl 4: 12 DOTCLKs
	LCD_WriteReg(0x0097,0x0000); // Panel interface ctrl 5: Gate non-overlap period: 0 clocks
	LCD_WriteReg(0x0098,0x0000); // Panel interface ctrl 6: Source output position: 0 clocks
	LCD_WriteReg(0x0007,(1<<8)|(1<<5)|(1<<4)|(1<<1)|(1<<0)); // Display control: BASEE=1; Normal display on
	Delay_ms(100);

//	LCD_Clear(RGB888to565(255,0,0)); // RED
//	LCD_Clear(RGB888to565(0,255,0)); // GREEN
//	LCD_Clear(RGB888to565(0,0,255)); // BLUE
	LCD_Clear(RGB888to565(0,0,0)); // BLACK

	LCD_Rect(0,0,320,240,RGB888to565(255,255,255));

	LCD_Text(5,5,"Hello world!",RGB888to565(255,0,255),RGB888to565(0,100,0));
	LCD_Text(6,32,"Look at this extremally slow screen :(",RGB888to565(0,255,255),RGB888to565(32,64,32));
	LCD_Text(40,50,"serial interface totally SUXX",RGB888to565(255,255,0),RGB888to565(0,0,0));
	LCD_Text(50,70,"... and 65K colors only ...",RGB888to565(255,128,0),RGB888to565(0,0,0));

	LCD_BMP_Mono(118,118,26,120,&FacePalm[0][0],RGB888to565(0,162,232));

	LCD_Text(5,220,"(c) Dimon",RGB888to565(255,55,55),RGB888to565(32,64,16));

	LCD_Rect(19,99,82,98,RGB888to565(0,0,255));
	LCD_BMP(20,100,80,96,&Face565[0]);
	LCD_Text(106,102,"< 16-bit color",RGB888to565(255,255,0),RGB888to565(0,0,0));

	while(1) {
    }
}
