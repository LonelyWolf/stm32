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

void LCD_SetCursor(uint16_t X, uint16_t Y) {
	LCD_WriteReg(0x0020,239-Y);
	LCD_WriteReg(0x0021,X);
}

void LCD_WriteData_Only(uint16_t data) {
	SPI_SendRecv(data >> 8);
	SPI_SendRecv(data & 0xff);
}

void LCD_SetPoint(uint16_t X, uint16_t Y, uint16_t C) {
	LCD_SetCursor(X,Y);
	LCD_WriteReg(0x0022,C);
}

void LCD_Clear(uint16_t color) {
	uint32_t i = 0;
	LCD_WriteReg(0x0020,0);
	LCD_WriteReg(0x0021,0);
	LCD_WriteIndex(0x0022);
	GPIO_WriteBit(GPIOA,GPIO_Pin_2,Bit_RESET); // SPI_CS_LOW
	SPI_SendRecv(SPI_START | SPI_WR | SPI_DATA);
	for (i = 0; i < 320*240; i++) {
		LCD_WriteData_Only(color);
	}
	GPIO_WriteBit(GPIOA,GPIO_Pin_2,Bit_SET);   // SPI_CS_HIGH
}

uint16_t RGB888to565(uint8_t R,uint8_t G,uint8_t B) {
	return ((R >> 3) << 11) | ((G >> 2) << 5) | (B >> 3);
}

void GetASCIICode(uint8_t* pBuffer, uint8_t ASCII) {
	memcpy(pBuffer,AsciiLib[ASCII-32],16);
}

void LCD_PutChar(uint16_t X, uint16_t Y, uint8_t Char, uint16_t chColor, uint16_t bgColor) {
	uint16_t i,j;
	uint8_t buffer[16],tmpCh;

    GetASCIICode(buffer,Char);

    for (i = 0; i < 16; i++) {
    	tmpCh = buffer[i];
    	for (j = 0; j < 8; j++) {
    		if (((tmpCh >> (7-j)) & 0x01) == 0x01) {
        		LCD_SetPoint(X+j,Y+i,chColor);
  			}
//    		LCD_SetPoint(X+j,Y+i,((tmpCh >> (7-j)) & 0x01) == 0x01 ? chColor : bgColor);
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

	for (i = 0; i < H; i++) {
		memcpy(buffer,FacePalm[i],W);
//		GetBMPCode(buffer,i,W);
		for (j = 0; j < W; j++) {
			for (k = 0; k < 8; k++) {
				if (((buffer[j] >> (7-k)) & 0x01) == 0x01) {
					LCD_SetPoint(X+(j << 3)+k,Y+i,Color);
//				} else {
//					LCD_SetPoint(X+(j << 3)+k,Y+i,65535-Color);
				}
			}
		}
	}
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

	uint16_t DevCode = LCD_ReadReg(0x0000);

	// Init LCD
	LCD_WriteReg(0x0000,0x0000);
	LCD_WriteReg(0x0001,0x0100);
	LCD_WriteReg(0x0002,0x0700);
	LCD_WriteReg(0x0003,0x1008);
	LCD_WriteReg(0x0004,0x0000);
	LCD_WriteReg(0x0008,0x0202);
	LCD_WriteReg(0x0009,0x0000);
	LCD_WriteReg(0x000a,0x0000);
	LCD_WriteReg(0x000c,(1<<0));
	LCD_WriteReg(0x000d,0x0000);
	LCD_WriteReg(0x000f,0x0000);
	Delay_ms(50);

	LCD_WriteReg(0x0007,0x0101);
	Delay_ms(50);

	LCD_WriteReg(0x0010,(1<<12)|(0<<8)|(1<<7)|(1<<6)|(0<<4));
	LCD_WriteReg(0x0011,0x0007);
	LCD_WriteReg(0x0012,(1<<8)|(1<<4)|(0<<0));
	LCD_WriteReg(0x0013,0x0b00);
	LCD_WriteReg(0x0029,0x0000);
	LCD_WriteReg(0x002b,(1<<14));
	LCD_WriteReg(0x0050,0x0000);
	LCD_WriteReg(0x0051,239);
	LCD_WriteReg(0x0052,0x0000);
	LCD_WriteReg(0x0053,319);
	Delay_ms(50);

	LCD_WriteReg(0x0060,0x2700);
	LCD_WriteReg(0x0061,0x0001);
	LCD_WriteReg(0x006a,0x0000);

	LCD_WriteReg(0x0080,0x0000);
	LCD_WriteReg(0x0081,0x0000);
	LCD_WriteReg(0x0082,0x0000);
	LCD_WriteReg(0x0083,0x0000);
	LCD_WriteReg(0x0084,0x0000);
	LCD_WriteReg(0x0085,0x0000);

	LCD_WriteReg(0x0090,(0<<7)|(16<<0));
	LCD_WriteReg(0x0092,0x0000);
	LCD_WriteReg(0x0093,0x0001);
	LCD_WriteReg(0x0095,0x0110);
	LCD_WriteReg(0x0097,(0<<8));
	LCD_WriteReg(0x0098,0x0000);
	LCD_WriteReg(0x0007,0x0133);
	Delay_ms(100);

	DevCode = LCD_ReadReg(0x0000);

	GPIO_WriteBit(GPIOC,DevCode == 0x0 ? GPIO_Pin_9 : GPIO_Pin_8,Bit_SET);

//	LCD_Clear(RGB888to565(255,0,0)); // RED
//	LCD_Clear(RGB888to565(0,255,0)); // GREEN
//	LCD_Clear(RGB888to565(0,0,255)); // BLUE
	LCD_Clear(0); // BLACK

	LCD_SetPoint(0,0,RGB888to565(255,0,0));
	LCD_SetPoint(319,239,RGB888to565(0,0,255));

	LCD_Text(10,10,"Hello world!",RGB888to565(255,0,255),RGB888to565(0,0,0));
	LCD_Text(10,38,"Look at this extremally slow screen :(",RGB888to565(0,255,255),RGB888to565(32,64,32));
	LCD_Text(40,60,"serial interface totally SUXX",RGB888to565(255,255,0),RGB888to565(0,0,0));
	LCD_Text(50,80,"... and 65K colors only ...",RGB888to565(255,128,0),RGB888to565(0,0,0));
//	LCD_Text(230,200,"/FACEPALM",RGB888to565(255,55,55),RGB888to565(32,64,16));

	LCD_BMP_Mono(120,120,26,120,&FacePalm[0][0],RGB888to565(0,162,232));

	LCD_Text(20,190,"(c) Dimon",RGB888to565(255,55,55),RGB888to565(32,64,16));

    while(1)
    {
    }
}
