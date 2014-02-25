#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_spi.h>

#include <pcf8812.h>
#include <delay.h>


uint8_t vRAM[917]; // Display buffer


void PCF8812_Init() {
	// Configure pins as output with Push-Pull
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitTypeDef PORT;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;

	PORT.GPIO_Pin = PCF8812_MOSI_PIN;
	GPIO_Init(PCF8812_MOSI_PORT,&PORT);

	PORT.GPIO_Pin = PCF8812_SCK_PIN;
	GPIO_Init(PCF8812_SCK_PORT,&PORT);

	PORT.GPIO_Pin = PCF8812_CS_PIN;
	GPIO_Init(PCF8812_CS_PORT,&PORT);

	PORT.GPIO_Pin = PCF8812_PWR_PIN;
	GPIO_Init(PCF8812_PWR_PORT,&PORT);

	PORT.GPIO_Pin = PCF8812_DC_PIN;
	GPIO_Init(PCF8812_DC_PORT,&PORT);

	PORT.GPIO_Pin = PCF8812_RES_PIN;
	GPIO_Init(PCF8812_RES_PORT,&PORT);

	CS_H();
	RES_H();
	DC_L();
	PWR_L();
}

// PCF8812 power on
void PCF8812_PowerOn(void) {
	CS_L();
	RES_L();
	Delay_ms(20);
	PWR_H();
	Delay_ms(20);
	RES_H();
}

// Hardware reset of PCF8812
void PCF8812_Reset(void) {
	RES_L();
	RES_H();
}

// Software SPI send byte
void PCF8812_Write(uint8_t data) {
	uint8_t i;

	for (i = 0; i < 8; i++) {
		if (data & 0x80) MOSI_H(); else MOSI_L();
		data <<= 1;
		SCK_L();
		SCK_H();
	}
}

// Set RAM address (Y - bank number, X - position in bank)
void PCF8812_SetXY(uint8_t X, uint8_t Y) {
	DC_L();
	PCF8812_Write(0x40 | Y); // Select display RAM bank (0..8)
	PCF8812_Write(0x80 | X); // Set X address (0..101)
}

// Send vRAM buffer into display
void PCF8812_Flush(void) {
	uint32_t i;

	DC_L();
	PCF8812_Write(0x40); // Select display RAM bank 0
	PCF8812_Write(0x80); // Set column 0
	DC_H();
	for (i = 0; i < 816; i++) PCF8812_Write(vRAM[i]);
}

// Fill vRAM with byte pattern
void PCF8812_Fill(uint8_t pattern) {
	uint32_t i;

	for (i = 0; i < 816; i++) vRAM[i] = pattern;
}

// Set pixel in vRAM buffer
void PCF8812_SetPixel(uint8_t X, uint8_t Y) {
	vRAM[((Y / 8) * 102) + X] |= 1 << (Y % 8);
}

// Clear pixel in vRAM buffer
void PCF8812_ResetPixel(uint8_t X, uint8_t Y) {
	vRAM[((Y / 8) * 102) + X] &= ~(1 << (Y % 8));
}
