#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <delay.h>
#include <lcd1602.h>


// Send strobe to LCD via E line
void LCD_strobe(void) {
	GPIO_WriteBit(GPIOB,pin_E,Bit_SET);
	Delay_us(1); // Due to datasheet E cycle time is about ~500ns
	GPIO_WriteBit(GPIOB,pin_E,Bit_RESET);
	Delay_us(1); // Due to datasheet E cycle time is about ~500ns
}

// Send low nibble of cmd to LCD via 4bit bus
void LCD_send_4bit(uc8 cmd) {
//	uint16_t tmp = 0;
//	tmp = GPIO_ReadOutputData(GPIOB) & 0x0FFF; // read PortB content and clean high nibble
//	tmp |= cmd<<12;
//	GPIO_Write(GPIOB,tmp);
	if (cmd & (1<<0)) GPIO_WriteBit(GPIOB,pin_DB4,Bit_SET); else GPIO_WriteBit(GPIOB,pin_DB4,Bit_RESET);
	if (cmd & (1<<1)) GPIO_WriteBit(GPIOB,pin_DB5,Bit_SET); else GPIO_WriteBit(GPIOB,pin_DB5,Bit_RESET);
	if (cmd & (1<<2)) GPIO_WriteBit(GPIOB,pin_DB6,Bit_SET); else GPIO_WriteBit(GPIOB,pin_DB6,Bit_RESET);
	if (cmd & (1<<3)) GPIO_WriteBit(GPIOB,pin_DB7,Bit_SET); else GPIO_WriteBit(GPIOB,pin_DB7,Bit_RESET);
	LCD_strobe();
}

// Send command to LCD via 4bit bus
void LCD_cmd_4bit(uc8 cmd) {
    GPIO_WriteBit(GPIOB,pin_RS,Bit_RESET);
    LCD_send_4bit(cmd>>4); // send high nibble
    LCD_send_4bit(cmd); // send low nibble
    Delay_us(40); // typical command takes about 39us
}

// Send data to LCD via 4bit bus
void LCD_data_4bit(uc8 data) {
    GPIO_WriteBit(GPIOB,pin_RS,Bit_SET);
    LCD_send_4bit(data>>4);                 // send high nibble
    LCD_send_4bit(data);                    // send low nibble
    GPIO_WriteBit(GPIOB,pin_RS,Bit_RESET);
    Delay_us(44);                           // write data to RAM takes about 43us
}

// Set cursor position on LCD
// column : Column position
// line   : Line position
void LCD_GotoXY(int column, int line) {
    LCD_cmd_4bit((column+(line<<6)) | 0x80);  // Set DDRAM address with coordinates
}

// Init LCD to 4bit bus mode
void LCD_Init(void) {
	Delay_ms(15);              // must wait >=30us after LCD Vdd rises to 4.5V
	LCD_send_4bit(0b00000011); // select 4-bit bus (still 8bit)
	Delay_ms(5);               // must wait more than 4.1ms
	LCD_send_4bit(0b00000011); // select 4-bit bus (still 8bit)
	Delay_us(100);             // must wait more than 100us
	LCD_send_4bit(0b00000011); // select 4-bit bus (still 8bit)
	LCD_send_4bit(0b00000010); // Function set: 4-bit bus (gotcha!)

	LCD_cmd_4bit(0x28); // LCD Function: 2 Lines, 5x8 matrix
	LCD_cmd_4bit(0x0C); // Display control: Display: on, cursor: off
	LCD_cmd_4bit(0x06); // Entry mode: increment, shift disabled
}

// Clear LCD display and set cursor at first position
void LCD_Cls(void) {
	LCD_cmd_4bit(0x01); // Clear display command
	Delay_ms(2); // Numb display does it at least 1.53ms
	LCD_cmd_4bit(0x02); // Return Home command
	Delay_ms(2); // Numb display does it at least 1.53ms
}

// Send string to LCD
void LCD_Print(char *string) {
    while (*string) { LCD_data_4bit(*string++); }
}

// Send integer to LCD
void LCD_PrintI(uint32_t num) {
	char str[11]; // 10 chars max for UINT32_MAX
	int i = 0;
	do { str[i++] = num % 10 + '0'; } while ((num /= 10) > 0);
	for (i--; i>=0; i--) { LCD_data_4bit(str[i]); }
}

// Send HEX integer to LCD
void LCD_PrintH(uint32_t num) {
	char str[11]; // 10 chars max for UINT32_MAX in HEX
	int i = 0;
	do { str[i++] = "0123456789ABCDEF"[num % 0x10]; } while ((num /= 0x10) > 0);
	str[i++] = 'x';
	str[i++] = '0';
	for (i--; i>=0; i--) { LCD_data_4bit(str[i]); }
}
