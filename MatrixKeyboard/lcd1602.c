#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <delay.h>
#include <lcd1602.h>


// Send strobe to LCD via E line
void LCD_strobe(void) {
	GPIO_WriteBit(GPIOB,pin_E,Bit_SET);
	Delay_us(4); // Due to datasheet E cycle time is about ~500ns
	GPIO_WriteBit(GPIOB,pin_E,Bit_RESET);
	Delay_us(4); // Due to datasheet E cycle time is about ~500ns
}

// Send low nibble of cmd to LCD via 4bit bus
void LCD_send_4bit(uc8 cmd) {
	GPIO_WriteBit(GPIOB,pin_DB4,cmd & (1<<0) ? Bit_SET : Bit_RESET);
	GPIO_WriteBit(GPIOB,pin_DB5,cmd & (1<<1) ? Bit_SET : Bit_RESET);
	GPIO_WriteBit(GPIOB,pin_DB6,cmd & (1<<2) ? Bit_SET : Bit_RESET);
	GPIO_WriteBit(GPIOB,pin_DB7,cmd & (1<<3) ? Bit_SET : Bit_RESET);
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
	Delay_ms(30);              // must wait >=30us after LCD Vdd rises to 4.5V
	LCD_send_4bit(0b00000011); // select 4-bit bus (still 8bit)
	Delay_ms(5);               // must wait more than 4.1ms
	LCD_send_4bit(0b00000011); // select 4-bit bus (still 8bit)
	Delay_us(150);             // must wait more than 100us
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

// Send BIN integer to LCD (max 8bit number)
void LCD_PrintB8(uint8_t num) {
	char str[8] = "00000000";
	int i = 0;
	do { str[i++] = num % 2 + '0'; } while ((num /= 2) > 0);
	for (i=7; i>=0; i--) { LCD_data_4bit(str[i]); }
}

// Send BIN integer to LCD (max 16bit number)
void LCD_PrintB16(uint16_t num) {
	char str[16] = "0000000000000000";
	int i = 0;
	do { str[i++] = num % 2 + '0'; } while ((num /= 2) > 0);
	for (i=15; i>=0; i--) { LCD_data_4bit(str[i]); }
}

