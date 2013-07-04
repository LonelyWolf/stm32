#include <stm32f10x_gpio.h>
#include <delay.h>
#include <ssd1289.h>

void write_command(uint16_t cmd) {
/*
    PORT_data_L = command;//выставили команду на шину
    PORT_data_H = 0x00;//
    PORT_command &= ~(1<<RS);//дрыгаем
    PORT_command |=  (1<<WR) | (1<<RD) | (1<<CS);//ногами
    PORT_command &= ~(1<<WR);//согласно
    PORT_command &= ~(1<<CS);//даташита
    asm("nop");asm("nop");asm("nop");asm("nop");//длительность строба
    PORT_command |=  (1<<CS);//
    PORT_command |=  (1<<WR);//
    PORT_data_L = 0x00;//
    PORT_data_H = 0x00;//
*/
}

void write_data(uint16_t data) {
/*
    PORT_data_L = (unsigned char)(data & 0x00FF);//выставили младший байт данных
    PORT_data_H = (unsigned char)(data>>8);//старший байт данных
    PORT_command |=  (1<<RS);//дрыгаем
    PORT_command |=  (1<<WR) | (1<<RD) | (1<<CS);//ногами
    PORT_command &= ~(1<<WR);//согласно
    PORT_command &= ~(1<<CS);//даташита
    asm("nop");asm("nop");asm("nop");asm("nop");//длительность строба
    PORT_command |=  (1<<CS);//
    PORT_command |=  (1<<WR);//
    PORT_data_L = 0x00;//
    PORT_data_H = 0x00;//
*/
}

void LCD_Reset(void) {
	GPIO_WriteBit(GPIOB,GPIO_Pin_10,Bit_SET);
	Delay_ms(5);
	GPIO_WriteBit(GPIOB,GPIO_Pin_10,Bit_RESET);
	Delay_ms(5);
	GPIO_WriteBit(GPIOB,GPIO_Pin_10,Bit_SET);
	Delay_ms(5);
}
