#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_i2c.h>

#include <uart.h>
#include <lm75.h>


int main(void)
{
	UART_Init();
	UART_SendStr("\nSTM32F103RET6 is online.\n");

	UART_SendStr("I2C init ... ");
	if (!LM75_Init(100000)) UART_SendStr("ready.\n"); else {
		UART_SendStr("fail.\n");
		UART_SendStr("MCU halted now.\n");
		while(1);
	}

	uint16_t value;

    value = LM75_ReadReg(0x00);
	UART_SendHex16(value); UART_SendChar('\n');
	value = LM75_ReadConf();
	UART_SendHex8(value); UART_SendChar('\n');
	value = LM75_ReadReg(0x02);
	UART_SendHex16(value); UART_SendChar('\n');
	value = LM75_ReadReg(0x03);
	UART_SendHex16(value); UART_SendChar('\n');

    LM75_Shutdown(DISABLE);

    int16_t temp = LM75_Temperature();
    UART_SendInt(temp / 10); UART_SendChar('.');
    temp %= 10;
    if (temp < 0) temp *= -1;
    UART_SendInt(temp % 10); UART_SendStr("C\n");

    while(1);
}
