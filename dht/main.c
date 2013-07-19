#include <stm32f10x.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_gpio.h>
#include <delay.h>
#include <uart.h>


uint32_t bits[40];

uint8_t  humidity_l,humidity_h;
uint8_t  temperature_l,temperature_h;
uint16_t humidity,temperature;
uint8_t  parity_rcv,parity_cmp;


uint32_t read_bit(void) {

	return 0;
}


int main(void)
{
	UART_Init();

	UART_SendStr("\nSTM32F103RET6 is online.\n");

	GPIO_InitTypeDef PORT;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Pin = GPIO_Pin_15;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&PORT);

	//GPIOB->BSRR = GPIO_Pin_15; // 2.9V
	//GPIOB->BRR  = GPIO_Pin_15; // 0.0V

	dTimerInit();

	uint32_t st,et;
	uint8_t i = 0;

	UART_SendStr("Give a few seconds to AM2302 for powerup stabilization...\n");
	//Delay_ms(2000); // Wait 2 seconds for AM2302 to stabilize after powerup

	UART_SendStr("Let's talk with AM2302...\n");
	// AM2302 Single-bus communications
	GPIOB->BRR = GPIO_Pin_15; // Pull down SDA (Bit_SET)
	Delay_ms(10); // Host start signal at least 800us
	GPIOB->BSRR = GPIO_Pin_15; // Release SDA (Bit_RESET)
	PORT.GPIO_Mode = GPIO_Mode_IPU; // Switch pin to input with Pull-Up
	GPIO_Init(GPIOB,&PORT);

	// Wait for AM2302 to start communicate
	st = dTimerGet_us();
	while ((GPIOB->IDR & GPIO_Pin_15) != (uint32_t)Bit_RESET && (dTimerGet_us()-st) < 250);
	et = dTimerGet_us();
	if (et-st >= 200) {
		UART_SendStr("No response from sensor.\n");
		UART_SendInt(et-st); UART_SendChar('\n');
		while(1);
	}

	st = dTimerGet_us();
	while (((GPIOB->IDR & GPIO_Pin_15) == (uint32_t)Bit_RESET) && ((dTimerGet_us()-st) < 100));
	et = dTimerGet_us();
	if ((et-st < 70) || (et-st > 90)) {
		UART_SendStr("Bad response from sensor.\n");
		UART_SendInt(et-st); UART_SendChar('\n');
		while(1);
	}

	st = dTimerGet_us();
	while ((GPIOB->IDR & GPIO_Pin_15) != (uint32_t)Bit_RESET && (dTimerGet_us()-st) < 100);
	et = dTimerGet_us();
	if ((et-st < 70) || (et-st > 90)) {
		UART_SendStr("Bad response from sensor.\n");
		UART_SendInt(et-st); UART_SendChar('\n');
		while(1);
	}

	// ACK strobe received, now receive 40 bits
	while (i < 40) {
		// Measure Tlow time (bit start impulse)
		st = dTimerGet_us();
		while ((GPIOB->IDR & GPIO_Pin_15) == (uint32_t)Bit_RESET);
		et = dTimerGet_us();

		if (et-st > 70) {
			// bit start fail
			bits[i] = 0xffff;
			while ((GPIOB->IDR & GPIO_Pin_15) != (uint32_t)Bit_RESET);
		} else {
			// bit start ok
			st = dTimerGet_us();
			while ((GPIOB->IDR & GPIO_Pin_15) != (uint32_t)Bit_RESET);
			et = dTimerGet_us();
			bits[i] = et-st;
		}
		i++;
	}

	humidity_h = 0;
	for (i = 0; i < 8; i++) {
		humidity_h <<= 1;
		if (bits[i] > 40) humidity_h |= 1;
	}
	humidity_l = 0;
	for (i = 8; i < 16; i++) {
		humidity_l <<= 1;
		if (bits[i] > 40) humidity_l |= 1;
	}
	temperature_h = 0;
	for (i = 16; i < 24; i++) {
		temperature_h <<= 1;
		if (bits[i] > 40) temperature_h |= 1;
	}
	temperature_l = 0;
	for (i = 24; i < 32; i++) {
		temperature_l <<= 1;
		if (bits[i] > 40) temperature_l |= 1;
	}
	for (i = 32; i < 40; i++) {
		parity_rcv <<= 1;
		if (bits[i] > 40) parity_rcv |= 1;
	}

	parity_cmp = humidity_h + humidity_l + temperature_h + temperature_l;
	humidity = (humidity_h << 8) + humidity_l;
	temperature = (temperature_h << 8) + temperature_l;

	UART_SendStr("Parity: ");
	UART_SendInt(parity_rcv);
	UART_SendStr(" = ");
	UART_SendInt(parity_cmp);
	UART_SendChar('\n');
	UART_SendStr("Humidity: ");
	UART_SendInt(humidity / 10); UART_SendChar('.');
	UART_SendInt(humidity % 10); UART_SendStr("%RH");
	UART_SendChar('\n');
	UART_SendStr("Temperature: ");
	UART_SendInt(temperature / 10); UART_SendChar('.');
	UART_SendInt(temperature % 10); UART_SendStr("C");
	UART_SendChar('\n');

	while (1);
}
