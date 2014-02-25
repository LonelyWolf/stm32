#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_spi.h>
#include <stm32f10x_exti.h>
#include <misc.h>

#include <uart.h>
#include <nRF24L01.h>
#include <delay.h>


#define RX_PAYLOAD           18   // nRF24L01 TX payload length


uint8_t buf[RX_PAYLOAD];
uint8_t have_data;


static uint8_t CRC7_one(uint8_t t, uint8_t data) {
	const uint8_t g = 0x89;
	uint8_t i;

	t ^= data;
	for (i = 0; i < 8; i++) {
		if (t & 0x80) t ^= g;
		t <<= 1;
	}

	return t;
}

uint8_t CRC7_buf(const uint8_t * p, uint8_t len) {
	uint8_t j,crc = 0;

	for (j = 0; j < len; j++) crc = CRC7_one(crc,p[j]);

	return crc >> 1;
}

void EXTI15_10_IRQHandler(void) {
	uint8_t i;
	uint8_t status;

	UART_SendStr("---IRQ-->>>\n");
	if (EXTI_GetITStatus(EXTI_Line10) != RESET) {
		for (i = 0; i < RX_PAYLOAD; i++) buf[i] = 0x00;
		UART_SendStr("RXPacket: ");
		status = nRF24_RXPacket(buf,RX_PAYLOAD);
		nRF24_ClearIRQFlags();
		UART_SendHex8(status);

		if (status == 0x0E) {
			UART_SendStr(" => FIFO Empty (fake alarm)\n");
		} else {
			have_data = 1;
			UART_SendChar('\n');
		}

		GPIOC->ODR ^= GPIO_Pin_9; // Toggle LED

		nRF24_RXMode(RX_PAYLOAD);
		nRF24_ClearIRQFlags();

		EXTI_ClearITPendingBit(EXTI_Line10);
	}

	UART_SendStr("---IRQ--<<<\n");
}


int main(void) {
	// Init LED pins
	GPIO_InitTypeDef PORT;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	PORT.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_8;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&PORT);

	UART_Init(115200);
	UART_SendStr("\nSTM32F103RET6 is online.\n");

	nRF24_init();

	if (nRF24_Check() != 0) {
		UART_SendStr("Got wrong answer from SPI device.\n");
		UART_SendStr("MCU is now halt.\n");
		while(1);
	}

	unsigned char i,v;

	for (i = 0; i < 0x1D; i++) {
		UART_SendHex8(i);
		UART_SendStr("=");
		v = nRF24_ReadReg(i);
		UART_SendHex8(v);
		UART_SendStr("  ");
	}
	UART_SendChar('\n');

	nRF24_RXMode(RX_PAYLOAD);
	nRF24_ClearIRQFlags();

	// Enable Alternative function (for EXTI)
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

	// EXTI pin
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB,GPIO_PinSource10);

	// Configure EXTI line1
	EXTI_InitTypeDef EXTIInit;
	EXTIInit.EXTI_Line = EXTI_Line10;             // EXTI will be on line 10
	EXTIInit.EXTI_LineCmd = ENABLE;               // EXTI1 enabled
	EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;     // Generate IRQ
	EXTIInit.EXTI_Trigger = EXTI_Trigger_Falling; // IRQ on signal falling
	EXTI_Init(&EXTIInit);

	// Configure EXTI1 interrupt
	NVIC_InitTypeDef NVICInit;
	NVICInit.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 0x0F;
	NVICInit.NVIC_IRQChannelSubPriority = 0x0F;
	NVIC_Init(&NVICInit);

	int16_t temp;
	uint8_t CRC7;
	uint16_t vrefint;
	uint16_t LSI_freq;
	uint8_t TR1, TR2, TR3;
	uint8_t DR1, DR2, DR3;
	uint8_t hours, minutes, seconds;
	uint8_t day, month, year;

	have_data = 0;
	while(1) {
		while(!have_data);

		have_data = 0;
		CRC7 = CRC7_buf(&buf[0],RX_PAYLOAD-1);

		UART_SendStr("RX=["); UART_SendBufHex((char*)&buf[0],RX_PAYLOAD);
		UART_SendStr("] CRC=("); UART_SendInt(buf[RX_PAYLOAD-1]);
		UART_SendStr("|"); UART_SendInt(CRC7);
		UART_SendStr((buf[RX_PAYLOAD-1] == CRC7) ? ") => ok\n" : ") => bad\n");
		UART_SendStr("Temperature: ");
		temp = (buf[0] << 8) | buf[1];
		UART_SendInt(temp / 10); UART_SendChar('.');
		temp %= 10;
		if (temp < 0) temp *= -1;
		UART_SendInt(temp % 10); UART_SendStr("C\n");

		UART_SendStr("Packet: #");
		UART_SendInt((uint32_t)((buf[2] << 24)|(buf[3] << 16)|(buf[4] << 8)|(buf[5])));
		UART_SendChar('\n');

		vrefint = (buf[6] << 8) + buf[7];
		UART_SendStr("Vcc: ");
		UART_SendInt(vrefint / 100);
		UART_SendChar('.');
		UART_SendInt0(vrefint % 100);
		UART_SendStr("V\n");

		LSI_freq = (buf[14] << 8) + buf[15];
		UART_SendStr("LSI: ");
		UART_SendInt(LSI_freq);
		UART_SendStr("Hz\n");

		UART_SendStr("OBSERVE_TX:\n  ");
		UART_SendHex8(buf[16] >> 4); UART_SendStr(" pckts lost\n  ");
		UART_SendHex8(buf[16] & 0x0F); UART_SendStr(" retries\n");

		TR1 = buf[8];
		TR2 = buf[9];
		TR3 = buf[10];
		DR1 = buf[11];
		DR2 = buf[12];
		DR3 = buf[13];
		seconds = ((TR1 >> 4) * 10) + (TR1 & 0x0F);
		minutes = ((TR2 >> 4) * 10) + (TR2 & 0x0F);
		hours   = (((TR3 & 0x30) >> 4) * 10) + (TR3 & 0x0F);
	    day   = ((DR1 >> 4) * 10) + (DR1 & 0x0F);
	    //dow   = DR2 >> 5;
	    month = (((DR2 & 0x1F) >> 4) * 10) + (DR2 & 0x0F);
	    year  = ((DR3 >> 4) * 10) + (DR3 & 0x0F);

		UART_SendStr("Uptime: ");
        UART_SendInt0(hours); UART_SendChar(':');
        UART_SendInt0(minutes); UART_SendChar(':');
        UART_SendInt0(seconds); UART_SendChar(' ');
        UART_SendInt0(day); UART_SendChar('.');
        UART_SendInt0(month); UART_SendStr(".20");
        UART_SendInt0(year); UART_SendChar('\n');
	}
}
