#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <sdcard.h>
#include <uart.h>


int main(void)
{
	UART_Init();

	UART_SendStr("\nSTM32F103RET6 is online.\n");

	SD_Init();

	uint8_t b = 0;
	UART_SendStr("SD_CardInit: ");
	b = SD_CardInit();
	UART_SendHex8(b);
	UART_SendStr("\nCard version: ");
	UART_SendHex8(SD_CardType);

	uint8_t response = 0;
	UART_SendStr("\nCSD: ");
	response = SD_Read_CSD();
	if (response != 0x00) {
		UART_SendStr("error = ");
		UART_SendHex8(response);
	} else UART_SendBufHex((char*)&SD_CSD[0],16);
	UART_SendStr("\nCID: ");
	response = SD_Read_CID();
	if (response != 0x00) {
		UART_SendStr("error = ");
		UART_SendHex8(response);
	} else UART_SendBufHex((char*)&SD_CID[0],16);
	UART_SendStr("\nMax bus clk.: ");
	UART_SendHex32(SD_MaxBusClkFreq);
	UART_SendStr("\nDevice size: ");
	UART_SendInt(SD_CardCapacity >> 10);
	UART_SendStr("Mb\n");

	response = SD_Read_Block(0);
	if (response != 0x00) {
		UART_SendStr("error = ");
		UART_SendHex8(response);
	} else {
		UART_SendBufHexFancy((char*)&SD_sector[0],512,32,'.');
		UART_SendStr("CRC16: ");
		UART_SendHex16(SD_CRC16_rcv);
		UART_SendStr(" == ");
		UART_SendHex16(SD_CRC16_cmp);
	}
	UART_SendChar('\n');

	response = SD_Read_Block(1);
	if (response != 0x00) {
		UART_SendStr("error = ");
		UART_SendHex8(response);
	} else {
		UART_SendBufHexFancy((char*)&SD_sector[0],512,32,'.');
		UART_SendStr("CRC16: ");
		UART_SendHex16(SD_CRC16_rcv);
		UART_SendStr(" == ");
		UART_SendHex16(SD_CRC16_cmp);
	}
	UART_SendChar('\n');

	while(1);
}
