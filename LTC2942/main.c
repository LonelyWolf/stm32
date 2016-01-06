#include "main.h"


int main(void) {
	// Initialize the MCU clock system
	SystemInit();
	SystemCoreClockUpdate();

	// Initialize debug output port (USART1)
	USART1_HandleInit();
	USART_Init(&hUSART1,USART_TX,2000000);
	printf("\r\n---STM32L151RDT6---\r\n");

	// I2C port initialization
	if (I2Cx_Init(I2C1,400000) != I2C_SUCCESS) {
		printf("I2C initialization failed\r\n");
		while(1);
	} else {
/*
		// Scan I2C bus for all possible 7-bit addresses
		printf("%sScan I2C bus:\r\n",sepstr);
		for (i = 15; i < 127; i++) {
			if (I2Cx_IsDeviceReady(I2C1,i << 1,10) == I2C_SUCCESS) printf("  respond: 0x%02X\r\n",i);
		}
*/
	}

	// Check if LTC2942 is responding
	printf("LTC2942 ping: ");
	if (I2Cx_IsDeviceReady(LTC2942_I2C_PORT,LTC2942_ADDR,10) == I2C_SUCCESS) {
		printf("OK\r\n");
	} else {
		printf("FAIL\r\n");
		while(1);
	}
	printf(sepstr);

	// Enable auto measurement of battery voltage and temperature
	LTC2942_SetADCMode(LTC2942_ADC_AUTO);

	// Enable analog section of the chip (in case if it disabled)
	LTC2942_SetAnalog(LTC2942_AN_ENABLED);

	// Set prescaler M value
	// M=16 for 600mAh battery, the 1LSB of AC value is 0,010625mAh
//	LTC2942_SetPrescaler(LTC2942_PSCM_16);
	// M=8 for 400mAh battery, the 1LSB of AC value is 0,0053125mAh
	LTC2942_SetPrescaler(LTC2942_PSCM_8);

	// Disable AL/CC pin
	LTC2942_SetALCCMode(LTC2942_ALCC_DISABLED);

	// Program accumulated charge value for fully charged battery
//	LTC2942_SetAC(0xFFFF);

	while(1) {
		// Battery voltage
		i = LTC2942_GetVoltage();

		// Chip temperature
		j = LTC2942_GetTemperature();

		// Accumulated charge
		k = LTC2942_GetAC();

		// Dump values to log
		printf("Vbat: %.3uV  Temp: %.2iC  Charge: %.2u%% ~%umAh [0x%04X %u]\r\n",
				i, /* Battery voltage */
				j, /* Chip temperature */
				(k * 10000) / 65535, /* Accumulated charge in percent */
				(k * 85 * 8) / 128000, /* Very rough current charge capacity (8 - prescaler M value) */
//				(k * 85 * 16) / 128000, /* Very rough current charge capacity (16 - prescaler M value) */
				k, /* Raw accumulated charge (HEX) */
				k  /* Raw accumulated charge (DEC) */
			);

		// Dummy delay
		for (i = 0x007FFFFF; i--; ) asm volatile ("nop");
	}
}
