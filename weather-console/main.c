#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_i2c.h>
#include <math.h>

#include <delay.h>
#include <uart.h>
#include <bmp180.h>
#include <dht22.h>


// BMP180
uint32_t u_temp,u_pres;
int32_t rt,rp;

// DHT22
uint32_t response;
uint16_t humidity,temperature;


float calc_dewpoint(float hum, float temp) {
	float gamma, dew_point;

	gamma = (17.27 * temp) / (237.7 + temp) + log10f(hum / 100);
	dew_point = (237.7 * gamma) / (17.27 - gamma);

	return dew_point;
}


int main(void) {
	UART_Init();
	UART_SendStr("\nSTM32F103RET6 is online.\n");

	UART_SendStr("BMP180\n");

	if (BMP180_Init(400000)) {
		UART_SendStr("I2C communications failed. MCU halted.\n");
		while(1);
	}

	BMP180_ReadCalibration();

	u_temp = BMP180_Read_UT();
	rt = BMP180_Calc_RT(u_temp);

	u_pres = BMP180_Read_PT(0);
	rp = BMP180_Calc_RP(u_pres,0);

	UART_SendStr("Temperature = ");
	UART_SendInt(rt / 10); UART_SendChar('.');
	UART_SendInt(rt % 10); UART_SendStr("C\n");

	float p_mmHg_i, p_mmHg_f;

	p_mmHg_f = modff(rp * 0.0075006375541921, &p_mmHg_i);

	UART_SendStr("Pressure:\n");
	UART_SendStr("  measured = ");
	UART_SendInt(rp / 1000); UART_SendChar('.');
	UART_SendInt(rp % 1000); UART_SendStr("kPa (");
	UART_SendInt((uint32_t)p_mmHg_i);
	UART_SendChar('.');
	UART_SendInt((uint32_t)(p_mmHg_f * 1000));
	UART_SendStr("mmHg)\n");

	rp += 2559;
	p_mmHg_f = modff(rp * 0.0075006375541921, &p_mmHg_i);
	UART_SendStr("  at sea level = ");
	UART_SendInt(rp / 1000); UART_SendChar('.');
	UART_SendInt(rp % 1000); UART_SendStr("kPa (");
	UART_SendInt((uint32_t)p_mmHg_i);
	UART_SendChar('.');
	UART_SendInt((uint32_t)(p_mmHg_f * 1000));
	UART_SendStr("mmHg)\n");

	UART_SendStr("---------------------------\n");

	UART_SendStr("DHT22\n");

	DHT22_Init();
	response = DHT22_GetReadings();
	if (response != DHT22_RCV_OK) {
		UART_SendStr("DHT22_GetReadings() error = ");
		UART_SendInt(response);
		UART_SendStr("\nMCU halted.\n");
		while(1);
	}

	response = DHT22_DecodeReadings();
	if ((response & 0xff) != (response >> 8)) {
		UART_SendStr("Wrong data received.\n");
		while(1);
	}

	temperature = DHT22_GetTemperature();
	humidity = DHT22_GetHumidity();

	UART_SendStr("Humidity: ");
	UART_SendInt(humidity / 10); UART_SendChar('.');
	UART_SendInt(humidity % 10); UART_SendStr("%RH\n");
	UART_SendStr("Temperature: ");
	if ((temperature & 0x8000) != 0) UART_SendChar('-');
	UART_SendInt((temperature & 0x7fff) / 10); UART_SendChar('.');
	UART_SendInt((temperature & 0x7fff) % 10); UART_SendStr("C\n");
	if (!(temperature & 0x8000)) {
		UART_SendStr("Dew point: ");
		float h, t, dp;
		h = humidity / 10.0;
		t = (temperature & 0x7fff) / 10.0;
		dp = calc_dewpoint(h,t);

		float dp_f, dp_i;
		dp_f = modff(dp, &dp_i);
		UART_SendInt((uint32_t)dp_i);
		UART_SendChar('.');
		UART_SendInt((uint32_t)(dp_f * 10));
		UART_SendStr("C\n");
	}

	UART_SendStr("---------------------------\n");

	while(1);
}
