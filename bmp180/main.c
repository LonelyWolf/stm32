#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_i2c.h>
#include <delay.h>
#include <uart.h>
#include <bmp180.h>


int main(void)
{
	UART_Init();
	UART_SendStr("\nSTM32F103RET6 is online.\n");

	UART_SendStr("I2C init ... ");
	if (!BMP180_Init(400000)) UART_SendStr("ready.\n"); else {
		UART_SendStr("fail.\n");
		UART_SendStr("MCU halted now.\n");
		while(1);
	}

	UART_SendStr("ChipID: ");
	uint8_t ChipID = BMP180_ReadReg(BMP180_CHIP_ID_REG);
	UART_SendHex8(ChipID); UART_SendChar('\n');

	UART_SendStr("Version: ");
	uint8_t Version = BMP180_ReadReg(BMP180_VERSION_REG);
	UART_SendHex8(Version); UART_SendChar('\n');

	BMP180_ReadCalibration();

/*
	BMP180_Calibration.AC1 = 408;
	BMP180_Calibration.AC2 = -72;
	BMP180_Calibration.AC3 = -14383;
	BMP180_Calibration.AC4 = 32741;
	BMP180_Calibration.AC5 = 32757;
	BMP180_Calibration.AC6 = 23153;
	BMP180_Calibration.B1  = 6190;
	BMP180_Calibration.B2  = 4;
	BMP180_Calibration.MB  = -32767;
	BMP180_Calibration.MC  = -8711;
	BMP180_Calibration.MD  = 2868;
*/

	UART_SendStr("E2PROM Calibration values:\n");
	UART_SendStr("  AC1 = "); UART_SendHex16(BMP180_Calibration.AC1); UART_SendChar(' ');
	UART_SendStr("  AC2 = "); UART_SendHex16(BMP180_Calibration.AC2); UART_SendChar(' ');
	UART_SendStr("  AC3 = "); UART_SendHex16(BMP180_Calibration.AC3); UART_SendChar(' ');
	UART_SendStr("  AC4 = "); UART_SendHex16(BMP180_Calibration.AC4); UART_SendChar('\n');
	UART_SendStr("  AC5 = "); UART_SendHex16(BMP180_Calibration.AC5); UART_SendChar(' ');
	UART_SendStr("  AC6 = "); UART_SendHex16(BMP180_Calibration.AC6); UART_SendChar(' ');
	UART_SendStr("  B1  = "); UART_SendHex16(BMP180_Calibration.B1);  UART_SendChar(' ');
	UART_SendStr("  B2  = "); UART_SendHex16(BMP180_Calibration.B2);  UART_SendChar('\n');
	UART_SendStr("  MB  = "); UART_SendHex16(BMP180_Calibration.MB);  UART_SendChar(' ');
	UART_SendStr("  MC  = "); UART_SendHex16(BMP180_Calibration.MC);  UART_SendChar(' ');
	UART_SendStr("  MD  = "); UART_SendHex16(BMP180_Calibration.MD);  UART_SendChar('\n');

	uint32_t u_temp,u_pres;
	int32_t rt,rp;

	u_temp = BMP180_Read_UT();
	//u_temp = 27898;
	rt = BMP180_Calc_RT(u_temp);

	u_pres = BMP180_Read_PT(0);
	//u_pres = 23843;
	rp = BMP180_Calc_RP(u_pres,0);

	UART_SendStr("Uncompensated temperature = "); UART_SendInt(u_temp); UART_SendChar('\n');
	UART_SendStr("Uncompensated pressure = "); UART_SendInt(u_pres); UART_SendChar('\n');

	UART_SendStr("Real temperature = ");
	UART_SendInt(rt); UART_SendStr(" -> ");
	UART_SendInt(rt / 10); UART_SendChar('.');
	UART_SendInt(rt % 10); UART_SendStr("C\n");

	UART_SendStr("Real pressure = ");
	UART_SendInt(rp); UART_SendStr(" -> ");
	UART_SendInt(rp / 1000); UART_SendChar('.');
	UART_SendInt(rp % 1000); UART_SendStr("kPa (");
	UART_SendInt(BMP180_kpa_to_mmhg(rp)); UART_SendStr("mmHg)\n");

	UART_SendStr("---------------------------\n");

    while(1);
}
