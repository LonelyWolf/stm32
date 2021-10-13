#include "main.h"


// Configure system clock
// HSE -> PLL -> SYSCLK
static void SetSysClock(void) {
	RCC_PLLInitTypeDef pll;
	RCC_CLKInitTypeDef clk;

	// AHB, APB1 and APB2 dividers
	clk.AHBdiv  = RCC_AHB_DIV1;
	clk.APB1div = RCC_APB1_DIV1;
	clk.APB2div = RCC_APB2_DIV1;

	// Main PLL configuration
	// Input clock Fin = 16MHz
	// Fvco   = (Fin * PLLN) / PLLM = (16 * 20) / 2 = 160MHz
	// Fpll_p = Fvco / PLLP = 160 / 7 = 22.857143MHz (unused)
	// Fpll_q = Fvco / PLLQ = 160 / 8 = 20MHz (unused)
	// Fpll_r = Fvco / PLLR = 160 / 2 = 80MHz (input for SYSCLK)
	pll.PLLN = 20U;
	pll.PLLR = RCC_PLLR_DIV2;
	pll.PLLQ = RCC_PLLQ_DIV8;
	pll.PLLP = RCC_PLLP_DIV7;

	// Configure the PLLs input clock divider (common for all PLLs)
	RCC_PLLMConfig(RCC_PLLM_DIV2);

#if (DEVICE_HSE_PRESENT)
	// Device with HSE

	// Try to start HSE clock
	if (RCC_HSEConfig(RCC_HSE_ON) == ERROR) {
		// The HSE did not started, disable it
		RCC_HSEConfig(RCC_HSE_OFF);

		// Configure MSI speed to 16MHz
		if (RCC_MSIConfig(RCC_MSI_16M) == ERROR) {
			// Something really bad had happened, the MSI did not start
			PWR_EnterSHUTDOWNMode();
		}

		// Set MSI as clock source for PLLs
		RCC_PLLSrcConfig(RCC_PLLSRC_MSI);

		// Configure AHB/APB dividers, configure main PLL and
		// try to switch main system clock source to PLL
		RCC_SetClockPLL(RCC_PLLSRC_MSI, &pll, &clk);
	} else {
		// Set HSE as clock source for PLLs
		RCC_PLLSrcConfig(RCC_PLLSRC_HSE);

		// Configure AHB/APB dividers, configure main PLL and
		// try to switch main system clock source to PLL
		if (RCC_SetClockPLL(RCC_PLLSRC_HSE, &pll, &clk) == SUCCESS) {
			// Since main clock successfully switched to PLL feed by HSE,
			// the MSI clock can be disabled
			RCC_MSIConfig(RCC_MSI_OFF);
		}
	}
#else // DEVICE_HSE_PRESENT
	// Device without HSE (e.g. Nucleo)

	// Configure MSI speed to 16MHz
	if (RCC_MSIConfig(RCC_MSI_16M) == ERROR) {
		// Something really bad had happened, the MSI did not start
		PWR_EnterSHUTDOWNMode();
	}

	// Set MSI as clock source for PLLs
	RCC_PLLSrcConfig(RCC_PLLSRC_MSI);

	// Configure AHB/APB dividers, configure main PLL and
	// try to switch main system clock source to PLL
	RCC_SetClockPLL(RCC_PLLSRC_MSI, &pll, &clk);
#endif // DEVICE_HSE_PRESENT
}


int main(void) {
	// Initialize the MCU clock system
	SetSysClock();
	SystemCoreClockUpdate();

	// Prefetch is useful if at least one wait state is needed to access the Flash memory
	if (RCC_GetLatency() != FLASH_ACR_LATENCY_0WS) {
		// Enable prefetch buffer (a.k.a ART)
		RCC_PrefetchEnable();
	}

	// Enable I-Cache and D-Cache (enabled by default)
	RCC_ICacheEnable();
	RCC_DCacheEnable();


	// Initialize debug output port (USART1)
	//   clock source: SYSCLK
	//   mode: transmit only
	USART1_HandleInit();
	RCC_SetClockUSART(RCC_USART2_CLK_SRC, RCC_PERIPH_CLK_SYSCLK);
	USART_Init(&hUSART1, USART_MODE_TX);

	// Configure USART:
	//   oversampling by 16
	//   115200, 8-N-1
	//   no hardware flow control
	USART_SetOversampling(&hUSART1, USART_OVERS16);
	USART_SetBaudRate(&hUSART1, 3000000U);
	USART_SetDataMode(&hUSART1, USART_DATAWIDTH_8B, USART_PARITY_NONE, USART_STOPBITS_1);
	USART_SetHWFlow(&hUSART1, USART_HWCTL_NONE);
	USART_Enable(&hUSART1);
	USART_CheckIdleState(&hUSART1, 0xC5C10); // Timeout of about 100ms at 80MHz CPU

	// I2C initialization
	RCC_SetClockI2C(RCC_I2C3_CLK_SRC, RCC_PERIPH_CLK_SYSCLK);
	I2C_Init(TMP116_I2C_PORT);
	I2C_ConfigFilters(TMP116_I2C_PORT, I2C_AF_ENABLE, 0);
	// I2C timings:
	//   - 80MHz source clock (I2CCLK)
	//   - 400kHz fast mode
	//   - 100ns rise time, 10ns fall time
	I2C_ConfigTiming(TMP116_I2C_PORT, 0x00F02B86);
	I2C_Enable(TMP116_I2C_PORT);

	// Initialize delay functions
	Delay_Init();


	// Check if the sensor is present
	printf("TMP116: ");
	if (I2C_IsDeviceReady(TMP116_I2C_PORT, TMP116_ADDR, 1) != I2C_SUCCESS) {
		printf("FAIL\r\n");
		while (1);
	}


#if (TMP116_RESET)
	// Do a chip software reset
	// NOTE: will reset all devices on the bus that support the general-call
	printf("RESET: ");
	TMP116_SoftReset();
#else
	printf("READY: ");
#endif // TMP116_RESET

	// Wait for the TMP116 to complete the reset sequence by monitoring EEPROM busy bit
	uint32_t i = 0xFFF;
	while (TMP116_IsEEPROMBusy() && --i) {
		printf(".");
	}
	if (i == 0) {
		// Some banana happens...
		printf(" TIMEOUT\r\n");
		while (1);
	} else {
		printf(" OK\r\n");
	}


	// Configure the sensor
	TMP116_Init();
	TMP116_SetModeTA(TMP116_BIT_RESET); // alert mode
	TMP116_SetAlertPin(TMP116_BIT_SET); // ALERT pin indicates temperature data is ready
	TMP116_SetAlertPolarity(TMP116_BIT_SET); // ALERT pin active HIGH
	TMP116_SetLimitHigh(3000); // alert high limit +30.0C
	TMP116_SetLimitLow(-2500); // alert low limit -25.0C
#if 0
	// Each parameter separately
	TMP116_SetConv(TMP116_CONV_4);
	TMP116_SetAvg(TMP116_AVG_8);
	TMP116_SetMode(TMP116_MODE_SD);
#else
	// All three parameters at once
	TMP116_SetConfig(TMP116_MODE_SD, TMP116_AVG_8, TMP116_CONV_4);
#endif


#if 0

	// Configure factory default values
	TMP116_SetConfig(TMP116_MODE_CC, TMP116_AVG_8, TMP116_CONV_4);
	TMP116_SetModeTA(TMP116_BIT_RESET);
	TMP116_SetAlertPin(TMP116_BIT_RESET);
	TMP116_SetAlertPolarity(TMP116_BIT_RESET);
	TMP116_SetLimitHigh(19200);
	TMP116_SetLimitLow(-25600);

	// Unlock EEPROM for programming
	TMP116_EEPROMUnlock();

	// Store current config and limits into EEPROM
	// Polling busy bit after each write operation
	TMP116_WriteConfig();
	while (TMP116_IsEEPROMBusy());
	TMP116_WriteLimitHigh();
	while (TMP116_IsEEPROMBusy());
	TMP116_WriteLimitLow();
	while (TMP116_IsEEPROMBusy());

	// Write dummy values to each EEPROM cell,
	// wiping the unique NIST traceability ID =)
	TMP116_EEPROMWrite(TMP116_EEPROM_1, 0x0123);
	while (TMP116_IsEEPROMBusy());
	TMP116_EEPROMWrite(TMP116_EEPROM_2, 0x4567);
	while (TMP116_IsEEPROMBusy());
	TMP116_EEPROMWrite(TMP116_EEPROM_3, 0x89AB);
	while (TMP116_IsEEPROMBusy());
	TMP116_EEPROMWrite(TMP116_EEPROM_4, 0xCDEF);
	while (TMP116_IsEEPROMBusy());

	// Lock the EEPROM for programming
	TMP116_EEPROMLock();

#endif


	char sign, sign_h, sign_l;
	int32_t temp, lim_h, lim_l;


	// Dump contents of the chip EEPROM
	printf("EEPROM: ");
	for (temp = TMP116_EEPROM_1; temp <= TMP116_EEPROM_4; temp++) {
		printf("0x%04X ", TMP116_EEPROMRead(temp));
	}
	printf("\r\n");


	// Read sensor high limit
	lim_h = TMP116_GetLimitHigh();
	if (lim_h < 0) {
		lim_h = 0 - lim_h;
		sign_h = '-';
	} else {
		sign_h = lim_h == 0 ? ' ' : '+';
	}

	// Read sensor low limit
	lim_l = TMP116_GetLimitLow();
	if (lim_l < 0) {
		lim_l = 0 - lim_l;
		sign_l = '-';
	} else {
		sign_l = lim_l == 0 ? ' ' : '+';
	}

	// Print configured limits
	printf("Limits: %c%i.%02iC..%c%i.%02iC\r\n",
			sign_l,
			lim_l / 100,
			lim_l % 100,
			sign_h,
			lim_h / 100,
			lim_h % 100
		);


	// The main loop
	while (1) {
		// Starts temperature measurements
		TMP116_SetMode(TMP116_MODE_OS);

		// NOTE: constant polling of the sensor is a bad idea (it causes unnecessary self-heating),
		// for the one-shot mode it is better to implement a delay for the necessary amount of time

		// Moreover, constant repeated reading of the CONFIGURATION register without delays between
		// attempts can cause a situation that the Data_Ready bit will be reset and the user software
		// will not catch the moment when temperature readings will be ready
		// The ALERT pin should be used if polling behavior is necessary

#if 0

		// Polls the chip waiting till temperature data is ready
		do {
			TMP116_GetStatus();
			// There should be some delay here (see NOTE above)
		} while (!TMP116_IsDataReady());

#else

		// Delay long enough for the sensor to complete the temperature conversion
		Delay_ms(150); // 125ms at AVG set to 8
		TMP116_GetStatus();

#endif

		// Reads and prints temperature
		temp = TMP116_GetTemp();

		// Puts sensor into shutdown mode
		TMP116_SetMode(TMP116_MODE_SD);

		// Prints out limit flags and temperature
		if (temp < 0) {
			temp = 0 - temp;
			sign = '-';
		} else {
			sign = temp == 0 ? ' ' : '+';
		}
		printf("A.Lo:%c A.Hi:%c Temp:%c%i.%02iC\r\n",
				TMP116_IsAlertLow() ? '+' : '-',
				TMP116_IsAlertHigh() ? '+' : '-',
				sign,
				temp / 100,
				temp % 100
			);

		// Sleeping to avoid taking measurements too often
		Delay_ms(900);
	}
}
