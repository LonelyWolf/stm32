// Common variables
static uint32_t i, j, k;

// RAW temperature and pressure values
int32_t UT, UP;

// Human-readable temperature and pressure
int32_t temperature;
uint32_t pressure;

#if (BMP280_FLOAT_FUNCTIONS)
// Temperature and pressure, float
float temperature_f;
float pressure_f;
#endif



// Rapidly blink Nucleo LED to indicate that BMP280 not found
void nucleo_blink_error(void) {
	while (1) {
		Delay_ms(100);
		GPIO_PIN_INVERT(GPIOA, GPIO_PIN_5);
	}
}


int main(void) {
	// Initialize the MCU clock system
	SetSysClock();
	SystemCoreClockUpdate();


	// Initialize debug output port (USART2)
	//   clock source: SYSCLK
	//   mode: transmit only
	USART2_HandleInit();
	RCC_SetClockUSART(RCC_USART2_CLK_SRC, RCC_PERIPH_CLK_SYSCLK);
	USART_Init(&hUSART2, USART_MODE_TX);

	// Configure USART:
	//   oversampling by 16
	//   115200, 8-N-1
	//   no hardware flow control
	USART_SetOversampling(&hUSART2, USART_OVERS16);
	USART_SetBaudRate(&hUSART2, 256000);
	USART_SetDataMode(&hUSART2, USART_DATAWIDTH_8B, USART_PARITY_NONE, USART_STOPBITS_1);
	USART_SetHWFlow(&hUSART2, USART_HWCTL_NONE);
	USART_Enable(&hUSART2);
	USART_CheckIdleState(&hUSART2, 0xC5C10); // Timeout of about 100ms at 80MHz CPU


	// Initialize delay functions
	Delay_Init();


	// Initialize the PA5 pin (LED on the Nucleo board)
	// Enable the GPIOA peripheral
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	// Configure PA5 as push-pull output without pull-up, at lowest speed
	GPIO_set_mode(GPIOA, GPIO_Mode_OUT, GPIO_PUPD_NONE, GPIO_PIN_5);
	GPIO_out_cfg(GPIOA, GPIO_OT_PP, GPIO_SPD_LOW, GPIO_PIN_5);


	// I2C clock source: SYSCLK
	RCC_SetClockI2C(RCC_I2C2_CLK_SRC, RCC_PERIPH_CLK_SYSCLK);
	// Initialize the I2C peripheral and its GPIO
	I2C_Init(I2C2);
	// Noise filters: analog filter is enabled, digital is disabled
	I2C_ConfigFilters(I2C2, I2C_AF_ENABLE, 0);
	// I2C timings:
	//   - 80MHz source clock (I2CCLK)
	//   - 400kHz fast mode
	//   - 100ns rise time, 10ns fall time
	I2C_ConfigTiming(I2C2, 0x00F02B86);
	// Enable I2C peripheral
	I2C_Enable(I2C2);


#if 0 // I2C bus scan
	// Scan I2C bus for all possible 7-bit addresses
	printf("I2C2 bus scan:\r\n   ");
	for (i = 0; i < 0x10; i++) printf("  %X", i);
	printf("\r\n00:         ");
	for (i = 3; i < 0x78; i++) {
		if (!(i & 0x0F)) printf("\r\n%02X:", i);
		if (I2C_IsDeviceReady(I2C2, i << 1, 5) == I2C_SUCCESS) {
			printf(" %02X", i);
		} else {
			printf(" --");
		}
	}
	printf("\r\n");
#endif // I2C bus scan


	// BMP280
	printf("BMP280: ");
	if (!BMP280_Check()) {
		printf("NONE\r\nHALTING\r\n");
		nucleo_blink_error();
	}
	// Version of the chip (BMP280 = 0x58)
	printf("ID=%02X\r\n", BMP280_GetVersion());

	// Chip working mode (must be SLEEP after reset)
	i = BMP280_GetMode();
	printf("Mode: [%02X] ", i);
	switch (i) {
		case BMP280_MODE_SLEEP:
			printf("SLEEP\r\n");
			break;
		case BMP280_MODE_FORCED:
		case BMP280_MODE_FORCED2:
			printf("FORCED\r\n");
			break;
		default:
			printf("NORMAL\r\n");
			break;
	}

	// Reset BMP280 chip
	printf("BMP280 chip reset: ");
	BMP280_Reset();

	// Chip startup time is 2ms (after either power up or soft reset),
	// the user software must wait until the chip become accessible
#if 0
	// So, at this point can be placed a delay of 2ms
	Delay_ms(3);
#else
	// In case when delay function is not available, can be used a trick...
	// After power-on/reset, chip copies the NVM data to registers and sets a bit in
	// the status register, thus the user software can poll that bit and
	// when it will be reset, the chip is ready to work
	i = 0xFFF;
	while ((BMP280_GetStatus() & BMP280_STATUS_IM_UPDATE) && --i) {
		USART_SendChar(USART2, '.');
	}
	if (i == 0) {
		// Some banana happens:
		// - no respond from chip after reset
		// - NVM bit is stuck for some reason
		printf(" TIMEOUT\r\n");
		nucleo_blink_error();
	}
#endif
	printf(" OK\r\n");

	// Read calibration values
	printf("Calibration: ");
	i = BMP280_Read_Calibration();
	printf("[%02X] %s\r\n", i, (i == BMP280_SUCCESS) ? "OK" : "FAIL");

#if 0
	// cal_param is the internal static struct of library
	printf("\tdig_T1:%u\tdig_T2:%i\tdig_T3:%i\r\n", cal_param.dig_T1, cal_param.dig_T2, cal_param.dig_T3);
	printf("\tdig_P1:%u\tdig_P2:%i\tdig_P3:%i\r\n", cal_param.dig_P1, cal_param.dig_P2, cal_param.dig_P3);
	printf("\tdig_P4:%i\tdig_P5:%i\tdig_P6:%i\r\n", cal_param.dig_P4, cal_param.dig_P5, cal_param.dig_P6);
	printf("\tdig_P7:%i\tdig_P8:%i\tdig_P9:%i\r\n", cal_param.dig_P7, cal_param.dig_P8, cal_param.dig_P9);
#endif

	// Set normal mode inactive duration (standby time)
	BMP280_SetStandby(BMP280_STBY_1s);

	// Set IIR filter constant
	BMP280_SetFilter(BMP280_FILTER_16);

	// Set oversampling for temperature
	BMP280_SetOSRST(BMP280_OSRS_T_x2);

	// Set oversampling for pressure
	BMP280_SetOSRSP(BMP280_OSRS_P_x16);

	// Set normal mode (perpetual periodic conversion)
	BMP280_SetMode(BMP280_MODE_NORMAL);

	// Chip working mode
	i = BMP280_GetMode();
	printf("Mode: [%02X] -> ", i);
	switch (i) {
		case BMP280_MODE_SLEEP:
			printf("SLEEP\r\n");
			break;
		case BMP280_MODE_FORCED:
		case BMP280_MODE_FORCED2:
			printf("FORCED\r\n");
			break;
		default:
			printf("NORMAL\r\n");
			break;
	}

	// The main loop
	while (1) {
		// Check status of chip
		i = BMP280_GetStatus();
		printf("Status: [%02X] %s %s\r\n",
				i,
				(i & BMP280_STATUS_MEASURING) ? "MEASURING" : "READY",
				(i & BMP280_STATUS_IM_UPDATE) ? "NVM_UPDATE" : "NVM_READY"
			);

		// Get raw readings from the chip
		i = BMP280_Read_UTP(&UT, &UP);
		printf("Raw: T=0x%05X P=0x%05X [R=%s]\r\n",
				UT,
				UP,
				i ? "OK" : "ERROR"
			);

		if (UT == 0x80000) {
			// Either temperature measurement is configured as 'skip' or first conversion is not completed yet
			printf("Temperature: no data\r\n");
			// There is no sense to calculate pressure without temperature readings
			printf("Pressure: no temperature readings\r\n");
		} else {
			// Temperature (must be calculated first)
			//  UT = 0x84D3C; // test raw value: 25.90C
			temperature = BMP280_CalcT(UT);
			printf("Temperature: %.2iC\r\n", temperature);

#if (BMP280_FLOAT_FUNCTIONS)
			// Same using floats
			temperature_f = BMP280_CalcTf(UT);
			printf("Temperature: %.2iC\r\n", (int32_t)(temperature_f * 100.0F));
#endif // BMP280_FLOAT_FUNCTIONS

			if (UP == 0x80000) {
				// Either pressure measurement is configured as 'skip' or first conversion is not completed yet
				printf("Pressure: no data\r\n");
			} else {
				// Pressure
				pressure = BMP280_CalcP(UP);
				printf("Pressure: %.3uPa [%.3uhPa]\r\n",
						pressure,
						pressure / 100
					);

				printf("mmHg: %.3u\r\n",
						BMP280_Pa_to_mmHg(pressure)
					);

#if (BMP280_FLOAT_FUNCTIONS)
				// Same using floats
				pressure_f = BMP280_CalcPf(UP);
				pressure = (uint32_t)(pressure_f * 1000.0F);
				printf("Pressure: %.3uPa [%.3uhPa]\r\n",
						pressure,
						pressure / 100
					);

				printf("mmHg: %.3u\r\n",
						(uint32_t)(BMP280_Pa_to_mmHgf(pressure_f) * 1000.0F)
					);
#endif // BMP280_FLOAT_FUNCTIONS

			}
		}

		printf("------------------------\r\n");

		// Invert state of the Nucleo LED
		GPIO_PIN_INVERT(GPIOA, GPIO_PIN_5);

		Delay_ms(1000);
	}
}
