#include "main.h"


// --- Clock-related functions ---

// Configure system clock
// HSE -> PLL -> SYSCLK
void SetSysClock(void) {
	RCC_PLLInitTypeDef pll;
	RCC_CLKInitTypeDef clk;
	ErrorStatus result;

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
	pll.PLLN = 20;
	pll.PLLR = RCC_PLLR_DIV2;
	pll.PLLQ = RCC_PLLQ_DIV8;
	pll.PLLP = RCC_PLLP_DIV7;

	// Configure the PLLs input clock divider (common for all PLLs)
	RCC_PLLMConfig(RCC_PLLM_DIV2);

	// Try to start HSE clock
	if (RCC_HSEConfig(RCC_HSE_ON) == ERROR) {
		// The HSE did not started, disable it
		RCC_HSEConfig(RCC_HSE_OFF);

		// Configure MSI speed to 16MHz
		if (RCC_MSIConfig(RCC_MSI_16M) == ERROR) {
			// Something really bad had happened, the MSI did not start
			while(1);
		}

		// Set MSI as clock source for PLLs
		RCC_PLLSrcConfig(RCC_PLLSRC_MSI);

		// Configure AHB/APB dividers, configure main PLL and
		// try to switch main system clock source to PLL
		result = RCC_SetClockPLL(RCC_PLLSRC_MSI, &pll, &clk);
	} else {
		// Set HSE as clock source for PLLs
		RCC_PLLSrcConfig(RCC_PLLSRC_HSE);

		// Configure AHB/APB dividers, configure main PLL and
		// try to switch main system clock source to PLL
		result = RCC_SetClockPLL(RCC_PLLSRC_HSE, &pll, &clk);

		if (result == SUCCESS) {
			// Since main clock successfully switched to PLL feed by HSE,
			// the MSI clock can be disabled
			RCC_MSIConfig(RCC_MSI_OFF);
		}
	}

	//
	if (result != SUCCESS) {
		while (1) {
			asm volatile ("nop");
		}
	}
}


// ------------------------------


int main(void) {
	// Initialize the MCU clock system
	SystemInit();
	SetSysClock();
	SystemCoreClockUpdate();


	// Initialize delay functions
	Delay_Init();


	// Initialize debug output port (USART2)
	//   clock source: SYSCLK
	//   mode: transmit only
	USART2_HandleInit();
	RCC_SetClockUSART(RCC_USART2_CLK_SRC, RCC_PERIPH_CLK_SYSCLK);
	USART_Init(&hUSART2, USART_MODE_TX);

	// Configure USART2:
	//   oversampling by 16
	//   256000, 8-N-1
	//   no hardware flow control
	USART_SetOversampling(&hUSART2, USART_OVERS16);
	USART_SetBaudRate(&hUSART2, 256000);
	USART_SetDataMode(&hUSART2, USART_DATAWIDTH_8B, USART_PARITY_NONE, USART_STOPBITS_1);
	USART_SetHWFlow(&hUSART2, USART_HWCTL_NONE);
	USART_Enable(&hUSART2);
	// FIXME: MAIN: need to poll TEACK bit in ISR register after enabling the USART


	// Say "hello world"
	RCC_ClocksTypeDef Clocks;
	RCC_GetClocksFreq(&Clocks);
	printf("\r\n---STM32L476RG---\r\n");
	printf("MAX44009 (%s @ %s)\r\n", __DATE__, __TIME__);
	printf("CPU: %.3uMHz\r\n", SystemCoreClock / 1000);
	printf("SYSCLK=%.3uMHz, HCLK=%.3uMHz\r\n", Clocks.SYSCLK_Frequency / 1000, Clocks.HCLK_Frequency / 1000);
	printf("APB1=%.3uMHz, APB2=%.3uMHz\r\n", Clocks.PCLK1_Frequency / 1000, Clocks.PCLK2_Frequency / 1000);
	printf("System clock: %s\r\n", _sysclk_src_str[RCC_GetSysClockSource()]);


	// I2C initialization
	printf("I2C: ");

	// I2C clock source: SYSCLK
	RCC_SetClockI2C(RCC_I2C1_CLK_SRC, RCC_PERIPH_CLK_SYSCLK);
	// Initialize the I2C peripheral and its GPIO
	I2C_Init(I2C1);
	// Noise filters: analog filter is enabled, digital is disabled
	I2C_ConfigFilters(I2C1, I2C_AF_ENABLE, 0);
	// I2C timings:
	//   - 80MHz source clock (I2CCLK)
	//   - 400kHz fast mode
	//   - 100ns rise time, 10ns fall time
	I2C_ConfigTiming(I2C1, 0x00F02B86);
	// Enable I2C peripheral
	I2C_Enable(I2C1);

	printf("OK\r\n");


	// Check if MAX44009 is present
	printf("MAX44009: ");
	if (I2C_IsDeviceReady(MAX44009_I2C_PORT, MAX44009_ADDR, 1) != I2C_SUCCESS) {
		printf("FAIL\r\n");
		while (1);
	}
	printf("OK\r\n");


	// Initialize the IC to its default state
	MAX44009_Init();


	// Get LUX readings and print to UART every second
	while(1) {
		als_lux = MAX44009_GetLux();
		if (als_lux != MAX44009_OVERRANGE) {
			printf("Lux=%.3u\r\n", als_lux);
		} else {
			printf("Lux=OVERRANGE\r\n");
		}
		Delay_ms(1000);
	}
}
