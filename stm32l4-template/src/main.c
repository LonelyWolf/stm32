#include "main.h"


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
			// The MSI has not started, this is really odd
			// Put the MCU to SHUTDOWN mode
			PWR_EnterSHUTDOWNMode();
		}

		// Set MSI as clock source for PLLs
		RCC_PLLSrcConfig(RCC_PLLSRC_MSI);

		// Configure AHB/APB dividers, configure main PLL and
		// try to switch main system clock source to PLL
		result = RCC_SetClockPLL(RCC_PLLSRC_MSI,&pll,&clk);
	} else {
		// Set HSE as clock source for PLLs
		RCC_PLLSrcConfig(RCC_PLLSRC_HSE);

		// Configure AHB/APB dividers, configure main PLL and
		// try to switch main system clock source to PLL
		result = RCC_SetClockPLL(RCC_PLLSRC_HSE,&pll,&clk);

		if (result == SUCCESS) {
			// Since main clock successfully switched to PLL feed by HSE,
			// the MSI clock can be disabled
			RCC_MSIConfig(RCC_MSI_OFF);
		}
	}

	// The PLL has not started, put the MCU to SHUTDOWN mode
	if (result != SUCCESS) {
		PWR_EnterSHUTDOWNMode();
	}
}

// Configure CLK48 clock
// note: PLL input clock must be already configured
void Clock_Clk48Config(void) {
	RCC_PLLInitTypeDef pll;

	// Input clock HSE = 16MHz
	// Fvco   = (HSE * PLLN) / PLLM = (16 * 12) / 2 = 96MHz
	// Fpll_p = Fvco / PLLP = 96 / 7 = 13.714286MHz (unused)
	// Fpll_q = Fvco / PLLQ = 96 / 2 = 48MHz (PLL48M2CLK, clock for USB, RNG and SDMMC)
	// Fpll_r = Fvco / PLLR = 96 / 2 = 48MHz (unused)

	pll.PLLN = 12;
	pll.PLLR = RCC_PLLSAI1R_DIV2;
	pll.PLLQ = RCC_PLLSAI1Q_DIV2;
	pll.PLLP = RCC_PLLSAI1P_DIV7;

	// Configure PLLSAI1
	RCC_PLLConfig(RCC_PLL_SAI1,&pll);

	// Enable PLLSAI1 PLLQ output
	RCC_PLLOutEnable(RCC_PLL_SAI1,RCC_PLL_OUTQ);

	// Set PLLSAI1Q output as CLK48 clock
	RCC_SetClock48M(RCC_CLK48_CLK_PLLSAI1Q);
}

// Disable CLK48 clock
void Clock_Clk48Disable(void) {
	RCC_SetClock48M(RCC_CLK48_CLK_NONE);
	RCC_PLLOutDisable(RCC_PLL_SAI1,RCC_PLL_OUTQ);
	RCC_PLLDisable(RCC_PLL_SAI1);
}


int main(void) {
	// Initialize the MCU clock system
	SystemInit();
	SetSysClock();
	SystemCoreClockUpdate();


	// Initialize debug output port (USART2)
	//   clock source: SYSCLK
	//   mode: transmit only
	USART2_HandleInit();
	RCC_SetClockUSART(RCC_USART2_CLK_SRC,RCC_PERIPH_CLK_SYSCLK);
	USART_Init(&hUSART2,USART_MODE_TX);

	// Configure USART:
	//   oversampling by 16
	//   115200, 8-N-1
	//   no hardware flow control
	USART_SetOversampling(&hUSART2,USART_OVERS16);
	USART_SetBaudRate(&hUSART2,115200);
	USART_SetDataMode(&hUSART2,USART_DATAWIDTH_8B,USART_PARITY_NONE,USART_STOPBITS_1);
	USART_SetHWFlow(&hUSART2,USART_HWCTL_NONE);
	USART_Enable(&hUSART2);
	USART_CheckIdleState(&hUSART2,0xC5C10); // Timeout of about 100ms at 80MHz CPU


	// Say "hello world"
	RCC_ClocksTypeDef Clocks;
	RCC_GetClocksFreq(&Clocks);
	printf("\r\n---STM32L476RG---\r\n");
	printf("Template (%s @ %s)\r\n",__DATE__,__TIME__);
	printf("CPU: %.3uMHz\r\n",SystemCoreClock / 1000);
	printf("SYSCLK=%.3uMHz, HCLK=%.3uMHz\r\n",Clocks.SYSCLK_Frequency / 1000,Clocks.HCLK_Frequency / 1000);
	printf("APB1=%.3uMHz, APB2=%.3uMHz\r\n",Clocks.PCLK1_Frequency / 1000,Clocks.PCLK2_Frequency / 1000);
	printf("System clock: %s\r\n",_sysclk_src_str[RCC_GetSysClockSource()]);


	// Initialize delay functions
	Delay_Init();


	// Initialize the PA5 pin (LED on the Nucleo board)
	// Enable the GPIOA peripheral
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	// Configure PA5 as push-pull output without pull-up, at lowest speed
	GPIO_set_mode(GPIOA,GPIO_Mode_OUT,GPIO_PUPD_NONE,GPIO_PIN_5);
	GPIO_out_cfg(GPIOA,GPIO_OT_PP,GPIO_SPD_LOW,GPIO_PIN_5);


	// The main loop
	while (1) {
		// Invert the PA5 pin state every half of second
		Delay_ms(500);
		GPIO_PIN_INVERT(GPIOA,GPIO_PIN_5);
	}
}
