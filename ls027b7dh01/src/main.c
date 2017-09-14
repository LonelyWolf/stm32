#include "main.h"


// Display connection
//  EXTCOMIN <- PB1  (PWM)
//  DISP     <- PB2  (Display On/Off)
//  SCS      <- PB14 (Chip select)
//  SI       <- PB15 (SPI2_MOSI)
//  SCLK     <- PB13 (SPI2_SCK)


static uint32_t i, j, k;


// Configure system clock
// HSE -> PLL -> SYSCLK
void SetSysClock(void) {
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
	RCC_PLLConfig(RCC_PLL_SAI1, &pll);

	// Enable PLLSAI1 PLLQ output
	RCC_PLLOutEnable(RCC_PLL_SAI1, RCC_PLL_OUTQ);

	// Set PLLSAI1Q output as CLK48 clock
	RCC_SetClock48M(RCC_CLK48_CLK_PLLSAI1Q);
}

// Disable CLK48 clock
void Clock_Clk48Disable(void) {
	RCC_SetClock48M(RCC_CLK48_CLK_NONE);
	RCC_PLLOutDisable(RCC_PLL_SAI1, RCC_PLL_OUTQ);
	RCC_PLLDisable(RCC_PLL_SAI1);
}

// Find SPI baud rate divider to achieve specified target frequency
// input:
//   SPIx - pointer to the SPI port handle
//   tgt_freq - target frequency
// return: SPI baud rate divider
// examples:
//   APB frequency = 80MHz, target = 16MHz => return SPI_BR_8   (F.spi = 10MHz)
//   APB frequency = 32MHz, target = 16MHz => return SPI_BR_2   (F.spi = 16MHz)
//   APB frequency = 40MHz, target = 2MHz  => return SPI_BR_32  (F.spi = 1.25MHz)
//   APB frequency = 80MHz, target = 10Hz  => return SPI_BR_256 (F.spi = 312.5KHz)
uint32_t Clock_SPICalcDivider(SPI_HandleTypeDef *SPI, uint32_t tgt_freq) {
	static const struct {
		uint32_t val;
		uint32_t br;
	} spi_br[8] = {
			{   2,   SPI_BR_2 },
			{   4,   SPI_BR_4 },
			{   8,   SPI_BR_8 },
			{  16,  SPI_BR_16 },
			{  32,  SPI_BR_32 },
			{  64,  SPI_BR_64 },
			{ 128, SPI_BR_128 },
			{ 256, SPI_BR_256 }
	};

	uint32_t src_freq;
	uint32_t res_freq;
	uint32_t idx = 0;

	src_freq = RCC_GetSYSCLKFreq();
	src_freq = RCC_GetHCLKFreq(src_freq);
	if (SPI->Instance == SPI1) {
		// SPI1 feed from APB2
		src_freq = RCC_GetPCLK2Freq(src_freq);
	} else {
		// SPI2 and SPI3 feed from APB1
		src_freq = RCC_GetPCLK1Freq(src_freq);
	}

	// Find SPI divider to achieve frequency value closest to specified target frequency
	do {
		res_freq = src_freq / spi_br[idx].val;
		if (res_freq <= tgt_freq) {
			break;
		}
		idx++;
	} while (idx < 8);
	if (idx > 7) {
		idx = 7;
	}

	return spi_br[idx].br;
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


	// Say "hello world"
	RCC_ClocksTypeDef Clocks;
	RCC_GetClocksFreq(&Clocks);
	printf("\r\n---STM32L476RG---\r\n");
	printf("LS027B4DH01 (%s @ %s)\r\n", __DATE__, __TIME__);
	printf("CPU: %.3uMHz\r\n", SystemCoreClock / 1000);
	printf("SYSCLK=%.3uMHz, HCLK=%.3uMHz\r\n", Clocks.SYSCLK_Frequency / 1000, Clocks.HCLK_Frequency / 1000);
	printf("APB1=%.3uMHz, APB2=%.3uMHz\r\n", Clocks.PCLK1_Frequency / 1000, Clocks.PCLK2_Frequency / 1000);
	printf("System clock: %s\r\n", _sysclk_src_str[RCC_GetSysClockSource()]);


	// Initialize delay functions
	Delay_Init();


	// Initialize the PA5 pin (LED on the Nucleo board)
	// Enable the GPIOA peripheral
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	// Configure PA5 as push-pull output without pull-up, at lowest speed
	GPIO_set_mode(GPIOA, GPIO_Mode_OUT, GPIO_PUPD_NONE, GPIO_PIN_5);
	GPIO_out_cfg(GPIOA, GPIO_OT_PP, GPIO_SPD_LOW, GPIO_PIN_5);


	// Configure PWM signal for display EXTCOMIN pin (PB1 -> TIM3_CH4)

	// Enable PWM GPIO peripheral
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;

	// PWM pin: alternate function output, no pull-up/down, lowest speed
	GPIO_set_mode(GPIOB, GPIO_Mode_AF, GPIO_PUPD_NONE, GPIO_PIN_1);
	GPIO_out_cfg(GPIOB, GPIO_OT_PP, GPIO_SPD_LOW, GPIO_PIN_1);
	GPIO_af_cfg(GPIOB, GPIO_PinSource1, GPIO_AF2);

	// Reset PWM TIM peripheral
	RCC->APB1RSTR1 |=  RCC_APB1RSTR1_TIM3RST;
	RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_TIM3RST;

	// Enable PWM TIM peripheral
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;

	// Configure PWM timer (TIM3_CH4, 1Hz, 50% PWM)
	TIM3->CR1   |= TIM_CR1_ARPE; // Auto-preload enable
	TIM3->CR1   |= TIM_CR1_URS; // Only counter overflow/underflow generates an update interrupt
	TIM3->CCMR2 |= TIM_CCMR2_OC4PE; // Output compare 4 preload enable
	TIM3->CCMR2 |= TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1; // PWM mode 1
	TIM3->PSC    = SystemCoreClock / 10000; // 1Hz
//	TIM3->PSC    = SystemCoreClock / 160000; // 16Hz
	TIM3->ARR    = 9999; // auto reload value
	TIM3->CCR4   = 4999; // 50% duty cycle
	TIM3->CCER  |= TIM_CCER_CC4E; // TIMx output compare 4 enable, active high
	TIM3->EGR    = TIM_EGR_UG; // Generate an update event to reload the prescaler value immediately
	TIM3->CR1   |= TIM_CR1_CEN; // Counter enable


	// SPI2:
	//   1 line TX-only
	//   Clock line: idle low
	//   Clock phase: 1st edge
	//   Baudrate: as much close to 16MHz as possible (from below)
	//   Frame: 8bit
	SPI2_HandleInit();
	SPI_Init(&hSPI2, SPI_CLK_PL_E1, SPI_DIR_TX);
	SPI_SetDataWidth(&hSPI2, SPI_DW_8BIT);
	SPI_SetRXFIFOThreshold(&hSPI2, SPI_RXFIFO_THQ);
	SPI_SetBaudrate(&hSPI2, Clock_SPICalcDivider(&hSPI2, 16000000));
	SPI_Enable(&hSPI2);


	// Initialize display
	SMLCD_InitGPIO();
	SMLCD_Init();
	SMLCD_Enable();
	SMLCD_Clear();

#define ORI 0
	uint8_t ori;
#if (ORI == 0)
	ori = LCD_ORIENT_NORMAL;
#elif (ORI == 1)
	ori = LCD_ORIENT_180;
#elif (ORI == 2)
	ori = LCD_ORIENT_CW;
#else
	ori = LCD_ORIENT_CCW;
#endif
	SMLCD_Orientation(ori);


	// Clear video buffer
	LCD_Clear();


	LCD_PixelMode = LCD_PSET;
//	LCD_PixelMode = LCD_PRES;
//	LCD_PixelMode = LCD_PINV;


	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;


#if 0 // test screen orientation

	uint32_t ticks = 0;

	while (1) {
		LCD_Clear();

		DWT->CYCCNT = 0;

#if 1
		// Draw a fake interface

		LCD_PixelMode = LCD_PSET;

		LCD_Rect(0, 0, scr_width - 1, scr_height - 1);
		LCD_Rect(2, 2, scr_width - 3, scr_height - 3);

		// RTC :)
		i  = 10;
		j  = 10;
		i += LCD_PutStr(i, j, "Time:", fnt7x10);
		i  = 10;
		j += 14;
		i += LCD_PutIntLZ(i, j, 23, 2, fnt_dig_big) + 1; // Hours
		LCD_FillRect(i, j + 6, i + 2, j + 14);
		LCD_FillRect(i, j + 19, i + 2, j + 27);
		i += 5;
		i += LCD_PutIntLZ(i, j, 48, 2, fnt_dig_big) + 1; // Minutes
		LCD_FillRect(i, j + 6, i + 2, j + 14);
		LCD_FillRect(i, j + 19, i + 2, j + 27);
		i += 5;
		i += LCD_PutIntLZ(i, j, 59, 2, fnt_dig_big); // Seconds

		// Horizontal divider
		j += 38;
		LCD_FillRect(2, j, scr_width - 3, j + 3);

		// Vertical divider
		LCD_FillRect(i + 5, 2, i + 8, j);

		// Rectangle filled with black as a cell header
		k  = 4;
		i += 10;
		LCD_FillRect(i, k, scr_width - 5, k + 16);

		// The battery...
		k += 3;
		i += 10;
		// Change drawing mode to INVERT pixels
		LCD_PixelMode = LCD_PINV;
		// So the text will be drawn on the black rectangle with INVERT mode
		// As a result we will see a white text
		LCD_PutStr(i, k, "BATTERY", fnt7x10);

		// Change drawing mode to SET pixels (normal mode)
		LCD_PixelMode = LCD_PSET;
		k += 17;
		i += 4;
		LCD_PutIntF(i, k, 4153, 3, dig8x16); // 4153 means 4.153V
		LCD_PutChar(i + 40, k + 3, 'V', fnt7x10);
		k += 19;
		i += 8;
		LCD_PutIntU(i, k, 95, dig8x16); // 95% :)
		LCD_PutChar(i + 20, k + 3, '%', fnt7x10);

		// Pretend we have an environment BME280 sensor...
		i  = 10;
		j += 8;
		i += LCD_PutStr(i, j, "BME280:", fnt7x10);

		i  = 10;
		j += 15;
		i += LCD_PutStr(i, j, "Temperature:", fnt7x10);
		i += LCD_PutIntF(i, j, 2468, 2, fnt7x10) + 2; // Let's say 2468 means 24.68C
		i += LCD_PutChar(i, j, 'C', fnt7x10);
		LCD_HLine(7, scr_width - 8, j + 12);

		i  = 10;
		j += 15;
		i += LCD_PutStr(i, j, "Pressure   :", fnt7x10);
		i += LCD_PutIntF(i, j, 738542, 3, fnt7x10) + 2; // 738542 means 738.542mmHg
		i += LCD_PutStr(i, j, "mmHg", fnt7x10);
		LCD_HLine(7, scr_width - 8, j + 12);

		i  = 10;
		j += 15;
		i += LCD_PutStr(i, j, "Humidity   :", fnt7x10);
		i += LCD_PutIntF(i, j, 32469, 3, fnt7x10) + 2; // 32469 means 32.469%RH
		i += LCD_PutStr(i, j, "%RH", fnt7x10);

		// Horizontal divider
		j += 15;
		LCD_FillRect(2, j, scr_width - 3, j + 3);

		// And ambient light sensor...
		i  = 10;
		j += 8;
		i += LCD_PutStr(i, j, "MAX44009:", fnt7x10);

		i  = 10;
		j += 15;
		i += LCD_PutStr(i, j, "Illuminance:", fnt7x10);
		i += LCD_PutIntF(i, j, 471382, 3, fnt7x10) + 2; // 471382 means 471.382LUX
		i += LCD_PutStr(i, j, "LUX", fnt7x10);

		// Draw a filled black rectangle at the bottom of screen
		j += 15;
		LCD_FillRect(2, j, scr_width - 3, scr_height - 3);

		// Change drawing mode to RESET pixels -> drawing with "white" color
		LCD_PixelMode = LCD_PRES;
		i  = 23;
		j += 7;
		// This will be white on black
		i += LCD_PutStr(i, j + 3, "Counter:", fnt7x10) + 2;
		i += LCD_PutIntLZ(i, j, 5783, 10, dig8x16);

		j += 20;
		LCD_PutIntLZ(40, j, k, 10, &digits_medium);

#endif

#if 0
		// Draw bitmaps
		if (ori & (LCD_ORIENT_CW | LCD_ORIENT_CCW)) {
			LCD_PixelMode = LCD_PINV;
			LCD_DrawBitmap(16, scr_height - 110, 77, 93, bmp_lol_guy);
			LCD_DrawBitmap(scr_width - 145, scr_height - 138, 128, 128, bmp_pandajs_logo);
		}
#endif

#if 0

		// Draw horizontal lines
		j = 0;
		for (i = 0; i < scr_height / 2; i += 3) {
			LCD_HLine(j, j + 36, i);
			j += 1;
		}
		j = 10;
		for (i = 0; i < scr_height / 2; i += 2) {
			LCD_HLine(0, j, i + (scr_height / 2));
			j += 3;
		}

#endif

#if 0

		// Draw filled rectangles
		j = 10;
		for (i = 2; i < scr_height - 15; i += 14) {
//			LCD_FillRect(j, i + 3, j + 4, i + 8);
//			LCD_Rect(j - 2, i + 1, j + 6, i + 10);
			LCD_FillRect(j, i + 3, j + 100, i + 8);
			LCD_Rect(j - 2, i + 1, j + 102, i + 10);
			j += 3;
		}

#endif

#if 0

		// Draw circles
		if (scr_width > scr_height) {
			j = scr_height / 2;
		} else {
			j = scr_width / 2;
		}
		for (i = 4; i < j; i += 4) {
			LCD_Circle((scr_width / 2) - 1, (scr_height / 2) - 1, i);
		}

#endif


//		LCD_Invert(2, 2, (scr_width / 2) - 1, (scr_height / 2) - 1);
//		LCD_Invert(0, 0, scr_width, scr_height);
//		LCD_InvertFull();

		ticks = DWT->CYCCNT;

//		DWT->CYCCNT = 0;
		SMLCD_Flush();
//		ticks = DWT->CYCCNT;


		printf("orientation=%u %04b -> ", ori, ori);
		if (ori == LCD_ORIENT_NORMAL) {
			printf("NORM");
			ori = LCD_ORIENT_CW;
		} else if (ori == LCD_ORIENT_CW) {
			printf("CW  ");
			ori = LCD_ORIENT_180;
		} else if (ori == LCD_ORIENT_180) {
			printf("180 ");
			ori = LCD_ORIENT_CCW;
		} else {
			ori = LCD_ORIENT_NORMAL;
			printf("CCW ");
		}
		printf("\t%u\r\n", ticks);
		if (ori == LCD_ORIENT_NORMAL) {
			printf("----------------------------\r\n");
		}

//		ori = LCD_ORIENT_NORMAL;

		SMLCD_Orientation(ori);

		GPIO_PIN_INVERT(GPIOA, GPIO_PIN_5);
		Delay_ms(1000);
	}

#endif // test screen orientation


	SMLCD_Flush();


	SMLCD_Orientation(LCD_ORIENT_CW);

	j = scr_height - 65;

	int32_t px, py;
	int32_t dx, dy;

	uint32_t x1, x2;
	uint32_t y1, y2;

	px = 0;
	py = 0;
	dx = 4;
	dy = 2;

	x1 = 10;
	y1 = scr_height - 150;
	x2 = scr_width - 78;
	y2 = scr_height - 11;

	uint32_t w = x2 - x1 + 1;
	uint32_t h = y2 - y1 + 1;

	// The main loop
	while (1) {
		// Invert the PA5 pin state every half of second
		Delay_ms(50);
		GPIO_PIN_INVERT(GPIOA, GPIO_PIN_5);

		LCD_Clear();
		for (i = 10; i < scr_height - 200; i += 76) {
			LCD_PutIntU(12, i, k, fnt_dig_big);
			LCD_PutIntU(12, i + 36, ~k, fnt_dig_big);
		}
		k += 4321;


		LCD_Rect(x1, y1, x2, y2);
		LCD_Line(x1, y1, x1 + px, y1 + py);
		LCD_Line(x1, y2, x1 + px, y1 + py);
		LCD_Line(x2, y1, x1 + px, y1 + py);
		LCD_Line(x2, y2, x1 + px, y1 + py);
		px += dx;
		py += dy;
		if (px >= w) {
			dx *= -1;
		}
		if (py <= 0) {
			dy *= -1;
		}
		if (py >= h) {
			dy *= -1;
		}
		if (py <= 0) {
			dy *= -1;
		}


		LCD_DrawBitmap(scr_width - 68, --j, 64, 64, bmp_spider);
		if (j == 0) {
			j = scr_height - 65;
		}


		SMLCD_Flush();
	}
}
