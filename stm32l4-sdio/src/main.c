#include "main.h"


// Define size of the buffer for multi-block reads/writes
// Note: should be multiple of 512
//#define BUFFER_SIZE (512U * 1U)  // 0.5KB
//#define BUFFER_SIZE (512U * 6U)  // 2.5KB
#define BUFFER_SIZE (512U * 32U) // 16KB
//#define BUFFER_SIZE (512U * 64U) // 32KB
//#define BUFFER_SIZE (512U * 128U) // 64KB

// Define non zero to enable high-speed mode
// If enabled, will try to switch SD card to HS mode
// and enable SDIO clock bypass
#define SD_HIGH_SPEED 1

// Duration of each test cycle (seconds)
// Note:
// 1 second is too short for an adequate result
// 2 seconds can cause incorrect results on very slow cards
// 3-5 seconds is pretty enough
// more - will not harm but doesn't have sense
#define TEST_CYCLE_TIME 4

// Define start address for read/write tests
// Note: if the specified address will out of bounds for the tested card,
// the start address will be set to beginning of the card
#define TEST_START_ADDR 0x00000000

// Define non zero to enable single read test
// This test reads the same data block using polling and DMA methods
// Then it compares the data CRCs of each method
#define TEST_SINGLE_READ 1

// Define non zero to enable read tests
// This test continuously reads data from the card,
// the duration is set by TEST_CYLCE_TIME, defined above
// The read test consists of four parts:
// - single-block reading using the polling
// - multi-block reading using the polling
// - single-block reading using the DMA
// - multi-block reading using the DMA
#define TEST_READ 1

// Define non zero to enable write tests
// WARNING: tests will destroy existing data on card
// This test continuously writes data to the card,
// the duration is set by TEST_CYLCE_TIME, defined above
// The write test consists of four parts:
// - single-block write using the polling
// - multi-block write using the polling
// - single-block write using the DMA
// - multi-block write using the DMA
#define TEST_WRITE 0


// Start timer for measurements
#define __START_TIMER do {     \
	ms_count = 0;              \
	TIM17->CNT = 0;            \
	TIM17->CR1 |= TIM_CR1_CEN; \
} while (0);

// Stop timer for measurements
#define __STOP_TIMER do {        \
	TIM17->CR1 &= ~TIM_CR1_CEN;  \
	ms_count += TIM17->CNT / 10; \
} while (0);

// Increase address
#define __ADDR_INC do {                     \
	addr += blk_size;                       \
	if ((addr >> 9) >= SDCard.BlockCount) { \
		addr = start_addr;                  \
	}                                       \
} while (0);


#if 0
// Place buffers in SRAM2 region
static uint8_t __attribute__((section(".sram2"))) sd_buf[BUFFER_SIZE] __attribute__((aligned(4)));
#else
// Place buffers in SRAM region
static uint8_t sd_buf[BUFFER_SIZE] __attribute__((aligned(4)));
#endif


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

// TIM17 interrupt handler
void TIM1_TRG_COM_TIM17_IRQHandler(void) {
	if (TIM17->SR & TIM_SR_UIF) {
		TIM17->SR = ~TIM_SR_UIF;
		ms_count += 100;
	}
}

// Dump buffer to USART in human-readable format (formatted HEX and RAW values)
void USART_SendBufHexPretty(USART_TypeDef *USARTx, const char *buf, uint32_t length, uint8_t column_width, char subst) {
	uint32_t i = 0;
	uint32_t len;
	uint32_t pos;
	uint8_t tmp_buf[column_width];

	while (i < length) {
		// Print line number
		USART_SendHexLZ(USARTx, i, 8);
		USART_SendChar(USARTx, ':');
		USART_SendChar(USARTx, ' ');

		// Copy data for single line to temporary buffer
		len = (i + column_width >= length) ? length - i : column_width;
		memcpy(tmp_buf, &buf[i], len);

		// Print Hex data
		pos = 0;
		while (pos < len) USART_SendHexLZ(USARTx, tmp_buf[pos++], 2);
		if (pos == column_width) {
			USART_SendChar(USARTx, ' ');
			USART_SendChar(USARTx, '|');
			USART_SendChar(USARTx, ' ');
		} else {
			do { USART_SendChar(USARTx, ' '); } while (pos++ <= column_width);
			USART_SendChar(USARTx, '|');
			USART_SendChar(USARTx, ' ');
		}

		// Print raw data
		pos = 0;
		do {
			USART_SendChar(USARTx, tmp_buf[pos] > 32 ? tmp_buf[pos] : subst);
		} while (++pos < len);

		USART_SendChar(USARTx, '\r');
		USART_SendChar(USARTx, '\n');

		i += len;
	}
}

// Calculate CRC of the data buffer (using CRC peripheral)
// input:
//   pBuf - pointer to the data buffer
//   length - length of the buffer in bytes
uint8_t CRC8_CCITT(uint8_t *pBuf, uint32_t length) {
	CRC_Reset();
	CRC_CalcBuffer((uint32_t *)pBuf, length);
	return CRC_GetData8();
}

#if 0
// Calculate CRC-8/CCITT of the data buffer (software)
// input:
//   pBuf - pointer to the data buffer
//   length - length of the buffer in bytes
// CRC parameters: Polynomial=0x07, Init=0x00, RefIn=false, RefOut=false, XorOut=0x00
uint8_t CRC8_CCITT(uint8_t *pBuf, uint32_t length) {
	uint16_t crc = 0;
	uint8_t i;

	while (length--) {
		crc ^= (*pBuf++ << 8);
		for (i = 8; i; i--) {
			if (crc & 0x8000) crc ^= 0x8380; // polynomial value = ((0x07 << 7) & 0x8000)
			crc <<= 1;
		}
	}

	return (uint8_t)(crc >> 8);
}
#endif

// De-initialize and halt
void DeInit_and_Halt(void) {
	// De-initialize the SDIO GPIO lines and disable the Clk48
	SD_GPIO_DeInit();
	SD_SDIO_DeInit();
	Clock_Clk48Disable();

	// Disable timer
	TIM17->CR1 &= ~TIM_CR1_CEN;
	NVIC_DisableIRQ(TIM1_TRG_COM_TIM17_IRQn);

	printf("Halt\r\n");
	// Wait for last USART transmit completed and then disable USART
	while (!USART_GetFlags(&hUSART1, USART_FLAG_TC));
	USART_Disable(&hUSART1);

	//PWR_EnterSLEEPMode(PWR_SENTRY_WFI);
	PWR_EnterSTANDBYMode();
}

// Print results of read/write tests (uses global variables)
void PrintResults(void) {
	if (sd_error != 0) {
		printf("success/errors count: %u/%u\r\n",
				sd_count,
				sd_error
			);
	} else {
		printf("%u blocks (%.3u MB/%u bytes); perf: %.2u blocks/s, %.2u bytes/s (%.3uMB/s)\r\n",
				sd_count * (blk_size >> 9),
				sd_amount >> 10,
				sd_amount,
				(uint32_t)(((float)(sd_count * (blk_size >> 9)) * 100000.0f) / (float)ms_count),
				(uint32_t)(((float)sd_amount * 100000.0f) / (float)ms_count),
				(uint32_t)((((float)sd_amount / 1048576.0f) * 1000000.0f) / (float)ms_count)
			);
	}
}

// Initialize variables for tests
void InitTestVars(uint32_t start_address, uint32_t block_size) {
	start_addr = start_address;
	if ((start_addr >> 9) > SDCard.BlockCount) {
		start_addr = 0x00000000;
	}
	addr       = start_addr;
	blk_size   = block_size;
	sd_count   = 0;
	sd_error   = 0;
	sd_crc     = 0;
	sd_amount  = 0;
}


int main(void) {
	// Initialize the MCU clock system
	SetSysClock();
	SystemCoreClockUpdate();


	// Enable debugging when the MCU is in low power modes
	DBGMCU->CR |= DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY;


	// Prefetch is useful if at least one wait state is needed to access the Flash memory
	if (RCC_GetLatency() != FLASH_ACR_LATENCY_0WS) {
		// Enable prefetch buffer (a.k.a ART)
		RCC_PrefetchEnable();
	}


	// Initialize debug output port (USART1)
	//   clock source: SYSCLK
	//   mode: transmit only
	USART1_HandleInit();
	RCC_SetClockUSART(RCC_USART1_CLK_SRC, RCC_PERIPH_CLK_SYSCLK);
	USART_Init(&hUSART1, USART_MODE_TX);

	// Configure USART:
	//   oversampling by 16
	//   115200, 8-N-1
	//   no hardware flow control
	USART_SetOversampling(&hUSART1, USART_OVERS16);
	USART_SetBaudRate(&hUSART1, 3000000);
	USART_SetDataMode(&hUSART1, USART_DATAWIDTH_8B, USART_PARITY_NONE, USART_STOPBITS_1);
	USART_SetHWFlow(&hUSART1, USART_HWCTL_NONE);
	USART_Enable(&hUSART1);
	USART_CheckIdleState(&hUSART1, 0xC5C10); // Timeout of about 100ms at 80MHz CPU


	// Say "hello world"
	RCC_ClocksTypeDef Clocks;
	RCC_GetClocksFreq(&Clocks);
	printf("\r\n---STM32L476RG---\r\n");
	printf("SDIO (%s @ %s)\r\n", __DATE__, __TIME__);
	printf("CPU: %.3uMHz\r\n", SystemCoreClock / 1000);
	printf("SYSCLK=%.3uMHz, HCLK=%.3uMHz\r\n", Clocks.SYSCLK_Frequency / 1000, Clocks.HCLK_Frequency / 1000);
	printf("APB1=%.3uMHz, APB2=%.3uMHz\r\n", Clocks.PCLK1_Frequency / 1000, Clocks.PCLK2_Frequency / 1000);
	printf("System clock: %s\r\n", _sysclk_src_str[RCC_GetSysClockSource()]);


	// Initialize delay functions
	Delay_Init();


	// Initialize the PA10 pin (SDCard power enable on WBC v3.6)
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	GPIO_set_mode(GPIOA, GPIO_Mode_OUT, GPIO_PUPD_NONE, GPIO_PIN_10);
	GPIO_out_cfg(GPIOA, GPIO_OT_PP, GPIO_SPD_LOW, GPIO_PIN_10);

	// Enable and configure the PLLSAI1 to use as CLK48 clock
	Clock_Clk48Config();

	// Initialize the SDIO GPIO lines
	SD_GPIO_Init();

	// Initialize the SDIO peripheral
	SD_SDIO_Init();

	// Enable DMA2 peripheral (for SDIO)
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	// Enable TIM17 peripheral (time measurements for tests)
	RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;


	// Enable power for SD card
	GPIO_PIN_SET(GPIOA, GPIO_PIN_10);
	// Wait for HW soft-start (about 150ms)
	Delay_ms(200);


	// Try to initialize the SD card
	j = SD_Init();
	printf("SDCard init: %02X\r\n", j);
	if (j != SDR_Success) {
		DeInit_and_Halt();
	}

	printf("SDCard info:\r\n");
	printf("\tRCA     : %X\r\n\ttype    : ", SDCard.RCA);
	switch (SDCard.Type) {
		case SDCT_SDSC_V1: printf("SDSCv1");  break;
		case SDCT_SDSC_V2: printf("SDSCv2");  break;
		case SDCT_MMC:     printf("MMC");     break;
		case SDCT_SDHC:    printf((SDCard.BlockCount >= 65535) ? "SDXC" : "SDHC"); break;
		default:           printf("unknown"); break;
	}
	printf("\r\n\tcapacity: %.3u%c [%u blocks]",
			((SDCard.Capacity >> 10) * 1000) >> 10,
			(SDCard.Type == SDCT_SDHC) ? 'G' : 'M',
			SDCard.BlockCount
		);
	printf("\r\n\tmax.freq: %uMHz\r\n", SDCard.MaxBusClkFreq);
#if 1
	// CSD: version of CSD
	printf("\tCSD     : v%u.00\r\n", (SDCard.CSDVer == 0) ? 1 : 2);
	// PNM: product name
	printf("\tPNM     : \"%5s\"\r\n", SDCard.PNM);
	// PRV: product revision
	printf("\tPRV     : %u.%u\r\n", SDCard.PRV >> 4, SDCard.PRV & 0x0F);
	// MID: manufacturer ID, OID: OEM/Application ID
	printf("\tMID/OID : %02X/%04X\r\n", SDCard.MID, SDCard.OID);
	// PSN: product serial number
	printf("\tPSN     : %u [0x%08X]\r\n", SDCard.PSN, SDCard.PSN);
	// MDT: month and year when the card was manufactured
	printf("\tMDT     : %02u/%04u [0x%04X]\r\n",
			SDCard.MDT & 0x0F,
			(SDCard.MDT >> 4) + 2000,
			SDCard.MDT
		);
#endif

#if (SDIO_USE_4BIT)
	if (SDCard.Type != SDCT_MMC) {
		// MMC doesn't support 4-bit bus
		if (SDCard.SCR[1] & 0x05) {
			// Set 4-bit bus width
			SD_SetBusWidth(SD_BUS_4BIT);
		}
	}
#endif // SDIO_USE_4BIT

#if (SD_HIGH_SPEED != 0)
	// Try to switch to High-speed mode
	printf("High-speed mode: ");
	i = SD_HighSpeed();
	if (i == SDR_Success) {
		printf("OK\r\n");

		// Enable SDIO clock bypass
		SDMMC1->CLKCR |= SDMMC_CLKCR_BYPASS;
	} else {
		printf("FAIL [%02X]\r\n", i);
	}
#endif // SD_HIGH_SPEED

	// Print parameters of the SDIO bus
	printf("SDIO: ");
	switch (SDMMC1->CLKCR & SDMMC_CLKCR_WIDBUS) {
		case SD_BUS_4BIT: printf("4-bit"); break;
		case SD_BUS_8BIT: printf("8-bit"); break;
		default:          printf("1-bit"); break;
	}
	// Presume that SDMMC clock is 48MHz ...
	i = 48000000;
	if (SDMMC1->CLKCR & SDMMC_CLKCR_BYPASS) {
		printf(", BYPASS -> %.3uMHz\r\n", i / 1000);
	} else {
		printf(", div=%u -> %.3uMHz\r\n",
				SDMMC1->CLKCR & SDMMC_CLKCR_CLKDIV,
				i / ((SDMMC1->CLKCR & SDMMC_CLKCR_CLKDIV) + 2) / 1000
			);
	}


	// Configure TIM17 (overflow every 100ms)
	TIM17->CR1  |= TIM_CR1_ARPE | TIM_CR1_URS;
	TIM17->PSC   = (SystemCoreClock / 10000) - 1;
	TIM17->ARR   = 1000 - 1;
	TIM17->DIER |= TIM_DIER_UIE;
	TIM17->EGR   = TIM_EGR_UG;
	TIM17->SR    = ~TIM_SR_UIF;

	// Enable TIM17 IRQ
	NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);


	// Configure CRC peripheral for 8-bit CCITT calculation
	// CCITT CRC-8 (Poly=0x07, Init=0x00, RefIn=false, RefOut=false)
	CRC_Enable();
	CRC_SetInRevMode(CRC_IN_NORMAL);
	CRC_SetOutRevMode(CRC_OUT_NORMAL);
	CRC_SetPolynomialSize(CRC_PSIZE_8B);
	CRC_SetPolynomial(0x07);
	CRC_SetInitValue(0x00);


#if (TEST_SINGLE_READ != 0)
	// Single read test: read a same block of data using polling and DMA,
	// then compare results CRCs

	// Starting address to read
	InitTestVars(TEST_START_ADDR, 512);

	memset(sd_buf, 0xAA, blk_size);
	j = SD_ReadBlock(addr, (uint32_t *)sd_buf, blk_size);
	if (j == SDR_Success) {
		crc_ref = CRC8_CCITT(sd_buf, blk_size);
		printf("Poll: 0x%02X\r\n", crc_ref);
		//USART_SendBufHexPretty(DBG_USART, sd_buf, blk_size, 32, '.');
	} else {
		crc_ref = 0;
		printf("ERB:%02X\r\n", j);
	}

	// Configure the SDIO DMA channel to read data from SDIO
	SD_Configure_DMA((uint32_t *)sd_buf, blk_size, SDIO_DMA_DIR_RX);

	memset(sd_buf, 0x55, blk_size);
	j = SD_ReadBlock_DMA(addr, (uint32_t *)sd_buf, blk_size);
	if (j == SDR_Success) {
		j = SD_CheckRead(blk_size);
		if (j == SDR_Success) {
			crc_val = CRC8_CCITT(sd_buf, blk_size);
			printf("DMA:  0x%02X\r\n", crc_val);
			//USART_SendBufHexPretty(DBG_USART, sd_buf, blk_size, 32, '.');
		} else {
			printf("ECR:%02X\r\n", j);
		}
	} else {
		printf("ERB:%02X\r\n", j);
	}

	printf("Result: %02X:%02X -> %s\r\n",
			crc_ref,
			crc_val,
			(crc_ref == crc_val) ? "PASSED" : "FAIL"
		);
#endif // TEST_SINGLE_READ


#if (TEST_READ != 0)
	// Perform read tests

	printf("READ (polling) ...\r\n");

	printf("Single block: ");
	InitTestVars(TEST_START_ADDR, 512);
	__START_TIMER;
	do {
		j = SD_ReadBlock(addr, (uint32_t *)sd_buf, blk_size);
		if (j == SDR_Success) {
			sd_count++;
			sd_amount += blk_size;
		} else {
			printf("ERB:%02X ", j);
			sd_error++;
		}
		__ADDR_INC;
	} while (ms_count < 1000 * TEST_CYCLE_TIME);
	__STOP_TIMER;
	PrintResults();

	printf("Multiple block: ");
	InitTestVars(TEST_START_ADDR, BUFFER_SIZE);
	__START_TIMER;
	do {
		j = SD_ReadBlock(addr, (uint32_t *)sd_buf, blk_size);
		if (j == SDR_Success) {
			sd_count++;
			sd_amount += blk_size;
		} else {
			printf("ERB:%02X ", j);
			sd_error++;
		}
		__ADDR_INC;
	} while (ms_count < 1000 * TEST_CYCLE_TIME);
	__STOP_TIMER;
	PrintResults();

	printf("READ (DMA) ...\r\n");

	// Configure the SDIO DMA channel to read data from SDIO
	SD_Configure_DMA((uint32_t *)sd_buf, BUFFER_SIZE, SDIO_DMA_DIR_RX);

	printf("Single block: ");
	InitTestVars(TEST_START_ADDR, 512);
	__START_TIMER;
	do {
		j = SD_ReadBlock_DMA(addr, (uint32_t *)sd_buf, blk_size);
		if (j == SDR_Success) {
			j = SD_CheckRead(blk_size);
			if (j == SDR_Success) {
				sd_count++;
				sd_amount += blk_size;
			} else {
				printf("ECR:%02X ", j);
				sd_error++;
			}
		} else {
			printf("ERB:%02X ", j);
			sd_error++;
		}
		__ADDR_INC;
	} while (ms_count < 1000 * TEST_CYCLE_TIME);
	__STOP_TIMER;
	PrintResults();

	printf("Multiple block: ");
	InitTestVars(TEST_START_ADDR, BUFFER_SIZE);
	__START_TIMER;
	do {
		j = SD_ReadBlock_DMA(addr, (uint32_t *)sd_buf, blk_size);
		if (j == SDR_Success) {
			j = SD_CheckRead(blk_size);
			if (j == SDR_Success) {
				sd_count++;
				sd_amount += blk_size;
			} else {
				printf("ECR:%02X ", j);
				sd_error++;
			}
		} else {
			printf("ERB:%02X ", j);
			sd_error++;
		}
		__ADDR_INC;
	} while (ms_count < 1000 * TEST_CYCLE_TIME);
	__STOP_TIMER;
	PrintResults();

#endif // Read tests


#if (TEST_WRITE != 0)
	// Perform write tests

	printf("WRITE (polling) ...\r\n");

	printf("Single block: ");
	memset(sd_buf, 0xAA, BUFFER_SIZE);
	InitTestVars(TEST_START_ADDR, 512);
	__START_TIMER;
	do {
		j = SD_WriteBlock(addr, (uint32_t *)sd_buf, blk_size);
		if (j == SDR_Success) {
			sd_count++;
			sd_amount += blk_size;
		} else {
			printf("EWB:%02X ", j);
			sd_error++;
		}
		__ADDR_INC;
	} while (ms_count < 1000 * TEST_CYCLE_TIME);
	__STOP_TIMER;
	PrintResults();

	printf("Multiple block: ");
	memset(sd_buf, 0x55, BUFFER_SIZE);
	InitTestVars(TEST_START_ADDR, BUFFER_SIZE);
	__START_TIMER;
	do {
		j = SD_WriteBlock(addr, (uint32_t *)sd_buf, blk_size);
		if (j == SDR_Success) {
			sd_count++;
			sd_amount += blk_size;
		} else {
			printf("EWB:%02X ", j);
			sd_error++;
		}
		__ADDR_INC;
	} while (ms_count < 1000 * TEST_CYCLE_TIME);
	__STOP_TIMER;
	PrintResults();

	printf("WRITE (DMA) ...\r\n");

	// Configure the SDIO DMA channel to write data to SDIO
	SD_Configure_DMA((uint32_t *)sd_buf, BUFFER_SIZE, SDIO_DMA_DIR_TX);

	printf("Single block: ");
	memset(sd_buf, 0xAA, BUFFER_SIZE);
	InitTestVars(TEST_START_ADDR, 512);
	__START_TIMER;
	do {
		j = SD_WriteBlock_DMA(addr, (uint32_t *)sd_buf, blk_size);
		if (j == SDR_Success) {
			j = SD_CheckWrite(blk_size);
			if (j == SDR_Success) {
				sd_count++;
				sd_amount += blk_size;
			} else {
				printf("ECW:%02X ", j);
				sd_error++;
			}
		} else {
			printf("EWB:%02X ", j);
			sd_error++;
		}
		__ADDR_INC;
	} while (ms_count < 1000 * TEST_CYCLE_TIME);
	__STOP_TIMER;
	PrintResults();

	printf("Multiple block: ");
	memset(sd_buf, 0x55, BUFFER_SIZE);
	InitTestVars(TEST_START_ADDR, BUFFER_SIZE);
	__START_TIMER;
	do {
		j = SD_WriteBlock_DMA(addr, (uint32_t *)sd_buf, blk_size);
		if (j == SDR_Success) {
			j = SD_CheckWrite(blk_size);
			if (j == SDR_Success) {
				sd_count++;
				sd_amount += blk_size;
			} else {
				printf("ECW:%02X ", j);
				sd_error++;
			}
		} else {
			printf("EWB:%02X ", j);
			sd_error++;
		}
		__ADDR_INC;
	} while (ms_count < 1000 * TEST_CYCLE_TIME);
	__STOP_TIMER;
	PrintResults();

#endif // Write tests


	// Shutdown :)
	DeInit_and_Halt();


	// The main loop
	while (1);
}
