#include "main.h"

//#define buf_size (512U * 6)  // 2.5KB
//#define buf_size (512U * 32) // 16KB
//#define buf_size (512U * 64) // 32KB
#define buf_size (512U * 128) // 64KB

#if 0
// Place buffers in SRAM2 region
static uint8_t __attribute__((section(".sram2"))) sd_buf[buf_size] __attribute__((aligned(4)));
#else
// Place buffers in SRAM region
static uint8_t sd_buf[buf_size] __attribute__((aligned(4)));
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

// TIM17 interrupt handler
void TIM1_TRG_COM_TIM17_IRQHandler(void) {
	if (TIM17->SR & TIM_SR_UIF) {
		TIM17->SR &= ~TIM_SR_UIF;

		tim_flag = !0;
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
		USART_SendHexLZ(USARTx,i,8);
		USART_SendChar(USARTx,':');
		USART_SendChar(USARTx,' ');

		// Copy data for single line to temporary buffer
		len = (i + column_width >= length) ? length - i : column_width;
		memcpy(tmp_buf,&buf[i],len);

		// Print Hex data
		pos = 0;
		while (pos < len) USART_SendHexLZ(USARTx,tmp_buf[pos++],2);
		if (pos == column_width) {
			USART_SendChar(USARTx,' ');
			USART_SendChar(USARTx,'|');
			USART_SendChar(USARTx,' ');
		} else {
			do { USART_SendChar(USARTx,' '); } while (pos++ <= column_width);
			USART_SendChar(USARTx,'|');
			USART_SendChar(USARTx,' ');
		}

		// Print raw data
		pos = 0;
		do {
			USART_SendChar(USARTx,tmp_buf[pos] > 32 ? tmp_buf[pos] : subst);
		} while (++pos < len);

		USART_SendChar(USARTx,'\r');
		USART_SendChar(USARTx,'\n');

		i += len;
	}
}

// Calculate CRC-8/CCITT of the data buffer
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


int main(void) {
	// Initialize the MCU clock system
	SystemInit();
	SetSysClock();
	SystemCoreClockUpdate();


	// Initialize debug output port (USART1)
	//   clock source: SYSCLK
	//   mode: transmit only
	USART1_HandleInit();
	RCC_SetClockUSART(RCC_USART1_CLK_SRC,RCC_PERIPH_CLK_SYSCLK);
	USART_Init(&hUSART1,USART_MODE_TX);

	// Configure USART:
	//   oversampling by 16
	//   115200, 8-N-1
	//   no hardware flow control
	USART_SetOversampling(&hUSART1,USART_OVERS16);
	USART_SetBaudRate(&hUSART1,3000000);
	USART_SetDataMode(&hUSART1,USART_DATAWIDTH_8B,USART_PARITY_NONE,USART_STOPBITS_1);
	USART_SetHWFlow(&hUSART1,USART_HWCTL_NONE);
	USART_Enable(&hUSART1);
	USART_CheckIdleState(&hUSART1,0xC5C10); // Timeout of about 100ms at 80MHz CPU


	// Say "hello world"
	RCC_ClocksTypeDef Clocks;
	RCC_GetClocksFreq(&Clocks);
	printf("\r\n---STM32L476RG---\r\n");
	printf("SDIO (%s @ %s)\r\n",__DATE__,__TIME__);
	printf("CPU: %.3uMHz\r\n",SystemCoreClock / 1000);
	printf("SYSCLK=%.3uMHz, HCLK=%.3uMHz\r\n",Clocks.SYSCLK_Frequency / 1000,Clocks.HCLK_Frequency / 1000);
	printf("APB1=%.3uMHz, APB2=%.3uMHz\r\n",Clocks.PCLK1_Frequency / 1000,Clocks.PCLK2_Frequency / 1000);
	printf("System clock: %s\r\n",_sysclk_src_str[RCC_GetSysClockSource()]);


	// Initialize delay functions
	Delay_Init();


	// Initialize the PA10 pin (SDCard power enable on WBC v3.6)
	// Enable the GPIOA peripheral
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	// Configure PA5 as push-pull output without pull-up, at lowest speed
	GPIO_set_mode(GPIOA,GPIO_Mode_OUT,GPIO_PUPD_NONE,GPIO_PIN_10);
	GPIO_out_cfg(GPIOA,GPIO_OT_PP,GPIO_SPD_LOW,GPIO_PIN_10);

	// Enable power for SD card
	GPIO_PIN_SET(GPIOA,GPIO_PIN_10);
	Delay_ms(50);

	// Enable and configure the PLLSAI1 to use as CLK48 clock
	Clock_Clk48Config();

	// Initialize the SDIO GPIO lines
	SD_GPIO_Init();

	// Initialize the SDIO peripheral
	SD_SDIO_Init();

	// Enable DMA2 peripheral (for SDIO)
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	// Try to initialize the SD card
	j = SD_Init();
	printf("SDCard init: %02X\r\n",j);
	if (j != SDR_Success) {
		// De-initialize the SDIO GPIO lines and disable the Clk48
		SD_GPIO_DeInit();
		SD_SDIO_DeInit();
		Clock_Clk48Disable();

		printf("Halt\r\n");
		// Wait for last USART transmit completed and then disable USART
		while (!USART_GetFlags(&hUSART1,USART_FLAG_TC));
		USART_Disable(&hUSART1);

		PWR_EnterSTANDBYMode();
	}

	printf("SDCard info:\r\n");
	printf("\tRCA     : %X\r\n\ttype    : ",SDCard.RCA);
	switch (SDCard.Type) {
		case SDCT_SDSC_V1: printf("SDSCv1");  break;
		case SDCT_SDSC_V2: printf("SDSCv2");  break;
		case SDCT_MMC:     printf("MMC");     break;
		case SDCT_SDHC:    printf("SDHC");    break;
		default:           printf("unknown"); break;
	}
	printf("\r\n\tcapacity: ");
	if (SDCard.Type == SDCT_SDHC) {
		printf("%.3uGb",((SDCard.Capacity / 1024) * 1000) / 1024);
	} else {
		printf("%.3uMb",((SDCard.Capacity / 1024) * 1000) / 1024);
	}
	printf("\r\n\tmax.freq: %uMHz\r\n",SDCard.MaxBusClkFreq);
/*
	printf("\tCSD     : v%u.00\r\n",(SDCard.CSDVer == 0) ? 1 : 2);
	printf("\tPNM     : \"%5s\"\r\n",SDCard.PNM);
	printf("\tPRV     : %u.%u\r\n",SDCard.PRV >> 4,SDCard.PRV & 0x0f);
	printf("\tMID/OID : %02X/%04X\r\n",SDCard.MID,SDCard.OID);
	printf("\tPSN     : %u\r\n",SDCard.PSN);
	printf("\tMDT     : %03X\r\n",SDCard.MDT);
*/

#if (SDIO_USE_4BIT)
	if (SDCard.Type != SDCT_MMC) {
		// MMC doesn't support 4-bit bus
		if (SDCard.SCR[1] & 0x05) {
			// Set 4-bit bus width
			SD_SetBusWidth(SD_BUS_4BIT);
		}
	}
#endif // SDIO_USE_4BIT

	printf("Bus: ");
	switch (SDMMC1->CLKCR & SDMMC_CLKCR_WIDBUS) {
		case SD_BUS_4BIT: printf("4-bit"); break;
		case SD_BUS_8BIT: printf("8-bit"); break;
		default:          printf("1-bit"); break;
	}
	printf(", div=%u -> %.3uMHz\r\n",
			SDMMC1->CLKCR & 0xFF,
			48000000 / ((SDMMC1->CLKCR & 0xFF) + 2) / 1000
		);

#if 0
	// Try to switch to High-speed mode
	i = SD_HighSpeed();
	// For SDv1,SDv2 and MMC card must set block size (SDHC/SDXC always have fixed size 512bytes)
	if ((SDCard.Type == SDCT_SDSC_V1) || (SDCard.Type == SDCT_SDSC_V2) || (SDCard.Type == SDCT_MMC)) {
		i = SD_SetBlockSize(512);
	}
	// Enable SDIO clock bypass
	SDMMC1->CLKCR |= SDMMC_CLKCR_BYPASS;
#endif


	// Initialize test variables
	sd_count  = 0;
	sd_error  = 0;
	sd_crc    = 0;
	sd_amount = 0;

	// Test duration (1 to 32 seconds)
	tim_seconds = 2;


/*
	// Configure CRC unit
	CRC_Enable();
	CRC_SetInRevMode(CRC_IN_NORMAL);
	CRC_SetOutRevMode(CRC_OUT_NORMAL);
	CRC_SetPolynomialSize(CRC_PSIZE_8B);
	CRC_SetPolynomial(0x07);
	CRC_SetInitValue(0x00);
	CRC_Reset();
*/


	// Configure TIM17
	RCC->APB2ENR |= RCC_APB2ENR_TIM17EN;
	TIM17->CR1 |= TIM_CR1_ARPE;
	TIM17->CR1 |= TIM_CR1_URS;
	TIM17->CR1 |= TIM_CR1_OPM;
	TIM17->PSC  = SystemCoreClock / (1999 + 1);
	TIM17->ARR  = 2000 * tim_seconds;
	TIM17->DIER |= TIM_DIER_UIE;
	TIM17->EGR  = TIM_EGR_UG;
	TIM17->SR  &= ~TIM_SR_UIF;

	// Enable TIM17 IRQ
	NVIC_EnableIRQ(TIM1_TRG_COM_TIM17_IRQn);


	memset(sd_buf,0x88,buf_size);
	j = SD_ReadBlock(0x5000000,(uint32_t *)sd_buf,buf_size);
	crc_ref = CRC8_CCITT(sd_buf,buf_size);
//	USART_SendBufHexPretty(DBG_USART,sd_buf,buf_size,32,'.');
	printf("ReadBlock: %02X [CRC:%02X]\r\n",j,crc_ref);

	// Configure the SDIO DMA channel to read data from SDIO
	SD_Configure_DMA((uint32_t *)sd_buf, buf_size, SDIO_DMA_DIR_RX);

	memset(sd_buf,0x11,buf_size);
	j = SD_ReadBlock_DMA(0x5000000,(uint32_t *)sd_buf,buf_size);
	if (j == SDR_Success) {
		j = SD_CheckRead(buf_size);
		if (j == SDR_Success) {
			crc_val = CRC8_CCITT(sd_buf,buf_size);
			printf("DMA CRC: %02X\r\n",crc_val);
//			USART_SendBufHexPretty(DBG_USART,sd_buf,buf_size,32,'.');
		} else {
			printf("CR:%02X\r\n",j);
		}
	} else {
		printf("RB:%02X\r\n",j);
	}


#if 0
	// Perform read tests

	printf("Single block read POLL: ");
	sd_count  = 0;
	sd_error  = 0;
	sd_crc    = 0;
	sd_amount = 0;
	tim_flag = 0;
	TIM17->CR1 |= TIM_CR1_CEN;
	while (!tim_flag) {
		j = SD_ReadBlock(0x5000000, (uint32_t *)sd_buf, 512);
		if (j == SDR_Success) {
			sd_count++;
			sd_amount += 512;
		} else {
			printf("RB:%02X ", j);
			sd_error++;
		}
	}
	if (sd_error != 0) {
		printf("success/errors count: %u/%u\r\n",
				sd_count,
				sd_error
			);
	} else {
		printf("%u bytes (%.3u MB), perf: %u blocks/s, %u bytes/s (%.3uMB/s)\r\n",
				sd_amount,
				sd_amount >> 10,
				sd_count / tim_seconds,
				sd_amount / tim_seconds,
				(uint32_t)((sd_amount / 1048576.0) * 1000.0) / tim_seconds
			);
	}

	printf("Multiple block read POLL: ");
	sd_count  = 0;
	sd_error  = 0;
	sd_crc    = 0;
	sd_amount = 0;
	tim_flag = 0;
	TIM17->CR1 |= TIM_CR1_CEN;
	while (!tim_flag) {
		j = SD_ReadBlock(0x5000000, (uint32_t *)sd_buf, buf_size);
		if (j == SDR_Success) {
			sd_count++;
			sd_amount += buf_size;
		} else {
			printf("RB:%02X ", j);
			sd_error++;
		}
	}
	if (sd_error != 0) {
		printf("success/errors count: %u/%u\r\n",
				sd_count,
				sd_error
			);
	} else {
		printf("%u bytes (%.3u MB), perf: %u blocks/s, %u bytes/s (%.3uMB/s)\r\n",
				sd_amount,
				sd_amount >> 10,
				((buf_size >> 9) * sd_count) / tim_seconds,
				sd_amount / tim_seconds,
				(uint32_t)((sd_amount / 1048576.0) * 1000.0) / tim_seconds
			);
	}

	printf("Single block read DMA: ");
	sd_count  = 0;
	sd_error  = 0;
	sd_crc    = 0;
	sd_amount = 0;
	tim_flag = 0;
	TIM17->CR1 |= TIM_CR1_CEN;
	while (!tim_flag) {
		j = SD_ReadBlock_DMA(0x5000000, (uint32_t *)sd_buf, 512);
		if (j == SDR_Success) {
			j = SD_CheckRead(512);
			if (j == SDR_Success) {
				sd_count++;
				sd_amount += 512;
			} else {
				printf("CR:%02X ", j);
				sd_error++;
			}
		} else {
			printf("RB:%02X ", j);
			sd_error++;
		}
	}
	if (sd_error != 0) {
		printf("success/errors count: %u/%u\r\n",
				sd_count,
				sd_error
			);
	} else {
		printf("%u bytes (%.3u MB), perf: %u blocks/s, %u bytes/s (%.3uMB/s)\r\n",
				sd_amount,
				sd_amount >> 10,
				sd_count / tim_seconds,
				sd_amount / tim_seconds,
				(uint32_t)((sd_amount / 1048576.0) * 1000.0) / tim_seconds
			);
	}

	printf("Multiple block read DMA: ");
	sd_count  = 0;
	sd_error  = 0;
	sd_crc    = 0;
	sd_amount = 0;
	tim_flag = 0;
	TIM17->CR1 |= TIM_CR1_CEN;
	while (!tim_flag) {
		j = SD_ReadBlock_DMA(0x5000000, (uint32_t *)sd_buf, buf_size);
		if (j == SDR_Success) {
			j = SD_CheckRead(buf_size);
			if (j == SDR_Success) {
				sd_count++;
				sd_amount += buf_size;
			} else {
				printf("CR:%02X ", j);
				sd_error++;
			}
		} else {
			printf("RB:%02X ", j);
			sd_error++;
		}
	}
	if (sd_error != 0) {
		printf("success/errors count: %u/%u\r\n",
				sd_count,
				sd_error
			);
	} else {
		printf("%u bytes (%.3u MB), perf: %u blocks/s, %u bytes/s (%.3uMB/s)\r\n",
				sd_amount,
				sd_amount >> 10,
				((buf_size >> 9) * sd_count) / tim_seconds,
				sd_amount / tim_seconds,
				(uint32_t)((sd_amount / 1048576.0) * 1000.0) / tim_seconds
			);
	}

#endif // Read tests


/*
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	sd_addr = 0;
	for (k = 0; k < 16; k++) {
		DWT->CYCCNT = 0;
		j = SD_WriteBlock(sd_addr, (uint32_t *)sd_buf, buf_size);
		i = DWT->CYCCNT;
		printf("WB:%02X\t\tDWT:%u\r\n", j, i);
		sd_addr += buf_size;
	}
*/

/*
	DWT->CYCCNT = 0;
	j = SD_ReadBlock(0x0, (uint32_t *)sd_buf, buf_size);
	i = DWT->CYCCNT;
	printf("RB:%02X  DWT:%u\r\n", j, i);
*/

/*
	SD_Configure_DMA((uint32_t *)sd_buf, 32768, SDIO_DMA_DIR_TX);
	j = SD_WriteBlock_DMA(0x0, (uint32_t *)sd_buf, 32768);
	if (j == SDR_Success) {
		j = SD_CheckWrite(buf_size);
		printf("CW:%02X\r\n", j);
	}
*/




#if 1
	// Perform write tests

	// Single block
	printf("Single block write POLL: ");
	memset(sd_buf, 0xCC, 512);
	sd_count  = 0;
	sd_error  = 0;
	sd_crc    = 0;
	sd_amount = 0;
	tim_flag = 0;
	TIM17->CR1 |= TIM_CR1_CEN;
	while (!tim_flag) {
		j = SD_WriteBlock(0x4FF7E00, (uint32_t *)sd_buf, 512);
		if (j == SDR_Success) {
			sd_count++;
			sd_amount += 512;
		} else {
			printf("WB:%02X ", j);
			sd_error++;
		}
	}
	if (sd_error != 0) {
		printf("success/errors count: %u/%u\r\n",
				sd_count,
				sd_error
			);
	} else {
		printf("%u bytes (%.3u MB), perf: %u blocks/s, %u bytes/s (%.3uMB/s)\r\n",
				sd_amount,
				sd_amount >> 10,
				sd_count / tim_seconds,
				sd_amount / tim_seconds,
				(uint32_t)((sd_amount / 1048576.0) * 1000.0) / tim_seconds
			);
	}

	// Multiple block
	printf("Multiple block write POLL: ");
	memset(sd_buf, 0x33, buf_size);
	sd_count  = 0;
	sd_error  = 0;
	sd_crc    = 0;
	sd_amount = 0;
	tim_flag = 0;
	TIM17->CR1 |= TIM_CR1_CEN;
	while (!tim_flag) {
		j = SD_WriteBlock(0x4FF8000, (uint32_t *)sd_buf, buf_size);
		if (j == SDR_Success) {
			sd_count++;
			sd_amount += buf_size;
		} else {
			printf("WB:%02X ", j);
			sd_error++;
		}
	}
	if (sd_error != 0) {
		printf("success/errors count: %u/%u\r\n",
				sd_count,
				sd_error
			);
	} else {
		printf("%u bytes (%.3u MB), perf: %u blocks/s, %u bytes/s (%.3uMB/s)\r\n",
				sd_amount,
				sd_amount >> 10,
				((buf_size >> 9) * sd_count) / tim_seconds,
				sd_amount / tim_seconds,
				(uint32_t)((sd_amount / 1048576.0) * 1000.0) / tim_seconds
			);
	}

/*
	// Configure the SDIO DMA channel to write data to SDIO
	SD_Configure_DMA((uint32_t *)sd_buf, buf_size, SDIO_DMA_DIR_TX);

	// Single block with DMA
	printf("Single block write DMA: ");
	memset(sd_buf, 0xCC, 512);
	sd_count  = 0;
	sd_error  = 0;
	sd_crc    = 0;
	sd_amount = 0;
	tim_flag = 0;
	TIM17->CR1 |= TIM_CR1_CEN;
	while (!tim_flag) {
		j = SD_WriteBlock_DMA(0x4FF7E00, (uint32_t *)sd_buf, 512);
		if (j == SDR_Success) {
			j = SD_CheckWrite(512);
			if (j == SDR_Success) {
				sd_count++;
				sd_amount += 512;
			} else {
				printf("CW:%02X ", j);
				sd_error++;
			}
		} else {
			printf("WB:%02X ", j);
			sd_error++;
		}
	}
	if (sd_error != 0) {
		printf("success/errors count: %u/%u\r\n",
				sd_count,
				sd_error
			);
	} else {
		printf("%u bytes (%.3u MB), perf: %u blocks/s, %u bytes/s (%.3uMB/s)\r\n",
				sd_amount,
				sd_amount >> 10,
				sd_count / tim_seconds,
				sd_amount / tim_seconds,
				(uint32_t)((sd_amount / 1048576.0) * 1000.0) / tim_seconds
			);
	}

	// Multiple block with DMA
	printf("Multiple block write DMA: ");
	memset(sd_buf, 0x33, buf_size);
	sd_count  = 0;
	sd_error  = 0;
	sd_crc    = 0;
	sd_amount = 0;
	tim_flag = 0;
	TIM17->CR1 |= TIM_CR1_CEN;
	while (!tim_flag) {
		j = SD_WriteBlock_DMA(0x4FF8000, (uint32_t *)sd_buf, buf_size);
		if (j == SDR_Success) {
			j = SD_CheckWrite(buf_size);
			if (j == SDR_Success) {
				sd_count++;
				sd_amount += buf_size;
			} else {
				printf("CW:%02X ", j);
				sd_error++;
			}
		} else {
			printf("WB:%02X ", j);
			sd_error++;
		}
	}
	if (sd_error != 0) {
		printf("success/errors count: %u/%u\r\n",
				sd_count,
				sd_error
			);
	} else {
		printf("%u bytes (%.3u MB), perf: %u blocks/s, %u bytes/s (%.3uMB/s)\r\n",
				sd_amount,
				sd_amount >> 10,
				((buf_size >> 9) * sd_count) / tim_seconds,
				sd_amount / tim_seconds,
				(uint32_t)((sd_amount / 1048576.0) * 1000.0) / tim_seconds
			);
	}
*/

#endif // Write tests


	// The main loop
	while (1) {
	}
}
