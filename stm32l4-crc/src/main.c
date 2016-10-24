#include "main.h"


// Sample data for CRC calculation
uint8_t data_buf[] = {
		0x54, 0x68, 0x69, 0x73, 0x20, 0x69, 0x73, 0x20,
		0x61, 0x20, 0x74, 0x65, 0x73, 0x74, 0x20, 0x6f,
		0x66, 0x20, 0x74, 0x68, 0x65, 0x20, 0x43, 0x52,
		0x43, 0x20, 0x70, 0x65, 0x72, 0x69, 0x70, 0x68,
		0x65, 0x72, 0x61, 0x6c, 0x2e
};
uint32_t buf_size = sizeof(data_buf);


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
	printf("CRC peripheral (%s @ %s)\r\n",__DATE__,__TIME__);
	printf("CPU: %.3uMHz\r\n",SystemCoreClock / 1000);
	printf("SYSCLK=%.3uMHz, HCLK=%.3uMHz\r\n",Clocks.SYSCLK_Frequency / 1000,Clocks.HCLK_Frequency / 1000);
	printf("APB1=%.3uMHz, APB2=%.3uMHz\r\n",Clocks.PCLK1_Frequency / 1000,Clocks.PCLK2_Frequency / 1000);
	printf("System clock: %s\r\n",_sysclk_src_str[RCC_GetSysClockSource()]);


	// Enable the CRC peripheral
	CRC_Enable();

	// Disable reverse for input and output data
	CRC_SetInRevMode(CRC_IN_NORMAL);
	CRC_SetOutRevMode(CRC_OUT_NORMAL);


	// First column in output is the calculated CRC, second - reference value

	// 8-bit CRC tests
	CRC_SetPolynomialSize(CRC_PSIZE_8B);
	printf("CRC-8 tests:\r\n");

	// CCITT CRC-8 (Poly=0x07, Init=0x00, RefIn=false, RefOut=false)
	CRC_SetPolynomial(0x07);
	CRC_SetInitValue(0x00);
	CRC_Reset();
	CRC_CalcBuffer((uint32_t *)data_buf, buf_size);
	printf("CCITT    : 0x%02X . 0x%02X\r\n", CRC_GetData8(), 0x34);

	// CDMA2000 CRC-8 (Poly=0x9B, Init=0xFF, RefIn=false, RefOut=false)
	CRC_SetPolynomial(0x9B);
	CRC_SetInitValue(0xFF);
	CRC_Reset();
	CRC_CalcBuffer((uint32_t *)data_buf, buf_size);
	printf("CDMA2000 : 0x%02X . 0x%02X\r\n", CRC_GetData8(), 0xCE);

	// WCDMA CRC-8 (Poly=0x9B, Init=0x00, RefIn=true, RefOut=true)
	CRC_SetInitValue(0x00);
	CRC_SetInRevMode(CRC_IN_REV_BYTE);
	CRC_SetOutRevMode(CRC_OUT_REVERSED);
	CRC_Reset();
	CRC_CalcBuffer((uint32_t *)data_buf, buf_size);
	printf("WCDMA    : 0x%02X . 0x%02X\r\n", CRC_GetData8(), 0xEA);

	// I-CODE CRC-8 (Poly=0x1D, Init=0xFD, RefIn=false, RefOut=false)
	CRC_SetPolynomial(0x1D);
	CRC_SetInitValue(0xFD);
	CRC_SetInRevMode(CRC_IN_NORMAL);
	CRC_SetOutRevMode(CRC_OUT_NORMAL);
	CRC_Reset();
	CRC_CalcBuffer((uint32_t *)data_buf, buf_size);
	printf("I-CODE   : 0x%02X . 0x%02X\r\n", CRC_GetData8(), 0x87);

	// ROHC CRC-8 (Poly=0x07, Init=0xFF, RefIn=true, RefOut=true)
	CRC_SetPolynomial(0x07);
	CRC_SetInitValue(0xFF);
	CRC_SetInRevMode(CRC_IN_REV_BYTE);
	CRC_SetOutRevMode(CRC_OUT_REVERSED);
	CRC_Reset();
	CRC_CalcBuffer((uint32_t *)data_buf, buf_size);
	printf("ROHC     : 0x%02X . 0x%02X\r\n", CRC_GetData8(), 0x1A);


	// 16-bit CRC tests
	CRC_SetPolynomialSize(CRC_PSIZE_16B);
	printf("CRC-16 tests:\r\n");

	// MODBUS (Poly=0x8005, Init=0xFFFF, RefIn=true, RefOut=true)
	CRC_SetPolynomial(0x8005);
	CRC_SetInitValue(0xFFFF);
	CRC_SetInRevMode(CRC_IN_REV_BYTE);
	CRC_SetOutRevMode(CRC_OUT_REVERSED);
	CRC_Reset();
	CRC_CalcBuffer((uint32_t *)data_buf, buf_size);
	printf("MODBUS   : 0x%04X . 0x%04X\r\n", CRC_GetData16(), 0x3BD1);

	// CDMA2000 (Poly=0xC867, Init=0xFFFF, RefIn=false, RefOut=false)
	CRC_SetPolynomial(0xC867);
	CRC_SetInRevMode(CRC_IN_NORMAL);
	CRC_SetOutRevMode(CRC_OUT_NORMAL);
	CRC_Reset();
	CRC_CalcBuffer((uint32_t *)data_buf, buf_size);
	printf("CDMA2000 : 0x%04X . 0x%04X\r\n", CRC_GetData16(), 0x52F7);

	// XMODEM (Poly=0x1021, Init=0x0000, RefIn=false, RefOut=false)
	CRC_SetPolynomial(0x1021);
	CRC_SetInitValue(0x0000);
	CRC_Reset();
	CRC_CalcBuffer((uint32_t *)data_buf, buf_size);
	printf("XMODEM   : 0x%04X . 0x%04X\r\n", CRC_GetData16(), 0x047B);

	// USB (Poly=0x8005, Init=0xFFFF, RefIn=true, RefOut=true, XorOut=0xFFFF)
	CRC_SetPolynomial(0x8005);
	CRC_SetInitValue(0xFFFF);
	CRC_SetInRevMode(CRC_IN_REV_BYTE);
	CRC_SetOutRevMode(CRC_OUT_REVERSED);
	CRC_Reset();
	CRC_CalcBuffer((uint32_t *)data_buf, buf_size);
	printf("USB      : 0x%04X . 0x%04X\r\n", CRC_GetData16() ^ 0xFFFF, 0xC42E);


	// 32-bit CRC tests
	CRC_SetPolynomialSize(CRC_PSIZE_32B);
	printf("CRC-32 tests:\r\n");

	// MPEG-2 (Poly=0x04C11DB7, Init=0xFFFFFFFF, RefIn=true, RefOut=true)
	CRC_SetPolynomial(0x04C11DB7);
	CRC_SetInitValue(0xFFFFFFFF);
	CRC_SetInRevMode(CRC_IN_NORMAL);
	CRC_SetOutRevMode(CRC_OUT_NORMAL);
	CRC_Reset();
	CRC_CalcBuffer((uint32_t *)data_buf, buf_size);
	printf("MPEG-2   : 0x%08X . 0x%08X\r\n", CRC_GetData32(), 0xF5C62CB4);

	// POSIX (Poly=0x04C11DB7, Init=0x00000000, RefIn=false, RefOut=false, XorOut=0xFFFFFFFF)
	CRC_SetPolynomial(0x04C11DB7);
	CRC_SetInitValue(0x00000000);
	CRC_Reset();
	CRC_CalcBuffer((uint32_t *)data_buf, buf_size);
	printf("POSIX    : 0x%08X . 0x%08X\r\n", CRC_GetData32() ^ 0xFFFFFFFF, 0xD338A790);

	// CRC-32D (Poly=0xA833982B, Init=0xFFFFFFFF, RefIn=true, RefOut=true, XorOut=0xFFFFFFFF)
	CRC_SetPolynomial(0xA833982B);
	CRC_SetInitValue(0xFFFFFFFF);
	CRC_SetInRevMode(CRC_IN_REV_BYTE);
	CRC_SetOutRevMode(CRC_OUT_REVERSED);
	CRC_Reset();
	CRC_CalcBuffer((uint32_t *)data_buf, buf_size);
	printf("CRC-32D  : 0x%08X . 0x%08X\r\n", CRC_GetData32() ^ 0xFFFFFFFF, 0xD2178F40);

	// XFER (Poly=0x000000AF, Init=0x00000000, RefIn=false, RefOut=false)
	CRC_SetPolynomial(0x000000AF);
	CRC_SetInitValue(0x00000000);
	CRC_SetInRevMode(CRC_IN_NORMAL);
	CRC_SetOutRevMode(CRC_OUT_NORMAL);
	CRC_Reset();
	CRC_CalcBuffer((uint32_t *)data_buf, buf_size);
	printf("XFER     : 0x%08X . 0x%08X\r\n", CRC_GetData32(), 0x5D4143CF);


	// The main loop
	while (1) {
	}
}
