#include <stm32l1xx_rcc.h>
#include <stm32l1xx_gpio.h>
#include <stm32l1xx_syscfg.h>
#include <misc.h>

// Debug
#include <stdio.h>

// Libraries
#include "i2c.h"
#include "RTC.h"
#include "bmp180.h"
#include "bmc050.h"
#include "tsl2581.h"
#include "beeper.h"
#include "delay.h"
#include "wolk.h"
#include "sdcard-sdio.h"
#include "log.h"

// USB stuff
#include "usb_lib.h"
#include "VCP.h"

// DOSFS
#include "dosfs.h"




// GPS enable (PC1)
#define PWR_GPS_ENABLE_PORT       GPIOC
#define PWR_GPS_ENABLE_PIN        GPIO_Pin_1
#define PWR_GPS_ENABLE_PERIPH     RCC_AHBPeriph_GPIOC
#define PWR_GPS_ENABLE_H()        PWR_GPS_ENABLE_PORT->BSRRL = PWR_GPS_ENABLE_PIN
#define PWR_GPS_ENABLE_L()        PWR_GPS_ENABLE_PORT->BSRRH = PWR_GPS_ENABLE_PIN

// Vbat measure enable (PC3)
#define PWR_VBAT_ENABLE_PORT      GPIOC
#define PWR_VBAT_ENABLE_PIN       GPIO_Pin_3
#define PWR_VBAT_ENABLE_PERIPH    RCC_AHBPeriph_GPIOC
#define PWR_VBAT_ENABLE_H()       PWR_VBAT_ENABLE_PORT->BSRRL = PWR_VBAT_ENABLE_PIN
#define PWR_VBAT_ENABLE_L()       PWR_VBAT_ENABLE_PORT->BSRRH = PWR_VBAT_ENABLE_PIN

// LCD boost enable (PA4)
#define PWR_LCD_ENABLE_PORT       GPIOA
#define PWR_LCD_ENABLE_PIN        GPIO_Pin_4
#define PWR_LCD_ENABLE_PERIPH     RCC_AHBPeriph_GPIOA
#define PWR_LCD_ENABLE_H()        PWR_LCD_ENABLE_PORT->BSRRL = PWR_LCD_ENABLE_PIN
#define PWR_LCD_ENABLE_L()        PWR_LCD_ENABLE_PORT->BSRRH = PWR_LCD_ENABLE_PIN

// SD card enable (PA9)
#define PWR_SD_ENABLE_PORT        GPIOA
#define PWR_SD_ENABLE_PIN         GPIO_Pin_9
#define PWR_SD_ENABLE_PERIPH      RCC_AHBPeriph_GPIOA
#define PWR_SD_ENABLE_H()         PWR_SD_ENABLE_PORT->BSRRL = PWR_SD_ENABLE_PIN
#define PWR_SD_ENABLE_L()         PWR_SD_ENABLE_PORT->BSRRH = PWR_SD_ENABLE_PIN

// USB sense pin (PA10)
#define USB_SENS_PORT             GPIOA
#define USB_SENS_PIN              GPIO_IDR_IDR_10
#define USB_SENS_PERIPH           RCC_AHBPeriph_GPIOA
#define USB_SENS_EXTI             (1 << 10)

// Charger STAT pin (PC2)
#define CHRG_STAT_PORT            GPIOC
#define CHRG_STAT_PIN             GPIO_IDR_IDR_2
#define CHRG_STAT_PERIPH          RCC_AHBPeriph_GPIOC

// SD detect pin (PB3)
#define SD_DETECT_PORT            GPIOB
#define SD_DETECT_PIN             GPIO_IDR_IDR_3
#define SD_DETECT_PERIPH          RCC_AHBPeriph_GPIOB




GPIO_InitTypeDef PORT;
volatile uint32_t _USB_int_cntr = 0;
volatile uint32_t _USB_connected = 0;
uint32_t i,j,k;
uint8_t sector[2048];

uint16_t d0,d1;
BMP180_RESULT BR;
int16_t RT;
int32_t RP;
int16_t X,Y,Z,ACCT;

uint16_t ADC1_raws[16],Vrefint_raws[16];
uint32_t ADC1_raw,Vrefint_raw;
uint32_t Vbat,Vrefint,Vcpu;


NVIC_InitTypeDef NVICInit;




void RTC_WKUP_IRQHandler(void) {
	if (RTC->ISR & RTC_ISR_WUTF) {
		// RTC Wake-up interrupt

		// Disable sleep-on-exit (return to main loop from IRQ)
		SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;

		PWR->CR  |= PWR_CR_DBP; // Access to RTC, RTC Backup and RCC CSR registers enabled
		RTC->ISR &= ~RTC_ISR_WUTF; // Clear the RTC wake-up timer flag
		PWR->CR  &= ~PWR_CR_DBP; // Access to RTC, RTC Backup and RCC CSR registers disabled

		EXTI->PR = RTC_EXTI_LINE;
	}
}

// EXTI[10..15] lines IRQ handler
void EXTI15_10_IRQHandler(void) {
	if (EXTI->PR & USB_SENS_EXTI) {
		_USB_int_cntr++;
		_USB_connected = (USB_SENS_PORT->IDR & USB_SENS_PIN) ? 1 : 0;
		if (_USB_connected) {
			BEEPER_PlayTones(tones_USB_con);
		} else {
			BEEPER_PlayTones(tones_USB_dis);
		}
		EXTI->PR = USB_SENS_EXTI; // Clear IT bit for EXTI_Line10
	}
}

// SDIO IRQ handler
void SDIO_IRQHandler(void) {
	printf("\r\n###> IRQ SDIO [%X] CNT=%u\r\n",SDIO->STA,SDIO->FIFOCNT);

	// Clear SDIO interrupt bit
//	SDIO->ICR = 0x00FFFFFF;
	SDIO->ICR = SDIO_ICR_TXUNDERRC;
}

// DMA2 channel4 IRQ handler (SDIO DMA)
void DMA2_Channel4_IRQHandler(void) {
	uint32_t cntr;

	cntr = SDIO_DMA_CH->CNDTR;
	printf("\r\n###> IRQ DMA2 [%X] [%u]\r\n",DMA2->ISR,cntr);

	// Clear DMA interrupt bits
	DMA2->IFCR = DMA_IFCR_CGIF4 | DMA_IFCR_CHTIF4 | DMA_IFCR_CTCIF4 | DMA_IFCR_CTEIF4;
}

uint32_t InterquartileMean(uint16_t *array, uint32_t numOfSamples) {
	uint32_t sum=0;
	uint32_t index, maxindex;

	// discard the lowest and the highest data samples
	maxindex = 3 * numOfSamples / 4;
	for (index = (numOfSamples / 4); index < maxindex; index++) {
		sum += array[index];
	}
	// return the mean value of the remaining samples value

	return (sum / (numOfSamples / 2));
}




int main(void) {
	// Enable debugging when the MCU is in low power modes
//	DBGMCU->CR |= DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY;




	// Enable the system configuration controller
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;




	// Enable and configure RTC
//	RTC_Config();
//	RTC_SetWakeUp(10);




	Delay_Init(NULL);
	BEEPER_Init();




	// Enable power control lines GPIO
	RCC_AHBPeriphClockCmd(
			PWR_GPS_ENABLE_PERIPH |
			PWR_SD_ENABLE_PERIPH |
			PWR_VBAT_ENABLE_PERIPH |
			PWR_LCD_ENABLE_PERIPH,
			ENABLE); // Enable power control peripherals

	// Configure power control lines as push-pull output without pull-up
	PORT.GPIO_Mode  = GPIO_Mode_OUT;
	PORT.GPIO_Speed = GPIO_Speed_400KHz;
	PORT.GPIO_OType = GPIO_OType_PP;
	PORT.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	PORT.GPIO_Pin = PWR_GPS_ENABLE_PIN;
	GPIO_Init(PWR_GPS_ENABLE_PORT,&PORT);
	PWR_GPS_ENABLE_L(); // Turn off

	PORT.GPIO_Pin = PWR_SD_ENABLE_PIN;
	GPIO_Init(PWR_SD_ENABLE_PORT,&PORT);
	PWR_SD_ENABLE_L(); // Turn off

	PORT.GPIO_Pin = PWR_VBAT_ENABLE_PIN;
	GPIO_Init(PWR_VBAT_ENABLE_PORT,&PORT);
	PWR_VBAT_ENABLE_L(); // Turn off

	PORT.GPIO_Pin = PWR_LCD_ENABLE_PIN;
	GPIO_Init(PWR_LCD_ENABLE_PORT,&PORT);
	PWR_LCD_ENABLE_L(); // Turn off




	// Configure MCO out
	PORT.GPIO_Pin = GPIO_Pin_8;
	PORT.GPIO_Mode = GPIO_Mode_AF;
	PORT.GPIO_Speed = GPIO_Speed_40MHz;
	PORT.GPIO_OType = GPIO_OType_PP;
	PORT.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,&PORT);
	RCC_MCOConfig(RCC_MCOSource_PLLCLK,RCC_MCODiv_16);




	// Configure USB sense pin
	RCC->AHBENR |= USB_SENS_PERIPH; // Enable USB sense pin port peripheral

	// Configure GPIO pin as input with pull-up
	USB_SENS_PORT->MODER &= ~GPIO_MODER_MODER10; // Input mode (reset state)
	USB_SENS_PORT->PUPDR &= ~GPIO_PUPDR_PUPDR10; // Floating (clear bits)
//	USB_SENS_PORT->PUPDR |=  GPIO_PUPDR_PUPDR10_0; // Pull-up
	USB_SENS_PORT->PUPDR |=  GPIO_PUPDR_PUPDR10_1; // Pull-down

	// Configure the EXTI line 10 (USB sense pin)
	EXTI->PR    =  USB_SENS_EXTI; // Clear IT pending bit for EXTI
	EXTI->IMR  |=  USB_SENS_EXTI; // Enable interrupt request from EXTI
	EXTI->EMR  &= ~USB_SENS_EXTI; // Disable event on EXTI
	EXTI->RTSR |=  USB_SENS_EXTI; // Trigger rising edge enabled
	EXTI->FTSR |=  USB_SENS_EXTI; // Trigger falling edge enabled

/*
	// 2 bit for pre-emption priority, 2 bits for subpriority
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	// Enable the USB sense pin interrupt
	NVICInit.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 2;
	NVICInit.NVIC_IRQChannelSubPriority = 0;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVICInit);
*/

	NVIC_EnableIRQ(EXTI15_10_IRQn);

	_USB_connected = (USB_SENS_PORT->IDR & USB_SENS_PIN) ? 1 : 0;




	if (_USB_connected) {
//		Configure the USB peripheral
		USB_HWConfig();
//		Initialize the USB device
		USB_Init();
	}




	// Configure charger STAT pin
	RCC->AHBENR |= CHRG_STAT_PERIPH; // Enable charger STAT pin port peripheral

	// Configure GPIO pin as input with pull-up
	CHRG_STAT_PORT->MODER &= ~GPIO_MODER_MODER2; // Input mode (reset state)
	CHRG_STAT_PORT->PUPDR &= ~GPIO_PUPDR_PUPDR2; // Floating (clear bits)
	CHRG_STAT_PORT->PUPDR |=  GPIO_PUPDR_PUPDR2_0; // Pull-up




	// Configure SD detect pin
	RCC->AHBENR |= SD_DETECT_PERIPH; // Enable charger STAT pin port peripheral

	// ------------ WARNING -------------
	// PB3 mapped as JTDO after reset, it must be switched to GPIO function here!
	// ------------ WARNING -------------

	// Configure GPIO pin as input with pull-up
	SD_DETECT_PORT->MODER &= ~GPIO_MODER_MODER3; // Input mode (reset state)
	SD_DETECT_PORT->PUPDR &= ~GPIO_PUPDR_PUPDR3; // Floating (clear bits)
//	SD_DETECT_PORT->PUPDR |=  GPIO_PUPDR_PUPDR3_0; // Pull-up




	/////////////////////////////////////////////////////////////////////////////////////////////
	// To use the serial wire DP to release some GPIOs, the user software must change the GPIO //
	// (PA15, PB3 and PB4) configuration mode in the GPIO_MODER register. This releases        //
	// PA15, PB3 and PB4 which now become available as GPIOs.                                  //
	/////////////////////////////////////////////////////////////////////////////////////////////





	if (CoreDebug->DHCSR & (1 << CoreDebug_DHCSR_C_DEBUGEN_Pos)) {
		// Application executed under debugger control
		BEEPER_Enable(333,3);
		Delay_ms(500);
	} else {
		if (PWR->CSR & PWR_CSR_SBF) {
			// Device has been in STANDBY mode
			PWR->CR |= (PWR_CR_CWUF | PWR_CR_CSBF);
			BEEPER_Enable(4000,3);
			Delay_ms(5000);
		} else {
			// Device has not been in STANDBY mode
			BEEPER_Enable(1000,3);
			Delay_ms(1500);
		}
	}




	// Check Vbat measurement

	// Enable the PORTA peripheral
	RCC->AHBENR |= RCC_AHBPeriph_GPIOA;

	// Configure PA1 as analog (for ADC_IN1)
	GPIOA->MODER |=  GPIO_MODER_MODER1; // Analog mode

	// Enable Vbat divider
	PWR_VBAT_ENABLE_H(); // Turn on

	// Initialize the HSI clock
	RCC->CR |= RCC_CR_HSION; // Enable HSI
	while (!(RCC->CR & RCC_CR_HSIRDY)); // Wait until HSI stable

	// Initialize the ADC
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Enable the ADC1 peripheral clock
	ADC->CCR = ADC_CCR_TSVREFE; // Enable Vrefint and temperature sensor, ADC prescaler = HSI/1
	while(!(PWR->CSR & PWR_CSR_VREFINTRDYF)); // Wait until Vrefint stable

	// Configure the ADC
	ADC1->CR1 &= ~ADC_CR1_RES; // 12-bit resolution (Tconv = 12 ADCCLK cycles)
	ADC1->CR1 |= ADC_CR1_SCAN | ADC_CR1_PDI; // Scan mode, power down during the idle phase
	ADC1->CR2 &= ~ADC_CR2_ALIGN; // Right alignment

	// ADC channels sample rate
	ADC1->SMPR3 |= ADC_SMPR3_SMP1; // Channel 1: 384 cycles
	ADC1->SMPR2 |= ADC_SMPR2_SMP17; // Channel 17: 384 cycles

	// Injected sequence (2 conversions: first ADC_IN1, then ADC_IN17)
	ADC1->JSQR = ADC_JSQR_JL_0 | ADC_JSQR_JSQ3_0 | ADC_JSQR_JSQ4_0 | ADC_JSQR_JSQ4_4;

	// Enable the ADC
	ADC1->CR2 |= ADC_CR2_ADON;
	while (!(ADC1->SR & ADC_SR_ADONS)); // Wait until ADC is on




	// Initialize the I2C peripheral
	if (I2Cx_Init(I2C1,400000) != I2C_SUCCESS) {
		printf("I2C initialization failed\r\n");
		SleepStandby();
	}



	// Check the ALS
	TSL2581_Init();
	TSL2581_SetGain(TSL2581_GAIN8);
	d0 = TSL2581_GetData0();
	d1 = TSL2581_GetData1();
//	TSL2581_PowerOff();



	// Check the BMP180
    BMP180_Reset(); // Send reset command to BMP180
    Delay_ms(15); // Wait for BMP180 startup time (10ms by datasheet)
	if (BMP180_Check() == BMP180_SUCCESS) {
		BMP180_ReadCalibration();
		BR = BMP180_GetReadings(&RT,&RP,BMP180_ADVRES);
	}




	// Check the BMC050
	BMC050_ACC_SoftReset();
	Delay_ms(5); // must wait for start-up time of accelerometer (2ms)
	BMC050_Init();

	// Enable I2C watchdog timer with 50ms
	BMC050_ACC_InterfaceConfig(ACC_IF_WDT_50ms);

	BMC050_ACC_SetBandwidth(ACC_BW8); // Accelerometer readings filtering (lower or higher better?)
	BMC050_ACC_SetIRQMode(ACC_IM_NOLATCH); // No IRQ latching
	BMC050_ACC_ConfigSlopeIRQ(0,8); // Motion detection sensitivity
	BMC050_ACC_IntPinMap(ACC_IM1_SLOPE); // Map slope interrupt to INT1 pin
	BMC050_ACC_SetIRQ(ACC_IE_SLOPEX | ACC_IE_SLOPEY | ACC_IE_SLOPEZ); // Detect motion by all axes
	BMC050_ACC_LowPower(ACC_SLEEP_1000); // Low power with sleep duration 1s
//	BMC050_ACC_LowPower(ACC_SLEEP_1); // Low power with sleep duration 1ms

//	BMC050_ACC_Suspend();




	PWR_SD_ENABLE_H(); // Turn SD on

	// Enable the DMA2 peripheral clock
	RCC->AHBENR |= RCC_AHBENR_DMA2EN;

	// Enable the DMA2 channel4 interrupt
//	NVIC_EnableIRQ(DMA2_Channel4_IRQn);

	// Enable the SDIO interrupt
//	NVIC_EnableIRQ(SDIO_IRQn);

	SD_SDIO_GPIO_Init();
	j = SD_Init();
	printf("SD_Init: %X\r\n",j);

	if (j == SDR_Success) {
		if (SDCard.Type != SDCT_MMC) {
			// MMC doesn't support 4-bit bus
			if (SDCard.SCR[1] & 0x05) {
				// Set 4-bit bus width
				SD_SetBusWidth(SD_BUS_4BIT);
			}
		}

///*
		// DMA read block test
		for (i = 0; i < 2048; i++) sector[i] = '#';
		i = 2048; // block size
		j = SD_ReadBlock_DMA(0,(uint32_t *)sector,i);
		printf("SD_ReadBlock_DMA = %X\r\n",j);
		if (j == SDR_Success) {
			// Wait till data transfered by DMA
			j = SD_CheckRead(i);
			printf("SD_CheckRead = %X\r\n",j);
			if (j == SDR_Success) VCP_SendBufPrintable(sector,i,'.');
			printf("\r\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\r\n");
		}
//*/

///*
		SD_SetBusWidth(SD_BUS_1BIT);
		// DMA write block test
		for (i = 0; i < 512; i++) {
			sector[i] = sector[i + 1024] + 1;
			if (sector[i] < 33) sector[i] = 127;
			if (sector[i] > 127) sector[i] = 33;
		}
		j = SD_WriteBlock_DMA(1024,(uint32_t *)sector,512);
		printf("SD_WriteBlock_DMA = %X\r\n",j);
		if (j == SDR_Success) {
			// Wait till data transfered by DMA
			j = SD_CheckWrite(512);
			printf("SD_CheckWrite = %X\r\n",j);
		}
		printf("---------------------------------------------\r\n");
//*/

///*
		SD_SetBusWidth(SD_BUS_4BIT);
		// DMA write block test
		for (i = 0; i < 512; i++) {
			sector[i] = sector[i + 1536] + 1;
			if (sector[i] < 33) sector[i] = 127;
			if (sector[i] > 127) sector[i] = 33;
		}
		j = SD_WriteBlock_DMA(1536,(uint32_t *)sector,512);
		printf("SD_WriteBlock_DMA = %X\r\n",j);
		if (j == SDR_Success) {
			// Wait till data transfered by DMA
			j = SD_CheckWrite(512);
			printf("SD_CheckWrite = %X\r\n",j);
		}
		printf("---------------------------------------------\r\n");
//*/

/*
		// Write block test
		for (i = 0; i < 512; i++) sector[i] = '~';
		j = SD_WriteBlock(1024,(uint32_t *)sector,512);
		printf("Write block: %2X\r\n",j);
*/

///*
		// DMA read block test
		for (i = 0; i < 2048; i++) sector[i] = '#';
		i = 2048; // block size
		j = SD_ReadBlock_DMA(0,(uint32_t *)sector,i);
		printf("SD_ReadBlock_DMA = %X\r\n",j);
		if (j == SDR_Success) {
			// Wait till data transfered by DMA
			j = SD_CheckRead(i);
			printf("SD_CheckRead = %X\r\n",j);
			if (j == SDR_Success) VCP_SendBufPrintable(sector,i,'.');
			printf("\r\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\r\n");
		}
//*/

/*
		// DMA read speed test
		d0 = 0; d1 = 0; k = 0; i = 0;
		printf("Started------>\r\n");
		do {
			j = SD_ReadBlock_DMA(0,(uint32_t *)sector,512);
			if (j == SDR_Success) {
				j = SD_CheckRead(512);
				if (j != SDR_Success) d0++;
			} else d1++;
			k += 512;
			i++;
		} while (k < 10485760); // 10MB read
//		} while (k < 104857600); // 100MB read
		printf("<--------Ended\r\n");
		printf("Failed: %u[%u] from %u\r\n",d0,d1,i);
		printf("---------------------------------\r\n");
*/

		BEEPER_PlayTones(tones_USB_con);
//		while(1);

/*
		// Write block test
		for (i = 0; i < 1024; i++) sector[i] = '%';
		j = SD_WriteBlock(1024,(uint32_t *)sector,1024);
		for (i = 0; i < 2048; i++) sector[i] = '#';
		printf("Write block: %2X\r\n",j);
*/

/*
		// Read block speed test
		d0 = 0; k = 0; i = 0;
		printf("Started------>\r\n");
		do {
			j = SD_ReadBlock(0,(uint32_t *)sector,512);
			if (j != SDR_Success) d0++;
			k += 512;
			i++;
		} while (k < 10485760);
		printf("<--------Ended\r\n");
		printf("Failed: %u from %u\r\n",d0,i);
		printf("---------------------------------\r\n");
*/




/*
		// Log file create test
		j = LOG_Init();
		printf("LOG_Init: #%u\r\n",j);
		if (j == LOG_OK) {
			i = 0;
			j = LOG_NewFile(&i);

			printf("LOG_NewFile: %X [#%u]\r\n",(unsigned int)j,i);

			if (j == LOG_OK) {
				LOG_WriteStr("WBC!\r\n");
				LOG_WriteStr("Shoop da whoop!\r\n");
				LOG_WriteStr("!WBC\r\n");
				LOG_FileSync();
			}
		}
*/



		// DOSFS test
		uint32_t pstart, psize, cfree;
		uint8_t pactive, ptype;
		VOLINFO vi;
		DIRINFO di;
		DIRENT de;
		uint32_t total_space;
		uint32_t free_space;
		uint32_t used_space;

		// Find partition start
		pstart = DFS_GetPtnStart(0,sector,0,&pactive,&ptype,&psize);
		if (pstart == DFS_ERRMISC) {
			printf("Cannot find first partition.\r\n");
		} else {
			printf("Partition start: %X [active: %u, type: %u]\r\n",pstart,pactive,ptype);
			printf("Size: %u sectors\r\n",psize);
		}

		if (DFS_GetVolInfo(0,sector,pstart,&vi)) {
			printf("Error getting volume information.\r\n");
		} else {
			printf("Volume label: \"%s\"\r\n",vi.label);
			printf("File system: ");
			if (vi.filesystem == FAT12)
				printf("FAT12\r\n");
			else if (vi.filesystem == FAT16)
				printf("FAT16\r\n");
			else if (vi.filesystem == FAT32)
				printf("FAT32\r\n");
			else
				printf("[unknown]\r\n");
			printf("Sector : %u bytes\r\n",vi.sectorsize);
			printf("Cluster: %u sectors [%u bytes]\r\n",vi.secperclus,vi.clustersize);
			printf("Volume : %u clusters [%u sectors]\r\n",vi.numclusters,vi.numsecs);

			i = DFS_GetFree(&vi,sector,&cfree);
			printf("Free   : %u clusters\r\n",cfree);

			total_space = vi.numclusters * vi.clustersize;
			free_space  = cfree * vi.clustersize;
			used_space  = (vi.numclusters - cfree) * vi.clustersize;

			printf("Used space: %10ub [%uMB]\r\n",used_space,used_space / 1048576);
			printf("Free space: %10ub [%uMB]\r\n",free_space,free_space / 1048576);
			printf("Capacity  : %10ub [%uMB]\r\n",total_space,total_space / 1048576);

			di.scratch = sector;
			if (DFS_OpenDir(&vi,(uint8_t *)"",&di)) {
				printf("Error opening root directory.\r\n");
			} else {
				while (!DFS_GetNext(&vi,&di,&de)) {
					if (de.name[0]) {
						// Warning: there is no zero terminator in name
						printf("file: \"%s\" [%02X] %10u\r\n",de.name,(unsigned int)de.attr,(de.filesize_3 << 24) | (de.filesize_2 << 16) | (de.filesize_1 << 8) | de.filesize_0);
					}
				}
			}

			if (DFS_OpenDir(&vi,(uint8_t *)LOG_DIR_LOGS,&di)) {
				printf("Error opening \".\\%s\" directory.\r\n",LOG_DIR_LOGS);
			} else {
				while (!DFS_GetNext(&vi,&di,&de)) {
					if (de.name[0]) {
						// Warning: there is no zero terminator in name
						printf("file: \"%s\" [%02X] %10u\r\n",de.name,de.attr,(de.filesize_3 << 24) | (de.filesize_2 << 16) | (de.filesize_1 << 8) | de.filesize_0);
					}
				}
			}
		}

		// END OF SD CARD THESTS
	}




	printf("-----------------------\r\n");




	BEEPER_PlayTones(tones_USB_dis);




	while(1) {
		// Measure Vbat
/*
		ADC1_raw = 0;
		Vrefint_raw = 0;
		for (i = 0; i < 16; i++) {
			ADC1->CR2 |= ADC_CR2_JSWSTART; // Start conversion of injected channels
			while (!(ADC1->SR & ADC_SR_JEOC)); // Wait until ADC conversions end
			ADC1_raw    += ADC1->JDR1; // Read injected data register1 (ADC_IN1)
			Vrefint_raw += ADC1->JDR2; // Read injected data register2 (ADC_IN17)
			if (i) {
				ADC1_raw >>= 1;
				Vrefint_raw >>= 1;
			}
			ADC1->SR &= ~ADC_SR_JEOC; // Clear JEOC bit (is this necessary?)
		}
*/

		for (i = 0; i < 16; i++) {
			ADC1->CR2 |= ADC_CR2_JSWSTART; // Start conversion of injected channels
			while (!(ADC1->SR & ADC_SR_JEOC)); // Wait until ADC conversions end
			ADC1_raws[i]    = ADC1->JDR1; // Read injected data register1 (ADC_IN1)
			Vrefint_raws[i] = ADC1->JDR2; // Read injected data register2 (ADC_IN17)
		}
		ADC1_raw = InterquartileMean(ADC1_raws,16);
		Vrefint_raw = InterquartileMean(Vrefint_raws,16);

/*
		// Calculate rough average of ADC readings
		Vrefint_raw = Vrefint_raws[0];
		ADC1_raw = ADC1_raws[0];
		for (i = 1; i < 16; i++) {
			Vrefint_raw += Vrefint_raws[i];
			ADC1_raw += ADC1_raws[i];
			Vrefint_raw >>= 1;
			ADC1_raw >>= 1;
		}
*/

		Vbat = (uint16_t)(((*VREFINT_CAL * ADC1_raw * 3.0)/(Vrefint_raw * 4095.0)) * 1000);
		Vrefint = (uint16_t)(((Vrefint_raw * 3.0) / 4095.0) * 1000);
		Vcpu = (uint16_t)(((*VREFINT_CAL * 3.0) / Vrefint_raw) * 1000);

		printf("VOL: Bat=[%u.%03uV -> %u.%03uV] Ref=%u.%03uV CPU=%u.%03uV\r\n",
				(unsigned int)Vbat / 1000, (unsigned int)Vbat % 1000,
				((unsigned int)Vbat / 1000) * 2, ((unsigned int)Vbat % 1000) * 2,
				(unsigned int)Vrefint / 1000, (unsigned int)Vrefint % 1000,
				(unsigned int)Vcpu / 1000, (unsigned int)Vcpu % 1000);

		// Get ALS readings
		d0 = TSL2581_GetData0();
		d1 = TSL2581_GetData1();
		printf("ALS: D0=%u D1=%u\r\n", (unsigned int)d0, (unsigned int)d1);

		// Get accelerometer readings
		BMC050_ACC_GetXYZ(&X,&Y,&Z);
		ACCT = BMC050_ReadTemp();
		printf("ACC: X=%d Y=%d Z=%d T=%d.%dC\r\n", X, Y, Z, ACCT / 10, ACCT % 10);

		// Get barometer readings
		BR = BMP180_GetReadings(&RT,&RP,BMP180_ADVRES);
		if (BR == BMP180_SUCCESS) {
			RP = BMP180_hPa_to_mmHg(RP);
			printf("BAR: T=%d.%dC P=%u.%ummHg\r\n", RT / 10, RT % 10, (unsigned int)RP / 10, (unsigned int)RP % 10);
		} else {
			printf("BAR: failed\r\n");
		}

		// Get USB sense pin state
		printf("USB: %s (%d)\r\n",_USB_connected ? "connected" : "disconnected",(unsigned int)_USB_int_cntr);

		// Get SD detect pin state
		printf("SDC: %s\r\n",(SD_DETECT_PORT->IDR & SD_DETECT_PIN) ? "no card" : "present");

		// Get charger STAT pin state
		i = 0;
		CHRG_STAT_PORT->PUPDR &= ~GPIO_PUPDR_PUPDR2; // Floating (clear bits)
		Delay_ms(100);
		printf("CHG: NONE=%d ",(unsigned int)(CHRG_STAT_PORT->IDR & CHRG_STAT_PIN));
		if (CHRG_STAT_PORT->IDR & CHRG_STAT_PIN) i++;

		CHRG_STAT_PORT->PUPDR &= ~GPIO_PUPDR_PUPDR2; // Floating (clear bits)
		CHRG_STAT_PORT->PUPDR |=  GPIO_PUPDR_PUPDR2_1; // Pull-down
		Delay_ms(100);
		printf("PD=%d ",(unsigned int)(CHRG_STAT_PORT->IDR & CHRG_STAT_PIN));
		if (CHRG_STAT_PORT->IDR & CHRG_STAT_PIN) i++;

		CHRG_STAT_PORT->PUPDR &= ~GPIO_PUPDR_PUPDR2; // Floating (clear bits)
		CHRG_STAT_PORT->PUPDR |=  GPIO_PUPDR_PUPDR2_0; // Pull-up
		Delay_ms(100);
		printf("PU=%d\r\n",(unsigned int)(CHRG_STAT_PORT->IDR & CHRG_STAT_PIN));

		CHRG_STAT_PORT->PUPDR &= ~GPIO_PUPDR_PUPDR2; // Floating (clear bits)

		j = SD_ReadBlock(0,(uint32_t *)sector,2048);
		printf("SD_ReadBlock = %2X\r\n",j);

		printf("------------------\r\n");

		Delay_ms(2000);
	}




	PWR->CSR |= PWR_CSR_EWUP1; // Enable WKUP pin 1 (PA0)
	PWR->CSR |= PWR_CSR_EWUP2; // Enable WKUP pin 2 (PC13)
	SleepStandby();




	while(1);
}
