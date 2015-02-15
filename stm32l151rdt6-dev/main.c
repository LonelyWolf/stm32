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
#include "spi.h"
#include "nRF24.h"

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
#define USB_SENS_EXTI_N           EXTI15_10_IRQn

// Charger STAT pin (PC2)
#define CHRG_STAT_PORT            GPIOC
#define CHRG_STAT_PIN             GPIO_IDR_IDR_2
#define CHRG_STAT_PERIPH          RCC_AHBPeriph_GPIOC

// SD detect pin (PB3)
#define SD_DETECT_PORT            GPIOB
#define SD_DETECT_PIN             GPIO_IDR_IDR_3
#define SD_DETECT_PERIPH          RCC_AHBPeriph_GPIOB
#define SD_DETECT_EXTI            (1 << 3)
#define SD_DETECT_EXTI_N          EXTI3_IRQn

// BMC050 IRQ pin (PC13)
#define ACC_IRQ_PORT              GPIOC
#define ACC_IRQ_PIN               GPIO_Pin_13
#define ACC_IRQ_PERIPH            RCC_AHBPeriph_GPIOC
#define ACC_IRQ_EXTI              (1 << 13)
#define ACC_IRQ_EXTI_N            EXTI15_10_IRQn




// Reset source
#define RESET_SRC_UNKNOWN         (uint32_t)0x00000000 // Unknown reset source
#define RESET_SRC_POR             (uint32_t)0x00000001 // POR/PDR (Power On Reset or Power Down Reset)
#define RESET_SRC_SOFT            (uint32_t)0x00000002 // Software reset
#define RESET_SRC_PIN             (uint32_t)0x00000004 // Reset from NRST pin
#define RESET_SRC_STBY            (uint32_t)0x00000008 // Reset after STANBY, wake-up flag set (RTC or WKUP# pins)
#define RESET_SRC_STBY_WP1        (uint32_t)0x00000010 // Reset after STANBY, wake-up from WKUP1 pin
#define RESET_SRC_STBY_WP2        (uint32_t)0x00000020 // Reset after STANBY, wake-up from WKUP2 pin




// It's obvious!
uint32_t i,j,k;

// Reset source
uint32_t _reset_source = RESET_SRC_UNKNOWN;

// USB
volatile uint32_t _USB_int_cntr = 0;
volatile uint32_t _USB_connected = 0;

// SD
volatile uint32_t _SD_int_cntr = 0;
volatile uint32_t _SD_connected = 0;
volatile uint32_t _SD_last_state = 0;
uint8_t __attribute__((aligned(4))) sector[2048];

// ALS variables
uint16_t d0,d1;

// BMP180 variables
BMP180_RESULT BR;
int16_t RT;
int32_t RP;
int16_t X,Y,Z,ACCT;

// ADC variables
uint16_t ADC1_raws[16],Vrefint_raws[16];
uint32_t ADC1_raw,Vrefint_raw;
uint32_t Vbat,Vrefint,Vcpu;

// nRF24 variables
//#define NRF24_SOLAR // if defined - receive solar powered temperature sensor, WBC otherwise
#ifdef NRF24_SOLAR
// Yes, this is redefinition from wolk.h (must be warn from compiler for the following lines)
#define nRF24_RX_Addr               "WolkT" // RX address for nRF24L01
#define nRF24_RX_Addr_Size                5 // RX address size
#define nRF24_RF_CHANNEL                110 // nRF24L01 channel (110CH = 2510MHz)
#define nRF24_RX_PAYLOAD                 18 // nRF24L01 payload length
uint8_t nRF24_RX_Buf[nRF24_RX_PAYLOAD]; // nRF24L01 payload buffer
bool _logging = FALSE;
#else
uint8_t nRF24_RX_Buf[nRF24_RX_PAYLOAD]; // nRF24L01 payload buffer for WBC
#endif
volatile bool _new_packet = FALSE;
uint32_t _packets_rcvd = 0; // Received packets counter

// Date/Time
RTC_TimeTypeDef _time;
RTC_DateTypeDef _date;

// SPL
GPIO_InitTypeDef PORT;
NVIC_InitTypeDef NVICInit;




// Determine source of reset
uint32_t GetResetSource(void) {
	uint32_t result = RESET_SRC_UNKNOWN;
	uint32_t reg;

	// Reset source
	reg = RCC->CSR;
	if (reg & RCC_CSR_PORRSTF) result |= RESET_SRC_POR;
	if (reg & RCC_CSR_SFTRSTF) result |= RESET_SRC_SOFT;
	if (reg & RCC_CSR_PINRSTF) result |= RESET_SRC_PIN;

	// Clear the reset flags
	RCC->CSR |= RCC_CSR_RMVF;

	// Enable the PWR peripheral to deal with it CSR register
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;

	// WKUP pin or RTC alarm
	reg = PWR->CSR;
	if (reg & PWR_CSR_WUF) {
		if (reg & PWR_CSR_SBF) result |= RESET_SRC_STBY;

		// Clear the wake-up and standby flags
		PWR->CR |= PWR_CR_CWUF | PWR_CR_CSBF;

		// Remember value of the AHBENR register
		reg = RCC->AHBENR;

		// Determine state of the WKUP# pins
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN;
		if (GPIOA->IDR & GPIO_IDR_IDR_0)  result |= RESET_SRC_STBY_WP1;
		if (GPIOC->IDR & GPIO_IDR_IDR_13) result |= RESET_SRC_STBY_WP2;

		// Restore value of the AHBENR register
		RCC->AHBENR = reg;
	}

	return result;
}

// RTC wake-up IRQ handler
void RTC_WKUP_IRQHandler(void) {
	if (RTC->ISR & RTC_ISR_WUTF) {
		// RTC Wake-up interrupt

		// Get current date/time
		RTC_GetDateTime(&_time,&_date);

		// Disable sleep-on-exit (return to main loop from IRQ)
		SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;

		PWR->CR  |= PWR_CR_DBP; // Access to RTC, RTC Backup and RCC CSR registers enabled
		RTC->ISR &= ~RTC_ISR_WUTF; // Clear the RTC wake-up timer flag
		PWR->CR  &= ~PWR_CR_DBP; // Access to RTC, RTC Backup and RCC CSR registers disabled

		EXTI->PR = RTC_EXTI_LINE;
	}
}

// EXTI1 line IRQ handler
void EXTI1_IRQHandler(void) {
	nRF24_RX_PCKT_TypeDef RX_status;

	if (EXTI->PR & nRF24_IRQ_EXTI) {
		RX_status = nRF24_RXPacket(nRF24_RX_Buf,nRF24_RX_PAYLOAD);
		if (RX_status == nRF24_RX_PCKT_PIPE0) {
			_packets_rcvd++;
			_new_packet = TRUE;
		}

//		BEEPER_Enable(700,2);

		// Disable sleep-on-exit (return to main loop from IRQ)
		SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;

		// Clear IT bit for EXTI_Line
		EXTI->PR = nRF24_IRQ_EXTI;
	}
}

// EXTI3 line IRQ handler
void EXTI3_IRQHandler(void) {
	if (EXTI->PR & SD_DETECT_EXTI) {
		_SD_int_cntr++;
		_SD_connected = (SD_DETECT_PORT->IDR & SD_DETECT_PIN) ? 0 : 1;
		if (_SD_connected) {
			BEEPER_PlayTones(tones_USB_con);
		} else {
			BEEPER_PlayTones(tones_USB_dis);
		}

		// Clear IT bit for EXTI_Line
		EXTI->PR = SD_DETECT_EXTI;
	}
}

// EXTI[10..15] lines IRQ handler
void EXTI15_10_IRQHandler(void) {
	uint16_t acc_irq;

	if (EXTI->PR & USB_SENS_EXTI) {
		// USB sense pin
		_USB_int_cntr++;
		_USB_connected = (USB_SENS_PORT->IDR & USB_SENS_PIN) ? 1 : 0;
		if (_USB_connected) {
			BEEPER_PlayTones(tones_USB_con);
		} else {
			BEEPER_PlayTones(tones_USB_dis);
		}
		EXTI->PR = USB_SENS_EXTI; // Clear IT bit for EXTI_Line
	}

	if (EXTI->PR & ACC_IRQ_EXTI) {
		// Accelerometer IRQ pin
		acc_irq = BMC050_ACC_GetIRQStatus();
		if (acc_irq & ACC_IRQ_SLOPE) {
			// Slope IRQ high
			BEEPER_Enable(444,1);
		} else {
			// Slope IRQ low
			BEEPER_Enable(3333,1);
		}
		printf("ACC: %s [%04X]\r\n",(ACC_IRQ_PORT->IDR & ACC_IRQ_PIN) ? "HIGH" : "LOW",acc_irq);
		EXTI->PR = ACC_IRQ_EXTI; // Clear IT bit for EXTI_line
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

// Compute interquartile mean from array of values
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
	// Determine wake-up source
	_reset_source = GetResetSource();




	// Just for fun...
	// If wake-up was from RTC then do BEEP and go back STANDBY mode
	// If wake-up was from WKUP2 pin (accelerometer IRQ) then do BEEP, wait while IRQ asserted
	//                     do another BEEP and go back STANBY mode
	// If wake-up was from WKUP1 pin (ESCAPE button), then skip this and go to normal operation
	if ((_reset_source & RESET_SRC_STBY) && !(_reset_source & RESET_SRC_STBY_WP1)) {
		// At this time the MSI clock is used as system clock (~2.097MHz)
		// Rise it up to ~4.194MHz to do proper beep
		i = (RCC->ICSCR & ~RCC_ICSCR_MSIRANGE); // Clear MSIRANGE bits
		RCC->ICSCR = i | RCC_ICSCR_MSIRANGE_6; // Set MSI range 6 (around 4.194MHz)

		// Compute the actual MCU clock speed (for proper BEEPER initialization)
		SystemCoreClockUpdate();

		// Initialize BEEPER and do BEEP to indicate what MCU is awake
//		BEEPER_Init();

		if (_reset_source & RESET_SRC_STBY_WP2) {
			// Initialize BEEPER and do BEEP to indicate what MCU is awake
			BEEPER_Init();

			// This is wake-up from accelerometer IRQ pin
			BEEPER_Enable(2222,2);

			// Enable the accelerometer IRQ GPIO pin and wait until it becomes low
			RCC->AHBENR |= ACC_IRQ_PERIPH; // Enable GPIO port peripheral
			ACC_IRQ_PORT->MODER &= ~GPIO_MODER_MODER13; // Input mode (reset state)
			ACC_IRQ_PORT->PUPDR &= ~GPIO_PUPDR_PUPDR13; // Floating (clear bits)
			while (ACC_IRQ_PORT->IDR & ACC_IRQ_PIN);

			// Do another BEEP and wait till it ends
			BEEPER_Enable(444,1);
			while (_beep_duration);
		} else {
			// Clear the RTC wake-up timer flag
			// The PWR peripheral must be enabled (in GetResetSource)
			PWR->CR  |= PWR_CR_DBP; // Access to RTC, RTC Backup and RCC CSR registers enabled
			RTC->ISR &= ~RTC_ISR_WUTF;
			PWR->CR  &= ~PWR_CR_DBP; // Access to RTC, RTC Backup and RCC CSR registers disabled

			// This is wake-up from RTC, do BEEP and wait until it ends
//			BEEPER_Enable(1111,1);
//			while (_beep_duration);
		}

		// Put MCU into STANDBY mode
		PWR->CSR |= PWR_CSR_EWUP1; // Enable WKUP pin 1 (PA0)
		PWR->CSR |= PWR_CSR_EWUP2; // Enable WKUP pin 2 (PC13)
		SleepStandby();
	}




	// Setup the microcontroller system
	SystemInit();




	// Enable debugging when the MCU is in low power modes
	DBGMCU->CR |= DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY;




/*
	// NVIC: 2 bit for pre-emption priority, 2 bits for subpriority
	// WARNING: this stuff will be re-setup in USB init routine
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
*/




	// Enable the system configuration controller
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;




	// Enable and configure RTC
	RTC_Config();
	if (_reset_source & RESET_SRC_POR) {
		// Set time only in case of POR/PDR (power on/power down reset)
		_time.RTC_Hours   = 0;
		_time.RTC_Minutes = 0;
		_time.RTC_Seconds = 0;
		_date.RTC_Date    = 13;
		_date.RTC_Month   = 02;
		_date.RTC_Year    = 15;
		_date.RTC_WeekDay = 5;
		RTC_SetDateTime(&_time,&_date);
	}
	RTC_GetDateTime(&RTC_Time,&RTC_Date);

//	RTC_SetWakeUp(10); // Wake every 10 seconds
	RTC_SetWakeUp(30); // Wake every 30 seconds
//	RTC_SetWakeUp(60); // Wake every minute
//	RTC_SetWakeUp(120); // Wake every 2 minutes




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




/*
	// Configure the MCO out
	PORT.GPIO_Pin = GPIO_Pin_8;
	PORT.GPIO_Mode = GPIO_Mode_AF;
	PORT.GPIO_Speed = GPIO_Speed_40MHz;
	PORT.GPIO_OType = GPIO_OType_PP;
	PORT.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,&PORT);
	RCC_MCOConfig(RCC_MCOSource_PLLCLK,RCC_MCODiv_16);
*/




	// Configure USB sense pin
	RCC->AHBENR |= USB_SENS_PERIPH; // Enable USB sense pin port peripheral

	// Configure GPIO pin as input with pull-down
	USB_SENS_PORT->MODER &= ~GPIO_MODER_MODER10; // Input mode (reset state)
	USB_SENS_PORT->PUPDR &= ~GPIO_PUPDR_PUPDR10; // Floating (clear bits)
//	USB_SENS_PORT->PUPDR |=  GPIO_PUPDR_PUPDR10_0; // Pull-up
	USB_SENS_PORT->PUPDR |=  GPIO_PUPDR_PUPDR10_1; // Pull-down

	// Configure the EXTI line (USB sense pin)
	EXTI->PR    =  USB_SENS_EXTI; // Clear IT pending bit for EXTI
	EXTI->IMR  |=  USB_SENS_EXTI; // Enable interrupt request from EXTI
	EXTI->EMR  &= ~USB_SENS_EXTI; // Disable event on EXTI
	EXTI->RTSR |=  USB_SENS_EXTI; // Trigger rising edge enabled
	EXTI->FTSR |=  USB_SENS_EXTI; // Trigger falling edge enabled

	// PA10 as source input for EXTI10
	// In fact this is not necessary because it is the default condition after reset
	SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR3_EXTI10; // Clear bits (Set PA10 pin as input source)
	SYSCFG->EXTICR[2] |=  SYSCFG_EXTICR3_EXTI10_PA; // Set PA10 pin as input source

/*
	// Enable the USB sense pin interrupt
	NVICInit.NVIC_IRQChannel = USB_SENS_EXTI_N;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 2;
	NVICInit.NVIC_IRQChannelSubPriority = 0;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVICInit);
*/

	// Simple enable the USB sense pin interrupt
	NVIC_EnableIRQ(USB_SENS_EXTI_N);

	_USB_connected = (USB_SENS_PORT->IDR & USB_SENS_PIN) ? 1 : 0;

///*
	if (_USB_connected) {
		// Configure the USB peripheral
		USB_HWConfig();
		// Initialize the USB device
		USB_Init();
	}
//*/




	// For debug purposes
	if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) {
		// Application executed under debugger control
		BEEPER_Enable(333,3);
		Delay_ms(500);
	} else {
		if (_reset_source & RESET_SRC_STBY) {
			// Device has been in STANDBY mode
			BEEPER_Enable(4000,3);
		} else {
			// Device has not been in STANDBY mode
			BEEPER_Enable(1000,3);
		}
		Delay_ms(2000);
	}




	// Reset or wake-up source
	if (_reset_source == RESET_SRC_UNKNOWN) {
		printf("Wakeup: [UNKNOWN]\r\n");
	} else {
		printf("Wakeup: %s%s%s%s%s%s\r\n",
				_reset_source & RESET_SRC_PIN      ? "PIN " : "",
				_reset_source & RESET_SRC_POR      ? "POR/PDR " : "",
				_reset_source & RESET_SRC_SOFT     ? "SOFT " : "",
				_reset_source & RESET_SRC_STBY     ? "STBY " : "",
				_reset_source & RESET_SRC_STBY_WP1 ? "WKUP1 " : "",
				_reset_source & RESET_SRC_STBY_WP2 ? "WKUP2 " : ""
				);
	}




	// System core clock
	i = SystemCoreClock;
	printf("CPU: %u.%03uMHz\r\n",i / 1000000,(i / 1000) % 1000);
/*
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);
	printf("RCC: HCLK=%u PCLK1=%u PCLK2=%u SYSCLK=%u\r\n",
			RCC_Clocks.HCLK_Frequency,
			RCC_Clocks.PCLK1_Frequency,
			RCC_Clocks.PCLK2_Frequency,
			RCC_Clocks.SYSCLK_Frequency);
*/




	// Debugger report
	printf("Core debugger: %s\r\n",(CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk) ? "PRESENT" : "NONE");




	// Configure charger STAT pin
	RCC->AHBENR |= CHRG_STAT_PERIPH; // Enable charger STAT pin port peripheral

	// Configure GPIO pin as input with pull-up
	CHRG_STAT_PORT->MODER &= ~GPIO_MODER_MODER2; // Input mode (reset state)
	CHRG_STAT_PORT->PUPDR &= ~GPIO_PUPDR_PUPDR2; // Floating (clear bits)
	CHRG_STAT_PORT->PUPDR |=  GPIO_PUPDR_PUPDR2_0; // Pull-up




	// Configure SD detect pin
	RCC->AHBENR |= SD_DETECT_PERIPH; // Enable SD detect pin port peripheral

	// ------------ WARNING -------------
	// PB3 mapped as JTDO after reset, it must be switched to GPIO function here!
	// ------------ WARNING -------------

	/////////////////////////////////////////////////////////////////////////////////////////////
	// To use the serial wire DP to release some GPIOs, the user software must change the GPIO //
	// (PA15, PB3 and PB4) configuration mode in the GPIO_MODER register. This releases        //
	// PA15, PB3 and PB4 which now become available as GPIOs.                                  //
	/////////////////////////////////////////////////////////////////////////////////////////////

	// Configure GPIO pin as input with pull-up
	SD_DETECT_PORT->MODER &= ~GPIO_MODER_MODER3; // Input mode (reset state)
	SD_DETECT_PORT->PUPDR &= ~GPIO_PUPDR_PUPDR3; // Floating (clear bits)

	// Configure the EXTI line (SD detect pin)
	EXTI->PR    =  SD_DETECT_EXTI; // Clear IT pending bit for EXTI
	EXTI->IMR  |=  SD_DETECT_EXTI; // Enable interrupt request from EXTI
	EXTI->EMR  &= ~SD_DETECT_EXTI; // Disable event on EXTI
	EXTI->RTSR |=  SD_DETECT_EXTI; // Trigger rising edge enabled
	EXTI->FTSR |=  SD_DETECT_EXTI; // Trigger falling edge enabled

	// PB3 as source input for EXTI3
	SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI3; // Clear bits (PA[x] pin as input source)
	SYSCFG->EXTICR[0] |=  SYSCFG_EXTICR1_EXTI3_PB; // Set PB[x] pin as input source

/*
	// Enable the SD detect pin interrupt
	NVICInit.NVIC_IRQChannel = SD_DETECT_EXTI_N;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 2;
	NVICInit.NVIC_IRQChannelSubPriority = 0;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVICInit);
*/

	// Simple enable the SD detect pin interrupt
	NVIC_EnableIRQ(SD_DETECT_EXTI_N);

	_SD_connected = (SD_DETECT_PORT->IDR & SD_DETECT_PIN) ? 0 : 1;




	// nRF24 SPI port initialization
	SPIx_Init(nRF24_SPI_PORT);
	SPIx_SetSpeed(nRF24_SPI_PORT,SPI_BR_2); // Highest speed (16MHz with 32MHz CPU)

	// Initialize and configure nRF24
	nRF24_Init();
	j = nRF24_Check();
	printf("nRF24L01+: %s\r\n",(j) ? "PRESENT" : "FAIL");

	// Dump nRF24L01+ registers
	if (j) {
		printf("nRF24L01+ registers:\r\n");
		for (i = 0; i < 0x1d; i++) {
			j = nRF24_ReadReg(i);
			printf("R%02X=0x%02X%s",i,j,((i + 1) % 8) ? " " : "\r\n");
		}

		// Configure nRF24 IRQ EXTI line
		EXTI->PR    =  nRF24_IRQ_EXTI; // Clear IT pending bit for EXTI
		EXTI->IMR  |=  nRF24_IRQ_EXTI; // Enable interrupt request from EXTI
		EXTI->EMR  &= ~nRF24_IRQ_EXTI; // Disable event on EXTI
		EXTI->RTSR &= ~nRF24_IRQ_EXTI; // Trigger rising edge disabled
		EXTI->FTSR |=  nRF24_IRQ_EXTI; // Trigger falling edge enabled

		// PB1 as source input for EXTI1
		SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI1; // Clear bits (PA[x] pin as input source)
		SYSCFG->EXTICR[0] |=  SYSCFG_EXTICR1_EXTI1_PB; // Set PB[x] pin as input source

/*
		// Enable the nRF24 IRQ pin interrupt
		NVICInit.NVIC_IRQChannel = nRF24_IRQ_EXTI_N;
		NVICInit.NVIC_IRQChannelPreemptionPriority = 2;
		NVICInit.NVIC_IRQChannelSubPriority = 0;
		NVICInit.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVICInit);
*/

		// Simple enable the nRF24 IRQ pin interrupt
		NVIC_EnableIRQ(nRF24_IRQ_EXTI_N);

		// Configure nRF24 receive mode
		nRF24_Wake();
#ifdef NRF24_SOLAR
		// Solar powered temperature sensor
		nRF24_RXMode(
				nRF24_RX_PIPE0,           // RX on PIPE#0
				nRF24_ENAA_P0,            // Auto acknowledgment enabled for PIPE#0
				nRF24_RF_CHANNEL,         // RF Channel
				nRF24_DataRate_1Mbps,     // Data rate
				nRF24_CRC_2byte,          // CRC scheme
				(uint8_t *)nRF24_RX_Addr, // RX address
				nRF24_RX_Addr_Size,       // RX address size
				nRF24_RX_PAYLOAD,         // Payload length
				nRF24_TXPower_0dBm        // TX power for auto acknowledgment
				);
#else
		// WBC
		nRF24_RXMode(
				nRF24_RX_PIPE0,           // RX on PIPE#0
				nRF24_ENAA_OFF,           // Auto acknowledgment
				nRF24_RF_CHANNEL,         // RF Channel
				nRF24_DataRate_250kbps,   // Data rate
				nRF24_CRC_2byte,          // CRC scheme
				(uint8_t *)nRF24_RX_Addr, // RX address
				nRF24_RX_Addr_Size,       // RX address size
				nRF24_RX_PAYLOAD,         // Payload length
				nRF24_TXPower_0dBm        // TX power for auto acknowledgment
				);
#endif
	    nRF24_ClearIRQFlags();
	    nRF24_FlushRX();

//	    nRF24_PowerDown();

	    printf("\r\n---------------------------------------------\r\n");
	}



///*
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
//*/




	// Initialize the I2C peripheral
	if (I2Cx_Init(I2C1,400000) != I2C_SUCCESS) {
		printf("I2C initialization failed\r\n");
	} else {
		// Check if I2C devices respond
		printf("I2C test:\r\n");
		printf("  ...ping TSL2581: %s\r\n",(I2Cx_IsDeviceReady(I2C1,TSL2581_ADDR,10) == I2C_SUCCESS) ? "OK" : "NO RESPOND");
		printf("  ...ping  BMP180: %s\r\n",(I2Cx_IsDeviceReady(I2C1,BMP180_ADDR,10) == I2C_SUCCESS) ? "OK" : "NO RESPOND");
		printf("  ...ping   FAKE1: %s\r\n",(I2Cx_IsDeviceReady(I2C1,0xCC,10) == I2C_SUCCESS) ? "OK" : "NO RESPOND");
		printf("  ...ping BMC050A: %s\r\n",(I2Cx_IsDeviceReady(I2C1,BMC050_ACC_ADDR,10) == I2C_SUCCESS) ? "OK" : "NO RESPOND");
		printf("  ...ping BMC050M: %s\r\n",(I2Cx_IsDeviceReady(I2C1,BMC050_MAG_ADDR,10) == I2C_SUCCESS) ? "OK" : "NO RESPOND");
		printf("  ...ping   FAKE2: %s\r\n",(I2Cx_IsDeviceReady(I2C1,0xDD,10) == I2C_SUCCESS) ? "OK" : "NO RESPOND");
	    printf("---------------------------------------------\r\n");
	}




	// Check the ALS
	printf("Ambient light sensor ");
	if (I2Cx_IsDeviceReady(I2C1,TSL2581_ADDR,10) == I2C_SUCCESS) {
		printf("ID: %02X\r\n",TSL2581_GetDeviceID());
		TSL2581_Init();
		TSL2581_SetGain(TSL2581_GAIN8);
		d0 = TSL2581_GetData0();
		d1 = TSL2581_GetData1();
//		TSL2581_PowerOff();
	} else {
		printf("FAIL\r\n");
	}




	// Check the BMP180
	printf("Pressure sensor ");
	if (I2Cx_IsDeviceReady(I2C1,BMP180_ADDR,10) == I2C_SUCCESS) {
		printf("ID: %02X\r\n",BMP180_GetVersion());
		BMP180_Reset(); // Send reset command to BMP180
		Delay_ms(15); // Wait for BMP180 startup time (10ms by datasheet)
		if (BMP180_Check() == BMP180_SUCCESS) {
			BMP180_ReadCalibration();
			BR = BMP180_GetReadings(&RT,&RP,BMP180_ADVRES);
		} else {
			printf("BMP180 check failed\r\n");
			while(1);
		}
	} else {
		printf("FAIL\r\n");
	}



	// Check the BMC050
	printf("eCompass ");
	if (I2Cx_IsDeviceReady(BMC050_I2C_PORT,BMC050_ACC_ADDR,10) == I2C_SUCCESS) {
		printf("ID: %02X\r\n",BMC050_ACC_GetDeviceID());
		BMC050_ACC_SoftReset();
		Delay_ms(5); // must wait for start-up time of accelerometer (2ms)
		BMC050_Init();

		// Enable I2C watchdog timer with 50ms
		BMC050_ACC_InterfaceConfig(ACC_IF_WDT_50ms);

		// Configure accelerometer
		BMC050_ACC_SetIRQ(ACC_IE_DISABLE); // Disable all interrupts
		BMC050_ACC_SetBandwidth(ACC_BW8); // Accelerometer readings filtering (lower or higher better?)
//		BMC050_ACC_SetIRQMode(ACC_IM_NOLATCH); // No IRQ latching
		BMC050_ACC_SetIRQMode(ACC_IM_500ms); // Temporary latch IRQ for 500ms
		BMC050_ACC_ConfigSlopeIRQ(3,4); // Motion detection sensitivity
		BMC050_ACC_IntPinMap(ACC_IM1_SLOPE); // Map slope interrupt to INT1 pin
		BMC050_ACC_SetIRQ(ACC_IE_SLOPEX | ACC_IE_SLOPEY | ACC_IE_SLOPEZ); // Detect motion by all axes
		BMC050_ACC_LowPower(ACC_SLEEP_1000); // Low power with sleep duration 1s
//		BMC050_ACC_LowPower(ACC_SLEEP_1); // Low power with sleep duration 1ms
		BMC050_ACC_SetIRQMode(ACC_IM_RESET); // Clear IRQ bits

//		BMC050_ACC_Suspend();

		// Configure Accelerometer IRQ pin
		RCC->AHBENR |= ACC_IRQ_PERIPH; // Enable GPIO port peripheral

		// Configure GPIO pin as floating input (external pull-down)
		ACC_IRQ_PORT->MODER &= ~GPIO_MODER_MODER13; // Input mode (reset state)
		ACC_IRQ_PORT->PUPDR &= ~GPIO_PUPDR_PUPDR13; // Floating (clear bits)

		// Configure the EXTI line (Accelerometer IRQ pin)
		EXTI->PR    =  ACC_IRQ_EXTI; // Clear IT pending bit for EXTI
		EXTI->IMR  |=  ACC_IRQ_EXTI; // Enable interrupt request from EXTI
		EXTI->EMR  &= ~ACC_IRQ_EXTI; // Disable event on EXTI
		EXTI->RTSR |=  ACC_IRQ_EXTI; // Trigger rising edge enabled
		EXTI->FTSR |=  ACC_IRQ_EXTI; // Trigger falling edge enabled

		// PC13 as source input for EXTI13
		SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI13; // Clear bits (Set PCxx pin as input source)
		SYSCFG->EXTICR[3] |=  SYSCFG_EXTICR4_EXTI13_PC; // Set PCxx pin as input source

	/*
		// Enable the Accelerometer IRQ pin interrupt
		NVICInit.NVIC_IRQChannel = ACC_IRQ_EXTI_N;
		NVICInit.NVIC_IRQChannelPreemptionPriority = 2;
		NVICInit.NVIC_IRQChannelSubPriority = 0;
		NVICInit.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVICInit);
	*/

		// Simple enable the Accelerometer IRQ pin interrupt
		NVIC_EnableIRQ(ACC_IRQ_EXTI_N);
	} else {
		printf("FAIL\r\n");
	}




	// End of I2C devices test
    printf("---------------------------------------------\r\n");




	// Put MCU into STANDBY mode
//	PWR->CSR |= PWR_CSR_EWUP1; // Enable WKUP pin 1 (PA0)
//	PWR->CSR |= PWR_CSR_EWUP2; // Enable WKUP pin 2 (PC13)
//	SleepStop();
//	SleepStandby();
//	while(1);




	PWR_SD_ENABLE_H(); // Turn SD on

	// Enable the DMA2 peripheral clock
	RCC->AHBENR |= RCC_AHBENR_DMA2EN;

	// Enable the DMA2 channel4 interrupt
//	NVIC_EnableIRQ(DMA2_Channel4_IRQn);

	// Enable the SDIO interrupt
//	NVIC_EnableIRQ(SDIO_IRQn);

//	if (!(SD_DETECT_PORT->IDR & SD_DETECT_PIN)) {
	if (_SD_connected) {
		SD_SDIO_GPIO_Init();
		j = SD_Init();
		printf("SD_Init: %X\r\n",j);
	} else j = SDR_NoResponse;

	if (j == SDR_Success) {
		if (SDCard.Type != SDCT_MMC) {
			// MMC doesn't support 4-bit bus
			if (SDCard.SCR[1] & 0x05) {
				// Set 4-bit bus width
				SD_SetBusWidth(SD_BUS_4BIT);
			}
		}

		// DMA read block test
		for (i = 0; i < 2048; i++) sector[i] = '#';
		i = 2048; // block size
		j = SD_ReadBlock_DMA(0,(uint32_t *)sector,i);
		printf("SD_ReadBlock_DMA = %X\r\n",j);
		if (j == SDR_Success) {
			// Wait till data transfered by DMA
			j = SD_CheckRead(i);
			printf("SD_CheckRead = %X\r\n",j);
//			if (j == SDR_Success) VCP_SendBufPrintable(sector,i,'.');
//			printf("\r\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\r\n");
		}

/*
		// DMA write block test
		for (i = 0; i < 512; i++) {
			sector[i] = sector[i + 1024] - 1;
			if (sector[i] < 33) sector[i] = 127;
			if (sector[i] > 127) sector[i] = 33;
		}
		for (i = 512; i < 1024; i++) {
			sector[i] = sector[i + 1024] + 1;
			if (sector[i] < 33) sector[i] = 127;
			if (sector[i] > 127) sector[i] = 33;
		}
//		j = SD_WriteBlock_DMA(1024,(uint32_t *)sector,2048);
		j = SD_WriteBlock_DMA(1024,(uint32_t *)sector,512);
		printf("SD_WriteBlock_DMA = %X\r\n",j);
		if (j == SDR_Success) {
			// Wait till data transfered by DMA
//			j = SD_CheckWrite(1024);
			j = SD_CheckWrite(512);
			printf("SD_CheckWrite = %X\r\n",j);
			if (j == SDR_Success) d0++; else d1++;
		} else d1++;
		j = SD_GetCardState(&sector[0]);
		printf("CardState = %X [%X]\r\n",sector[0],j);
	    printf("---------------------------------------------\r\n");
*/

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
//			if (j == SDR_Success) VCP_SendBufPrintable(sector,i,'.');
//			printf("\r\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\r\n");
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
	    printf("---------------------------------------------\r\n");
*/

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
	    printf("---------------------------------------------\r\n");
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
				for (i = 0; i < 10000; i++) {
					LOG_WriteStr("CNTR=");
					LOG_WriteIntU(i);
					LOG_WriteStr("\r\n");
				}
				LOG_WriteStr("!WBC\r\n");
				LOG_FileSync();
			}
		}
*/

#ifdef NRF24_SOLAR
		// Log file create test
		j = LOG_Init();
		printf("LOG_Init: #%u\r\n",j);
		if (j == LOG_OK) {
			i = 0;
			j = LOG_NewFile(&i);

			printf("LOG_NewFile: %X [#%u]\r\n",(unsigned int)j,i);

			if (j == LOG_OK) {
				LOG_WriteStr("Solar temperature sensor\r\n");
				LOG_WriteStr("SENSOR_DATE;SENSOR_TIME;PACKET_NUM;TEMPERATURE;SENSOR_VBAT;SENSOR_LSI;SENSOR_OTX_LOST;SENSOR_OTX_RETRIES\r\n");
				LOG_FileSync();
				_logging = TRUE;
			}
		}
#endif


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

			// List files in root directory of SD card
			if (DFS_OpenDir(&vi,(uint8_t *)"",&di)) {
				printf("Error opening root directory.\r\n");
			} else {
				while (!DFS_GetNext(&vi,&di,&de)) {
					if (de.name[0]) {
						// Warning: there is no zero terminator in name
						printf("file: \"%s\" [%c%c%c%c%c] %10u\r\n",
								de.name,
								(de.attr & ATTR_DIRECTORY) ? 'D' : '-',
								(de.attr & ATTR_READ_ONLY) ? 'R' : '-',
								(de.attr & ATTR_HIDDEN)    ? 'H' : '-',
								(de.attr & ATTR_SYSTEM)    ? 'S' : '-',
								(de.attr & ATTR_ARCHIVE)   ? 'A' : '-',
								(de.filesize_3 << 24) | (de.filesize_2 << 16) | (de.filesize_1 << 8) | de.filesize_0);
					}
				}
			}

			// List files in LOG directory
			if (DFS_OpenDir(&vi,(uint8_t *)LOG_DIR_LOGS,&di)) {
				printf("Error opening \".\\%s\" directory.\r\n",LOG_DIR_LOGS);
			} else {
				while (!DFS_GetNext(&vi,&di,&de)) {
					if (de.name[0]) {
						// Warning: there is no zero terminator in name
						printf("file: \"%s\" [%c%c%c%c%c] %10u\r\n",
								de.name,
								(de.attr & ATTR_DIRECTORY) ? 'D' : '-',
								(de.attr & ATTR_READ_ONLY) ? 'R' : '-',
								(de.attr & ATTR_HIDDEN)    ? 'H' : '-',
								(de.attr & ATTR_SYSTEM)    ? 'S' : '-',
								(de.attr & ATTR_ARCHIVE)   ? 'A' : '-',
								(de.filesize_3 << 24) | (de.filesize_2 << 16) | (de.filesize_1 << 8) | de.filesize_0);
					}
				}
			}
		}

		// END OF SD CARD THESTS
	}




    printf("---------------------------------------------\r\n");




	BEEPER_Enable(2500,1);




	while(1) {
		// Date/time
		RTC_GetDateTime(&_time,&_date);
		printf("DTM: %s, %02u:%02u:%02u %02u.%02u.20%02u\r\n",
				sDOW[_date.RTC_WeekDay - 1],
				_time.RTC_Hours,
				_time.RTC_Minutes,
				_time.RTC_Seconds,
				_date.RTC_Date,
				_date.RTC_Month,
				_date.RTC_Year
			);

		// Measure Vbat
/*
		// Do 16 ADC measurements and calculate rough average
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

		// Do 16 ADC measurements
		for (i = 0; i < 16; i++) {
			ADC1->CR2 |= ADC_CR2_JSWSTART; // Start conversion of injected channels
			while (!(ADC1->SR & ADC_SR_JEOC)); // Wait until ADC conversions end
			ADC1_raws[i]    = ADC1->JDR1; // Read injected data register1 (ADC_IN1)
			Vrefint_raws[i] = ADC1->JDR2; // Read injected data register2 (ADC_IN17)
		}
		// Calculate interquartile mean values of ADC readings
		ADC1_raw = InterquartileMean(ADC1_raws,16);
		Vrefint_raw = InterquartileMean(Vrefint_raws,16);

		// Convert ADC readings to voltage
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

		// USB sense pin state
		printf("USB: %s (%d)\r\n",_USB_connected ? "connected" : "disconnected",(unsigned int)_USB_int_cntr);
/*
		if (_USB_connected) {
			BEEPER_Enable(4444,1);
			// USB cable plugged
			if (bDeviceState != CONFIGURED) {
				// USB device is not configured, try to initialize it
				USBdev_Init();
			}
		} else {
			// USB cable unplugged
			if (bDeviceState == CONFIGURED) {
				// USB device is configured, deinitialize it
				PowerOff();
				bDeviceState = UNCONNECTED;
				pInformation->Current_Configuration = 0;
			}
		}
*/

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

		// SD card state
		printf("SDC: %s (%d)\r\n",_SD_connected ? "present" : "no card",(unsigned int)_SD_int_cntr);
		if (_SD_last_state != _SD_connected) {
			if (_SD_connected) {
				// The SD card was inserted, need to initialize it
				j = SD_Init();
				printf("SD_Init: %X\r\n",j);
			}
		}
		_SD_last_state = _SD_connected;

		// Read some blocks from the SD card
		if (_SD_connected) {
/*
			for (i = 0; i < 2048; ) sector[i++] = '#';
			j = SD_ReadBlock(0,(uint32_t *)sector,2048);
			printf("SD_ReadBlock = %02X, ",j);
			if (j != SDR_Success) BEEPER_Enable(1111,1);
			j = SD_GetCardState(&sector[0]);
			printf("SD_GetCardState = %02X [%X]\r\n",j,sector[0]);
*/
			for (i = 0; i < 2048; ) sector[i++] = '#';
			j = SD_ReadBlock_DMA(0,(uint32_t *)sector,2048);
			printf("ReadBlock_DMA = %02X, ",j);
			if (j == SDR_Success) {
				j = SD_CheckRead(2048);
				printf("CheckRead = %02X, ",j);
			}
			if (j != SDR_Success) BEEPER_Enable(4444,1);
			j = SD_GetCardState(&sector[0]);
			printf("CardState = %02X [%X]\r\n",j,sector[0]);
		}

#ifdef NRF24_SOLAR
		// Yep, the following code is so ugly =)

		// Solar temperature sensor packet
		printf("NRF: %upkts%s",_packets_rcvd,(_new_packet) ? " " : "\r\n");
		if (_new_packet) {
			RT = (nRF24_RX_Buf[0] << 8) | nRF24_RX_Buf[1]; // Temperature
			Vrefint = (nRF24_RX_Buf[6] << 8) + nRF24_RX_Buf[7]; // Sensor voltage
			printf("[#%u TEMP: %d.%dC VREF: %u.%uV LSI: %uHz OTX: %u/%u %02u:%02u:%02u %02u.%02u.20%02u]\r\n",
					(uint32_t)((nRF24_RX_Buf[2] << 24)|(nRF24_RX_Buf[3] << 16)|(nRF24_RX_Buf[4] << 8)|(nRF24_RX_Buf[5])), /* Packet number */
					RT / 10, RT % 10,
					Vrefint / 100, Vrefint % 100,
					(nRF24_RX_Buf[14] << 8) + nRF24_RX_Buf[15], /* Sensor MCU LSI frequency */
					nRF24_RX_Buf[16] >> 4, nRF24_RX_Buf[16] & 0x0F, /* Sensor nRF24 OTX register: lost packets/number of retries */
					(((nRF24_RX_Buf[10] & 0x30) >> 4) * 10) + (nRF24_RX_Buf[10] & 0x0F), /* Time: hours */
					((nRF24_RX_Buf[9] >> 4) * 10) + (nRF24_RX_Buf[9] & 0x0F), /* Time: minutes */
					((nRF24_RX_Buf[8] >> 4) * 10) + (nRF24_RX_Buf[8] & 0x0F), /* Time: seconds */
					((nRF24_RX_Buf[11] >> 4) * 10) + (nRF24_RX_Buf[11] & 0x0F), /* Date: day */
					(((nRF24_RX_Buf[12] & 0x1f) >> 4) * 10) + (nRF24_RX_Buf[12] & 0x0F), /* Date: month */
					((nRF24_RX_Buf[13] >> 4) * 10) + (nRF24_RX_Buf[13] & 0x0F) /* Date: year */
					);

			// Write data to log file
			if (_logging) {
				// Sensor time
				LOG_WriteTime(
						(((nRF24_RX_Buf[10] & 0x30) >> 4) * 10) + (nRF24_RX_Buf[10] & 0x0F),
						((nRF24_RX_Buf[9] >> 4) * 10) + (nRF24_RX_Buf[9] & 0x0F),
						((nRF24_RX_Buf[8] >> 4) * 10) + (nRF24_RX_Buf[8] & 0x0F));
				LOG_WriteStr(";");
				// Sensor date
				LOG_WriteDate(
						((nRF24_RX_Buf[11] >> 4) * 10) + (nRF24_RX_Buf[11] & 0x0F),
						(((nRF24_RX_Buf[12] & 0x1f) >> 4) * 10) + (nRF24_RX_Buf[12] & 0x0F),
						((nRF24_RX_Buf[13] >> 4) * 10) + (nRF24_RX_Buf[13] & 0x0F));
				LOG_WriteStr(";");
				// Packet number
				LOG_WriteIntU((uint32_t)((nRF24_RX_Buf[2] << 24)|(nRF24_RX_Buf[3] << 16)|(nRF24_RX_Buf[4] << 8)|(nRF24_RX_Buf[5])));
				LOG_WriteStr(";");
				// Temperature
				LOG_WriteInt(RT / 10);
				LOG_WriteStr(".");
				LOG_WriteInt(RT % 10);
				LOG_WriteStr("C;");
				// Sensor voltage
				LOG_WriteIntF(Vrefint,2);
				LOG_WriteStr("V;");
				// Sensor MCU LSI frequency
				LOG_WriteIntU((nRF24_RX_Buf[14] << 8) + nRF24_RX_Buf[15]);
				LOG_WriteStr("Hz;");
				// Sensor nRF24 OTX register (shockburst)
				LOG_WriteIntU(nRF24_RX_Buf[16] >> 4); // Lost packets
				LOG_WriteStr(";");
				LOG_WriteIntU(nRF24_RX_Buf[16] & 0x0F); // Number of retries
				LOG_WriteStr("\r\n");
				LOG_FileSync(); // Save data buffer to SD card
			}

			_new_packet = FALSE;
		}
#else
		// WBC packet
		printf("NRF: %upkts%s",_packets_rcvd,(_new_packet) ? " " : "\r\n");
		if (_new_packet) {
			i = CRC8_CCITT(nRF24_RX_Buf,nRF24_RX_PAYLOAD - 1);
			j = ((nRF24_RX_Buf[6] & 0x03) << 8) + nRF24_RX_Buf[7];
			printf("[CRC: %02X%c%02X VBAT: %u.%uV WAKE: %u]\r\n",
					i,(i == nRF24_RX_Buf[nRF24_RX_PAYLOAD - 1]) ? '=' : '!',nRF24_RX_Buf[nRF24_RX_PAYLOAD - 1],
					j / 100, j % 100,
					(nRF24_RX_Buf[8] << 8) + nRF24_RX_Buf[9]);
			_new_packet = FALSE;
		}
#endif

		i = BMC050_ACC_GetIRQStatus();
		printf("ACC: %s [%04X]\r\n",(ACC_IRQ_PORT->IDR & ACC_IRQ_PIN) ? "HIGH" : "LOW",i);
//		BMC050_ACC_SetIRQMode(ACC_IM_RESET); // Clear IRQ bits

	    printf("---------------------------------------------\r\n");

//		BEEPER_Enable(111,1);

//		Delay_ms(2000);

	    // Put MCU into SLEEP mode
//		SleepWait();

	    // Put MCU into STOP mode
//	    SleepStop();

		// Put MCU into STANDBY mode
		PWR->CSR |= PWR_CSR_EWUP1; // Enable WKUP pin 1 (PA0)
		PWR->CSR |= PWR_CSR_EWUP2; // Enable WKUP pin 2 (PC13)
		SleepStandby();
	}




	// Put MCU into STANDBY mode
	PWR->CSR |= PWR_CSR_EWUP1; // Enable WKUP pin 1 (PA0)
	PWR->CSR |= PWR_CSR_EWUP2; // Enable WKUP pin 2 (PC13)
	SleepStandby();

	// After STANDBY the MCU will go RESET routine, not here
	while(1);
}
