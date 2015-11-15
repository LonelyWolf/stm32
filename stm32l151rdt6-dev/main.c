#include <stm32l1xx_rcc.h>
#include <stm32l1xx_gpio.h>
#include <stm32l1xx_syscfg.h>
#include <misc.h>
#include <string.h>
#include <math.h> // Don't forget to add '-lm' linker option (damn CoIDE)

// Debug
#include <stdio.h>

// Libraries
#include "wolk.h"
#include "i2c.h"
#include "RTC.h"
#include "bmp180.h"
#include "bmc050.h"
#include "tsl2581.h"
#include "beeper.h"
#include "delay.h"
#include "sdcard-sdio.h"
#include "log.h"
#include "spi.h"
#include "nRF24.h"
#include "uart.h"
#include "ST7541.h"
#include "NMEA.h"
#include "GPS.h"

// USB stuff
#include "usb_lib.h"
#include "VCP.h"

// DOSFS
#include "dosfs.h"

// Resources
#include "font3x5.h" // Very small digits
#include "font5x7.h"
#include "font7x10.h"
#include "toppler.h"
#include "font_digits.h" // Big digits
#include "bitmaps.h" // Various bitmaps




// Ignore 'format' compiler warnings
#pragma GCC diagnostic ignored "-Wformat"




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

// LCD backlight pin (PB4)
#define LCD_BL_PIN_PERIPH         RCC_AHBENR_GPIOBEN
#define LCD_BL_PORT               GPIOB
#define LCD_BL_PIN                GPIO_Pin_4
#define LCD_BL_PIN_SRC            GPIO_PinSource4
#define LCD_BL_GPIO_AF            GPIO_AF_TIM3
#define LCD_BL_H()                LCD_BL_PORT->BSRRL = LCD_BL_PIN
#define LCD_BL_L()                LCD_BL_PORT->BSRRH = LCD_BL_PIN
#define LCD_BL_TIM_RCC            RCC->APB1ENR
#define LCD_BL_TIM_PERIPH         RCC_APB1ENR_TIM3EN
#define LCD_BL_TIM                TIM3


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
uint8_t sector[2048] __attribute__((aligned(4)));

// ALS variables
uint16_t d0,d1;
uint32_t lux;

// BMP180 variables
BMP180_RESULT BR;
int16_t RT;
int32_t RP;
int16_t X,Y,Z,ACCT;

// ADC variables
#define ADC_count 16  // Number of ADC conversions
uint16_t ADC_raw[ADC_count * 2]; // array for 16 pairs of values ADC_IN1 and Vrefint (size must be power of 2)
uint16_t ADC_IN1_raws[ADC_count],Vrefint_raws[ADC_count];
uint32_t ADC_IN1_raw,Vrefint_raw;
uint32_t Vbat,Vrefint,Vcpu;
volatile bool _ADC_completed = FALSE;

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
uint32_t _packets_lost = 0; // Lost packets counter
uint32_t _prev_wake_cntr = 0xDEADBEEF; // Prevous wake counter (to count lost packets)
uint32_t _time_no_signal = 0; // Time elapsed since the last received packet
#define NO_SIGNAL_TIME 15 // Sensor signal timeout (seconds)
bool _icon_RF = TRUE; // Flag for RF icon flashing

// Date/Time
RTC_TimeTypeDef _time;
RTC_DateTypeDef _date;

// LCD debug
uint8_t lcd_backlight = 10;

// RTC
volatile bool _new_time = FALSE;
uint32_t _wakeup_interval = 1;

// USART
#define  USART_FIFO_SIZE         64    // USART FIFO buffer size (must be power of 2)
uint8_t  USART_FIFO[USART_FIFO_SIZE];  // USART FIFO receive buffer
uint16_t USART_FIFO_pos;               // Write position in USART FIFO

// GPS
uint32_t _NMEA_total_count; // Count of received NMEA sentences
uint32_t _NMEA_total_size; // Total amount of NMEA data received (bytes)

// Charger STAT pin
uint8_t _STAT_ft = 0;
uint8_t _STAT_pu = 0;
uint8_t _STAT_pd = 0;

// SPL
GPIO_InitTypeDef PORT;
NVIC_InitTypeDef NVICInit;




// Determine source of reset
uint32_t GetResetSource(void) {
	uint32_t result = RESET_SRC_UNKNOWN;
	uint32_t reg;

	// Reset source
	reg = RCC->CSR;
	if (reg & RCC_CSR_SFTRSTF) {
		result |= RESET_SRC_SOFT;
	} else if (reg & RCC_CSR_PORRSTF) {
		result |= RESET_SRC_POR;
	} else if (reg & RCC_CSR_PINRSTF) {
		result |= RESET_SRC_PIN;
	}

	// Clear the reset flags
	RCC->CSR |= RCC_CSR_RMVF;

	// Enable the PWR peripheral to deal with it CSR register
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	__DSB();

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
		__DSB();
		if (GPIOA->IDR & GPIO_IDR_IDR_0)  result |= RESET_SRC_STBY_WP1;
		if (GPIOC->IDR & GPIO_IDR_IDR_13) result |= RESET_SRC_STBY_WP2;

		// Restore value of the AHBENR register
		RCC->AHBENR = reg;
	}

	return result;
}

// Inquiry status of the button
void Button_Inquiry(BTN_TypeDef *button) {
	if (button == &BTN[BTN_ESCAPE]) {
		// Escape button have inverted input
		if (!(button->PORT->IDR & button->PIN)) {
		    // Button released
			button->hold_cntr = 0;
			if (button->state != BTN_Hold) button->cntr++;
			button->state = BTN_Released;
			// Disable sleep-on-exit (return to main loop from IRQ)
			SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;

			BEEPER_Enable(200,1);
		} else {
			// Button pressed
			button->hold_cntr = 0;
			button->state = BTN_Pressed;

			BEEPER_Enable(2000,1);
		}
	} else {
		if (button->PORT->IDR & button->PIN) {
		    // Button released
			button->hold_cntr = 0;
			if (button->state != BTN_Hold) button->cntr++;
			button->state = BTN_Released;
			// Disable sleep-on-exit (return to main loop from IRQ)
			SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;

			BEEPER_Enable(200,1);
		} else {
			// Button pressed
			button->hold_cntr = 0;
			button->state = BTN_Pressed;

			BEEPER_Enable(2000,1);
		}
	}
}

// RTC wake-up IRQ handler
void RTC_WKUP_IRQHandler(void) {
	if (RTC->ISR & RTC_ISR_WUTF) {
		// RTC Wake-up interrupt

		// Get current date/time
		RTC_GetDateTime(&_time,&_date);

		_new_time = TRUE;
		_time_no_signal += _wakeup_interval;

		// Disable sleep-on-exit (return to main loop from IRQ)
		SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;

		PWR->CR  |= PWR_CR_DBP; // Access to RTC, RTC Backup and RCC CSR registers enabled
		RTC->ISR &= ~RTC_ISR_WUTF; // Clear the RTC wake-up timer flag
		PWR->CR  &= ~PWR_CR_DBP; // Access to RTC, RTC Backup and RCC CSR registers disabled

		EXTI->PR = RTC_WKUP_EXTI; // Clear the EXTI pending bit
	}
}

// RTC alarm IRQ handler
void RTC_Alarm_IRQHandler(void) {
	if (RTC->ISR & RTC_ISR_ALRAF) {
		// RTC alarm A interrupt

		BEEPER_PlayTones(tones_3beep);

		PWR->CR  |= PWR_CR_DBP; // Access to RTC, RTC Backup and RCC CSR registers enabled
		RTC->ISR &= ~RTC_ISR_ALRAF; // Clear the RTC alarm A flag
		PWR->CR  &= ~PWR_CR_DBP; // Access to RTC, RTC Backup and RCC CSR registers disabled

		EXTI->PR = RTC_ALARM_EXTI; // Clear the EXTI pending bit
	}
}

// USART2 IRQ handler
void USART2_IRQHandler(void) {
	uint16_t SR;
	uint16_t rcvd;

	// Read the USART SR register and then DR to clean all possible flags
	SR = GPS_USART_PORT->SR;
	(void)GPS_USART_PORT->DR;

	// USART IDLE line detected
	if (SR & USART_SR_IDLE) {
		// Copy received data from the USART FIFO buffer to the GPS buffer
		rcvd = USART_FIFO_SIZE - USART_FIFO_pos - (uint16_t)DMA1_Channel6->CNDTR; // Amount of received data
		if (rcvd >= (USART_FIFO_SIZE >> 1)) rcvd -= (USART_FIFO_SIZE >> 1); // Data in last half of FIFO
		// Trim GPS data counter to prevent overflow
		if (GPS_buf_cntr + rcvd > GPS_BUFFER_SIZE - 1) rcvd = GPS_BUFFER_SIZE - GPS_buf_cntr;
		memcpy(&GPS_buf[GPS_buf_cntr],&USART_FIFO[USART_FIFO_pos],rcvd);
		USART_FIFO_pos += rcvd;
		GPS_buf_cntr += rcvd;
		GPS_new_data = TRUE;
	}

	// USART overrun error
	if (SR & USART_SR_ORE) {
		printf(">>> OE: %u\r\n",GPS_buf_cntr);
		GPS_buf_cntr = 0;
	}
}

// DMA1 channel1 IRQ handler (ADC1)
void DMA1_Channel1_IRQHandler() {
	// Clear the DMA1 channel1 transfer complete and global interrupt flags
	DMA1->IFCR = DMA_IFCR_CTCIF1 | DMA_IFCR_CGIF1;

	// Disable the DMA1 channel1
	DMA1_Channel1->CCR &= ~DMA_CCR1_EN;

	// Disable the DMA bit in the ADC1_CR2 register
	// It must be written to 0 before starting a new conversion
	// And disable continuous mode
	ADC1->CR2 &= ~(ADC_CR2_DMA | ADC_CR2_CONT);

	// Set flag to indicate conversion ends
	_ADC_completed = TRUE;
}

// DMA1 channel6 IRQ handler (USART_RX)
void DMA1_Channel6_IRQHandler() {
	uint16_t rcvd;

	// Channel6 half-transfer
	if (DMA1->ISR & DMA_ISR_HTIF6) {
		// Copy first half of the FIFO buffer to the GPS buffer
		rcvd = (USART_FIFO_SIZE >> 1) - USART_FIFO_pos; // Amount of received data
		// Trim GPS data counter to prevent overflow
		if (GPS_buf_cntr + rcvd > GPS_BUFFER_SIZE - 1) rcvd = GPS_BUFFER_SIZE - GPS_buf_cntr;
		memcpy(&GPS_buf[GPS_buf_cntr],&USART_FIFO[USART_FIFO_pos],rcvd);
		USART_FIFO_pos = USART_FIFO_SIZE >> 1;
		GPS_buf_cntr += rcvd;

		// Clear the DMA1 channel6 half-transfer flag
		DMA1->IFCR = DMA_IFCR_CHTIF6;
	}

	// Channel6 transfer complete
	if (DMA1->ISR & DMA_ISR_TCIF6) {
		// Copy last half of the FIFO buffer to the GPS buffer
		rcvd = USART_FIFO_SIZE - USART_FIFO_pos; // Amount of received data
		// Trim GPS data counter to prevent overflow
		if (GPS_buf_cntr + rcvd > GPS_BUFFER_SIZE - 1) rcvd = GPS_BUFFER_SIZE - GPS_buf_cntr;
		memcpy(&GPS_buf[GPS_buf_cntr],&USART_FIFO[USART_FIFO_pos],rcvd);
		USART_FIFO_pos = 0;
		GPS_buf_cntr += rcvd;

		// Clear the DMA1 channel6 transfer complete flag
		DMA1->IFCR = DMA_IFCR_CTCIF6;
	}
}

// EXTI0 line IRQ handler
void EXTI0_IRQHandler(void) {
	if (EXTI->PR & BTN3_EXTI) {
		// EXTI0 (Button#3 -> "ESCAPE")
		Button_Inquiry(&BTN[BTN_ESCAPE]);
		EXTI->PR = BTN3_EXTI; // Clear IT bit for EXTI_line
	}
}

// EXTI1 line IRQ handler
void EXTI1_IRQHandler(void) {
	nRF24_RX_PCKT_TypeDef RX_status;
	uint16_t *ptr = (uint16_t *)&nRF24_RX_Buf;

	if (EXTI->PR & nRF24_IRQ_EXTI) {
		RX_status = nRF24_RXPacket(nRF24_RX_Buf,nRF24_RX_PAYLOAD);
		if (RX_status == nRF24_RX_PCKT_PIPE0) {
			// Bytes of 16-bit values must be swapped due to different endianness of STM8L and STM32
/*
			nRF24_Packet.cntr_SPD  =  (nRF24_RX_Buf[0] << 8) | nRF24_RX_Buf[1];
			nRF24_Packet.tim_SPD   =  (nRF24_RX_Buf[2] << 8) | nRF24_RX_Buf[3];
			nRF24_Packet.tim_CDC   =  (nRF24_RX_Buf[4] << 8) | nRF24_RX_Buf[5];
			nRF24_Packet.vrefint   = ((nRF24_RX_Buf[6] << 8) | nRF24_RX_Buf[7]) & 0x03ff;
			nRF24_Packet.cntr_wake =  (nRF24_RX_Buf[8] << 8) | nRF24_RX_Buf[9];
			nRF24_Packet.CRC8      =   nRF24_RX_Buf[10];
*/
			// This code looks cooler and its size is 40 bytes less
			nRF24_Packet.cntr_SPD  = __builtin_bswap16(*ptr++);
			nRF24_Packet.tim_SPD   = __builtin_bswap16(*ptr++);
			nRF24_Packet.tim_CDC   = __builtin_bswap16(*ptr++);
			nRF24_Packet.vrefint   = __builtin_bswap16(*ptr++);
			nRF24_Packet.cntr_wake = __builtin_bswap16(*ptr++);
			nRF24_Packet.CRC8 = nRF24_RX_Buf[10];

			_new_packet = TRUE;
			_time_no_signal = 0;

/*
			CRC_local = CRC8_CCITT(nRF24_RX_Buf,nRF24_RX_PAYLOAD - 1);
			if (CRC_local == nRF24_Packet.CRC8) {
				if (_prev_wake_cntr == 0xDEADBEEF) _prev_wake_cntr = nRF24_Packet.cntr_wake;
				if (nRF24_Packet.cntr_wake < _prev_wake_cntr) _prev_wake_cntr = nRF24_Packet.cntr_wake;
				wake_diff = nRF24_Packet.cntr_wake - _prev_wake_cntr;
				printf("wake: %u[%u] diff: %u vbat=%u.%02uV pkts: %u\\%u [%u%%]\r\n",
						nRF24_Packet.cntr_wake,
						_prev_wake_cntr,
						wake_diff,
						nRF24_Packet.vrefint / 100,nRF24_Packet.vrefint % 100,
						_packets_rcvd,
						_packets_lost,
						(_packets_lost * 100) / (_packets_rcvd + _packets_lost)
				);
				_prev_wake_cntr = nRF24_Packet.cntr_wake;
				if (wake_diff > 1) {
					_packets_lost += wake_diff - 1;
//					BEEPER_Enable(1700,1);
				}
			} else {
				printf("CRC: %02X <> %02X [%s]\r\n",nRF24_Packet.CRC8,CRC_local,(nRF24_Packet.CRC8 == CRC_local) ? "OK" : "BAD");
//				BEEPER_Enable(333,1);
			}
			_packets_rcvd++;
*/
		} else {
//			BEEPER_Enable(5000,3);
		}

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

/*
	// EXTI10 of USB sense conflicts with EXTI10 of button "UP"
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
*/

	if (EXTI->PR & ACC_IRQ_EXTI) {
		// Accelerometer IRQ pin
		acc_irq = BMC050_ACC_GetIRQStatus(); // <--- Bad idea to call this in IRQ handler
		if (acc_irq & ACC_IRQ_SLOPE) {
			// Slope IRQ high
			BEEPER_Enable(444,1);
		} else {
			// Slope IRQ low
			BEEPER_Enable(3333,1);
		}
		printf("---> ACC IRQ: [%s] <---\r\n",(acc_irq & ACC_IRQ_SLOPE) ? "HIGH" : "LOW");
		EXTI->PR = ACC_IRQ_EXTI; // Clear IT bit for EXTI_line
	}

	if (EXTI->PR & BTN0_EXTI) {
		// EXTI10 (Button#0 -> "UP")
		Button_Inquiry(&BTN[BTN_UP]);
		EXTI->PR = BTN0_EXTI; // Clear IT bit for EXTI_line
	}

	if (EXTI->PR & BTN1_EXTI) {
		// EXTI11 (Button#1 -> "DOWN")
		Button_Inquiry(&BTN[BTN_DOWN]);
		EXTI->PR = BTN1_EXTI; // Clear IT bit for EXTI_line
	}

	if (EXTI->PR & BTN2_EXTI) {
		// EXTI15 (Button#2 -> "ENTER")
		Button_Inquiry(&BTN[BTN_ENTER]);
		EXTI->PR = BTN2_EXTI; // Clear IT bit for EXTI_line
	}
}

// DMA1 channel3 IRQ handler (display SPI DMA TX)
void DMA1_Channel3_IRQHandler(void) {
	// Handle the DMA IRQ
	SPIx_DMA_Handler(&ST7541_SPI_PORT.DMA_TX);
	// Disable the DMA channel
	SPIx_SetDMA(&ST7541_SPI_PORT,SPI_DMA_TX,DISABLE);
	// Ensure that the last SPI communication is complete
	while (!(ST7541_SPI_PORT.Instance->SR & SPI_SR_TXE));
	while (ST7541_SPI_PORT.Instance->SR & SPI_SR_BSY);
	// Deassert the display CS pin
	ST7541_CS_H();
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
	uint32_t sum = 0;
	uint32_t index, maxindex;

	// discard the lowest and the highest data samples
	maxindex = 3 * (numOfSamples >> 2);
	for (index = numOfSamples >> 2; index < maxindex; index++) sum += array[index];

	// return the mean value of the remaining samples value
	return (sum / (numOfSamples >> 1));
}




int main(void) {
	// Determine wake-up source
	_reset_source = GetResetSource();




	// PB4 after reset acts as NJTRST input with pull-up (PB4 configure as alternative function)
	// The LED backlight PWM output must be tied to ground ASAP
	// Configure PB4 as GPIO
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	__DSB();
	GPIOB->MODER &= ~GPIO_MODER_MODER4; // Input
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR4; // No pull-up, pull-down




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

		// Enable the PWR peripheral to deal with it CSR register
		RCC->APB1ENR |= RCC_APB1ENR_PWREN;
		__DSB();

		// Compute the actual MCU clock speed (for proper BEEPER initialization)
		SystemCoreClockUpdate();

		// Initialize BEEPER and do BEEP to indicate what MCU is awake
		BEEPER_Init();

		if (_reset_source & RESET_SRC_STBY_WP2) {
			// Initialize BEEPER and do BEEP to indicate what MCU is awake
//			BEEPER_Init();

			// This is wake-up from accelerometer IRQ pin
			BEEPER_Enable(2222,2);

			// Enable the accelerometer IRQ GPIO pin and wait until it becomes low
			RCC->AHBENR |= ACC_IRQ_PERIPH; // Enable GPIO port peripheral
			__DSB();
			ACC_IRQ_PORT->MODER &= ~GPIO_MODER_MODER13; // Input mode (reset state)
			ACC_IRQ_PORT->PUPDR &= ~GPIO_PUPDR_PUPDR13; // Floating (clear bits)
			while (ACC_IRQ_PORT->IDR & ACC_IRQ_PIN);

			// Do another BEEP and wait till it ends
			BEEPER_Enable(444,1);
			while (_beep_duration);
		} else {
			// Initialize display SPI port
			SPI1_HandleInit();
			SPIx_Init(&ST7541_SPI_PORT,SPI_DIR_TX,SPI_BR_2);

			// Initialize display control pins
			ST7541_InitGPIO();

			ST7541_CS_L();
			ST7541_cmd(0x2f); // Power control: VC,VR,VF = 1,1,1 (internal voltage booster)
			ST7541_CS_H();

			// Adjust partial display and lower contrast ratio
			ST7541_SetDisplayPartial(32,0,64);
			ST7541_Contrast(5,5,10);

			// Draw time
			ST7541_Fill(0x0000);
			RTC_GetDateTime(&_time,&_date);
			i = 0;
			i += PutIntLZ(i,42,_time.RTC_Hours,2,fnt5x7);
			i += DrawChar(i,42,':',fnt5x7);
			i += PutIntLZ(i,42,_time.RTC_Minutes,2,fnt5x7);
			i += DrawChar(i,42,':',fnt5x7);
			i += PutIntLZ(i,42,_time.RTC_Seconds,2,fnt5x7);
			i += 3;
			i += PutIntLZ(i,42,_date.RTC_Date,2,fnt5x7);
			i += DrawChar(i,42,'.',fnt5x7);
			i += PutIntLZ(i,42,_date.RTC_Month,2,fnt5x7);
			i += DrawChar(i,42,'.',fnt5x7);
			i += PutIntLZ(i,42,_date.RTC_Year,2,fnt5x7);
			PutStr(i + 3,42,RTC_DOW_STR[_date.RTC_WeekDay],fnt5x7);
			ST7541_Flush();

			// This is wake-up from RTC, do BEEP and wait until it ends
			BEEPER_Enable(1111,1);
			while (_beep_duration);
		}

		// Clear the RTC wake-up timer flag
		// The PWR peripheral must be enabled (in GetResetSource)
		PWR->CR  |= PWR_CR_DBP; // Access to RTC, RTC Backup and RCC CSR registers enabled
		RTC->ISR &= ~RTC_ISR_WUTF;
		PWR->CR  &= ~PWR_CR_DBP; // Access to RTC, RTC Backup and RCC CSR registers disabled

		// Put MCU into STANDBY mode
		PWR->CSR |= PWR_CSR_EWUP1; // Enable WKUP pin 1 (PA0)
		PWR->CSR |= PWR_CSR_EWUP2; // Enable WKUP pin 2 (PC13)
		SleepStandby();
	}




	// Setup the microcontroller system
	SystemInit();
	SystemCoreClockUpdate();




	// Enable debugging when the MCU is in low power modes
	DBGMCU->CR |= DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY;




/*
	// NVIC: 2 bit for preemption priority, 2 bits for subpriority
	// WARNING: this stuff will be re-setup in USB init routine
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
*/




	// Enable the system configuration controller
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;




/*
	// UART port initialization
	UARTx_Init(USART2,USART_RX | USART_TX,1382400);
	UART_SendStr(USART2,"--- STM32L151RDT6 ---\r\n");
*/




/*
	// Enable the RTC wake-up interrupt
	NVICInit.NVIC_IRQChannel = RTC_WKUP_IRQn;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 0x0f; // 0x0f - lowest priority
	NVICInit.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Init(&NVICInit);

	// Enable the RTC alarm interrupt
	NVICInit.NVIC_IRQChannel = RTC_Alarm_IRQn;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 0x0f; // 0x0f - lowest priority
	NVICInit.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Init(&NVICInit);
*/

	// Simple enable the RTC wake-up interrupt
	NVIC_EnableIRQ(RTC_WKUP_IRQn);

	// Simple enable the RTC alarm IRQ
	NVIC_EnableIRQ(RTC_Alarm_IRQn);

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
		_date.RTC_WeekDay = RTC_DOW_FRIDAY;
		RTC_SetDateTime(&_time,&_date);
	}
	RTC_ITConfig(RTC_IT_WUT,ENABLE);
	RTC_GetDateTime(&_time,&_date);

	_wakeup_interval = 1; // Wake every 10 seconds
	RTC_SetWakeUp(_wakeup_interval); // Wake every 10 seconds




	// Initialize delay
	Delay_Init(NULL);




	// Initialize beeper
	BEEPER_Init();




	// Enable power control lines GPIO peripherals
	RCC->AHBENR |= (PWR_GPS_ENABLE_PERIPH | PWR_SD_ENABLE_PERIPH | PWR_VBAT_ENABLE_PERIPH | PWR_LCD_ENABLE_PERIPH);

	// Configure power control lines as push-pull output without pull-up
	PORT.GPIO_Mode  = GPIO_Mode_OUT;
	PORT.GPIO_Speed = GPIO_Speed_400KHz;
	PORT.GPIO_OType = GPIO_OType_PP;
	PORT.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	PORT.GPIO_Pin = PWR_GPS_ENABLE_PIN;
	GPIO_Init(PWR_GPS_ENABLE_PORT,&PORT);
	PWR_GPS_ENABLE_L(); // Turn it off

	PORT.GPIO_Pin = PWR_SD_ENABLE_PIN;
	GPIO_Init(PWR_SD_ENABLE_PORT,&PORT);
	PWR_SD_ENABLE_L(); // Turn it off

	PORT.GPIO_Pin = PWR_VBAT_ENABLE_PIN;
	GPIO_Init(PWR_VBAT_ENABLE_PORT,&PORT);
	PWR_VBAT_ENABLE_L(); // Turn it off

	PORT.GPIO_Pin = PWR_LCD_ENABLE_PIN;
	GPIO_Init(PWR_LCD_ENABLE_PORT,&PORT);
	PWR_LCD_ENABLE_L(); // Turn it off




/*
	// Configure the MCO out
	PORT.GPIO_Pin = GPIO_Pin_8;
	PORT.GPIO_Mode = GPIO_Mode_AF;
	PORT.GPIO_Speed = GPIO_Speed_40MHz;
	PORT.GPIO_OType = GPIO_OType_PP;
	PORT.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,&PORT);
	RCC_MCOConfig(RCC_MCOSource_HSI,RCC_MCODiv_1);
*/




	// Enable the DMA1 peripheral clock
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	// Enable DC-DC for LCD
//	PWR_LCD_ENABLE_H();
	PWR_LCD_ENABLE_L();

	// Display SPI port initialization: 1-line TX
	SPI1_HandleInit();
//	SPIx_Init(&ST7541_SPI_PORT,SPI_DIR_TX,SPI_BR_256); // lowest speed (125kHz on 32MHz CPU)
//	SPIx_Init(&ST7541_SPI_PORT,SPI_DIR_TX,SPI_BR_2);   // highest speed (16MHz on 32MHz CPU)
	SPIx_Init(&ST7541_SPI_PORT,SPI_DIR_TX,SPI_BR_4);   // 8MHz on 32MHz CPU
//	SPIx_Init(&ST7541_SPI_PORT,SPI_DIR_TX,SPI_BR_8);   // 4MHz on 32MHz CPU
//	SPIx_Init(&ST7541_SPI_PORT,SPI_DIR_TX,SPI_BR_16);  // 2MHz on 32MHz CPU

	// Initialize display and clear screen
	ST7541_InitGPIO();
	ST7541_Init();
//	ST7541_Contrast(6,6,36); // External booster 10.6V
//	ST7541_Contrast(7,7,24); // Internal booster/External booster 12.5V
	ST7541_Contrast(6,6,32); // Internal booster
	ST7541_Fill(0x0000);

	// Enable the display SPI port DMA TX interrupt
	ST7541_SPI_PORT.DMA_TX.Channel->CCR |= DMA_CCR1_TCIE; // Enable the DMA TC IRQ
	NVIC_EnableIRQ(DMA1_Channel3_IRQn);




	// Draw big gray-scaled toppler (logo)
	uint8_t xx,yy,bb;
	ST7541_Fill(0x0000);
	for (yy = 0; yy < 48; yy++) {
		for (xx = 0; xx < 12; xx++) {
			bb = ~toppler_big[(yy * 12) + xx];
			for (i = 4; i--; ) {
				Pixel(40 + (xx << 2) + i,10 + yy,(bb & 0x03));
				bb >>= 2;
			}
		}
	}
	DrawBitmapGS(40,60,48,48,toppler_big);

	DrawBitmap(0,0,30,25,bmp_bike_man);

	ST7541_Flush_DMA(NOBLOCK);

//	while(1);

	// Initialize the display backlight

	// Backlight pin GPIO configuration
	PORT.GPIO_Mode  = GPIO_Mode_AF;
	PORT.GPIO_Speed = GPIO_Speed_40MHz;
	PORT.GPIO_OType = GPIO_OType_PP;
	PORT.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	PORT.GPIO_Pin   = LCD_BL_PIN;
	GPIO_Init(LCD_BL_PORT,&PORT);
	GPIO_PinAFConfig(LCD_BL_PORT,LCD_BL_PIN_SRC,LCD_BL_GPIO_AF);

	// TIM3_CH1 as PWM output
	LCD_BL_TIM_RCC |= LCD_BL_TIM_PERIPH; // Enable the TIMx peripheral
	LCD_BL_TIM->CR1   |= TIM_CR1_ARPE; // Auto-preload enable
	LCD_BL_TIM->CR2    = 0; // Clear this just for any case
	LCD_BL_TIM->CCMR1 |= TIM_CCMR1_OC1PE; // Output compare 1 preload enable
	LCD_BL_TIM->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1; // PWM mode 1
	LCD_BL_TIM->PSC    = SystemCoreClock / 40000; // 400Hz PWM
	LCD_BL_TIM->ARR    = 99; // 100 levels of backlight
	LCD_BL_TIM->CCR1   = 50; // 50% duty cycle
	LCD_BL_TIM->CCER  |= TIM_CCER_CC1NP | TIM_CCER_CC1E; // Output compare 1 enable, negative polarity
	LCD_BL_TIM->EGR    = 1; // Generate an update event to reload the prescaler value immediately
	LCD_BL_TIM->CR1   |= TIM_CR1_CEN; // Counter enable

//	i = 0;
//	int8_t di = 1;
//	while (1) {
//		LCD_BL_TIM->CCR1 = i;
//		i += di;
//		if ((i > 99) || (i == 0)) di *= -1;
//		Delay_ms(50);
//	}
	LCD_BL_TIM->CCR1 = lcd_backlight;




	// Configure USB sense pin
	RCC->AHBENR |= USB_SENS_PERIPH; // Enable USB sense pin port peripheral

	// Configure GPIO pin as input with pull-down
	USB_SENS_PORT->MODER &= ~GPIO_MODER_MODER10; // Input mode (reset state)
	USB_SENS_PORT->PUPDR &= ~GPIO_PUPDR_PUPDR10; // Floating (clear bits)
	USB_SENS_PORT->PUPDR |=  GPIO_PUPDR_PUPDR10_1; // Pull-down

/*
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
*/

/*
	// Enable the USB sense pin interrupt
	NVICInit.NVIC_IRQChannel = USB_SENS_EXTI_N;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 2;
	NVICInit.NVIC_IRQChannelSubPriority = 0;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVICInit);
*/

	// Simple enable the USB sense pin interrupt
//	NVIC_EnableIRQ(USB_SENS_EXTI_N);

	_USB_connected = (USB_SENS_PORT->IDR & USB_SENS_PIN) ? 1 : 0;

	// Configure the USB peripheral
	USB_HWConfig();
	// Initialize the USB device
	USB_Init();




	// Buttons initialization
	for (i = 0; i < 4; i++) memset(&BTN[i],0,sizeof(BTN[i]));
	BTN[0].PORT  = BTN0_PORT;
	BTN[0].PIN   = BTN0_PIN;
	BTN[0].EXTIn = BTN0_EXTI;

	BTN[1].PORT  = BTN1_PORT;
	BTN[1].PIN   = BTN1_PIN;
	BTN[1].EXTIn = BTN1_EXTI;

	BTN[2].PORT  = BTN2_PORT;
	BTN[2].PIN   = BTN2_PIN;
	BTN[2].EXTIn = BTN2_EXTI;

	BTN[3].PORT  = BTN3_PORT;
	BTN[3].PIN   = BTN3_PIN;
	BTN[3].EXTIn = BTN3_EXTI;

	// Enable button GPIOx peripherals (pins where buttons connected)
	RCC->AHBENR |= BTN0_PERIPH | BTN1_PERIPH | BTN2_PERIPH | BTN3_PERIPH;

	PORT.GPIO_Speed = GPIO_Speed_2MHz;
	PORT.GPIO_Mode  = GPIO_Mode_IN;
	PORT.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	// Button#0 (PB10)
	PORT.GPIO_Pin = BTN0_PIN;
	GPIO_Init(BTN0_PORT,&PORT);
	// Button#1 (PB11)
	PORT.GPIO_Pin = BTN1_PIN;
	GPIO_Init(BTN1_PORT,&PORT);
	// Button#2 (PA15)
	PORT.GPIO_Pin = BTN2_PIN;
	GPIO_Init(BTN2_PORT,&PORT);
	// Button#3 (PA0)
	PORT.GPIO_Pin = BTN3_PIN;
	GPIO_Init(BTN3_PORT,&PORT);

	// Configure buttons the EXTI lines
	// Button#0
	EXTI->PR    =  BTN0_EXTI; // Clear IT pending bit for EXTI
	EXTI->IMR  |=  BTN0_EXTI; // Enable interrupt request from EXTI
	EXTI->EMR  &= ~BTN0_EXTI; // Disable event on EXTI
	EXTI->RTSR |=  BTN0_EXTI; // Trigger rising edge enabled
	EXTI->FTSR |=  BTN0_EXTI; // Trigger falling edge enabled
	// PB10 as source input for EXTI10
	SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR3_EXTI10; // Clear bits (Set PA10 pin as input source)
	SYSCFG->EXTICR[2] |=  SYSCFG_EXTICR3_EXTI10_PB; // Set PB10 pin as input source

	// Button#1
	EXTI->PR    =  BTN1_EXTI; // Clear IT pending bit for EXTI
	EXTI->IMR  |=  BTN1_EXTI; // Enable interrupt request from EXTI
	EXTI->EMR  &= ~BTN1_EXTI; // Disable event on EXTI
	EXTI->RTSR |=  BTN1_EXTI; // Trigger rising edge enabled
	EXTI->FTSR |=  BTN1_EXTI; // Trigger falling edge enabled

	// PB11 as source input for EXTI11
	SYSCFG->EXTICR[2] &= ~SYSCFG_EXTICR3_EXTI11; // Clear bits (Set PA11 pin as input source)
	SYSCFG->EXTICR[2] |=  SYSCFG_EXTICR3_EXTI11_PB; // Set PB11 pin as input source

	// Button#2
	EXTI->PR    =  BTN2_EXTI; // Clear IT pending bit for EXTI
	EXTI->IMR  |=  BTN2_EXTI; // Enable interrupt request from EXTI
	EXTI->EMR  &= ~BTN2_EXTI; // Disable event on EXTI
	EXTI->RTSR |=  BTN2_EXTI; // Trigger rising edge enabled
	EXTI->FTSR |=  BTN2_EXTI; // Trigger falling edge enabled

	// PA15 as source input for EXTI15
	SYSCFG->EXTICR[3] &= ~SYSCFG_EXTICR4_EXTI15; // Clear bits (Set PA15 pin as input source)
	// In fact this is not necessary because it is the default condition after reset
	SYSCFG->EXTICR[3] |=  SYSCFG_EXTICR4_EXTI15_PA; // Set PA15 pin as input source

	// Button#3
	EXTI->PR    =  BTN3_EXTI; // Clear IT pending bit for EXTI
	EXTI->IMR  |=  BTN3_EXTI; // Enable interrupt request from EXTI
	EXTI->EMR  &= ~BTN3_EXTI; // Disable event on EXTI
	EXTI->RTSR |=  BTN3_EXTI; // Trigger rising edge enabled
	EXTI->FTSR |=  BTN3_EXTI; // Trigger falling edge enabled

	// PA0 as source input for EXTI0
	SYSCFG->EXTICR[0] &= ~SYSCFG_EXTICR1_EXTI0; // Clear bits (Set PA0 pin as input source)
	// In fact this is not necessary because it is the default condition after reset
	SYSCFG->EXTICR[0] |=  SYSCFG_EXTICR1_EXTI0_PA; // Set PA0 pin as input source

	// Simple enable the button pins interrupts
	NVIC_EnableIRQ(BTN0_EXTI_N);
	NVIC_EnableIRQ(BTN1_EXTI_N);
	NVIC_EnableIRQ(BTN2_EXTI_N);
	NVIC_EnableIRQ(BTN3_EXTI_N);




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
//	CHRG_STAT_PORT->PUPDR |=  GPIO_PUPDR_PUPDR2_0; // Pull-up




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




	// Configure Vbat measurement

	// Enable the PORTA peripheral
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA1 as analog (for ADC_IN1)
	GPIOA->MODER |=  GPIO_MODER_MODER1; // Analog mode

	// Enable the battery voltage divider
	PWR_VBAT_ENABLE_H();

	// Initialize the HSI clock
	RCC->CR |= RCC_CR_HSION; // Enable HSI
	while (!(RCC->CR & RCC_CR_HSIRDY)); // Wait until HSI stable

	// Initialize the ADC
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Enable the ADC1 peripheral clock
	ADC->CCR = ADC_CCR_TSVREFE; // Enable Vrefint and temperature sensor, ADC prescaler = HSI/1
	while(!(PWR->CSR & PWR_CSR_VREFINTRDYF)); // Wait until Vrefint stable

	// Continuous regular channels sequence ADC conversion with DMA

	// Enable the DMA1 peripheral
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	// Configure the DMA1 channel 1 (ADC1)
	// 16-bit peripheral to memory transfer, TC IRQ enabled, channel disabled
	DMA1_Channel1->CCR = DMA_CCR1_PL_0 | DMA_CCR1_MSIZE_0 | DMA_CCR1_PSIZE_0 | DMA_CCR1_MINC | DMA_CCR1_TCIE;
	DMA1_Channel1->CMAR = (uint32_t)ADC_raw; // Buffer address
	DMA1_Channel1->CPAR = (uint32_t)(&(ADC1->DR)); // Address of the peripheral data register
	DMA1_Channel1->CNDTR = ADC_count << 1; // Number of conversions

	// Enable the DMA1 channel 1 interrupt
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

	// Configure the ADC
	// Clear resolution bits: 12-bit resolution (Tconv = 12 ADCCLK cycles)
	ADC1->CR1 &= ~ADC_CR1_RES;
	// CR1_SCAN: scan mode enabled
	// CR1_PDI: power down during the idle phase
	// CR1_PDD: power down during the delay phase
	ADC1->CR1 |= ADC_CR1_SCAN | ADC_CR1_PDI | ADC_CR1_PDD;
	// Clear alignment bit: right alignment
	ADC1->CR2 &= ~ADC_CR2_ALIGN; // Right alignment
	// CR2_CONT: continuous conversion enabled
	// CR2_DMA: DMA mode enabled
	// CR2_DELS: delay selection (255APB clocks after end of conversion)
	ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_CONT | ADC_CR2_DELS;

	// ADC channels sample rate
	ADC1->SMPR3 |= ADC_SMPR3_SMP1; // Channel 1: 384 cycles
	ADC1->SMPR2 |= ADC_SMPR2_SMP17; // Channel 17: 384 cycles

	// Regular sequence
	ADC1->SQR1 = ADC_SQR1_L_0; // 2 conversions
	ADC1->SQR5 = ADC_SQR5_SQ2_0 | ADC_SQR5_SQ1_0 | ADC_SQR5_SQ1_4; // Channels: first ADC_IN1, then ADC_IN17

	// Enable the ADC
	ADC1->CR2 |= ADC_CR2_ADON;
	while (!(ADC1->SR & ADC_SR_ADONS)); // Wait until ADC is on

	// Enable the DMA channel 1 (ADC1)
	DMA1_Channel1->CCR |= DMA_CCR1_EN;

	// ..... The ADC is ready to do 32 measurements with DMA
	// ..... To start conversion just trigger SWSTART bit in the ADC1_CR2 register
	// Start conversion of regular channels
	ADC1->CR2 |= ADC_CR2_SWSTART;

/*
	// Single conversion injected channels sequence

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
*/




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
	}
    printf("---------------------------------------------\r\n");




	// Check the ALS
	printf("Ambient light sensor ");
	if (I2Cx_IsDeviceReady(I2C1,TSL2581_ADDR,10) == I2C_SUCCESS) {
		printf("ID: %02X\r\n",TSL2581_GetDeviceID());
		TSL2581_Init();
		// Set 99.9ms integration time (37 * 2.7ms)
//		TSL2581_SetTime(37);
		// Set 400ms integration time (148 * 2.7ms)
		TSL2581_SetTime(TSL2581_NOM_INTEG_CYCLE);
		// Set analog gain x1
//		TSL2581_SetGain(TSL2581_GAIN1);
		// Set analog gain x8
		TSL2581_SetGain(TSL2581_GAIN8);
		d0 = TSL2581_GetData0();
		d1 = TSL2581_GetData1();
		printf("D0 = %u D1 = %u Lux = %u\r\n",d0,d1,TSL2581_LuxCalc(d0,d1));
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
			printf("Temperature: %d.%uC\r\n",RT / 10,RT % 10);
			i = BMP180_hPa_to_mmHg(RP);
			printf("Pressure: %uPa [%u.%ummHg]\r\n",RP,i / 10,i % 10);
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
		BMC050_ACC_ConfigSlopeIRQ(3,8); // Motion detection sensitivity
		BMC050_ACC_IntPinMap(ACC_IM1_SLOPE); // Map slope interrupt to INT1 pin
		BMC050_ACC_SetIRQ(ACC_IE_SLOPEX | ACC_IE_SLOPEY | ACC_IE_SLOPEZ); // Detect motion by all axes
//		BMC050_ACC_LowPower(ACC_SLEEP_1000); // Low power with sleep duration 1s
		BMC050_ACC_LowPower(ACC_SLEEP_500); // Low power with sleep duration 500ms
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

	if (_SD_connected) {
		SD_SDIO_GPIO_Init();
		j = SD_Init();
		printf("SD_Init: %X\r\n",j);
	} else j = SDR_NoResponse;
	_SD_last_state = _SD_connected;

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
//			j = SD_CheckWrite(2048);
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
	    RTC_SetWakeUp(10);
	    _new_time = FALSE;
	    uint32_t bc = 0; // Block counter
	    uint32_t oc = 0; // 'Read OK' counter
	    uint32_t fc = 0; // 'Read FAIL' counter
	    while(1) {
	    	while (!_new_time) {
	    		j = SD_ReadBlock_DMA(0,(uint32_t *)sector,sizeof(sector));
	    		if (j == SDR_Success) {
	    			j = SD_CheckRead(sizeof(sector));
	    			if (j == SDR_Success) oc++; else fc++;
	    		} else fc++;
	    		bc++;
	    	}
	    	j = (oc * sizeof(sector));
	    	i = j / 10;
	    	printf("DMA read (block = %ub): %u blocks [%u bytes] (%u OK / %u FAIL) SPEED: ~%u b/s\r\n",sizeof(sector),bc,j,oc,fc,i);
	    	_new_time = FALSE;
	    	oc = 0; fc = 0; bc = 0;
	    }
	    // Single block read (512b):
	    // 16MHz 1-bit: 20850 blocks [10675200 bytes] (20850 OK / 0 FAIL) SPEED: ~1067520 b/s
	    // 16MHz 4-bit: 34664 blocks [17747968 bytes] (34664 OK / 0 FAIL) SPEED: ~1774796 b/s
	    // 24MHz 1-bit: 25415 blocks [13012480 bytes] (25415 OK / 0 FAIL) SPEED: ~1301248 b/s
	    // 24MHz 4-bit: 37650 blocks [19276800 bytes] (37650 OK / 0 FAIL) SPEED: ~1927680 b/s
	    // Multiple block read (2048b):
	    // 16MHz 4-bit: 16611 blocks [34019328 bytes] (16611 OK / 0 FAIL) SPEED: ~3401932 b/s
	    // 24MHz 4-bit: 16607 blocks [34011136 bytes] (16607 OK / 0 FAIL) SPEED: ~3401113 b/s
	    // Multiple block read (8192b):
	    // 24MHz 4-bit: 5716 blocks [46825472 bytes] (5716 OK / 0 FAIL) SPEED: ~4682547 b/s
	    // Multiple block read (16384b):
	    // 24MHz 4-bit: 3050 blocks [49971200 bytes] (3050 OK / 0 FAIL) SPEED: ~4997120 b/s
	    // Multiple block read (32768b):
	    // 24MHz 4-bit: 1578 blocks [51707904 bytes] (1578 OK / 0 FAIL) SPEED: ~5170790 b/s
*/

/*
		// Write block test
		for (i = 0; i < 1024; i++) sector[i] = '%';
		j = SD_WriteBlock(1024,(uint32_t *)sector,1024);
		for (i = 0; i < 2048; i++) sector[i] = '#';
		printf("Write block: %2X\r\n",j);
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
						printf("\"%s\" [%c%c%c%c%c] %10u\r\n",
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
						printf("\"%s\" [%c%c%c%c%c] %10u\r\n",
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




	// nRF24 SPI port initialization: 2 lines full duplex, highest speed (16MHz at 32MHz CPU)
    SPI2_HandleInit();
	SPIx_Init(&nRF24_SPI_PORT,SPI_DIR_DUPLEX,SPI_BR_2);

	// Initialize and configure nRF24
	nRF24_Init();
	j = nRF24_Check();
	printf("nRF24L01+: %s\r\n",(j) ? "PRESENT" : "FAIL");

	if (j) {
		// Dump nRF24L01+ registers
		printf("nRF24L01+ registers:\r\n");
		for (i = 0; i < 0x1d; i++) {
			j = nRF24_ReadReg(i);
			printf("R%02X=0x%02X%s",i,j,((i + 1) % 8) ? " " : "\r\n");
		}
		printf("\r\n");

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

	}
    printf("---------------------------------------------\r\n");




    // GPS
	printf("GPS initialization...\r\n");

	// Reset GPS variables
	USART_FIFO_pos = 0;
	GPS_buf_cntr = 0;
	_NMEA_total_size = 0;
	_NMEA_total_count = 0;
	GPS_new_data = FALSE;
	GPS_parsed = FALSE;
	NMEA_InitData();

	// The GPS USART port initialization
	UARTx_Init(GPS_USART_PORT,USART_TX | USART_RX,9600); // Use slow speed at startup
	UARTx_InitIRQ(GPS_USART_PORT,USART_IRQ_IDLE,0xff); // Enable the USART IDLE IRQ with standard priority

	// Configure the GPS USART DMA
	RCC->AHBENR |= RCC_AHBENR_DMA1EN; // Enable the DMA1 peripheral clock
	UARTx_ConfigureDMA(GPS_USART_PORT,USART_DMA_RX,USART_DMA_BUF_CIRC,USART_FIFO,USART_FIFO_SIZE); // Configure the USART RX DMA channel
	DMA1_Channel6->CCR |= DMA_CCR6_TCIE | DMA_CCR6_HTIE; // Enable the DMA channel6 HT/TC IRQs
	NVIC_EnableIRQ(DMA1_Channel6_IRQn); // Enable the DMA channel6 IRQ
	DMA1->IFCR = DMA_IFCR_CGIF6 | DMA_IFCR_CHTIF6 | DMA_IFCR_CTCIF6 | DMA_IFCR_CTEIF6; // Clear the DMA channel6 IRQ flags
	DMA1_Channel6->CCR |= DMA_CCR1_EN; // Enable the DMA channel6
	GPS_USART_PORT->CR3 |= USART_CR3_DMAR; // Enable the USART receiver DMA

	// Enable power to the GPS module
	PWR_GPS_ENABLE_H();

	// Initialize GPS module
	GPS_Init();




	printf("---------------------------------------------\r\n");
	BEEPER_Enable(2500,1);




/*
	while (1) {
		// Get charger STAT pin state
		i = 0;
		CHRG_STAT_PORT->PUPDR &= ~GPIO_PUPDR_PUPDR2; // Floating (clear bits)
		printf("FL=%u ",(unsigned int)((CHRG_STAT_PORT->IDR & CHRG_STAT_PIN) ? 1 : 0));
		if (CHRG_STAT_PORT->IDR & CHRG_STAT_PIN) i++;

		CHRG_STAT_PORT->PUPDR &= ~GPIO_PUPDR_PUPDR2; // Floating (clear bits)
		CHRG_STAT_PORT->PUPDR |=  GPIO_PUPDR_PUPDR2_0; // Pull-up
		printf("PU=%u ",(unsigned int)((CHRG_STAT_PORT->IDR & CHRG_STAT_PIN) ? 1 : 0));

		CHRG_STAT_PORT->PUPDR &= ~GPIO_PUPDR_PUPDR2; // Floating (clear bits)
		CHRG_STAT_PORT->PUPDR |=  GPIO_PUPDR_PUPDR2_1; // Pull-down
		printf("PD=%u\r\n",(unsigned int)((CHRG_STAT_PORT->IDR & CHRG_STAT_PIN) ? 1 : 0));

		CHRG_STAT_PORT->PUPDR &= ~GPIO_PUPDR_PUPDR2; // Floating (clear bits)

		Delay_ms(5000);
	}
*/




/*
    // Configure the RTC alarm A at 30th second on every minute
	_time.RTC_Hours   = 00;
	_time.RTC_Minutes = 00;
	_time.RTC_Seconds = 30;

	RTC_AlarmCmd(RTC_ALARM_A,DISABLE); // Disable the alarm A
	RTC_SetAlarm(RTC_ALARM_A,&_time,0,RTC_ALARM_MASK_DAY | RTC_ALARM_MASK_HRS | RTC_ALARM_MASK_MIN);
	RTC_ITConfig(RTC_IT_ALRA,ENABLE); // Enable the alarm A interrupt
	RTC_AlarmCmd(RTC_ALARM_A,ENABLE); // Enable the alarm A

	// Configure the RTC alarm A at 10th minute of every hour
	_time.RTC_Hours   = 00;
	_time.RTC_Minutes = 10;
	_time.RTC_Seconds = 00;

	RTC_AlarmCmd(RTC_ALARM_A,DISABLE); // Disable the alarm A
	RTC_SetAlarm(RTC_ALARM_A,&_time,0,RTC_ALARM_MASK_DAY | RTC_ALARM_MASK_HRS | RTC_ALARM_MASK_SEC);
	RTC_ITConfig(RTC_IT_ALRA,ENABLE); // Enable the alarm A interrupt
	RTC_AlarmCmd(RTC_ALARM_A,ENABLE); // Enable the alarm A

	// Configure the RTC alarm A at 3:30:33 every day
	_time.RTC_Hours   = 03;
	_time.RTC_Minutes = 30;
	_time.RTC_Seconds = 33;

	RTC_AlarmCmd(RTC_ALARM_A,DISABLE); // Disable the alarm A
	RTC_SetAlarm(RTC_ALARM_A,&_time,0,RTC_ALARM_MASK_DAY);
	RTC_ITConfig(RTC_IT_ALRA,ENABLE); // Enable the alarm A interrupt
	RTC_AlarmCmd(RTC_ALARM_A,ENABLE); // Enable the alarm A
*/




	// This for nRF24 packet handling
	uint8_t CRC_local;
	uint16_t wake_diff;

	// Moving sprite
	int16_t  hX = (SCR_W / 2) - 8;
	int16_t  hY = SCR_H - 32;
	int16_t  dX = -1;
	int16_t  dY = 1;
	uint8_t  sprite = 0;
	int8_t   dspr   = 1;
	uint32_t fps    = 0;

	while(1) {
		while (!GPS_new_data && !_new_packet) {
			// Redraw screen only if DMA transaction completed
			if (ST7541_SPI_PORT.DMA_TX.State != DMA_STATE_BUSY) {
				ST7541_Fill(0x0000);

				// Draw FPS with shadow effect
				lcd_color = gs_ltgray;
				i = PutIntF(3,3,fps,1,fnt7x10) + 4;
				PutStr(i,3,"FPS",fnt7x10);
				lcd_color = gs_black;
				i = PutIntF(2,2,fps,1,fnt7x10) + 4;
				PutStr(i,2,"FPS",fnt7x10);

				// Draw packets count
				i = 64;
				i += PutStr(i,2,"Pkts:",fnt5x7);
				PutIntU(i,2,_packets_rcvd,fnt5x7);
				i = 64;
				i += PutStr(i,10,"Lost:",fnt5x7);
				PutIntU(i,10,_packets_lost,fnt5x7);

				// Draw time
				RTC_GetDateTime(&_time,&_date);
				i = 0;
				i += PutIntLZ(i,22,_time.RTC_Hours,2,fnt5x7);
				i += DrawChar(i,22,':',fnt5x7);
				i += PutIntLZ(i,22,_time.RTC_Minutes,2,fnt5x7);
				i += DrawChar(i,22,':',fnt5x7);
				i += PutIntLZ(i,22,_time.RTC_Seconds,2,fnt5x7);
				i += 3;
				i += PutIntLZ(i,22,_date.RTC_Date,2,fnt5x7);
				i += DrawChar(i,22,'.',fnt5x7);
				i += PutIntLZ(i,22,_date.RTC_Month,2,fnt5x7);
				i += DrawChar(i,22,'.',fnt5x7);
				i += PutIntLZ(i,22,_date.RTC_Year,2,fnt5x7);
				PutStr(i + 3,22,RTC_DOW_STR[_date.RTC_WeekDay],fnt5x7);

				// GPS info
				i = 0;
				i += PutStr(i,30,"NMEA:",fnt5x7) - 1;
				i += PutIntU(i,30,NMEA_sentences_parsed,fnt5x7) - 1;
				i += DrawChar(i,30,'/',fnt5x7) - 1;
				i += PutIntU(i,30,NMEA_sentences_unknown,fnt5x7) - 1;
				i += DrawChar(i,30,'/',fnt5x7) - 1;
				i += PutIntU(i,30,NMEA_sentences_invalid,fnt5x7) + 5;
				i += PutStr(i,30,"Size:",fnt5x7) - 1;
				i += PutIntU(i,30,GPS_buf_cntr,fnt5x7);

				i = 0;
				i += PutStr(i,38,"Sat:",fnt5x7) - 1;
				i += PutIntU(i,38,GPSData.sats_used,fnt5x7) - 1;
				i += DrawChar(i,38,'/',fnt5x7) - 1;
				i += PutIntU(i,38,GPSData.sats_view,fnt5x7) + 5;
				i += PutStr(i,38,"Fix:",fnt5x7) - 1;
				i += PutIntU(i,38,GPSData.fix,fnt5x7) + 5;
				i += DrawChar(i,38,GPSData.mode,fnt5x7) + 6;

				i = 0;
				i += PutIntU(i,15,_NMEA_total_count,fnt3x5) + 3;
				i += PutIntU(i,15,_NMEA_total_size,fnt3x5);

				if (GPSData.valid) {
					// Latitude + longitude
					i = 0;
					i += PutIntF(i,46,GPSData.latitude,6,fnt5x7);
					i += DrawChar(i,46,',',fnt5x7);
					i += PutIntF(i,46,GPSData.longitude,6,fnt5x7);
				} else {
					// No coordinates
					i = 0;
					i += PutStr(i,46,"No coordinates",fnt5x7);
				}

				// ALS readings
				i = 0;
				i += PutStr(i,54,"ALS:",fnt5x7) - 1;
				i += PutIntU(i,54,lux,fnt5x7);
				i += PutStr(i,54,"lux G",fnt5x7);
				i += PutIntU(i,54,TSL2581_gain,fnt5x7) + 3;
				i += PutIntU(i,54,d0,fnt5x7) + 3;
//				i += PutIntU(i,54,d1,fnt5x7) + 2;

				// Barometer readings
				i = 0;
				i += PutStr(i,62,"BAR:",fnt5x7) - 1;
				i += PutIntF(i,62,RT,1,fnt5x7);
				i += DrawChar(i,62,'C',fnt5x7) + 5;
				i += PutIntF(i,62,RP,1,fnt5x7);
				i += PutStr(i,62,"mmHg",fnt5x7);

				// Voltages
				i = 0;
				i += PutStr(i,70,"Vb:",fnt5x7) - 1;
				i += PutIntF(i,70,Vbat << 1,3,fnt5x7);
				i += DrawChar(i,70,'V',fnt5x7) + 3;
				i += PutStr(i,70,"Vc:",fnt5x7) - 1;
				i += PutIntF(i,70,Vcpu,3,fnt5x7);
				i += DrawChar(i,70,'V',fnt5x7) + 3;

				// Charger STAT pin
				i = 0;
				i += PutStr(i,78,"FT:",fnt5x7) - 1;
				i += PutIntU(i,78,_STAT_ft,fnt5x7) + 3;
				i += PutStr(i,78,"PU:",fnt5x7) - 1;
				i += PutIntU(i,78,_STAT_pu,fnt5x7) + 3;
				i += PutStr(i,78,"PD:",fnt5x7) - 1;
				i += PutIntU(i,78,_STAT_pd,fnt5x7) + 3;
				if ((_STAT_ft == 1) && (_STAT_pd == 1)) {
					// High
					j = 0;
				} else {
					if (_STAT_pu == 0) {
						// Low
						j = 1;
					} else {
						// HiZ
						j = 2;
					}
				}
				switch (j) {
					case 0:
						PutStr(i,78,"High",fnt5x7);
						break;
					case 1:
						PutStr(i,78,"Low",fnt5x7);
						break;
					case 2:
						PutStr(i,78,"HiZ",fnt5x7);
						break;
					default:
						PutStr(i,78,"WTF?",fnt5x7);
						break;
				}

				// Draw icons
				// RF icon
				if (_time_no_signal > NO_SIGNAL_TIME) {
					DrawBitmap(0,SCR_H - 7,13,7,bmp_icon_SPD_none);
				} else if (_icon_RF) {
					DrawBitmap(0,SCR_H - 7,13,7,bmp_icon_SPD_norm);
				} else {
					DrawBitmap(0,SCR_H - 7,13,7,bmp_icon_SPD_inv);
				}
				// GPS icon
				if (GPSData.fix != 2 && GPSData.fix != 3) {
					DrawBitmap(16,SCR_H - 7,13,7,bmp_icon_GPS_NA);
				} else if (GPSData.fix == 2) {
					DrawBitmap(16,SCR_H - 7,13,7,bmp_icon_GPS_2D);
				} else {
					DrawBitmap(16,SCR_H - 7,13,7,bmp_icon_GPS_3D);
				}
				// SD card icon
				if (_SD_connected) {
					DrawBitmap(32,SCR_H - 7,13,7,bmp_icon_SD_norm);
				} else {
					DrawBitmap(32,SCR_H - 7,13,7,bmp_icon_SD_none);
				}
				// USB icon
				if (_USB_connected) {
					DrawBitmap(48,SCR_H - 7,13,7,bmp_icon_USB_on);
				} else {
					DrawBitmap(48,SCR_H - 7,13,7,bmp_icon_USB_none);
				}

				switch (bDeviceState) {
					case UNCONNECTED:
						PutStr(64,SCR_H - 7,"UNCONNECT",fnt5x7);
						break;
					case ATTACHED:
						PutStr(64,SCR_H - 7,"ATTACHED",fnt5x7);
						break;
					case POWERED:
						PutStr(64,SCR_H - 7,"POWERED",fnt5x7);
						break;
					case SUSPENDED:
						PutStr(64,SCR_H - 7,"SUSPENDED",fnt5x7);
						break;
					case ADDRESSED:
						PutStr(64,SCR_H - 7,"ADDRESSED",fnt5x7);
						break;
					case CONFIGURED:
						PutStr(64,SCR_H - 7,"CONFIGURED",fnt5x7);
						break;
					default:
						PutStr(64,SCR_H - 7,"UNKNOWN",fnt5x7);
						break;
				}

				// Draw walking toppler (stupid not optimized drawing, but it works ^_^)
				for (yy = 0; yy < 16; yy++) {
					for (xx = 0; xx < 4; xx++) {
						if (dX > 0) {
							// Toppler moves from left to right
							bb = ~toppler_walk[(yy * 4) + xx + (sprite << 6)];
							for (i = 4; i--; ) {
								Pixel(hX + (xx << 2) + i,hY + yy,(bb & 0x03));
								bb >>= 2;
							}
						} else {
							// Toppler moves from right to left, must mirror sprite horizontally
							bb = ~toppler_walk[(yy * 4) + 3 - xx + (sprite << 6)];
							for (i = 4; i--; ) {
								Pixel(hX + (xx << 2) + i,hY + yy,(bb & 0xc0) >> 6);
								bb <<= 2;
							}
						}
					}
				}

				// Move and animate sprite every 16th frame
				if ((k % 16) == 0) {
					sprite += dspr;
					if ((sprite > 6) || (sprite < 1)) dspr *= -1;

					hX += dX;
					hY += dY;
					if ((hX <  1) || (hX > SCR_W - 17)) dX *= -1;
					if ((hY < 87) || (hY > SCR_H - 24)) dY *= -1;
				}

				ST7541_Flush_DMA(NOBLOCK);
				k++;

				if (BTN[BTN_UP].state == BTN_Pressed) {
			    	if (lcd_backlight < 95) lcd_backlight += 5; else lcd_backlight = 100;
			    	LCD_BL_TIM->CCR1 = lcd_backlight;

			    	if (bDeviceState != CONFIGURED) {
			    		// Enable the USB peripheral clock
			    		RCC->APB1ENR |= RCC_APB1ENR_USBEN;

			    		// Enable the USB interrupts
			    		NVIC_EnableIRQ(USB_LP_IRQn);
			    		NVIC_EnableIRQ(USB_FS_WKUP_IRQn);

			    		// Initialize the USB device
			    		USB_Init();
			    	}
			    }

				if (BTN[BTN_DOWN].state == BTN_Pressed) {
			    	if (lcd_backlight > 5) lcd_backlight -= 5; else lcd_backlight = 0;
			    	LCD_BL_TIM->CCR1 = lcd_backlight;

			    	if (bDeviceState != UNCONNECTED) {
			    		// Power off the USB
			    		PowerOff();

			    		// Disable the USB peripheral clock
			    		RCC->APB1ENR &= ~RCC_APB1ENR_USBEN;

			    		// Disable the USB interrupts
			    		NVIC_DisableIRQ(USB_LP_IRQn);
			    		NVIC_DisableIRQ(USB_FS_WKUP_IRQn);

			    		// Change USB device state
			    		bDeviceState = UNCONNECTED;
			    	}
			    }
			}

			if (_ADC_completed && _new_time) {
				// ADC measurements are ready

				// Separate a raw ADC values from common buffer
				uint16_t *ptr;
				ptr = ADC_raw;
				for (i = 0; i < ADC_count; i++) {
					ADC_IN1_raws[i] = *ptr++;
					Vrefint_raws[i] = *ptr++;
				}

				// Calculate interquartile mean values of ADC readings
				ADC_IN1_raw = InterquartileMean(ADC_IN1_raws,ADC_count);
				Vrefint_raw = InterquartileMean(Vrefint_raws,ADC_count);

				// Convert ADC readings to voltage (using floating calculations)
//				Vbat    = (uint16_t)(((*VREFINT_CAL * ADC_IN1_raw * 3.0) / (Vrefint_raw * 4095.0)) * 1000);
//				Vrefint = (uint16_t)(((Vrefint_raw * 3.0) / 4095.0) * 1000);
//				Vcpu    = (uint16_t)(((*VREFINT_CAL * 3.0) / Vrefint_raw) * 1000);

				// Convert ADC readings to voltage (using 64-bit integer calculations)
				// this is 128 bytes less code than float calculations
				Vbat    = (uint16_t)(((uint64_t)(*VREFINT_CAL * ADC_IN1_raw) * 3000) / ((uint32_t)Vrefint_raw << 12));
				Vrefint = (uint16_t)(((uint32_t)Vrefint_raw * 3000) >> 12);
				Vcpu    = (uint16_t)(((uint32_t)*VREFINT_CAL * 3000) / (uint32_t)Vrefint_raw);

///*
				// Output results
				printf("VOL: Bat=[%u.%03uV -> %u.%03uV] Ref=%u.%03uV CPU=%u.%03uV\r\n",
						Vbat / 1000, Vbat % 1000,               // Vbat (from divider)
						(Vbat / 1000) << 1, (Vbat % 1000) << 1, // Vbat calculated
						Vrefint / 1000, Vrefint % 1000,         // Internal Vref
						Vcpu / 1000, Vcpu % 1000                // MCU supply voltage
						);
//*/

				// Renew DMA channel counter (channel already must be disabled in IRQ routine)
				DMA1_Channel1->CMAR  = (uint32_t)ADC_raw; // Buffer address
				DMA1_Channel1->CNDTR = ADC_count << 1;
				DMA1_Channel1->CCR  |= DMA_CCR1_EN;

				// Start new conversion
				_ADC_completed = FALSE;
				ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_CONT | ADC_CR2_SWSTART;
			}

			if (_new_time) {
				// Get ALS readings
				d0 = TSL2581_GetData0();
				d1 = TSL2581_GetData1();
				lux = TSL2581_LuxCalc(d0,d1);

				// Dummy auto-gain
				if (d0 > 50000) {
					if (TSL2581_gain > TSL2581_GAIN1) {
						TSL2581_gain--;
						TSL2581_SetGain(TSL2581_gain);
					}
				} else if (d0 < 2000) {
					if (TSL2581_gain < TSL2581_GAIN111) {
						TSL2581_gain++;
						TSL2581_SetGain(TSL2581_gain);
					}
				}

				printf("ALS: D0=%u D1=%u G=%u %ulux\r\n",d0,d1,TSL2581_gain,lux);

				// Get barometer readings
				BR = BMP180_GetReadings(&RT,&RP,BMP180_ADVRES);
				if (BR == BMP180_SUCCESS) {
					RP = BMP180_hPa_to_mmHg(RP);
					printf("BAR: T=%d.%dC P=%u.%ummHg\r\n",RT / 10,RT % 10,RP / 10,RP % 10);
				} else {
					printf("BAR: failed\r\n");
				}

				fps = k * 10;
				k = 0;
				_new_time = FALSE;

				// Check if SD card present
				_SD_connected = (SD_DETECT_PORT->IDR & SD_DETECT_PIN) ? 0 : 1;

				// Check if VUSB line active
				_USB_connected = (USB_SENS_PORT->IDR & USB_SENS_PIN) ? 1 : 0;

				// Reinitialize the USB peripheral only if it was suspended
				if (_USB_connected && (bDeviceState == SUSPENDED)) {
		    		// Enable the USB peripheral clock
		    		RCC->APB1ENR |= RCC_APB1ENR_USBEN;

		    		// Enable the USB interrupts
		    		NVIC_EnableIRQ(USB_LP_IRQn);
		    		NVIC_EnableIRQ(USB_FS_WKUP_IRQn);

		    		// Initialize the USB device
		    		USB_Init();
				}

				CHRG_STAT_PORT->PUPDR &= ~GPIO_PUPDR_PUPDR2; // Clear bits
				CHRG_STAT_PORT->PUPDR |=  GPIO_PUPDR_PUPDR2_1; // Pull-down
				asm volatile ("nop; nop; nop; nop; nop; nop; nop");
				_STAT_pd = (CHRG_STAT_PORT->IDR & CHRG_STAT_PIN) ? 1 : 0;

				// Get charger STAT pin state
				CHRG_STAT_PORT->PUPDR &= ~GPIO_PUPDR_PUPDR2; // Floating
				asm volatile ("nop; nop; nop; nop; nop; nop; nop");
				_STAT_ft = (CHRG_STAT_PORT->IDR & CHRG_STAT_PIN) ? 1 : 0;

				CHRG_STAT_PORT->PUPDR &= ~GPIO_PUPDR_PUPDR2; // Clear bits
				CHRG_STAT_PORT->PUPDR |=  GPIO_PUPDR_PUPDR2_0; // Pull-up
				asm volatile ("nop; nop; nop; nop; nop; nop; nop");
				_STAT_pu = (CHRG_STAT_PORT->IDR & CHRG_STAT_PIN) ? 1 : 0;

				CHRG_STAT_PORT->PUPDR &= ~GPIO_PUPDR_PUPDR2; // Leave it floating (clear bits)
			}
		}

		if (GPS_new_data) {
			// Dump a GPS data to the USB VCP
			VCP_SendBuf(GPS_buf,GPS_buf_cntr);

			// Update total bytes counter
			_NMEA_total_size  += GPS_buf_cntr;

			// Parse data received from GPS receiver
			NMEA_ParseBuf(GPS_buf,&GPS_buf_cntr);

			// Reset the new GPS data flag (data were parsed)
			GPS_new_data = FALSE;

			// Set flag indicating what GPS data was parsed
			GPS_parsed = TRUE;

			// Update NMEA sentences counter
			_NMEA_total_count += NMEA_sentences_parsed + NMEA_sentences_invalid + NMEA_sentences_unknown;
		}

		if (GPS_parsed) {
			// GPS data parsed

			// Update related variables if at least one sentence was parsed
			if (NMEA_sentences_parsed) {
				NMEA_CheckUsedSats();
				if (GPSData.fix == 3) {
					// GPS altitude makes sense only in case of 3D fix
					CurData.GPSAlt = GPSData.altitude;
					if (CurData.GPSAlt > CurData.MaxGPSAlt) CurData.MaxGPSAlt = CurData.GPSAlt;
					if (CurData.GPSAlt < CurData.MinGPSAlt) CurData.MinGPSAlt = CurData.GPSAlt;
				}
				if (GPSData.fix == 2 || GPSData.fix == 3) {
					// GPS speed makes sense only in case of 2D or 3D position fix
					CurData.GPSSpeed = GPSData.speed;
					if (CurData.GPSSpeed > CurData.MaxGPSSpeed) CurData.MaxGPSSpeed = CurData.GPSSpeed;
				}
			}

///*
			printf("GPS: NMEA=%u/%u/%u SAT=%u/%u FIX=%u MODE=\"%c\" FIX=\"%sVALID\"\r\n",
					NMEA_sentences_parsed,
					NMEA_sentences_unknown,
					NMEA_sentences_invalid,
					GPSData.sats_used,
					GPSData.sats_view,
					GPSData.fix,
					GPSData.mode,
					GPSData.valid ? "" : "IN"
			);
//*/

			if (GPSData.valid) {
				printf("GPS: LAT=%i LON=%i SPD=%u.%02ukm/h CRS=%u.%02u ALT=%im\r\n",
						GPSData.latitude,
						GPSData.longitude,
						GPSData.speed / 100,
						GPSData.speed % 100,
						GPSData.course / 100,
						GPSData.course % 100,
						GPSData.altitude
					);
			}

/*
			printf("GPS: FIX[%02u:%02u:%02u %02u.%02u.%04u]\t",
					GPSData.fix_time.Hours,
					GPSData.fix_time.Minutes,
					GPSData.fix_time.Seconds,
					GPSData.fix_date.Date,
					GPSData.fix_date.Month,
					GPSData.fix_date.Year
				);
			printf("DATE[%02u:%02u:%02u %02u.%02u.%04u]\t",
					GPSData.time.Hours,
					GPSData.time.Minutes,
					GPSData.time.Seconds,
					GPSData.fix_date.Date,
					GPSData.fix_date.Month,
					GPSData.fix_date.Year
				);
			printf("Date is %sVALID\r\n",GPSData.datetime_valid ? "" : "IN");
*/

/*
			if (GPSData.datetime_valid) {
				// Date and time obtained from GPS
				_time.RTC_Hours   = GPSData.time.Hours;
				_time.RTC_Minutes = GPSData.time.Minutes;
				_time.RTC_Seconds = GPSData.time.Seconds;
				_date.RTC_Date    = GPSData.date.Date;
				_date.RTC_Month   = GPSData.date.Month;
				_date.RTC_Year    = GPSData.date.Year - ((GPSData.date.Year > 1980) ? 2000 : 1900);

				RTC_AdjustTimeZone(&_time,&_date,3); // 3 --> +3 GMT offset
//				RTC_CalcDOW(&_date);
				RTC_SetDateTime(&_time,&_date);
			}
*/

			GPS_parsed = FALSE;
		}

		if (_new_packet) {
			CRC_local = CRC8_CCITT(nRF24_RX_Buf,nRF24_RX_PAYLOAD - 1);
			if (CRC_local == nRF24_Packet.CRC8) {
				if (_prev_wake_cntr == 0xDEADBEEF) _prev_wake_cntr = nRF24_Packet.cntr_wake;
				if (nRF24_Packet.cntr_wake < _prev_wake_cntr) _prev_wake_cntr = nRF24_Packet.cntr_wake;
				wake_diff = nRF24_Packet.cntr_wake - _prev_wake_cntr;

///*
				printf("wake: %u[%u] diff: %u vbat=%u.%02uV pkts: %u\\%u [%u%%]\r\n",
						nRF24_Packet.cntr_wake,
						_prev_wake_cntr,
						wake_diff,
						nRF24_Packet.vrefint / 100,nRF24_Packet.vrefint % 100,
						_packets_rcvd,
						_packets_lost,
						(_packets_lost * 100) / (_packets_rcvd + _packets_lost)
				);
//*/

				_prev_wake_cntr = nRF24_Packet.cntr_wake;
				if (wake_diff > 1) {
					_packets_lost += wake_diff - 1;
				}
			}
			_packets_rcvd++;
			_new_packet = FALSE;
			_icon_RF = !_icon_RF; // Blink RF icon
		}

		printf("---------------------------------------------\r\n");
	}




/*
	// Measure LCD performance
	// GFX Demo (rotozoomer with checkerboard or XOR texture)
	float ca,sa;
	float scalee = 0.5;
	float dscalee = 0.05;
	float angle = 0.0;
	float x,y;
	float xrow = 0.0;
	float yrow = 0.0;

    RTC_SetWakeUp(10);
    _new_time = FALSE;
    ST7541_Fill(0x0000);
    d0 = 0;
	while(1) {
		ca = scalee * cosf(angle);
		sa = scalee * sinf(angle);
		xrow = sa;
		yrow = ca;
		for (j = 48; j < 127; j++) {
			x = xrow;
			y = yrow;
			for (i = 1; i < 127; i++) {
				x += ca;
				y += sa;
//				Pixel(i,j,((((int8_t)x >> 3) & 1) ^ (((int8_t)y >> 3) & 1)) ? gs_black : gs_white); // Black and white checker board
				Pixel(i,j,(((int8_t)x >> 3) & 3) ^ (((int8_t)y >> 3) & 3)); // 2-bit XOR texture
			}
			xrow += -sa;
			yrow +=  ca;
		}
		angle += 0.05;
		scalee += ((scalee <= 0.10) || (scalee >= 1.3)) ? dscalee *= -1: dscalee;

		Rect(0,47,127,127,gs_black);
		FillRect(0,0,127,46,gs_white);

		// Draw FPS with shadow effect
		lcd_color = gs_ltgray;
		i = PutIntF(4,4,d0,1,fnt7x10) + 4;
		PutStr(i,4,"FPS",fnt7x10);
		lcd_color = gs_dkgray;
		i = PutIntF(3,3,d0,1,fnt7x10) + 4;
		PutStr(i,3,"FPS",fnt7x10);
		lcd_color = gs_black;
		i = PutIntF(2,2,d0,1,fnt7x10) + 4;
		PutStr(i,2,"FPS",fnt7x10);

		// Draw packets count
		i = 64;
		i += PutStr(i,2,"Pkts:",fnt5x7);
		PutIntU(i,2,_packets_rcvd,fnt5x7);
		i = 64;
		i += PutStr(i,10,"Lost:",fnt5x7);
		PutIntU(i,10,_packets_lost,fnt5x7);

		// Draw time
		RTC_GetDateTime(&_time,&_date);
		i = 0;
		i += PutIntLZ(i,22,_time.RTC_Hours,2,fnt5x7);
		i += DrawChar(i,22,':',fnt5x7);
		i += PutIntLZ(i,22,_time.RTC_Minutes,2,fnt5x7);
		i += DrawChar(i,22,':',fnt5x7);
		i += PutIntLZ(i,22,_time.RTC_Seconds,2,fnt5x7);
		i += 3;
		i += PutIntLZ(i,22,_date.RTC_Date,2,fnt5x7);
		i += DrawChar(i,22,'.',fnt5x7);
		i += PutIntLZ(i,22,_date.RTC_Month,2,fnt5x7);
		i += DrawChar(i,22,'.',fnt5x7);
		i += PutIntLZ(i,22,_date.RTC_Year,2,fnt5x7);
		PutStr(i + 3,22,RTC_DOW_STR[_date.RTC_WeekDay],fnt5x7);

		// GPS info
		i = 0;
		i += PutStr(i,30,"NMEA:",fnt5x7) - 1;
		i += PutIntU(i,30,GPS_sentences_parsed,fnt5x7) - 1;
		i += DrawChar(i,30,'/',fnt5x7) - 1;
		i += PutIntU(i,30,GPS_sentences_unknown,fnt5x7) + 5;
		i += PutStr(i,30,"Fix:",fnt5x7) - 1;
		i += PutIntU(i,30,GPSData.fix,fnt5x7) + 5;
		i += DrawChar(i,30,GPSData.mode,fnt5x7);
		i = 0;
		i += PutStr(i,38,"Sat:",fnt5x7) - 1;
		i += PutIntU(i,38,GPSData.sats_used,fnt5x7) - 1;
		i += DrawChar(i,38,'/',fnt5x7) - 1;
		i += PutIntU(i,38,GPSData.sats_view,fnt5x7) - 1;


		ST7541_Flush_DMA(NOBLOCK);
		k++;

		if (GPS_new_data) {
			// New GPS packet received

			// Dump a GPS data to the USB VCP
			VCP_SendBuf(GPS_buf,GPS_buf_cntr);

			// Parse received data
			GPS_Parse();
		}

		if (GPS_parsed) {
			// GPS data parsed
			printf("GPS parsed: PCKT=%s DT=%s FIX=%u %u DATE=%u %u\r\n--------\r\n",
					GPSData.valid ? "VALID" : "INVALID",
					GPSData.datetime_valid ? "VALID" : "INVALID",
					GPSData.fix_time,
					GPSData.fix_date,
					GPSData.time,
					GPSData.date
			);

			// Date and time obtained from GPS
			_time.RTC_Hours   = GPSData.fix_time.Hours;
			_time.RTC_Minutes = GPSData.fix_time.Minutes;
			_time.RTC_Seconds = GPSData.fix_time.Seconds;
			i = GPSData.fix_date / 1000000;
			_date.RTC_Date  = i;
			_date.RTC_Month = (GPSData.fix_date - (i * 1000000)) / 10000;
			if (GPSData.fix_date % 10000 > 1980) {
				_date.RTC_Year = (GPSData.fix_date % 10000) - 2000;
			} else {
				_date.RTC_Year = (GPSData.fix_date % 10000) - 1900;
			}
//			RTC_AdjustTimeZone(&_time,&_date,3); // 3 --> +3 GMT offset
			RTC_CalcDOW(&_date);
			RTC_SetDateTime(&_time,&_date);

			printf("_time: %02u:%02u:%02u _date: %02u.%02u.%02u %s\r\n",
					_time.RTC_Hours,
					_time.RTC_Minutes,
					_time.RTC_Seconds,
					_date.RTC_Date,
					_date.RTC_Month,
					_date.RTC_Year,
					RTC_DOW_STR[_date.RTC_WeekDay]);

			GPS_parsed = FALSE;
		}

		if (_new_time) {
			d0 = k;
			_new_time = FALSE;
			k = 0;
		}

		if (_new_packet) {
			CRC_local = CRC8_CCITT(nRF24_RX_Buf,nRF24_RX_PAYLOAD - 1);
			if (CRC_local == nRF24_Packet.CRC8) {
				if (_prev_wake_cntr == 0xDEADBEEF) _prev_wake_cntr = nRF24_Packet.cntr_wake;
				if (nRF24_Packet.cntr_wake < _prev_wake_cntr) _prev_wake_cntr = nRF24_Packet.cntr_wake;
				wake_diff = nRF24_Packet.cntr_wake - _prev_wake_cntr;
				printf("wake: %u[%u] diff: %u vbat=%u.%02uV pkts: %u\\%u [%u%%]\r\n",
						nRF24_Packet.cntr_wake,
						_prev_wake_cntr,
						wake_diff,
						nRF24_Packet.vrefint / 100,nRF24_Packet.vrefint % 100,
						_packets_rcvd,
						_packets_lost,
						(_packets_lost * 100) / (_packets_rcvd + _packets_lost)
				);
				_prev_wake_cntr = nRF24_Packet.cntr_wake;
				if (wake_diff > 1) {
					_packets_lost += wake_diff - 1;
				}
			}
			_packets_rcvd++;
			_new_packet = FALSE;
		}

	    if (BTN[BTN_UP].state == BTN_Pressed) {
	    	if (lcd_backlight < 95) lcd_backlight += 5; else lcd_backlight = 100;
	    	LCD_BL_TIM->CCR1 = lcd_backlight;
	    }
	    if (BTN[BTN_DOWN].state == BTN_Pressed) {
	    	if (lcd_backlight > 5) lcd_backlight -= 5; else lcd_backlight = 0;
	    	LCD_BL_TIM->CCR1 = lcd_backlight;
	    }
	    if (BTN[BTN_ESCAPE].state == BTN_Pressed) {
	    	// Reset the MCU
//	    	NVIC_SystemReset();

	    	// Setup partial display mode
			ST7541_SetDisplayPartial(32,0,64);
			ST7541_Contrast(5,5,10);

	    	// Put MCU into STANDBY mode
	    	PWR->CSR |= PWR_CSR_EWUP1; // Enable WKUP pin 1 (PA0)
	    	PWR->CSR |= PWR_CSR_EWUP2; // Enable WKUP pin 2 (PC13)
	    	SleepStandby();
	    }
	}
*/

    ST7541_Fill(0x0000);
	k = 0;
    for (i = 0; i < 8; i++) {
		d0 = 16 + (i * 12);
    	for (j = 0; j < 8; j++) {
    		d1 = 29 + (j * 12);
    		FillRect(d0,d1,d0 + 11,d1 + 11,(k) ? gs_white : gs_ltgray);
    		Rect(d0,d1,d0 + 11,d1 + 11,gs_black);
    		k = !k;
    	}
    	k = !k;
    }
    Rect(0,0,127,127,gs_black);
    ST7541_Flush_DMA(NOBLOCK);
    while(1);

/*
    // Test partial display and power saving
	ST7541_Flush();
	Delay_ms(2000);
	ST7541_Contrast(5,5,10);
	ST7541_SetDisplayPartial(32,0,64);
	Delay_ms(4000);
	ST7541_PowerSave(ON);
	Delay_ms(10000);
	ST7541_PowerSave(OFF);
	ST7541_SetDisplayPartial(0,0,128);
	ST7541_Contrast(lcd_res_ratio,lcd_lcd_bias,lcd_el_vol);
*/

    RTC_SetWakeUp(10);
    _new_time = FALSE;
	while(1) {
		k = 0;
		while (!_new_time) {
			ST7541_Flush_DMA(NOBLOCK);
//			ST7541_Flush_DMA(BLOCK);
//			ST7541_Flush();
			k++;
		}
		FillRect(2,2,63,12,gs_white);
		i = PutIntF(2,2,k,1,fnt7x10) + 2;
		PutStr(i,2,"FPS",fnt7x10);
		printf("DTM: %s, %02u:%02u:%02u %02u.%02u.20%02u\r\n",
				RTC_DOW_STR[_date.RTC_WeekDay],
				_time.RTC_Hours,
				_time.RTC_Minutes,
				_time.RTC_Seconds,
				_date.RTC_Date,
				_date.RTC_Month,
				_date.RTC_Year
			);
		printf("FPS = %u.%u\r\n",k / 10,k % 10);
		_new_time = FALSE;
	}
	// Results with 125kHz
	// ST7541_data :   3.8FPS
	// SPIx_SendBuf:   3.8FPS
	// DMA         :   3.8FPS

	// Result with 2MHz
	// ST7541_data :  34.7FPS
	// SPIx_SendBuf:  60.9FPS
	// DMA         :  60.9FPS

	// Result with 4MHz
	// ST7541_data :  47.3FPS
	// SPIx_SendBuf: 121.7FPS
	// SPIx_SendBuf16: 121.7FPS
	// DMA         : 121.7FPS

	// Result with 8MHz
	// ST7541_data :  57.4FPS
	// SPIx_SendBuf: 149.9FPS
	// SPIx_SendBuf_2: 204.7FPS
	// SPIx_Sendbuf16: 242.9FPS
	// DMA         : 243.0FPS
	// DMA16       : 242.7FPS

	// Results with 16MHz
	// ST7541_data :  64.5FPS
	// SPIx_SendBuf: 149.9FPS
	// SPIx_SendBuf_2: 204.8FPS
	// SPIx_SendBuf16: 430.4FPS <-- FAIL (LCD is not fast enough for this)
	// DMA         : 430.2FPS <-- FAIL (LCD is not fast enough for this)
	// DMA16       : 482.8FPS <-- FAIL (LCD is not fast enough for this)




	while(1) {
		// Date/time
		RTC_GetDateTime(&_time,&_date);
		printf("DTM: %s, %02u:%02u:%02u %02u.%02u.20%02u\r\n",
				RTC_DOW_STR[_date.RTC_WeekDay],
				_time.RTC_Hours,
				_time.RTC_Minutes,
				_time.RTC_Seconds,
				_date.RTC_Date,
				_date.RTC_Month,
				_date.RTC_Year
			);

//		printf("BTN: #0 IDR=%s\r\n",(GPIOB->IDR & GPIO_Pin_10) ? "HI" : "LOW");
//		printf("BTN: #1 IDR=%s\r\n",(GPIOB->IDR & GPIO_Pin_11) ? "HI" : "LOW");
//		printf("BTN: #2 IDR=%s\r\n",(GPIOA->IDR & GPIO_Pin_15) ? "HI" : "LOW");
//		printf("BTN: #3 IDR=%s\r\n",(GPIOA->IDR & GPIO_Pin_0)  ? "HI" : "LOW");

		// Buttons
		for (i = 0; i < 4; i++) {
			printf("BTN: #%u CNTR=%u HOLD=%u STATE=%02X IDR=%s\r\n",
					i,
					BTN[i].cntr,
					BTN[i].hold_cntr,
					BTN[i].state,
					(BTN[i].PORT->IDR & BTN[i].PIN) ? "HI" : "LOW");
		}

		// Measure Vbat
/*
		// Do 16 ADC measurements and calculate rough average
		ADC_IN1_raw = 0;
		Vrefint_raw = 0;
		for (i = 0; i < 16; i++) {
			ADC1->CR2 |= ADC_CR2_JSWSTART; // Start conversion of injected channels
			while (!(ADC1->SR & ADC_SR_JEOC)); // Wait until ADC conversions end
			ADC_IN1_raw    += ADC1->JDR1; // Read injected data register1 (ADC_IN1)
			Vrefint_raw += ADC1->JDR2; // Read injected data register2 (ADC_IN17)
			if (i) {
				ADC_IN1_raw >>= 1;
				Vrefint_raw >>= 1;
			}
			ADC1->SR &= ~ADC_SR_JEOC; // Clear JEOC bit (is this necessary?)
		}
*/

		// Do 16 ADC measurements
		for (i = 0; i < 16; i++) {
			ADC1->CR2 |= ADC_CR2_JSWSTART; // Start conversion of injected channels
			while (!(ADC1->SR & ADC_SR_JEOC)); // Wait until ADC conversions end
			ADC_IN1_raws[i]    = ADC1->JDR1; // Read injected data register1 (ADC_IN1)
			Vrefint_raws[i] = ADC1->JDR2; // Read injected data register2 (ADC_IN17)
		}
		// Calculate interquartile mean values of ADC readings
		ADC_IN1_raw = InterquartileMean(ADC_IN1_raws,16);
		Vrefint_raw = InterquartileMean(Vrefint_raws,16);

		// Convert ADC readings to voltage
		Vbat = (uint16_t)(((*VREFINT_CAL * ADC_IN1_raw * 3.0)/(Vrefint_raw * 4095.0)) * 1000);
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
				if ((j == SDR_Success) && (SDCard.Type != SDCT_MMC)) {
					// MMC doesn't support 4-bit bus
					if (SDCard.SCR[1] & 0x05) SD_SetBusWidth(SD_BUS_4BIT);
				}
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
			printf("[#%u TEMP: %d.%dC VREF: %u.%02uV LSI: %uHz OTX: %u/%u %02u:%02u:%02u %02u.%02u.20%02u]\r\n",
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
			printf("[VBAT: %u.%02uV WAKE: %u]\r\n",
					nRF24_Packet.vrefint / 100, nRF24_Packet.vrefint % 100,
					nRF24_Packet.cntr_wake);
			_new_packet = FALSE;
		}
#endif

		i = BMC050_ACC_GetIRQStatus();
		printf("ACC: %s [%04X]\r\n",(ACC_IRQ_PORT->IDR & ACC_IRQ_PIN) ? "HIGH" : "LOW",i);
//		BMC050_ACC_SetIRQMode(ACC_IM_RESET); // Clear IRQ bits

	    printf("---------------------------------------------\r\n");

//		BEEPER_Enable(111,1);

//		Delay_ms(2000);

	    ST7541_Fill(0x0000);

/*
		// Gray shades
	    FillRect(0,100,31,127,gs_black);
	    FillRect(32,100,63,127,gs_dkgray);
	    FillRect(64,100,95,127,gs_ltgray);
	    FillRect(96,100,127,127,gs_white);
*/
///*
	    // Chess board
	    k = 0;
	    for (i = 0; i < 8; i++) {
    		d0 = 16 + (i * 12);
	    	for (j = 0; j < 8; j++) {
	    		d1 = 29 + (j * 12);
	    		FillRect(d0,d1,d0 + 11,d1 + 11,(k) ? gs_white : gs_ltgray);
	    		Rect(d0,d1,d0 + 11,d1 + 11,gs_black);
	    		k = !k;
	    	}
	    	k = !k;
	    }
//*/

	    Rect(0,0,127,127,gs_black);
	    ST7541_Flush_DMA(NOBLOCK);

	    // Put MCU into SLEEP mode
		SleepWait();

	    // Put MCU into STOP mode
//	    SleepStop();

		// Put MCU into STANDBY mode
//		PWR->CSR |= PWR_CSR_EWUP1; // Enable WKUP pin 1 (PA0)
//		PWR->CSR |= PWR_CSR_EWUP2; // Enable WKUP pin 2 (PC13)
//		SleepStandby();

/*
	    if (BTN[BTN_UP].state == BTN_Pressed) {
	    	BEEPER_PlayTones(tones_3beep);
	    	while(_tones_playing);
	    	RTC_SetWakeUp(0); // Disable RTC wake-up
	    	PWR->CSR |= PWR_CSR_EWUP1; // Enable WKUP pin 1 (PA0)
	    	PWR->CSR |= PWR_CSR_EWUP2; // Enable WKUP pin 2 (PC13)
	    	SleepStandby();
	    } else {
	    	SleepWait();
	    }
*/
	}




	// Put MCU into STANDBY mode
	PWR->CSR |= PWR_CSR_EWUP1; // Enable WKUP pin 1 (PA0)
	PWR->CSR |= PWR_CSR_EWUP2; // Enable WKUP pin 2 (PC13)
	SleepStandby();

	// After STANDBY the MCU will go RESET routine, not here
	while(1);
}
