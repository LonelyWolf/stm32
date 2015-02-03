///////////////////
// STM32L151RBT6 //
///////////////////


/////////////////////////////////////////////////////////////////////////

#define DEBUG

/////////////////////////////////////////////////////////////////////////


#include <stm32l1xx_rcc.h>
#include <stm32l1xx_gpio.h>
#include <stm32l1xx_rtc.h>
#include <stm32l1xx_exti.h>
#include <stm32l1xx_syscfg.h>
#include <string.h> // For memset, memmove
#include <misc.h>

#include <wolk.h>
#include <delay.h>
#include <spi.h>
#include <i2c.h>
#include <uart.h>
#include <uc1701.h>
#include <nRF24.h>
#include <bmp180.h>
#include <RTC.h>
#include <GUI.h>
#include <GPS.h>
#include <USB.h>
#include <beeper.h>
#include <sdcard.h>
#include <log.h>
#include <EEPROM.h>

#include <font5x7.h>
#include <font7x10.h>
#include <resources.h>


/////////////////////////////////////////////////////////////////////////


#define ALT_MEASURE_DUTY_CYCLE          60  // Temperature/pressure measurement duty cycle (seconds)
#define NO_SIGNAL_TIME                  10  // Sensor signal timeout (seconds)
#define GPS_TIME_SYNC_DUTY_CYCLE       120  // Duty cycle to sync system time with GPS time (seconds)


/////////////////////////////////////////////////////////////////////////


uint8_t nRF24_RX_Buf[nRF24_RX_PAYLOAD];     // nRF24L01 payload buffer

bool _new_packet;                           // TRUE if new packet was received but not parsed
bool _new_time;                             // TRUE if time was updated
bool _bmp180_present;                       // TRUE if BMP180 sensor responded on I2C bus
bool _icon_RF;                              // Flag for RF icon flashing
bool _icon_LOG;                             // Flag for LOG icon flashing

RTC_TimeTypeDef _time;                      // Temporary structure for time
RTC_DateTypeDef _date;                      // Temporary structure for date

uint16_t _prev_cntr_SPD;                    // Last received cntr_SPD value
uint16_t _prev_tim_SPD;                     // Last received tim_SPD value
uint16_t _tim_excess;                       // Timer excess from 1 second ratio
uint32_t _cadence_cntr;                     // Cadence counter (for AVG calculation)
uint32_t _cadence_accum;                    // Cadence accumulator (for AVG calculation)
uint32_t _GPS_time_duty_cycle;              // Duty cycle to sync system time with GPS time
uint32_t _altitude_duty_cycle;              // Altitude measurement duty cycle

int16_t altitude_history[128];              // Last 128 altitude values

uint16_t _cdc[5];
uint32_t _cdc_avg;

uint16_t _DMA_cntr;                         // Last value of UART RX DMA counter (to track RX timeout)

uint32_t i,j;                               // THIS IS UNIVERSAL VARIABLES
uint32_t ccc;                               // Wake-ups count, for debug purposes


/////////////////////////////////////////////////////////////////////////


void ParsePacket(void);
void ParseGPS(void);
bool UpdateBMP180(void);


/////////////////////////////////////////////////////////////////////////


// Configure the different system clocks
void RCC_Configuration(void) {
	RCC_AHBPeriphClockCmd(
			RCC_AHBPeriph_GPIOA |     // PORTA
			RCC_AHBPeriph_GPIOC,      // PORTC
			ENABLE);

	RCC_APB1PeriphClockCmd(
			RCC_APB1Periph_TIM7,      // TIM7 peripheral (DMA timeout check)
			ENABLE);

	RCC_APB2PeriphClockCmd(
			RCC_APB2Periph_SYSCFG,    // System configuration controller
			ENABLE);
}

// Configure pins
void GPIO_Configuration(void) {
	GPIO_InitTypeDef PORT;

	// Charge STAT pin (PA15)
	PORT.GPIO_Mode  = GPIO_Mode_IN;
	PORT.GPIO_Speed = GPIO_Speed_2MHz;
	PORT.GPIO_PuPd  = GPIO_PuPd_UP;
	PORT.GPIO_Pin   = GPIO_Pin_15;
	GPIO_Init(GPIOA,&PORT);

	// Button#0 (PA5)
	PORT.GPIO_Mode = GPIO_Mode_IN;
	PORT.GPIO_PuPd = GPIO_PuPd_UP;
	PORT.GPIO_Pin  = BTN0_PIN;
	GPIO_Init(BTN0_PORT,&PORT);

	// Button#1 (PA7)
	PORT.GPIO_Pin  = BTN1_PIN;
	GPIO_Init(BTN1_PORT,&PORT);

	// Button#2 pin (PC10)
	PORT.GPIO_Pin  = BTN2_PIN;
	GPIO_Init(BTN2_PORT,&PORT);

	// Button#3 pin (PC11)
	PORT.GPIO_Pin  = BTN3_PIN;
	GPIO_Init(BTN3_PORT,&PORT);
}

// Configure the interrupt controller
void NVIC_Configuration(void) {
	NVIC_InitTypeDef NVICInit;
	EXTI_InitTypeDef EXTIInit;

	// Configure priority group: 4 bits for preemption priority, 0 bits for subpriority.
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	// TIM7 IRQ (USART DMA timeout control)
	NVICInit.NVIC_IRQChannel = TIM7_IRQn;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 0x0e; // lowest priority
	NVICInit.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Init(&NVICInit);

	// PC6 -> EXTI line 6 (nRF24L01 IRQ)
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource6);
	EXTIInit.EXTI_Line = EXTI_Line6;
	EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTIInit.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTIInit.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTIInit);

	// EXTI9_5 interrupt
	NVICInit.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 0x00; // highest priority
	NVICInit.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Init(&NVICInit);

	// EXTI15_10 interrupt
	NVICInit.NVIC_IRQChannelPreemptionPriority = 0x01; // highest priority
	NVICInit.NVIC_IRQChannelSubPriority = 0x01;
	NVICInit.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_Init(&NVICInit);

	// PA5 -> EXTI line 5 (Button#0)
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource5);
	EXTIInit.EXTI_Line = EXTI_Line5;
	EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTIInit.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTIInit);

	// PC10 -> EXTI line 10  (Button#2)
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource10);
	EXTIInit.EXTI_Line = EXTI_Line10;
	EXTI_Init(&EXTIInit);

	// PC10 -> EXTI line 11  (Button#3)
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource11);
	EXTIInit.EXTI_Line = EXTI_Line11;
	EXTI_Init(&EXTIInit);

	// PC12 -> EXTI line 12  (Button#1)
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource12);
	EXTIInit.EXTI_Line = EXTI_Line12;
	EXTI_Init(&EXTIInit);
}

// Initialize display:
//   - turn on and configure peripheral
//   - set backlight level
//   - set contrast
//   - set orientation
void Display_Init(void) {
	UC1701_Init();
	UC1701_Contrast(4,24);
	UC1701_Orientation(scr_normal);
	UC1701_SetBacklight(Settings.LCD_brightness);
}

// Configure peripherals
void InitPeripherals(void) {
	RCC_Configuration();
	GPIO_Configuration();
	NVIC_Configuration();

	// Initialize buzzer out
	BEEPER_Init();

	// Initialize delay timer without callback function
	Delay_Init(NULL);

	// UART port initialization
	UARTx_Init(GPS_USART_PORT,9600); // Use slow speed at startup
	UARTx_InitRxIRQ(GPS_USART_PORT,0x0c); // Configure UART RX IRQ (IRQ priority 0x0c)
	UARTx_InitRxDMA(GPS_USART_PORT,0x0d); // Configure UART RX DMA (IRQ priority 0x0d)

	// USB port initialization
//	USB_Init();

	// Configure basic timer TIM7 (for DMA timeout)
	TIM7->CR1  |= TIM_CR1_ARPE; // Auto-preload enable
	TIM7->PSC   = SystemCoreClock / 16000; // prescaler
	TIM7->ARR   = 999; // auto reload value
	TIM7->EGR   = 1; // Generate an update event to reload the prescaler value immediately
	TIM7->SR   &= ~TIM_SR_UIF; // Clear TIM IRQ flag
	TIM7->DIER |= TIM_DIER_UIE; // Enable TIMx interrupt

	// SPI1 port initialization (SD card)
	SPIx_Init(SPI1);
	SPIx_SetSpeed(SPI1,SPI_BR_256); // Lowest SPI1 speed

	// SPI2 port initialization (display and nRF24)
	SPIx_Init(SPI2);
	SPIx_SetSpeed(SPI2,SPI_BR_2); // Maximum SPI2 speed

	// Initialize and configure LCD
	Display_Init();
}

// Turn on transceiver and configure it for RX mode
void nRF24_SetRXMode(void) {
	nRF24_Wake();
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
    nRF24_ClearIRQFlags();
    nRF24_FlushRX();
}

// Turn off transceiver
void nRF24_Sleep(void) {
	nRF24_PowerDown();
}

// Inquiry status of the button
void Button_Inquiry(BTN_TypeDef *button) {
	if (button->PORT->IDR & button->PIN) {
	    // Button released
		button->hold_cntr = 0;
		if (button->state != BTN_Hold) button->cntr++;
		button->state = BTN_Released;

		// Disable sleep-on-exit (return to main loop from IRQ)
		SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;
	} else {
		// Button pressed
		button->hold_cntr = 0;
		button->state = BTN_Pressed;
		BEEPER_Enable(2000,5);
	}
	if (!_screensaver) UC1701_SetBacklight(Settings.LCD_brightness);
	_screensaver = FALSE;
	_time_idle = 0;
	_time_scr_timeout = 0;
}


/////////////////////////////////////////////////////////////////////////


// EXTI[5..9] lines IRQ handler
void EXTI9_5_IRQHandler(void) {
	nRF24_RX_PCKT_TypeDef RX_status;

	if (EXTI_GetITStatus(EXTI_Line6) != RESET) {
		// EXTI6 (nRF24L01 IRQ)
		UC1701_PauseSPI();
		RX_status = nRF24_RXPacket(nRF24_RX_Buf,nRF24_RX_PAYLOAD);
		nRF24_ClearIRQFlags();
		UC1701_ResumeSPI();
		if (RX_status == nRF24_RX_PCKT_PIPE0) {
			ParsePacket();
			_time_no_signal = 0;
			if (_time_scr_timeout > Settings.LCD_timeout) _time_scr_timeout = Settings.LCD_timeout;
//			BEEPER_Enable(444,1);
		}
		EXTI_ClearITPendingBit(EXTI_Line6);
	}

	if (EXTI_GetITStatus(EXTI_Line5) != RESET) {
		// EXTI5 (Button#1 -> "Up")
		Button_Inquiry(&BTN[BTN_UP]);
		EXTI_ClearITPendingBit(EXTI_Line5);
	}
}

// EXTI[10..15] lines IRQ handler
void EXTI15_10_IRQHandler(void) {
	if (EXTI_GetITStatus(EXTI_Line10) != RESET) {
		// EXTI10 (Button#3 -> "Enter")
		Button_Inquiry(&BTN[BTN_ENTER]);
		EXTI_ClearITPendingBit(EXTI_Line10);
	}

	if (EXTI_GetITStatus(EXTI_Line11) != RESET) {
		// EXTI11 (Button#4 -> "Escape")
		Button_Inquiry(&BTN[BTN_ESCAPE]);
		EXTI_ClearITPendingBit(EXTI_Line11);
	}

	if (EXTI_GetITStatus(EXTI_Line12) != RESET) {
		// EXTI12 (Button#2 -> "Down")
		Button_Inquiry(&BTN[BTN_DOWN]);
		EXTI_ClearITPendingBit(EXTI_Line12);
	}
}

// RTC wake-up IRQ handler
void RTC_WKUP_IRQHandler(void) {
	if (RTC->ISR & RTC_ISR_WUTF) {
		// RTC Wake-up interrupt
		RTC_GetDateTime(&RTC_Time,&RTC_Date);

		_time_no_signal++;
		_GPS_time_duty_cycle++;
		_altitude_duty_cycle++;
		_time_idle++;
		_time_scr_timeout++;
		_new_time = TRUE;

		// Counters overflow protection  FIXME: is it really necessary?
		if (_time_no_signal > 36000) _time_no_signal = 36000;
		if (_time_idle > 36000) _time_idle = 36000;
		if (_time_scr_timeout > 36000) _time_scr_timeout = 36000;
		if (_GPS_time_duty_cycle > 36000) _GPS_time_duty_cycle = 36000;
		if (_altitude_duty_cycle > 36000) _altitude_duty_cycle = 36000;

		if (_time_no_signal > NO_SIGNAL_TIME) {
			// Data packets did not received within desired time
			CurData.Speed     = 0;
			CurData.Cadence   = 0;
			_prev_cntr_SPD    = 0;
		}

		_screensaver = (_time_scr_timeout > GUI_SCREENSAVER_TIMEOUT) ? TRUE : FALSE;
		if (!_screensaver) {
			// The following procedures must be executed only when no screensaver active
			if (_time_scr_timeout > Settings.LCD_timeout && Settings.LCD_timeout) {
				UC1701_SetBacklight(0);
			} else {
				UC1701_SetBacklight(Settings.LCD_brightness);
			}
		}

		PWR->CR  |= PWR_CR_DBP; // Access to RTC, RTC Backup and RCC CSR registers enabled
		RTC->ISR &= ~RTC_ISR_WUTF; // Clear the RTC wake-up timer flag
		PWR->CR  &= ~PWR_CR_DBP; // Access to RTC, RTC Backup and RCC CSR registers disabled
		EXTI_ClearITPendingBit(EXTI_Line20);
	}
}

// TIM7 IRQ handler
void TIM7_IRQHandler(void) {
	TIM7->SR &= ~TIM_SR_UIF; // Clear the TIM7's interrupt pending bit (TIM7 rises only UPDATE IT)

	if (_DMA_cntr == (uint16_t)DMA1_Channel6->CNDTR) {
		// DMA pointer unchanged from last TIM IRQ -> UART timeout
		TIM7->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN)); // Disable TIM7
		DMA1_Channel6->CCR &= (uint16_t)(~DMA_CCR6_EN); // Disable DMA
		GPS_USART_PORT->CR3 &= ~USART_CR3_DMAR; // Disable DMA for USART RX
		// Copy rest of data from the FIFO buffer to the GPS buffer
		_DMA_cntr = USART_FIFO_SIZE - (uint16_t)DMA1_Channel6->CNDTR;
		// Trim data buffer to prevent overflow
		if (GPS_buf_cntr + _DMA_cntr >= GPS_BUFFER_SIZE) _DMA_cntr = GPS_BUFFER_SIZE - GPS_buf_cntr;
		memcpy(&GPS_buf[GPS_buf_cntr],USART_FIFO,_DMA_cntr);
		GPS_buf_cntr += _DMA_cntr;
		_DMA_cntr = 0;
		ParseGPS(); // About 3ms with full buffer
		// Enable the USART RX complete interrupt after the GPS data was parsed (ready to get new)
		GPS_USART_PORT->CR1 |= USART_CR1_RXNEIE;
		GPS_new_data = TRUE;
	} else {
		// Data is still coming from the USART
		_DMA_cntr = (uint16_t)DMA1_Channel6->CNDTR;
	}
}

// USART2 IRQ handler
void USART2_IRQHandler(void) {
	if (GPS_USART_PORT->SR & USART_SR_RXNE) {
		// Received data ready to be read
		GPS_USART_PORT->CR1 &= ~USART_CR1_RXNEIE; // Disable USART RX complete interrupt
		GPS_USART_PORT->CR3 |= USART_CR3_DMAR; // Enable DMA for USART RX
		DMA1_Channel6->CCR &= (uint16_t)(~DMA_CCR6_EN); // Disable DMA (it should be disabled though)
		DMA1->IFCR = DMA_IFCR_CTCIF6; // Clear the DMA1 channel6 transfer complete flag
		DMA1_Channel6->CNDTR = USART_FIFO_SIZE;
		DMA1_Channel6->CCR |= DMA_CCR6_EN; // Enable DMA
		TIM7->CNT  = 0;
		TIM7->CR1 |= TIM_CR1_CEN; // Enable TIM7
	}

	if (GPS_USART_PORT->SR & USART_SR_ORE) {
		// Overrun error
		(void)GPS_USART_PORT->DR; // Read the DR register to clear overrun flag
		// Whereas GPS data is broken disable UART RX DMA
		TIM7->CR1 &= ~TIM_CR1_CEN; // Disable TIM7
		DMA1_Channel6->CCR &= (uint16_t)(~DMA_CCR6_EN); // Disable DMA
		GPS_USART_PORT->CR1 &= ~USART_CR1_RXNEIE; // Disable USART RX complete interrupt
		GPS_buf_cntr = 0;
	}
}


// UART2_RX DMA IRQ handler
void DMA1_Channel6_IRQHandler() {
	uint16_t b_rcvd;

	// Channel 6 transfer complete
	if (DMA1->ISR & DMA_ISR_TCIF6) {
		// Clear the DMA1 channel6 transfer complete flag
		DMA1->IFCR = DMA_IFCR_CTCIF6;
		// Amount of received data (in fact should be equal to USART_FIFO_SIZE)
		b_rcvd = USART_FIFO_SIZE - (uint16_t)DMA1_Channel6->CNDTR;
		// Disable DMA (to reload counter value)
		DMA1_Channel6->CCR  &= (uint16_t)(~DMA_CCR6_EN);
		// Prevent data buffer overflow
		if (GPS_buf_cntr + b_rcvd >= GPS_BUFFER_SIZE) b_rcvd = GPS_BUFFER_SIZE - GPS_buf_cntr;
		memcpy(&GPS_buf[GPS_buf_cntr],USART_FIFO,b_rcvd); // Copy data to the GPS buffer
		GPS_buf_cntr += b_rcvd;
		// Reload DMA counter
		DMA1_Channel6->CNDTR = USART_FIFO_SIZE;
		// If the data buffer is full the DMA should be remain disabled until the buffer is processed.
		if (GPS_buf_cntr <= GPS_BUFFER_SIZE) DMA1_Channel6->CCR |= DMA_CCR6_EN; // Enable DMA
	};
}


/////////////////////////////////////////////////////////////////////////


// Callback function for delay timer overflow IRQ
void callback_Delay(void) {
	uint8_t i;

	// Check if a buttons is in hold state
	for (i = 0; i < 4; i++) {
		if (!(BTN[i].PORT->IDR & BTN[i].PIN)) {
			BTN[i].hold_cntr++;
			if (BTN[i].hold_cntr > 3) BTN[i].state = BTN_Hold;
		}
	}

	// Disable sleep-on-exit for return to main loop
	SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;

	// Set GUI refresh flag (twice per second)
	GUI_refresh = TRUE;
}


/////////////////////////////////////////////////////////////////////////


// Parse received nRF24L01 data packet
void ParsePacket(void) {
	uint32_t diff_SPD;
	uint32_t i;
	uint8_t CRC_local;

	// Check CRC of the packet and ignore it if CRC did not match
	CRC_local = CRC8_CCITT(nRF24_RX_Buf,nRF24_RX_PAYLOAD - 1);
	if (CRC_local != nRF24_RX_Buf[nRF24_RX_PAYLOAD - 1]) return;

	// memcpy doesn't work here due to struct alignments
	nRF24_Packet.cntr_SPD  = (nRF24_RX_Buf[0] << 8) + nRF24_RX_Buf[1];
	nRF24_Packet.tim_SPD   = (nRF24_RX_Buf[2] << 8) + nRF24_RX_Buf[3];
	nRF24_Packet.tim_CDC   = (nRF24_RX_Buf[4] << 8) + nRF24_RX_Buf[5];
	nRF24_Packet.vrefint   = ((nRF24_RX_Buf[6] & 0x03) << 8) + nRF24_RX_Buf[7];
	nRF24_Packet.cntr_wake = (nRF24_RX_Buf[8] << 8) + nRF24_RX_Buf[9];

	// Convert SPD impulses period into speed
	if (nRF24_Packet.tim_SPD > 0) {
//		CurData.Speed = Settings.WheelCircumference * (992.9696969 * 0.36) / nRF24_Packet.tim_SPD;
		CurData.Speed = (Settings.WheelCircumference * 357.469090884) / nRF24_Packet.tim_SPD;
//		CurData.Speed = Settings.WheelCircumference * (1007.08 * 0.36) / nRF24_Packet.tim_SPD;
//		CurData.Speed = (Settings.WheelCircumference * 362.5488) / nRF24_Packet.tim_SPD;
		if (CurData.Speed > 999) CurData.Speed = 999; // Maximum 99.9km/h can be displayed
	} else CurData.Speed = 0;

	// Convert CDC impulses period into cadence
	if (nRF24_Packet.tim_CDC > 0) {
//		CurData.Cadence = (uint32_t)((60.0 / nRF24_Packet.tim_CDC) * 992.9696969);
		CurData.Cadence = (uint32_t)(59578.181814 / nRF24_Packet.tim_CDC);
//		CurData.Cadence = (uint32_t)((60.0 / nRF24_Packet.tim_CDC) * 1007.08);
//		CurData.Cadence = (uint32_t)(60424.8 / nRF24_Packet.tim_CDC);
		// TODO: filter some crazy high values
	} else CurData.Cadence = 0;

	// Average cadence for last five values
	memmove(&_cdc[1],&_cdc[0],sizeof(_cdc) - sizeof(*_cdc));
	_cdc[0] = CurData.Cadence;
	_cdc_avg = 0;
	for (i = 0; i < 5; i++) _cdc_avg += _cdc[i];
	_cdc_avg /= 5;

	// Update maximum values
	if (CurData.Speed > CurData.MaxSpeed) CurData.MaxSpeed = CurData.Speed;
	if (CurData.Cadence > CurData.MaxCadence) CurData.MaxCadence = CurData.Cadence;

	// Update current trip time
	// Transceiver send packet every second, so don't fuss and simply add one second to trip time
	CurData.TripTime++;
/*
	if (nRF24_Packet.tim_SPD != _prev_tim_SPD) {
	if (nRF24_Packet.tim_SPD < 1007) {
		// Pulse interval was shorter than one second therefore just
		// add one second to the trip counter
		CurData.TripTime++;
	} else {
		// Pulse interval was longer than one second, therefore convert it
		// to seconds and remember fractional part for further usage
		tmp = (nRF24_Packet.tim_SPD / 1007.08) + (_tim_excess / 1000.0);
		CurData.TripTime += (uint32_t)tmp;
		_tim_excess = (uint32_t)(tmp * 1000) - ((uint32_t)tmp * 1000);
	}
	_prev_tim_SPD  = nRF24_Packet.tim_SPD;
*/

	// Update odometer and current trip distance
	if (nRF24_Packet.cntr_SPD != 0 && nRF24_Packet.cntr_SPD != _prev_cntr_SPD) {
		if (nRF24_Packet.cntr_SPD >= _prev_cntr_SPD) {
			diff_SPD = nRF24_Packet.cntr_SPD - _prev_cntr_SPD;
		} else {
			diff_SPD = 65535 - _prev_cntr_SPD + nRF24_Packet.cntr_SPD;
		}
		CurData.dbg_cntr_diff = diff_SPD;
		if (diff_SPD > 1000) diff_SPD = 1; // FIXME: this is lame workaround for crazy high values
		CurData.Odometer += Settings.WheelCircumference * diff_SPD;
		CurData.TripDist += Settings.WheelCircumference * diff_SPD;
	} else {
		CurData.dbg_cntr_diff = 0;
	}
	_prev_cntr_SPD = nRF24_Packet.cntr_SPD;
	CurData.dbg_prev_cntr = _prev_cntr_SPD;

	// Update average values
	CurData.AvgSpeed = (uint32_t)((CurData.TripDist * 0.36) / CurData.TripTime);
	if (CurData.Cadence > 0) {
		// A simple calculation of the average cadence.
		_cadence_cntr++;
		// At constant value of cadence 300RPM this accumulator will overflow
		// after about 4000 hours of ride
		_cadence_accum += CurData.Cadence;
		CurData.AvgCadence = _cadence_accum / _cadence_cntr;
	}

	_new_packet = TRUE;
}

// Parse GPS data
void ParseGPS(void) {
//	BEEPER_Enable(222,1);
	GPS_InitData(); // Clear previously parsed GPS data
	while (GPS_msg.end < GPS_buf_cntr) {
		GPS_FindSentence(&GPS_msg,GPS_buf,GPS_msg.end,GPS_buf_cntr);
		if (GPS_msg.type != NMEA_BAD) {
			GPS_sentences_parsed++;
			GPS_ParseSentence(GPS_buf,&GPS_msg);
		} else GPS_sentences_unknown++;
	}
	GPS_buf_cntr = 0;
	if (GPS_sentences_parsed) {
		GPS_CheckUsedSats();
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
		GPS_new_data = FALSE;
		GPS_parsed = TRUE;
	}
}

// Update data from BMP180
// return: TRUE if data acquired successfully, FALSE otherwise
// note: really slow routine
bool UpdateBMP180(void) {
	bool result = FALSE;
	int16_t _raw_temp;
	int32_t _raw_press;

	if (_bmp180_present) {
		// TODO: it is worth trying to use the I2C IRQ instead of polling the I2C flags?
		if (BMP180_GetReadings(&_raw_temp,&_raw_press,BMP180_UHIRES)) {  // <--- UHIRES mode takes about 28ms
//		if (BMP180_GetReadings(&_raw_temp,&_raw_press,BMP180_ADVRES)) {  // <--- ADVRES mode takes about 80ms
			CurData.Temperature = _raw_temp;
			if (CurData.Temperature > CurData.MaxTemperature) CurData.MaxTemperature = CurData.Temperature;
			if (CurData.Temperature < CurData.MinTemperature) CurData.MinTemperature = CurData.Temperature;

			CurData.Pressure = _raw_press;
			if (CurData.Pressure > CurData.MaxPressure) CurData.MaxPressure = CurData.Pressure;
			if (CurData.Pressure < CurData.MinPressure) CurData.MinPressure = CurData.Pressure;

			CurData.Altitude = BMP180_hPa_to_Altitude(CurData.Pressure);
			if (CurData.Altitude > CurData.MaxAltitude) CurData.MaxAltitude = CurData.Altitude;
			if (CurData.Altitude < CurData.MinAltitude) CurData.MinAltitude = CurData.Altitude;

			memmove(&altitude_history[1],&altitude_history[0],sizeof(altitude_history) - sizeof(*altitude_history));
			altitude_history[0] = CurData.Altitude;

			// Reset this counter only if readings obtained successfully, otherwise next try will be on next cycle
			_altitude_duty_cycle = 0;

			GUI_new_BMP180 = TRUE;
			result = TRUE;
		}
	}
	return result;
}


/////////////////////////////////////////////////////////////////////////


int main(void) {
	// Update SystemCoreClock according to Clock Register Values
	SystemCoreClockUpdate();

#ifdef DEBUG
	// Enable debugging when the MCU is in low power modes
	DBGMCU->CR |= DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY;
#endif

/////////////////////////////////////////////////////////////////////////
//  Boot section
/////////////////////////////////////////////////////////////////////////

	// Variables initialization ------------------------------
	memset(&nRF24_RX_Buf,0,nRF24_RX_PAYLOAD);
	memset(&nRF24_Packet,0,sizeof(nRF24_Packet));
	memset(&CurData,0,sizeof(CurData));
	memset(&altitude_history,0,sizeof(altitude_history));
	memset(&GPS_buf,'#',sizeof(GPS_buf));
	memset(&USART_FIFO,0,USART_FIFO_SIZE);
	memset(&_cdc,0,sizeof(_cdc));
	GPS_InitData();

	_new_packet        = FALSE;
	_new_time          = FALSE;
	_screensaver       = FALSE;
	_bmp180_present    = FALSE;
	_SD_present        = FALSE;
	_logging           = FALSE;
	_icon_RF           = TRUE;
	_icon_LOG          = TRUE;
	GPS_new_data       = FALSE;
	GPS_parsed         = FALSE;
	GPS_buf_cntr       = 0;
	_DMA_cntr          = 0;
	_prev_cntr_SPD     = 0;
	_prev_tim_SPD      = 0;
	_tim_excess        = 0;
	_cadence_cntr      = 0;
	_cadence_accum     = 0;
	_time_no_signal    = 65535;
	_time_idle         = 0;
	_time_scr_timeout  = 0;
	_cdc_avg           = 0;
	_altitude_duty_cycle = ALT_MEASURE_DUTY_CYCLE + 1;
	_GPS_time_duty_cycle = GPS_TIME_SYNC_DUTY_CYCLE + 1;
	CurData.MinGPSAlt = 0x7FFFFFFF; // LONG_MAX - first time when altitude will be acquired it becomes normal value

	// Buttons initialization
	for (i = 0; i < 4; i++) memset(&BTN[i],0,sizeof(BTN[i]));
	BTN[0].PORT = BTN0_PORT;
	BTN[0].PIN  = BTN0_PIN;
	BTN[1].PORT = BTN1_PORT;
	BTN[1].PIN  = BTN1_PIN;
	BTN[2].PORT = BTN2_PORT;
	BTN[2].PIN  = BTN2_PIN;
	BTN[3].PORT = BTN3_PORT;
	BTN[3].PIN  = BTN3_PIN;

	// These values stored in EEPROM
	Settings.GMT_offset = 3;
	Settings.WheelCircumference = 206;
	Settings.altitude_home = 178;
	Settings.LCD_brightness = 50;
	Settings.LCD_timeout = 30;

	// Read settings from EEPROM
	ReadBuffer_EEPROM(DATA_EEPROM_START_ADDR,(uint32_t *)&Settings,sizeof(Settings));

	// -------------------- end of variables initialization

	InitPeripherals();

	// Boot screen
	UC1701_Fill(0x00);
	GUI_DrawBitmap(98,39,30,25,&bmp_bike_man[0]);
	PutStr(104,0,"WBC",fnt7x10);

	i = PutStr(0,scr_height - 7,"STAT:",fnt5x7) - 1;
	PutStr(i,scr_height - 7,GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15) == Bit_RESET ? "0" : "1",fnt5x7);

	// Init the RTC and configure it clock to LSE
	i = PutStr(0,0,"LSE:",fnt5x7) - 1;
	UC1701_Flush();
	RTC_Config();
	PutStr(i,0,"OK",fnt5x7);
	UC1701_Flush();

	// Configure nRF24L01+
	i = PutStr(0,8,"nRF24L01:",fnt5x7) - 1;
	UC1701_Flush();
    nRF24_Init();
    if (nRF24_Check()) {
    	nRF24_SetRXMode();
    	PutStr(i,8,"OK",fnt5x7);
    	UC1701_Flush();
    } else {
    	PutStr(i,8,"Fail",fnt5x7);
        UC1701_Flush();
        _screensaver = TRUE;
    	while(1) SleepStop(); // Infinite deep sleep
    }

    // I2C2 port initialization
    PutStr(0,16,"BMP180:",fnt5x7);
    UC1701_Flush();
    // I2C fast mode (400kHz)
    if (I2Cx_Init(I2C2,400000) == I2C_SUCCESS) {
        BMP180_Reset(); // Send reset command to BMP180
        Delay_ms(15); // Wait for BMP180 startup time (10ms by datasheet)
        if (BMP180_Check() == BMP180_SUCCESS) {
    		_bmp180_present = TRUE;
    		PutChar(42,16,'v',fnt5x7);
    		i = BMP180_GetVersion();
    		PutInt(48,16,i,fnt5x7);
    		BMP180_ReadCalibration();
			CurData.MinTemperature =  32767;
			CurData.MaxTemperature = -32767;
			CurData.MinPressure = 2147483647; // LONG_MAX - it becomes normal when the pressure will be acquired normally first time
    		if (UpdateBMP180()) {
        		GUI_PutTemperature(60,16,CurData.Temperature,fnt5x7);
        		GUI_PutPressure(60,24,CurData.Pressure,PT_mmHg,fnt5x7);
        		altitude_history[0] = BMP180_hPa_to_Altitude(CurData.Pressure);
    		} else {
    			PutStr(60,16,"Readings",fnt5x7);
    			PutStr(60,24,"failed",fnt5x7);
    		}
    	} else {
    		PutStr(42,16,"Not present",fnt5x7);
    	}
    } else {
    	PutStr(42,16,"I2C timeout",fnt5x7);
    }
	UC1701_Flush();

	// Initialization of SD card
	i = PutStr(0,32,"SD:",fnt5x7) - 1;
	UC1701_Flush();
	j = (uint32_t)SD_Init();
	if (j == SDR_Success) {
		SD_ReadCSD();
		_SD_present = TRUE;
		i += PutChar(i,32,'v',fnt5x7);
		i += PutInt(i,32,SDCard.CardType,fnt5x7) + 5;
		if (SDCard.CardType == SDCT_SDHC) {
			i += PutInt(i,32,SDCard.CardCapacity / 1024,fnt5x7);
		} else {
			i += PutInt(i,32,SDCard.CardCapacity / 1048576,fnt5x7);
		}
		PutStr(i,32,"Mb",fnt5x7);

		// Logging initialization (check for the file system)
		i = PutStr(0,40,"LOG:",fnt5x7) - 1;
		UC1701_Flush();
		j = LOG_Init();
		if (j == LOG_OK) {
			PutStr(i,40,"OK",fnt5x7);
		} else {
			PutHex(i,40,j,fnt5x7);
		}
	} else {
		PutHex(i,32,j,fnt5x7);
	}
	UC1701_Flush();

	// Set time only in case of POR (power on reset)
	if (RCC->CSR & RCC_CSR_PORRSTF) {
		_time.RTC_Hours   = 0;
		_time.RTC_Minutes = 0;
		_time.RTC_Seconds = 0;
		_date.RTC_Date    = 16;
		_date.RTC_Month   = 04;
		_date.RTC_Year    = 14;
		_date.RTC_WeekDay = 3;
		RTC_SetDateTime(&_time,&_date);
	}
	RTC_GetDateTime(&RTC_Time,&RTC_Date);
	RCC->CSR |= RCC_CSR_RMVF; // Clear the reset flags

	ccc = 0;

	// Reinitialize delay timer with callback function
	Delay_Init(callback_Delay);

	// Configure wake-up timer to wake every second and enable it
	RTC_SetWakeUp(1);

	// Enable USART RX IRQ
	GPS_USART_PORT->SR &= USART_SR_RXNE; // Clear RXNE flag
	GPS_USART_PORT->CR1 |= USART_CR1_RXNEIE;

	// Initialize GPS module
	GPS_Init();

	// Boot complete, do some noise
	BEEPER_PlayTones(tones_startup);


/////////////////////////////////////////////////////////////////////////
//	Main loop
/////////////////////////////////////////////////////////////////////////

	while(1) {
		if (_screensaver) {
			RTC_SetWakeUp(GUI_SCREENSAVER_UPDATE);
			nRF24_Sleep(); // Turn off the receiver
			GPS_SendCommand(PMTK_CMD_STANDBY_MODE); // Put GPS module into standby mode (not supported by EB-500)
			UC1701_SetBacklight(0);

			GUI_ScreenSaver();

 			// Reinitialize display
 			Display_Init();
			UC1701_Fill(0x00);

			GPS_SendCommand(PMTK_TEST); // Send dummy command to GPS module to wake it up (actually one byte is enough)
			nRF24_SetRXMode(); // Wake the receiver and configure it for RX mode
 			_GPS_time_duty_cycle = GPS_TIME_SYNC_DUTY_CYCLE + 1;
 			RTC_SetWakeUp(1); // Wake every second

 			GUI_refresh = TRUE;
		}

		if (_new_packet) {
			// New packet received and parsed
			_new_packet = FALSE;
			_icon_RF = !_icon_RF; // Blink RF icon
		}

/*
		if (GPS_new_data) {
			// New GPS packets received
			ParseGPS(); // About 3ms with full buffer
			// Enable the USART2 RX complete interrupt when the GPS data was parsed (ready to get new)
			GPS_USART_PORT->CR1 |= USART_CR1_RXNEIE;
		}
*/

		if (GPS_parsed) {
			// GPS data parsed
			if (GPSData.datetime_valid && _GPS_time_duty_cycle > GPS_TIME_SYNC_DUTY_CYCLE) {
				// Date and time obtained from GPS
				_time.RTC_Hours   =  GPSData.time / 3600;
				_time.RTC_Minutes = (GPSData.time / 60) % 60;
				_time.RTC_Seconds =  GPSData.time % 60;
				i = GPSData.date / 1000000;
				_date.RTC_Date  = i;
				_date.RTC_Month = (GPSData.date - (i * 1000000)) / 10000;
				_date.RTC_Year  = (GPSData.date % 10000) - 2000;
				RTC_AdjustTimeZone(&_time,&_date,Settings.GMT_offset);
				RTC_SetDateTime(&_time,&_date);
				_GPS_time_duty_cycle = 0;
			}
			GPS_parsed = FALSE;
		}

		if (_new_time) {
			// Time updated (every second)
			_new_time = FALSE;
			// ATTENTION! BMP180 polling is pretty slow!
			if (_bmp180_present && _altitude_duty_cycle > ALT_MEASURE_DUTY_CYCLE) UpdateBMP180();

			// Write debug information to the log file
			if (_logging) {
				_icon_LOG = !_icon_LOG; // Blink LOG icon

				LOG_WriteDate(RTC_Date.RTC_Date,RTC_Date.RTC_Month,RTC_Date.RTC_Year);
				LOG_WriteStr(";");
				LOG_WriteTime(RTC_Time.RTC_Hours,RTC_Time.RTC_Minutes,RTC_Time.RTC_Seconds);
				LOG_WriteStr(";");
				LOG_WriteIntU(nRF24_Packet.cntr_wake);
				LOG_WriteStr(";");
				LOG_WriteIntU(nRF24_Packet.cntr_SPD);
				LOG_WriteStr(";");
				LOG_WriteIntU(_prev_cntr_SPD);
				LOG_WriteStr(";");
				LOG_WriteIntU(CurData.dbg_cntr_diff);
				LOG_WriteStr(";");
				LOG_WriteIntU(CurData.dbg_prev_cntr);
				LOG_WriteStr(";");
				LOG_WriteIntF(CurData.Speed,1);
				LOG_WriteStr(";");
				LOG_WriteIntU(CurData.Cadence);
				LOG_WriteStr(";");
				LOG_WriteIntU(_cdc[0]);
				LOG_WriteStr(";");
				LOG_WriteIntU(_cdc[1]);
				LOG_WriteStr(";");
				LOG_WriteIntU(_cdc[2]);
				LOG_WriteStr(";");
				LOG_WriteIntU(_cdc[3]);
				LOG_WriteStr(";");
				LOG_WriteIntU(_cdc[4]);
				LOG_WriteStr(";");
				LOG_WriteIntU(_cdc_avg);
				LOG_WriteStr(";");
				LOG_WriteIntF(CurData.Odometer,5);
				LOG_WriteStr(";");
				if (GPSData.valid) {
					if (GPSData.datetime_valid) {
						LOG_WriteDateTimeTZ(GPSData.time,GPSData.date,0);
						LOG_WriteStr(";");
					} else {
						LOG_WriteStr(";;");
					}
					LOG_WriteIntF(GPSData.latitude,6);
					LOG_WriteStr(";");
					LOG_WriteIntF(GPSData.longitude,6);
					LOG_WriteStr(";");
					LOG_WriteInt(GPSData.altitude);
					LOG_WriteStr(";");
					LOG_WriteIntF(GPSData.speed,2);
					LOG_WriteStr(";");
					LOG_WriteIntF(GPSData.course,2);
					LOG_WriteStr(";");
					LOG_WriteIntF(GPSData.PDOP,2);
					LOG_WriteStr(";");
					LOG_WriteIntF(GPSData.VDOP,2);
					LOG_WriteStr(";");
					LOG_WriteIntF(GPSData.HDOP,2);
					LOG_WriteStr(";");
					if (GPSData.fix_quality == 2) {
						LOG_WriteStr("dgps;");
					} else if (GPSData.fix_quality == 3) {
						LOG_WriteStr("pps;"); // it's unlikely that it will occur (PPS - military signal)
					} else {
						if (GPSData.fix == 2) {
							LOG_WriteStr("2d;");
						} else {
							LOG_WriteStr("3d;");
						}
					}
					LOG_WriteIntU(GPSData.sats_used);
					LOG_WriteStr(";");
				} else {
					LOG_WriteStr(";;;;;;;;;none;0;");
				}
				LOG_WriteIntF(nRF24_Packet.vrefint,2);
				LOG_WriteStr(";");
				LOG_WriteIntF(CurData.Temperature,1);
				LOG_WriteStr(";");
				LOG_WriteIntF(BMP180_hPa_to_mmHg(CurData.Pressure),1);
				LOG_WriteStr("\r\n");
			}

			// Check if the UART are disabled and enable it if so
			if (!(TIM7->CR1 & TIM_CR1_CEN) &&
					!(DMA1_Channel6->CCR & DMA_CCR6_EN) &&
					!(GPS_USART_PORT->CR1 & USART_CR1_RXNEIE)) {
				// Enable USART2 RX complete interrupt
				GPS_USART_PORT->CR1 |= USART_CR1_RXNEIE;
				GPS_buf_cntr = 0;
			}
		}

		if (GUI_refresh) {
			// GUI must be redrawn
			UC1701_Fill(0x00);

			// Current speed with pace arrows
			GUI_DrawSpeed(scr_width - 55,0,CurData.Speed,CurData.AvgSpeed);

			// Fancy lines
			HLine(0,scr_width - 1,scr_height - 29,PSet);
			VLine(67,0,scr_height - 1,PSet);

			// RF icon
			if (_time_no_signal > NO_SIGNAL_TIME)
				GUI_DrawBitmap(0,0,13,7,&bmp_icon_13x7[65]);
			else if (_icon_RF) {
				GUI_DrawBitmap(0,0,13,7,&bmp_icon_13x7[39]);
			} else {
				GUI_DrawBitmap(0,0,13,7,&bmp_icon_13x7[52]);
			}

			// GPS icon
			if (GPSData.fix != 2 && GPSData.fix != 3)
				GUI_DrawBitmap(16,0,13,7,&bmp_icon_13x7[26]);
			else if (GPSData.fix == 2) GUI_DrawBitmap(16,0,13,7,&bmp_icon_13x7[0]);
			else GUI_DrawBitmap(16,0,13,7,&bmp_icon_13x7[13]);

			// SD card icon
			if (_SD_present)
				GUI_DrawBitmap(32,0,13,7,&bmp_icon_13x7[78]);
			else GUI_DrawBitmap(32,0,13,7,&bmp_icon_13x7[65]);

			// LOG icon
			if (_logging) {
				GUI_DrawBitmap(48,0,13,7,&bmp_icon_13x7[91]);
				if (_icon_LOG) InvertRect(48,0,13,7);
			} else GUI_DrawBitmap(48,0,13,7,&bmp_icon_13x7[65]);

			// FIXME: Hardcoded "cadence" value
			PutStr(scr_width - 30,scr_height - 27,"CDC",fnt5x7);
			if (_time_no_signal > NO_SIGNAL_TIME) {
				for (i = 0; i < 3; i++)	FillRect(scr_width - (i * 10) - 15,scr_height - 3,scr_width - (i * 10) - 7,scr_height - 1,PSet);
			} else {
				GUI_DrawNumber(-scr_width + 7,-scr_height + 1,CurData.Cadence,0,DS_Small);
			}
			GUI_DrawBitmap(scr_width - 5,scr_height - 19,5,19,&small_signs[15]);

			// FIXME: Hardcoded "ride time"
			PutStr(3,scr_height - 27,"Ride Time",fnt5x7);
			GUI_DrawRideTime(0,scr_height - 19,CurData.TripTime);

			if (_bmp180_present) {
				// Current pressure
				GUI_PutPressure(0,26,CurData.Pressure,PT_mmHg,fnt5x7);

				// Current temperature
				GUI_PutTemperature(0,18,CurData.Temperature,fnt5x7);
			}

			// Current time
			GUI_PutTimeSec(0,10,RTC_Time.RTC_Hours * 3600 + RTC_Time.RTC_Minutes * 60 + RTC_Time.RTC_Seconds,fnt5x7);

/*
			// Draw iterations counter
			i = PutInt(3,3,ccc,fnt5x7);
			FillRect(0,0,i + 4,12,PReset);
			Rect(1,1,i + 3,11,PSet);
			Rect(2,2,i + 2,10,PReset);
			PutInt(3,3,ccc,fnt5x7);
*/

			UC1701_Flush();
			GUI_refresh = FALSE;
		}

		// "Up" button pressed - just clear the counter, there is no function yet
		if (BTN[BTN_UP].cntr > 0) {
			BTN[BTN_UP].cntr = 0;
		}

		// "Down" button pressed - just clear the counter, there is no function yet
		if (BTN[BTN_DOWN].cntr > 0) {
			BTN[BTN_DOWN].cntr = 0;
		}

		// "Enter" button hold - show main menu
		if (BTN[BTN_ENTER].state == BTN_Hold || BTN[BTN_ENTER].cntr) { // <--- FIXME: Short press is temporary here
			GUI_MainMenu();
		}

		// "Escape" button pressed - just clear the counter, there is no function for yet
		if (BTN[BTN_ESCAPE].cntr) {
			BTN[BTN_ESCAPE].cntr = 0;
		}

		ccc++; // <--- Iterations counter, for debug purposes

		SleepWait(); // Sleep mode
	}

	// Something awful will happen when you reach this point
}
