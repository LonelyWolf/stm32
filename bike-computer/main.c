///////////////////
// STM32L151RBT6 //
///////////////////


/////////////////////////////////////////////////////////////////////////

#define DEBUG

/////////////////////////////////////////////////////////////////////////


#include <misc.h>
#include <stm32l1xx_rcc.h>
#include <stm32l1xx_gpio.h>
#include <stm32l1xx_rtc.h>
#include <stm32l1xx_exti.h>
#include <stm32l1xx_tim.h>
#include <stm32l1xx_usart.h>
#include <stm32l1xx_syscfg.h>
#include <stm32l1xx_dma.h>
#include <string.h> // For memset, memmove

#include <delay.h>
#include <spi.h>
#include <i2c.h>
#include <uart.h>
#include <uc1701.h>
#include <nRF24.h>
#include <bmp180.h>
#include <RTC.h>
#include <wolk.h>
#include <GUI.h>
#include <GPS.h>
#include <USB.h>
#include <beeper.h>

#include <font5x7.h>
#include <font7x10.h>
#include <resources.h>


/////////////////////////////////////////////////////////////////////////


#define SCREENSAVER_TIMEOUT           1800  // Timeout for screensaver activation (seconds)
#define SCREENSAVER_UPDATE              60  // Screensaver update interval (seconds)
#define ALT_MEASURE_DUTY_CYCLE          30  // Temperature/pressure measurement duty cycle (seconds)
#define NO_SIGNAL_TIME                  10  // Sensor signal timeout (seconds)

uint8_t nRF24_RX_Buf[nRF24_RX_PAYLOAD];     // nRF24L01 payload buffer

bool _new_packet;                           // TRUE if new packet was received but not parsed
bool _new_time;                             // TRUE if time was updated
bool _bmp180_present;                       // TRUE if BMP180 sensor responded on I2C bus
bool _RF_icon;                              // Flag for RF icon flashing

uint16_t _prev_cntr_SPD;                    // Last received cntr_SPD value
uint16_t _prev_tim_SPD;                     // Last received tim_SPD value
uint16_t _tim_excess;                       // Timer excess from 1 second ratio
uint32_t _cntr_cadence;                     // Cadence counter (for AVG calculation)
uint32_t _cadence_accum;                    // Cadence accumulator (for AVG calculation)

int16_t altitude_history[128];              // Last 128 altitude values
uint8_t _altitude_duty_cycle;               // Altitude measurement duty cycle

uint16_t _DMA_cntr;                         // Last value of UART RX DMA counter (to track RX timeout)

uint32_t i;                                 // THIS IS UNIVERSAL VARIABLE
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
			RCC_APB1Periph_TIM7,      // TIM7 peripheral
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
	NVICInit.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_Init(&NVICInit);

	// PA5 -> EXTI line 5 (Button#0)
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource5);
	EXTIInit.EXTI_Line = EXTI_Line5;
	EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTIInit.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTIInit);

	// PA7 -> EXTI line 7  (Button#1)
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource7);
	EXTIInit.EXTI_Line = EXTI_Line7;
	EXTI_Init(&EXTIInit);

	// PC10 -> EXTI line 10  (Button#2)
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource10);
	EXTIInit.EXTI_Line = EXTI_Line10;
	EXTI_Init(&EXTIInit);

	// PC10 -> EXTI line 11  (Button#3)
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource11);
	EXTIInit.EXTI_Line = EXTI_Line11;
	EXTI_Init(&EXTIInit);

	// RTC wake-up -> EXTI line 20
	EXTIInit.EXTI_Line = EXTI_Line20;
	EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising; // Must be rising edge
	EXTIInit.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTIInit);
	NVICInit.NVIC_IRQChannel = RTC_WKUP_IRQn;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 0x0f; // 0x0f - lowest priority
	NVICInit.NVIC_IRQChannelSubPriority = 0x00;
	NVIC_Init(&NVICInit);
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
	// Configure priority group: 4 bits for preemption priority, 0 bits for subpriority.
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	RCC_Configuration();
	GPIO_Configuration();
	NVIC_Configuration();

	// Initialize buzzer out
	BEEPER_Init();

	// Initialize delay timer without callback function
	Delay_Init(NULL);

	// UART port initialization
	UART2_Init(115200);

	// USB port initialization
//	USB_Init();

	// Configure basic timer TIM7 (for DMA timeout)
	TIM7->CR1  |= TIM_CR1_ARPE; // Auto-preload enable
	TIM7->PSC   = SystemCoreClock / 20000; // prescaler
	TIM7->ARR   = 999; // auto reload value
	TIM7->EGR   = 1; // Generate an update event to reload the prescaler value immediately
	TIM7->DIER |= TIM_DIER_UIE; // Enable TIMx interrupt

	// SPI2 port initialization
	SPI2_Init();

	// Initialize and configure LCD
	Display_Init();
}

// Turn on transceiver and configure it for RX mode
void nRF24_SetRXMode(void) {
	nRF24_Wake();
	nRF24_RXMode(nRF24_RX_PIPE0,nRF24_RF_CHANNEL,nRF24_DataRate_250kbps,nRF24_CRC_on,
			     nRF24_CRC_1byte,(uint8_t *)nRF24_RX_Addr,nRF24_RX_Addr_Size,nRF24_RX_PAYLOAD,
			     nRF24_TXPower_0dBm);
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
	} else {
		// Button pressed
		button->hold_cntr = 0;
		button->state = BTN_Pressed;
		BEEPER_Enable(2000,5);
	}
	if (!_screensaver) UC1701_SetBacklight(Settings.LCD_brightness);
	_screensaver = FALSE;
	_idle_time = 0;
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
		_no_signal_time = 0;
		if (_idle_time > Settings.LCD_timeout) _idle_time = Settings.LCD_timeout;
		if (RX_status == nRF24_RX_PCKT_PIPE0) ParsePacket();
		BEEPER_Enable(4000,1);
		EXTI_ClearITPendingBit(EXTI_Line6);
	}

	if (EXTI_GetITStatus(EXTI_Line7) != RESET) {
		// EXTI7 (Button#2 -> "Down")
		Button_Inquiry(&BTN[BTN_DOWN]);
		EXTI_ClearITPendingBit(EXTI_Line7);
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
}

// RTC wake-up IRQ handler
void RTC_WKUP_IRQHandler(void) {
	if (RTC->ISR & RTC_ISR_WUTF) {
		// RTC Wake-up interrupt
		RTC_GetDateTime();

		_no_signal_time++;
		_altitude_duty_cycle++;
		_idle_time++;
		_new_time = TRUE;

		if (_no_signal_time > 3600) _no_signal_time = 3600; // Counter overflow protection (1hour)
		if (_idle_time > 3600) _idle_time = 3600; // Counter overflow protection (1hour)

		if (_no_signal_time > NO_SIGNAL_TIME) {
			CurData.Speed   = 0;
			CurData.Cadence = 0;
		}
		if (_idle_time > SCREENSAVER_TIMEOUT) _screensaver = TRUE; else _screensaver = FALSE;
		if (!_screensaver) {
			// The following procedures must be executed only when no screensaver active
			if (_idle_time > Settings.LCD_timeout && Settings.LCD_timeout) UC1701_SetBacklight(0); else UC1701_SetBacklight(Settings.LCD_brightness);
			// ATTENTION! BMP180 polling pretty slow!
			// Execute it from this place is really a bad idea!
			if (_altitude_duty_cycle > ALT_MEASURE_DUTY_CYCLE) UpdateBMP180();
		}

		PWR->CR  |= PWR_CR_DBP; // Access to RTC, RTC Backup and RCC CSR registers enabled
		RTC->ISR &= ~RTC_ISR_WUTF; // Clear RTC wakeup timer flag
		PWR->CR  &= ~PWR_CR_DBP; // Access to RTC, RTC Backup and RCC CSR registers disabled
		EXTI_ClearITPendingBit(EXTI_Line20);
	}
}

// TIM7 IRQ handler
void TIM7_IRQHandler(void) {
	TIM7->SR = (uint16_t)~TIM_IT_Update; // Clear the TIM7's interrupt pending bit (TIM7 rises only UPDATE IT)
	if (_DMA_cntr == (uint16_t)DMA1_Channel6->CNDTR) {
		// DMA pointer unchanged from last IRQ -> UART timeout
		TIM7->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN)); // Disable TIM7
		GPS_buf_cntr = FIFO_BUFFER_SIZE - (uint16_t)DMA1_Channel6->CNDTR; // Remember how many bytes received in FIFO buffer
		DMA1_Channel6->CCR &= (uint16_t)(~DMA_CCR1_EN); // Disable DMA
		UART_PORT->CR1 |= (1<<5); // Enable USART2 RX complete interrupt
		_DMA_cntr = 0;
		// Copy data from the FIFO buffer to another one in case of new GPS data arrival
		memcpy(&GPS_buf[0],&USART_FIFO[0],GPS_buf_cntr);
		ParseGPS(); // About 3ms with full buffer
	} else {
		_DMA_cntr = (uint16_t)DMA1_Channel6->CNDTR;
	}
}

// USART2 IRQ handler
void USART2_IRQHandler(void) {
	if (UART_PORT->SR & USART_IT_RXNE) {
		// Received data read to be read
		UART_PORT->CR1 &= ~(1<<5); // Disable USART2 RX complete interrupt
		if ((DMA1_Channel6->CCR & DMA_CCR1_EN) == RESET) {
			// Byte received, DMA disabled -> enable DMA and TIM7
			DMA1_Channel6->CMAR = (uint32_t)&USART_FIFO[0];
			DMA1_Channel6->CNDTR = FIFO_BUFFER_SIZE;
			DMA1_Channel6->CCR |= DMA_CCR1_EN; // Enable DMA
			TIM7->CNT = 0;
			TIM7->EGR = 1;
			TIM7->CR1 |= TIM_CR1_CEN; // Enable TIM7
		}
	}
}

// UART2_RX DMA IRQ handler
void DMA1_Channel6_IRQHandler() {
	DMA1->IFCR |= DMA_ISR_TCIF6; // Clear DMA1 channel6 transfer complete flag
	UART_PORT->CR1 |= (1<<5); // Enable USART2 RX complete interrupt
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

	// Set GUI refresh flag twice per second
	GUI_refresh = TRUE;
}


/////////////////////////////////////////////////////////////////////////


// Parse received nRF24L01 data packet
void ParsePacket(void) {
	float tmp;
	int16_t diff_SPD;

	// memcpy doesn't work here due to struct alignments
	nRF24_Packet.cntr_SPD     = (nRF24_RX_Buf[0] << 8) + nRF24_RX_Buf[1];
	nRF24_Packet.tim_SPD      = (nRF24_RX_Buf[2] << 8) + nRF24_RX_Buf[3];
	nRF24_Packet.tim_CDC      = (nRF24_RX_Buf[4] << 8) + nRF24_RX_Buf[5];
	nRF24_Packet.vrefint      = ((nRF24_RX_Buf[6] & 0x03) << 8) + nRF24_RX_Buf[7];
	nRF24_Packet.observe_TX   =  nRF24_RX_Buf[8];
	nRF24_Packet.cntr_wake    = (nRF24_RX_Buf[9] << 8) + nRF24_RX_Buf[10];
	nRF24_Packet.packets_lost = (nRF24_RX_Buf[11] << 8) + nRF24_RX_Buf[12];
	nRF24_Packet.ride_time    = (nRF24_RX_Buf[13] << 8) + nRF24_RX_Buf[14];

	// Convert SPD impulses period into speed
	if (nRF24_Packet.tim_SPD > 0) {
//		CurData.Speed = Settings.WheelCircumference * (992.9696969 * 0.36) / nRF24_Packet.tim_SPD;
//		CurData.Speed = (Settings.WheelCircumference * 357.469090884) / nRF24_Packet.tim_SPD;
//		CurData.Speed = Settings.WheelCircumference * (1007.08 * 0.36) / nRF24_Packet.tim_SPD;
		CurData.Speed = (Settings.WheelCircumference * 362.5488) / nRF24_Packet.tim_SPD;
		if (CurData.Speed > 999) CurData.Speed = 999; // Maximum 99.9km/h can be displayed
	} else CurData.Speed = 0;

	// Convert CDC impulses period into cadence
	if (nRF24_Packet.tim_CDC > 0) {
//		CurData.Cadence = (uint32_t)((60.0 / nRF24_Packet.tim_CDC) * 992.9696969);
//		CurData.Cadence = (uint32_t)(59578.181814 / nRF24_Packet.tim_CDC);
//		CurData.Cadence = (uint32_t)((60.0 / nRF24_Packet.tim_CDC) * 1007.08);
		CurData.Cadence = (uint32_t)(60424.8 / nRF24_Packet.tim_CDC);
		if (CurData.Cadence > 250) CurData.Cadence = 250; // pedaling 250RPM, really?
	} else CurData.Cadence = 0;

	// Update maximum values
	if (CurData.Speed > CurData.MaxSpeed) CurData.MaxSpeed = CurData.Speed;
	if (CurData.Cadence > CurData.MaxCadence) CurData.MaxCadence = CurData.Cadence;

	// Update current trip time
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
			tmp_f = modff(tmp,&tmp_i);
			CurData.TripTime += tmp_i;
			_tim_excess = tmp_f * 1000;
		}
	}
	_prev_tim_SPD  = nRF24_Packet.tim_SPD;
*/
	if (nRF24_Packet.ride_time) {
		tmp = (nRF24_Packet.ride_time / 1007.08) + (_tim_excess / 1000.0);
		CurData.TripTime += (uint32_t)tmp;
		_tim_excess = (uint32_t)(tmp * 1000) - ((uint32_t)tmp * 1000);
	}

	// Update odometer and current trip distance
	if (nRF24_Packet.cntr_SPD != 0 && nRF24_Packet.cntr_SPD != _prev_cntr_SPD) {
		diff_SPD = nRF24_Packet.cntr_SPD - _prev_cntr_SPD;
		if (diff_SPD < 0) diff_SPD *= -1;
		CurData.Odometer += Settings.WheelCircumference * diff_SPD;
		CurData.TripDist += Settings.WheelCircumference * diff_SPD;
	}
	_prev_cntr_SPD = nRF24_Packet.cntr_SPD;

	// Update average values
	CurData.AvgSpeed = (uint32_t)((CurData.TripDist * 0.36) / CurData.TripTime);
	if (CurData.Cadence > 0) {
		// A simple calculation of the average cadence.
		// At a constant value in a fantastic 300RPM accumulator will overflow
		// after about 4000 hours of ride
		_cntr_cadence++;
		_cadence_accum += CurData.Cadence;
		CurData.AvgCadence = _cadence_accum / _cntr_cadence;
	}

	_new_packet = TRUE;
}

// Parse GPS data
void ParseGPS(void) {
	GPS_InitData(); // Clear previously parsed GPS data
	GPS_sentences_parsed = 0;
	GPS_msg = GPS_FindSentence(GPS_buf,0,FIFO_BUFFER_SIZE); // Find first NMEA sentence
	do {
		if (GPS_msg.type != NMEA_BAD) {
			GPS_sentences_parsed++;
			GPS_ParseSentence(GPS_buf,GPS_msg);
		}
		GPS_msg = GPS_FindSentence(GPS_buf,GPS_msg.end,FIFO_BUFFER_SIZE);
	} while (GPS_msg.end < GPS_buf_cntr);
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
		GPS_new_data = TRUE;
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
	memset(&GPS_buf,0,sizeof(GPS_buf));
	memset(&USART_FIFO,0,FIFO_BUFFER_SIZE);
	GPS_InitData();

	_new_packet     = FALSE;
	_new_time       = FALSE;
	_screensaver    = FALSE;
	_bmp180_present = FALSE;
	_RF_icon        = TRUE;
	GPS_new_data    = FALSE;
	GPS_buf_cntr    = 0;
	_DMA_cntr       = 0;
	_prev_cntr_SPD  = 0;
	_prev_tim_SPD   = 0;
	_tim_excess     = 0;
	_cntr_cadence   = 0;
	_cadence_accum  = 0;
	_no_signal_time = 32768;
	_idle_time      = 0;
	_altitude_duty_cycle = ALT_MEASURE_DUTY_CYCLE + 1;
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
	ReadSettings_EEPROM();

	// -------------------- end of variables initialization

	InitPeripherals();

	// Boot screen
	UC1701_Fill(0x00);
	GUI_DrawBitmap(98,39,30,25,&bmp_bike_man[0]);
	PutStr(0,0,"Wolk bike computer:",fnt5x7);
	PutStr(0,42,"CPU:",fnt5x7);
	i = 24 + PutIntF(24,42,SystemCoreClock / 1000,3,fnt5x7);
	PutStr(i,42,"MHz",fnt5x7);
	UC1701_Flush();

	PutStr(0,56,"STAT:",fnt5x7);
	PutStr(30,56,GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15) == Bit_RESET ? "0" : "1",fnt5x7);

	// Init the RTC and configure it clock to LSE
	PutStr(0,8,"LSE:",fnt5x7);
	UC1701_Flush();
	RTC_Config();
	PutStr(24,8,"Ok",fnt5x7);
	UC1701_Flush();

	// Configure nRF24L01+
	PutStr(0,16,"nRF24L01:",fnt5x7);
	UC1701_Flush();
    nRF24_init();
    if (nRF24_Check()) {
    	nRF24_SetRXMode();
    	PutStr(54,16,"Ok",fnt5x7);
    	UC1701_Flush();
    } else {
    	PutStr(54,16,"Fail",fnt5x7);
        UC1701_Flush();
    	while(1);
    }

    // I2C2 port initialization
    PutStr(0,24,"BMP180:",fnt5x7);
    UC1701_Flush();
    // I2C fast mode (400kHz)
    if (I2C2_Init(400000) == I2C_SUCCESS) {
        BMP180_Reset(); // Send reset command to BMP180
        Delay_ms(15); // Wait for BMP180 startup time (10ms by datasheet)
        if (BMP180_Check() == BMP180_SUCCESS) {
    		_bmp180_present = TRUE;
    		PutChar(42,24,'v',fnt5x7);
    		i = BMP180_GetVersion();
    		PutInt(48,24,i,fnt5x7);
    		BMP180_ReadCalibration();
			CurData.MinTemperature =  32767;
			CurData.MaxTemperature = -32767;
			CurData.MinPressure = 2147483647; // LONG_MAX - it becomes normal when the pressure will be acquired normally first time
    		if (UpdateBMP180()) {
        		GUI_PutTemperature(60,24,CurData.Temperature,fnt5x7);
        		GUI_PutPressure(60,32,CurData.Pressure,PT_mmHg,fnt5x7);
        		altitude_history[0] = BMP180_hPa_to_Altitude(CurData.Pressure);
    		} else {
    			PutStr(60,24,"Readings",fnt5x7);
    			PutStr(60,32,"failed",fnt5x7);
    		}
    	} else {
    		PutStr(42,24,"Not present",fnt5x7);
    	}
    } else {
    	PutStr(42,24,"I2C timeout",fnt5x7);
    }
	UC1701_Flush();

	BEEPER_PlayTones(tones_startup);
//	Delay_ms(2500); // Fancy startup delay

	UC1701_Fill(0x00);
    UC1701_Flush();

	// Set time
	RTC_Time.RTC_Hours   = 0;
	RTC_Time.RTC_Minutes = 0;
	RTC_Time.RTC_Seconds = 0;
	RTC_Date.RTC_Date    = 16;
	RTC_Date.RTC_Month   = 04;
	RTC_Date.RTC_Year    = 14;
	RTC_Date.RTC_WeekDay = 3;
	RTC_SetDateTime();

//	GPS_SendCommand("$PMTK104*"); // Reset EB500 to factory settings
//	GPS_SendCommand("$PMTK314,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1*"); // All sentences
//	GPS_SendCommand("$PMTK314,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*"); // GLL
//	GPS_SendCommand("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*"); // RMC
//	GPS_SendCommand("$PMTK314,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*"); // VTG
//	GPS_SendCommand("$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*"); // GGA
//	GPS_SendCommand("$PMTK314,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*"); // GSA
//	GPS_SendCommand("$PMTK314,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*"); // GSV
//	GPS_SendCommand("$PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0*"); // ZDA
	GPS_SendCommand("$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,1,0*"); // All supported sentences
	GPS_SendCommand("$PMTK101*"); // Hot restart

	ccc = 0;

	// Reinitialize delay timer with callback function
	Delay_Init(callback_Delay);

	// Configure wake-up timer to wake every second and enable it
	RTC_SetWakeUp(1);


/////////////////////////////////////////////////////////////////////////
//	Main loop
/////////////////////////////////////////////////////////////////////////

	while(1) {
		if (_screensaver) {
			RTC_SetWakeUp(SCREENSAVER_UPDATE);
			nRF24_Sleep(); // Turn off the receiver
			UC1701_SetBacklight(0);

			GUI_ScreenSaver();

 			// Reinitialize display
 			Display_Init();
			UC1701_Fill(0x00);

			nRF24_SetRXMode(); // Wake the receiver and configure it for RX mode
 			RTC_SetWakeUp(1); // Wake every second

			GUI_refresh = TRUE;
		}

		if (_new_packet) {
			_new_packet = FALSE;
			_RF_icon = !_RF_icon;
			GUI_refresh = TRUE;
		}

		if (GPS_new_data) {
			if (GPSData.datetime_valid && _new_time) {
				if (RTC_Time.RTC_Minutes != (GPSData.time / 60) % 60) {
					// Date and time obtained from GPS
					RTC_Time.RTC_Hours   =  GPSData.time / 3600;
					RTC_Time.RTC_Minutes = (GPSData.time / 60) % 60;
					RTC_Time.RTC_Seconds =  GPSData.time % 60;
					i = GPSData.date / 1000000;
					RTC_Date.RTC_Date  = i;
					RTC_Date.RTC_Month = (GPSData.date - (i * 1000000)) / 10000;
					RTC_Date.RTC_Year  = (GPSData.date % 10000) - 2000;
					RTC_AdjustTimeZone(&RTC_Time,&RTC_Date,Settings.GMT_offset);
					RTC_SetDateTime();
				}
			}
			GPSData.datetime_valid = FALSE;
			GPSData.time_valid = FALSE;
			GPS_new_data = FALSE;
			GUI_refresh = TRUE;
		}

		if (_new_time) {
			GUI_refresh = TRUE;
			_new_time = FALSE;
		}

		if (GUI_refresh) {
			UC1701_Fill(0x00);
			GUI_DrawSpeed(scr_width - 55,0,CurData.Speed,CurData.AvgSpeed);

			HLine(0,scr_width - 1,scr_height - 29,PSet);
			VLine(67,0,scr_height - 1,PSet);

			// RF icon
			if (_no_signal_time > NO_SIGNAL_TIME)
				GUI_DrawBitmap(0,0,13,7,&bmp_icon_13x7[65]);
			else if (_RF_icon) {
				GUI_DrawBitmap(0,0,13,7,&bmp_icon_13x7[39]);
			} else {
				GUI_DrawBitmap(0,0,13,7,&bmp_icon_13x7[52]);
			}

			// GPS icon
			if (GPSData.fix != 2 && GPSData.fix != 3)
				GUI_DrawBitmap(16,0,13,7,&bmp_icon_13x7[26]);
			else if (GPSData.fix == 2) GUI_DrawBitmap(16,0,13,7,&bmp_icon_13x7[0]);
			else GUI_DrawBitmap(16,0,13,7,&bmp_icon_13x7[13]);

			PutStr(scr_width - 30,scr_height - 27,"CDC",fnt5x7);
			if (_no_signal_time > NO_SIGNAL_TIME) {
				for (i = 0; i < 3; i++)	FillRect(scr_width - (i * 10) - 15,scr_height - 3,scr_width - (i * 10) - 7,scr_height - 1,PSet);
			} else {
				GUI_DrawNumber(-scr_width + 7,-scr_height + 1,CurData.Cadence,0,DS_Small);
			}
			GUI_DrawBitmap(scr_width - 5,scr_height - 19,5,19,&small_signs[15]);

			PutStr(3,scr_height - 27,"Ride Time",fnt5x7);
			GUI_DrawRideTime(0,scr_height - 19,CurData.TripTime);

			GUI_PutPressure(0,26,CurData.Pressure,PT_mmHg,fnt5x7);
			GUI_PutTemperature(0,18,CurData.Temperature,fnt5x7);

			GUI_PutTimeSec(0,10,RTC_Time.RTC_Hours * 3600 + RTC_Time.RTC_Minutes * 60 + RTC_Time.RTC_Seconds,fnt5x7);

			ccc++;
	/*
			i = PutInt(3,3,ccc,fnt5x7);
			Rect(0,0,i + 4,12,PReset);
			Rect(1,1,i + 3,11,PSet);
			Rect(2,2,i + 2,10,PReset);
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
		if (BTN[BTN_ENTER].state == BTN_Hold || BTN[BTN_ENTER].cntr) {
			GUI_MainMenu();
		}

		// "Escape" button pressed - just clear the counter, there is no function for yet
		if (BTN[BTN_ESCAPE].cntr) {
			BTN[BTN_ESCAPE].cntr = 0;
		}

		SleepWait(); // Sleep mode
	}

	// Something awful will happen when you reach this point
}
