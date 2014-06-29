///////////////////
// STM32L151RBT6 //
///////////////////


/////////////////////////////////////////////////////////////////////////

#define DEBUG

/////////////////////////////////////////////////////////////////////////


#include <misc.h>
#include <stm32l1xx_rcc.h>
#include <stm32l1xx_gpio.h>
#include <stm32l1xx_pwr.h>
#include <stm32l1xx_rtc.h>
#include <stm32l1xx_exti.h>
#include <stm32l1xx_tim.h>
#include <stm32l1xx_usart.h>
#include <stm32l1xx_syscfg.h>
#include <stm32l1xx_dma.h>
#include <string.h> // For memset, memmove
#include <math.h>

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
#include <resources.h>


/////////////////////////////////////////////////////////////////////////


#define LCD_BACKLIGHT_TIMEOUT           5   // Timeout for LCD backlight (seconds)
#define ALT_MEASURE_DUTY_CYCLE          10  // Temperature/pressure measurement duty cycle (seconds)

uint8_t nRF24_RX_Buf[nRF24_RX_PAYLOAD];     // nRF24L01 payload buffer

bool _new_packet;                           // TRUE if new packet was received but not parsed
bool _new_time;                             // TRUE if time was updated
bool _bmp180_present;                       // TRUE if BMP180 sensor responded on I2C bus

uint8_t _current_screen;     				// What currently displayed on screen

uint16_t _prev_cntr_SPD;                    // Last received cntr_SPD value
uint16_t _prev_tim_SPD;                     // Last received tim_SPD value
uint16_t _tim_excess;                       // Timer excess from 1 second ratio
uint32_t _cntr_cadence;                     // Cadence counter (for AVG calculation)
uint32_t _cadence_accum;                    // Cadence accumulator (for AVG calculation)

RTC_TimeTypeDef RTC_Time;                   // Current RTC time
RTC_DateTypeDef RTC_Date;                   // Current RTC date

uint32_t _no_signal_time;                   // Time since last packet received (seconds)
uint32_t _idle_time;                        // Time from last user event (button press)

int16_t altitude_history[128];              // Last 128 altitude values
uint8_t _altitude_duty_cycle;               // Altitude measurement duty cycle

int16_t _raw_temp;                          // Temporary variable for temperature
int32_t _raw_press;                         // Temporary variable for presssure

uint16_t _DMA_cntr;                         // Last value of UART RX DMA counter (to track RX timeout)

uint32_t i;                                 // THIS IS UNIVERSAL VARIABLE
uint32_t ccc;                               // Wakeups count, for debug purposes

// Peripherals initialization variables
GPIO_InitTypeDef PORT;
EXTI_InitTypeDef EXTIInit;
NVIC_InitTypeDef NVICInit;
DMA_InitTypeDef  DMAInit;


/////////////////////////////////////////////////////////////////////////


void ParsePacket(void);
void ParseGPS(void);


/////////////////////////////////////////////////////////////////////////


// EXTI[5..9] lines IRQ handler
void EXTI9_5_IRQHandler(void) {
	nRF24_RX_PCKT_TypeDef RX_status;

	if (EXTI_GetITStatus(EXTI_Line5) != RESET) {
		// EXTI5 (Button#1 -> "Up")
		if (BTN[BTN_UP].PORT->IDR & BTN[0].PIN) {
		    // Button released
			BTN[BTN_UP].hold_cntr = 0;
			if (BTN[BTN_UP].state != BTN_Hold) BTN[BTN_UP].cntr++;
			BTN[BTN_UP].state = BTN_Released;
		} else {
			// Button pressed
			BTN[BTN_UP].hold_cntr = 0;
			BTN[BTN_UP].state = BTN_Pressed;
		}
		_idle_time = 0;
		EXTI_ClearITPendingBit(EXTI_Line5);
	}

	if (EXTI_GetITStatus(EXTI_Line7) != RESET) {
		// EXTI7 (Button#2 -> "Down")
		if (BTN[BTN_DOWN].PORT->IDR & BTN[BTN_DOWN].PIN) {
		    // Button released
			BTN[BTN_DOWN].hold_cntr = 0;
			if (BTN[BTN_DOWN].state != BTN_Hold) BTN[BTN_DOWN].cntr++;
			BTN[BTN_DOWN].state = BTN_Released;
		} else {
			// Button pressed
			BTN[BTN_DOWN].hold_cntr = 0;
			BTN[BTN_DOWN].state = BTN_Pressed;
		}
		_idle_time = 0;
		EXTI_ClearITPendingBit(EXTI_Line7);
	}

	if (EXTI_GetITStatus(EXTI_Line6) != RESET) {
		// EXTI6 (nRF24L01 IRQ)
		UC1701_PauseSPI();
		RX_status = nRF24_RXPacket(nRF24_RX_Buf,nRF24_RX_PAYLOAD);
		if (RX_status == nRF24_RX_PCKT_PIPE0) _new_packet = TRUE;
		nRF24_ClearIRQFlags();
		UC1701_ResumeSPI();
		_no_signal_time = 0;
		EXTI_ClearITPendingBit(EXTI_Line6);
	}
}

// EXTI[10..15] lines IRQ handler
void EXTI15_10_IRQHandler(void) {
	if (EXTI_GetITStatus(EXTI_Line10) != RESET) {
		// EXTI10 (Button#3 -> "Enter")
		if (BTN[BTN_ENTER].PORT->IDR & BTN[BTN_ENTER].PIN) {
		    // Button released
			BTN[BTN_ENTER].hold_cntr = 0;
			if (BTN[BTN_ENTER].state != BTN_Hold) BTN[BTN_ENTER].cntr++;
			BTN[BTN_ENTER].state = BTN_Released;
		} else {
			// Button pressed
			BTN[BTN_ENTER].hold_cntr = 0;
			BTN[BTN_ENTER].state = BTN_Pressed;
		}
		_idle_time = 0;
		EXTI_ClearITPendingBit(EXTI_Line10);
	}

	if (EXTI_GetITStatus(EXTI_Line11) != RESET) {
		// EXTI11 (Button#4 -> "Escape")
		if (BTN[BTN_ESCAPE].PORT->IDR & BTN[BTN_ESCAPE].PIN) {
		    // Button released
			BTN[BTN_ESCAPE].hold_cntr = 0;
			if (BTN[BTN_ESCAPE].state != BTN_Hold) BTN[BTN_ESCAPE].cntr++;
			BTN[BTN_ESCAPE].state = BTN_Released;
		} else {
			// Button pressed
			BTN[BTN_ESCAPE].hold_cntr = 0;
			BTN[BTN_ESCAPE].state = BTN_Pressed;
		}
		_idle_time = 0;
		EXTI_ClearITPendingBit(EXTI_Line11);
	}
}

// RTC wakeup IRQ handler
void RTC_WKUP_IRQHandler(void) {
	if (RTC_GetITStatus(RTC_IT_WUT)) {
		// RTC Wakeup interrupt
		RTC_GetTime(RTC_Format_BIN,&RTC_Time);
		RTC_GetDate(RTC_Format_BIN,&RTC_Date);
		_new_time = TRUE;
		_altitude_duty_cycle++;
		_no_signal_time++;
		if (_no_signal_time > 3600) _no_signal_time = 3600; // Counter overflow protection (1hour)
		_idle_time++;
		if (_idle_time > 3600) _idle_time = 3600; // Counter overflow protection (1hour)
		RTC_ClearITPendingBit(RTC_IT_WUT);
		RTC_ClearFlag(RTC_FLAG_WUTF);
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

	if (_idle_time > LCD_BACKLIGHT_TIMEOUT)	UC1701_SetBacklight(0); else UC1701_SetBacklight(Settings.LCD_brightness);
}


/////////////////////////////////////////////////////////////////////////


// Parse received nRF24L01 data packet
void ParsePacket(void) {
	float tmp,tmp_f,tmp_i;
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

	// Magic number '992.9696969' - timer period in sensor MCU

	// Convert SPD impulses period into speed
	if (nRF24_Packet.tim_SPD > 0) {
		//tmp = Settings.WheelCircumference * (992.9696969 * 0.036) / nRF24_Packet.tim_SPD;
		tmp = (Settings.WheelCircumference * 35.7469090884) / nRF24_Packet.tim_SPD;
		tmp_f = modff(tmp,&tmp_i);
		CurData.Speed = ((uint32_t)tmp_i * 10) + (uint32_t)(tmp_f * 10);
		if (CurData.Speed > 999) CurData.Speed = 999; // Maximum 99.9km/h can be displayed
	} else CurData.Speed = 0;

	// Convert CDC impulses period into cadence
	if (nRF24_Packet.tim_CDC > 0) {
		//CurData.Cadence = (uint32_t)((60.0 / nRF24_Packet.tim_CDC) * 992.9696969);
		CurData.Cadence = (uint32_t)(59578.181814 / nRF24_Packet.tim_CDC);
		if (CurData.Cadence > 250) CurData.Cadence = 250; // 250RPM pedaling, really?
	} else CurData.Cadence = 0;

	// Update maximum values
	if (CurData.Speed > CurData.MaxSpeed) CurData.MaxSpeed = CurData.Speed;
	if (CurData.Cadence > CurData.MaxCadence) CurData.MaxCadence = CurData.Cadence;

	// Update trip time
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
		tmp_f = modff(tmp,&tmp_i);
		CurData.TripTime += tmp_i;
		_tim_excess = tmp_f * 1000;
	}

	// Update odometer
	if (nRF24_Packet.cntr_SPD != 0 && nRF24_Packet.cntr_SPD != _prev_cntr_SPD) {
		diff_SPD = nRF24_Packet.cntr_SPD - _prev_cntr_SPD;
		if (diff_SPD < 0) diff_SPD *= -1;
		CurData.Odometer += Settings.WheelCircumference * diff_SPD;
		CurData.TripDist += Settings.WheelCircumference * diff_SPD;
	}
	_prev_cntr_SPD = nRF24_Packet.cntr_SPD;

	// Update average values
	tmp = (CurData.TripDist / CurData.TripTime) * 0.036;
	tmp_f = modff(tmp,&tmp_i);
	CurData.AvgSpeed = (tmp_i * 10) + (uint32_t)(tmp_f * 10);
	if (CurData.Cadence > 0) {
		// A simple calculation of the average cadence.
		// At a constant value in a fantastic 300RPM accumulator will overflow
		// after about 4000 hours of ride
		_cntr_cadence++;
		_cadence_accum += CurData.Cadence;
		CurData.AvgCadence = _cadence_accum / _cntr_cadence;
	}

	_new_packet = FALSE;
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
		GPS_new_data = TRUE;
	}
}


/////////////////////////////////////////////////////////////////////////


int main(void) {
	// Update SystemCoreClock according to Clock Register Values
	SystemCoreClockUpdate();

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

	CurData.MinGPSAlt = 0x7FFFFFFF; // LONG_MAX - first time when altitude will be acquired it becomes normal value

	_new_packet = TRUE;
	_new_time   = TRUE;

	GPS_new_data = FALSE;
	GPS_buf_cntr = 0;

	_DMA_cntr = 0;

	_bmp180_present = FALSE;
	_altitude_duty_cycle = ALT_MEASURE_DUTY_CYCLE + 1;

	_current_screen = 0;

	_prev_cntr_SPD = 0;
	_prev_tim_SPD  = 0;
	_tim_excess    = 0;
	_cntr_cadence  = 0;
	_cadence_accum = 0;

	_no_signal_time = 0;
	_idle_time = 0;

	// Buttons initialization
	for (i = 0; i < 4; i++) memset(&BTN[i],0,sizeof(BTN[i]));
	BTN[0].PORT = BTN1_PORT;
	BTN[0].PIN  = BTN1_PIN;
	BTN[1].PORT = BTN2_PORT;
	BTN[1].PIN  = BTN2_PIN;
	BTN[2].PORT = BTN3_PORT;
	BTN[2].PIN  = BTN3_PIN;
	BTN[3].PORT = BTN4_PORT;
	BTN[3].PIN  = BTN4_PIN;

	// These values stored in EEPROM
	Settings.GMT_offset = 3;
	Settings.WheelCircumference = 206;
	Settings.altitude_home = 178;
	Settings.LCD_brightness = 50;

	// Read settings from EEPROM
	ReadSettings_EEPROM();

	// -------------------- end of variables initialization


	// Configure GPIO
	// Enable PORTA and PORTC peripheral
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOC,ENABLE);
	// Charge STAT pin (PA15)
	PORT.GPIO_Mode  = GPIO_Mode_IN;
	PORT.GPIO_Speed = GPIO_Speed_2MHz;
	PORT.GPIO_PuPd  = GPIO_PuPd_UP;
	PORT.GPIO_Pin   = GPIO_Pin_15;
	GPIO_Init(GPIOA,&PORT);

	// BTN1 pin (PA5)
	PORT.GPIO_Mode = GPIO_Mode_IN;
	PORT.GPIO_PuPd = GPIO_PuPd_UP;
	PORT.GPIO_Pin  = BTN1_PIN;
	GPIO_Init(BTN1_PORT,&PORT);
	// BTN2 pin (PA7)
	PORT.GPIO_Pin  = BTN2_PIN;
	GPIO_Init(BTN2_PORT,&PORT);
	// BTN3 pin (PC10)
	PORT.GPIO_Pin  = BTN3_PIN;
	GPIO_Init(BTN3_PORT,&PORT);
	// BTN4 pin (PC11)
	PORT.GPIO_Pin  = BTN4_PIN;
	GPIO_Init(BTN4_PORT,&PORT);

	// Initialize delay timer without callback function
	Delay_Init(NULL);

	// UART port initialization
	UART2_Init(115200);

	// Configure basic timer TIM7
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE); // Enable TIMx peripheral
	TIM7->CR1  |= TIM_CR1_ARPE; // Auto-preload enable
	TIM7->PSC   = 1600; // prescaler
	TIM7->ARR   = 999; // auto reload value
	TIM7->EGR   = 1; // Generate an update event to reload the prescaler value immediately
	TIM7->DIER |= TIM_DIER_UIE; // Enable TIMx interrupt
	// TIM7 IRQ
	NVICInit.NVIC_IRQChannel = TIM7_IRQn;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 0x05; // below middle priority
	NVICInit.NVIC_IRQChannelSubPriority = 0x05; // below middle priority
	NVIC_Init(&NVICInit);

	// SPI2 port initialization
	SPI2_Init();

	// Display initialization
	// Pinout:
	//   PB7  -> CS
	//   PB8  -> RST
	//   PB9  -> RS
	//   PB13 -> SCK (SPI SCK)
	//   PB15 -> SDA (SPI MOSI)
	//   PA1  -> LEDA (Backlight LED anode)
	UC1701_Init();
	UC1701_SetBacklight(Settings.LCD_brightness);
	UC1701_Contrast(4,24);
	UC1701_Orientation(scr_normal);
	UC1701_Fill(0x00);
	GUI_DrawBitmap(98,39,30,25,&bmp_bike_man[0]);

	// Boot screen
	PutStr5x7(0,0,"Wolk bike computer:",CT_opaque);
	PutStr5x7(0,42,"CPU:",CT_opaque);
	i = 24 + PutIntF5x7(24,42,SystemCoreClock / 1000,3,CT_opaque);
	PutStr5x7(i,42,"MHz",CT_opaque);
	UC1701_Flush();

	PutStr5x7(0,56,"STAT:",CT_opaque);
	PutStr5x7(30,56,GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15) == Bit_RESET ? "0" : "1",CT_opaque);

/*
	while(1) {
		UC1701_Fill(0x00);

		i = PutStr5x7(0,0,"STAT:",CT_opaque) - 1;


		UC1701_Flush();
	}
*/

	// Init the RTC and configure it clock to LSE
	PutStr5x7(0,8,"LSE:",CT_opaque);
	UC1701_Flush();
	RTC_Config();
	RTC_WaitForSynchro(); // Wait until the RTC Time and Date registers are synchronized with RTC APB clock
	PutStr5x7(24,8,"Ok",CT_opaque);
	UC1701_Flush();

	// Configure nRF24L01+
    // Pinout:
    //   PC7  -> CE
    //   PB12 -> CS
    //   PB13 -> SCK
    //   PB15 -> MOSI
    //   PB14 <- MISO
    //   PC6  <- IRQ
	PutStr5x7(0,16,"nRF24L01:",CT_opaque);
	UC1701_Flush();
    nRF24_init();
    if (nRF24_Check()) {
    	PutStr5x7(54,16,"Ok",CT_opaque);
    	UC1701_Flush();
    	nRF24_RXMode(nRF24_RX_PIPE0,nRF24_RF_CHANNEL,nRF24_DataRate_250kbps,nRF24_CRC_on,
    			     nRF24_CRC_1byte,(uint8_t *)nRF24_RX_Addr,nRF24_RX_Addr_Size,nRF24_RX_PAYLOAD,
    			     nRF24_TXPower_0dBm);
        nRF24_ClearIRQFlags();
        nRF24_FlushRX();
    } else {
    	PutStr5x7(54,16,"Fail",CT_opaque);
        UC1701_Flush();
    	while(1);
    }

    // I2C2 port initialization
    PutStr5x7(0,24,"BMP180:",CT_opaque);
    UC1701_Flush();
    // I2C fast mode (400kHz)
    if (I2C2_Init(400000) == I2C_SUCCESS) {
        BMP180_Reset(); // Send reset command to BMP180
        Delay_ms(15); // Wait for BMP180 startup time (10ms by datasheet)
        if (BMP180_Check() == BMP180_SUCCESS) {
    		PutChar5x7(42,24,'v',CT_opaque);
    		i = BMP180_GetVersion();
    		PutInt5x7(48,24,i,CT_opaque);
    		BMP180_ReadCalibration();
    		if (BMP180_GetReadings(&_raw_temp,&_raw_press,BMP180_ADVRES)) {
       			CurData.Temperature = _raw_temp;
    			CurData.MinTemperature = CurData.Temperature;
        		CurData.Pressure = _raw_press;
        		CurData.MinPressure = CurData.Pressure;
        		GUI_PutTemperature5x7(60,24,CurData.Temperature,CT_opaque);
        		GUI_PutPressure5x7(60,32,CurData.Pressure,PT_mmHg,CT_opaque);
    		} else {
    			CurData.Temperature = 0;
    			CurData.MinTemperature =  32767;
    			CurData.MaxTemperature = -32767;
    			CurData.Pressure = 0;
    			CurData.MinPressure = 2147483647; // LONG_MAX - it becomes normal when the pressure will be acquired normally first time
    			PutStr5x7(60,24,"Readings",CT_opaque);
    			PutStr5x7(60,32,"failed",CT_opaque);
    		}
    		_bmp180_present = TRUE;
    	} else {
    		PutStr5x7(42,24,"Not present",CT_opaque);
    	}
    } else {
    	PutStr5x7(42,24,"I2C timeout",CT_opaque);
    }
	UC1701_Flush();

    // External interrupts
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE); // Enable the system configuration controller

	// PC6 -> EXTI line 6 (nRF24L01 IRQ)
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource6);
	// Configure EXTI on Line6
	EXTIInit.EXTI_Line = EXTI_Line6;              // EXTI will be on line 6
	EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;     // Generate IRQ
	EXTIInit.EXTI_Trigger = EXTI_Trigger_Falling; // IRQ on edge fall
	EXTIInit.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTIInit);

	// Configure EXTI9_5 interrupt
	NVICInit.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 0x01; // 1 - highest priority
	NVICInit.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_Init(&NVICInit);

	// Configure EXTI15_10 interrupt
	NVICInit.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_Init(&NVICInit);

	// PA5 -> EXTI line 5 (Button#1)
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource5);
	// Configure EXTI on Line5
	EXTIInit.EXTI_Line = EXTI_Line5;              // EXTI will be on line 5
	EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;     // Generate IRQ
	EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising_Falling; // IRQ on edge fall
	EXTIInit.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTIInit);

	// PA7 -> EXTI line 7  (Button#2)
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA,EXTI_PinSource7);
	// Configure EXTI on Line7
	EXTIInit.EXTI_Line = EXTI_Line7;              // EXTI will be on line 7
	EXTI_Init(&EXTIInit);

	// PC10 -> EXTI line 10  (Button#3)
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource10);
	// Configure EXTI on Line10
	EXTIInit.EXTI_Line = EXTI_Line10;             // EXTI will be on line 10
	EXTI_Init(&EXTIInit);

	// PC10 -> EXTI line 11  (Button#4)
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource11);
	// Configure EXTI on Line11
	EXTIInit.EXTI_Line = EXTI_Line11;             // EXTI will be on line 11
	EXTI_Init(&EXTIInit);

	// RTC wakeup -> EXTI line 20
	EXTIInit.EXTI_Line = EXTI_Line20;             // RTC wakeup on EXTI line 20
	EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;     // Generate IRQ
	EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising;  // IRQ on rising edge
	EXTIInit.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTIInit);
	// RTC wakeup IRQ
	NVICInit.NVIC_IRQChannel = RTC_WKUP_IRQn;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 0x0f; // 0x0f - lowest priority
	NVICInit.NVIC_IRQChannelSubPriority = 0x0f;
	NVIC_Init(&NVICInit);

/*********************************************
    // Output clock on MCO pin (PA8)
	PORT.GPIO_Pin = GPIO_Pin_8;
	PORT.GPIO_Mode = GPIO_Mode_AF;
	PORT.GPIO_OType = GPIO_OType_PP;
	PORT.GPIO_Speed = GPIO_Speed_40MHz;
    GPIO_Init(GPIOA,&PORT);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_MCO); // Alternative function on PA8 -> SYS_MCO
//    RCC_MCOConfig(RCC_MCOSource_HSE,RCC_MCODiv_2); // Route HSE clock to MCO pin
    RCC_MCOConfig(RCC_MCOSource_LSE,RCC_MCODiv_2); // Route LSE clock to MCO pin
//    RCC_MCOConfig(RCC_MCOSource_SYSCLK,RCC_MCODiv_2); // Route System clock to MCO pin
//    RCC_MCOConfig(RCC_MCOSource_PLLCLK,RCC_MCODiv_4); // Route PLL clock to MCO pin
**********************************************/

//	Delay_ms(2500); // Fancy startup delay

	UC1701_Fill(0x00);
    UC1701_Flush();

	// Set custom time for debug purposes
	RTC_Time.RTC_Hours   = 23;
	RTC_Time.RTC_Minutes = 58;
	RTC_Time.RTC_Seconds = 51;
	RTC_SetTime(RTC_Format_BIN,&RTC_Time);
	RTC_Date.RTC_Date    = 16;
	RTC_Date.RTC_Month   = 04;
	RTC_Date.RTC_Year    = 14;
	RTC_Date.RTC_WeekDay = 3;
	RTC_SetDate(RTC_Format_BIN,&RTC_Date);

	if (_bmp180_present) {
		altitude_history[0] = BMP180_hPa_to_Altitude(BMP180_GetPressure(BMP180_ADVRES));
	}

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


/////////////////////////////////////////////////////////////////////////
//	Main loop
/////////////////////////////////////////////////////////////////////////

	while(1) {
//		if (_idle_time > LCD_BACKLIGHT_TIMEOUT)	UC1701_SetBacklight(0); else UC1701_SetBacklight(Settings.LCD_brightness);

		if (_new_packet) ParsePacket();

		if (_new_time) {
			if (_no_signal_time > 10) {
				CurData.Speed = 0;
				CurData.Cadence = 0;
			}

			if (_bmp180_present) {
				if (_altitude_duty_cycle > ALT_MEASURE_DUTY_CYCLE) {
					if (BMP180_GetReadings(&_raw_temp,&_raw_press,BMP180_ADVRES)) {
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
					}
				}
			}

			_new_time = FALSE;
		}

		if (GPS_new_data) {
			if (GPSData.datetime_valid) {
				// Date and time obtained from GPS
				RTC_Time.RTC_Hours   =  GPSData.time / 3600;
				RTC_Time.RTC_Minutes = (GPSData.time / 60) % 60;
				RTC_Time.RTC_Seconds =  GPSData.time % 60;
				i = GPSData.date / 1000000;
				RTC_Date.RTC_Date  = i;
				RTC_Date.RTC_Month = (GPSData.date - (i * 1000000)) / 10000;
				RTC_Date.RTC_Year  = (GPSData.date % 10000) - 2000;
				RTC_AdjustTimeZone(&RTC_Time,&RTC_Date,Settings.GMT_offset);
				RTC_SetTime(RTC_Format_BIN,&RTC_Time);
				RTC_SetDate(RTC_Format_BIN,&RTC_Date);
			}

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

			GPSData.datetime_valid = FALSE;
			GPSData.time_valid = FALSE;
			GPS_new_data = FALSE;
		}

		// "Up" button pressed - next screen
		if (BTN[BTN_UP].cntr > 0) {
			_current_screen++;
			if (_current_screen > 6) _current_screen = 0;
			BTN[BTN_UP].cntr = 0;
		}

		// "Down" button pressed - previous screen
		if (BTN[BTN_DOWN].cntr > 0) {
			_current_screen--;
			if (_current_screen > 6) _current_screen = 6;
			BTN[BTN_DOWN].cntr = 0;
		}

		// "Enter" button hold - show main menu
		if (BTN[BTN_ENTER].state == BTN_Hold || BTN[BTN_ENTER].cntr) {
			GUI_MainMenu();
		}

		// "Escape" button pressed - just clear counter, there is no function for it yet
		if (BTN[BTN_ESCAPE].cntr) BTN[BTN_ESCAPE].cntr = 0;

		switch(_current_screen) {
		default:
			UC1701_Fill(0x00);
			GUI_DrawSpeed(scr_width - 55,0,CurData.Speed,CurData.AvgSpeed);

			VLine(scr_width - 58,0,scr_height - 29,PSet);
			HLine(0,scr_width - 1,scr_height - 29,PSet);
			VLine(67,scr_height - 29,scr_height - 1,PSet);

			if (_no_signal_time > 10)
				GUI_DrawBitmap(0,0,13,7,&bmp_icon_13x7[52]);
			else
				GUI_DrawBitmap(0,0,13,7,&bmp_icon_13x7[39]);

			if (GPSData.fix != 2 && GPSData.fix != 3)
				GUI_DrawBitmap(16,0,13,7,&bmp_icon_13x7[26]);
			else if (GPSData.fix == 2) GUI_DrawBitmap(16,0,13,7,&bmp_icon_13x7[0]);
			else GUI_DrawBitmap(16,0,13,7,&bmp_icon_13x7[13]);

			PutStr5x7(scr_width - 30,scr_height - 27,"CDC",CT_opaque);
			GUI_DrawNumber(-scr_width + 7,-scr_height + 1,CurData.Cadence,0,DS_Small);
			GUI_DrawBitmap(scr_width - 5,scr_height - 19,5,19,&small_signs[15]);

			PutStr5x7(3,scr_height - 27,"Ride Time",CT_opaque);
			GUI_DrawRideTime(0,scr_height - 19,CurData.TripTime);

			PutStr5x7(0,18,"Alt:",CT_opaque);
			PutChar5x7(23 + PutInt5x7(23,18,CurData.Altitude,CT_opaque),18,'m',CT_opaque);
			GUI_PutTemperature5x7(0,26,CurData.Temperature,CT_opaque);
			break;
		case 1:
			UC1701_Fill(0x00);
			GUI_DrawTime(12,16,&RTC_Time,TT_Full,DS_Big);
			GUI_PutDate5x7(33,scr_height - 7,(RTC_Date.RTC_Date * 1000000) + (RTC_Date.RTC_Month * 10000) + RTC_Date.RTC_Year + 2000,CT_opaque);
			break;
		case 2:
			GUI_Screen_CurVal1();
			break;
		case 3:
			GUI_Screen_CurVal2();
			break;
		case 4:
			GUI_Screen_CurVal3();
			break;
		case 5:
			GUI_Screen_SensorRAW();
			break;
		case 6:
			if (_bmp180_present) {
				UC1701_Fill(0x00);
				GUI_PutPressure5x7(0,0,CurData.Pressure,PT_mmHg,CT_opaque);
				PutChar5x7(53 + PutInt5x7(53,0,altitude_history[0],CT_opaque),0,'m',CT_opaque);
				GUI_DrawGraph(0,8,128,56,&altitude_history[0],GT_line);
			} else {
				UC1701_Fill(0x00);
				PutStr5x7(0,0,"BMP180 is absent",CT_opaque);
			}
			break;
/*
		case xxx:
			UC1701_Fill(0x00);
			PutStr5x7(0,0,"Buttons:",CT_opaque);
			for (i = 0; i < 4; i++) {
				PutInt5x7(PutStr5x7(0,10 + i * 10,"BTN",CT_opaque),10 + i * 10,i + 1,CT_opaque);
				switch(BTN[i].state) {
				case BTN_Hold:
					PutStr5x7(29,10 + i * 10,"Hold",CT_opaque);
					break;
				case BTN_Pressed:
					PutStr5x7(29,10 + i * 10,"Pressed",CT_opaque);
					break;
				default:
					PutStr5x7(29,10 + i * 10,"Released",CT_opaque);
					break;
				}
				PutInt5x7(90,10 + i * 10,BTN[i].cntr,CT_opaque);
			}
			break;
*/
		}

		ccc++;
/*
		i = PutInt5x7(3,3,ccc,CT_opaque);
		Rect(0,0,i + 4,12,PReset);
		Rect(1,1,i + 3,11,PSet);
		Rect(2,2,i + 2,10,PReset);
*/

		UC1701_Flush();

		SleepWait(); // Sleep mode
	}

	// Something awful will happen when you reach this point
}
