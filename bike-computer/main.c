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


#define LCD_BACKLIGHT_TIMEOUT           30  // Timeout for LCD backlight (seconds)
#define BMP180_MEASURE_DUTY_CYCLE       10  // Temperature/pressure measurement duty cycle (seconds)

#define ALT_MEASURE_DUTY_CYCLE          10  // Temperature/pressure measurement duty cycle (seconds)

uint8_t nRF24_RX_Buf[nRF24_RX_PAYLOAD];     // nRF24L01 payload buffer

bool _new_packet;                           // TRUE if received new packet
bool _new_time;                             // TRUE if time was updated
bool _bmp180_present;                       // TRUE if BMP180 sensor responded on I2C bus

uint8_t _current_screen;     				// What currently displayed on screen

uint16_t WheelCircumference;                // Wheel size in centimeters

uint16_t _prev_cntr_SPD;                    // Last received cntr_SPD value
uint16_t _prev_tim_SPD;                     // Last received tim_SPD value
uint16_t _tim_excess;                       // Timer excess from 1 second ratio
uint32_t _cntr_cadence;                     // Cadence counter (for AVG calculation)
uint32_t _cadence_accum;                    // Cadence accumulator (for AVG calculation)

RTC_TimeTypeDef RTC_Time;                   // Current RTC time
RTC_DateTypeDef RTC_Date;                   // Current RTC date

uint8_t LCD_brightness;                     // LCD backlight brightness (0..100)

uint32_t _no_signal_time;                   // Time since last packet received (seconds)
uint32_t _idle_time;                        // Time from last user event (button press)

int32_t altitude_history[128];              // Last 128 altitude values
uint8_t _altitude_duty_cycle;               // Altitude measurement duty cycle
int32_t altitude_home;                      // Home altitude

int16_t _raw_temp;                          // Temporary variable for temperature
int32_t _raw_press;                         // Temporary variable for presssure

bool _new_GPS;                              // TRUE if received new GPS packet
uint8_t GPS_buf[FIFO_BUFFER_SIZE];          // Buffer with data from EB500
uint16_t GPS_buf_cntr;                      // Number of bytes contained in GPS buffer
NMEASentence_TypeDef msg;                   // NMEA sentence
uint8_t GPS_sentences_parsed;               // NMEA sentences parsed
uint16_t _DMA_cntr;                         // Last value of UART RX DMA counter (to track RX timeout)

int8_t _time_GMT_offset;                    // Time offset from GMT (hours)

uint32_t i;
uint32_t ccc;
uint32_t bbb;

// Init structures
GPIO_InitTypeDef PORT;
EXTI_InitTypeDef EXTIInit;
NVIC_InitTypeDef NVICInit;
DMA_InitTypeDef  DMAInit;

// Buttons
#define      BTN1_PORT   GPIOA
#define      BTN1_PIN    GPIO_Pin_5
#define      BTN2_PORT   GPIOA
#define      BTN2_PIN    GPIO_Pin_7
#define      BTN3_PORT   GPIOC
#define      BTN3_PIN    GPIO_Pin_10
#define      BTN4_PORT   GPIOC
#define      BTN4_PIN    GPIO_Pin_11

BTN_TypeDef BTN[4];                         // Buttons


/////////////////////////////////////////////////////////////////////////


// EXTI[5..9] lines IRQ handler
void EXTI9_5_IRQHandler(void) {
	nRF24_RX_PCKT_TypeDef RX_status;

	if (EXTI_GetITStatus(EXTI_Line5) != RESET) {
		// EXTI5 (Button#1)
		if (BTN[0].PORT->IDR & BTN[0].PIN) {
		    // Button released
			BTN[0].hold_cntr = 0;
			BTN[0].cntr++;
			BTN[0].state = BTN_Released;
		} else {
			// Button pressed
			BTN[0].hold_cntr = 0;
			BTN[0].state = BTN_Pressed;
		}
		EXTI_ClearITPendingBit(EXTI_Line5);
	}

	if (EXTI_GetITStatus(EXTI_Line7) != RESET) {
		// EXTI7 (Button#2)
		if (BTN[1].PORT->IDR & BTN[1].PIN) {
		    // Button released
			BTN[1].hold_cntr = 0;
			BTN[1].cntr++;
			BTN[1].state = BTN_Released;
		} else {
			// Button pressed
			BTN[1].hold_cntr = 0;
			BTN[1].state = BTN_Pressed;
		}
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
		// EXTI10 (Button#3)
		if (BTN[2].PORT->IDR & BTN[2].PIN) {
		    // Button released
			BTN[2].hold_cntr = 0;
			BTN[2].cntr++;
			BTN[2].state = BTN_Released;
		} else {
			// Button pressed
			BTN[2].hold_cntr = 0;
			BTN[2].state = BTN_Pressed;
		}
		EXTI_ClearITPendingBit(EXTI_Line10);
	}

	if (EXTI_GetITStatus(EXTI_Line11) != RESET) {
		// EXTI11 (Button#4)
		if (BTN[3].PORT->IDR & BTN[3].PIN) {
		    // Button released
			BTN[3].hold_cntr = 0;
			BTN[3].cntr++;
			BTN[3].state = BTN_Released;
		} else {
			// Button pressed
			BTN[3].hold_cntr = 0;
			BTN[3].state = BTN_Pressed;
		}
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
//	UART_SendStr("WKUP\r\n");
}

// TIM7 IRQ handler
void TIM7_IRQHandler(void) {
	TIM7->SR = (uint16_t)~TIM_IT_Update; // Clear the TIM6's interrupt pending bit (TIM7 produce only UPDATE IT)
//	UART_SendStr("TIM7  ");
	if (_DMA_cntr == (uint16_t)DMA1_Channel6->CNDTR) {
		// DMA pointer unchanged from last IRQ -> UART timeout
		TIM7->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN)); // Disable TIM7
		GPS_buf_cntr = FIFO_BUFFER_SIZE - (uint16_t)DMA1_Channel6->CNDTR; // Remember how many bytes received in FIFO buffer
		DMA1_Channel6->CCR &= (uint16_t)(~DMA_CCR1_EN); // Disable DMA
		UART_PORT->CR1 |= (1<<5); // Enable USART2 RX complete interrupt
		_DMA_cntr = 0;
		_new_GPS = TRUE;
//		UART_SendStr("rcvd: ");
//		UART_SendInt(GPS_buf_cntr);
	} else {
//		UART_SendStr("DMA: ");
//		UART_SendInt(_DMA_cntr);
		_DMA_cntr = (uint16_t)DMA1_Channel6->CNDTR;
	}
//	UART_SendStr("\r\n");
}

// USART2 IRQ handler
void USART2_IRQHandler(void) {
	if (UART_PORT->SR & USART_IT_RXNE) {
//		UART_SendStr("RXNE\r\n");
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


// Parse received nRF24L01 data packet
void ParsePacket(void) {
	float tmp,tmp_f,tmp_i;
	int16_t diff_SPD;

	// memcpy doesn't work here due to struct alignments
	nRF24_Packet.cntr_SPD     = (nRF24_RX_Buf[0] << 8) + nRF24_RX_Buf[1];
	nRF24_Packet.tim_SPD      = (nRF24_RX_Buf[2] << 8) + nRF24_RX_Buf[3];
	nRF24_Packet.tim_CDC      = (nRF24_RX_Buf[4] << 8) + nRF24_RX_Buf[5];
	nRF24_Packet.tx_power     = (nRF24_RX_Buf[6] >> 2) & 0x03;
	nRF24_Packet.vrefint      = ((nRF24_RX_Buf[6] & 0x03) << 8) + nRF24_RX_Buf[7];
	nRF24_Packet.observe_TX   =  nRF24_RX_Buf[8];
	nRF24_Packet.cntr_wake    = (nRF24_RX_Buf[9] << 8) + nRF24_RX_Buf[10];
	nRF24_Packet.packets_lost = (nRF24_RX_Buf[11] << 8) + nRF24_RX_Buf[12];
	nRF24_Packet.ride_time    = (nRF24_RX_Buf[13] << 8) + nRF24_RX_Buf[14];

	// Magic number '992.9696969' - timer period in sensor MCU

	// Convert SPD impulses period into speed
	if (nRF24_Packet.tim_SPD > 0) {
		//tmp = WheelCircumference * (992.9696969 * 0.036) / nRF24_Packet.tim_SPD;
		tmp = (WheelCircumference * 35.7469090884) / nRF24_Packet.tim_SPD;
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
		CurData.Odometer += WheelCircumference * diff_SPD;
		CurData.TripDist += WheelCircumference * diff_SPD;
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
	// Copy data from the FIFO buffer to another one
	// This must be done for case when new GPS data will arrive
	memcpy(&GPS_buf[0],&USART_FIFO[0],GPS_buf_cntr);

	// Clear previously parsed GPS data
	memset(&GPSData,0,sizeof(GPSData));
	for (i = 0; i < 12; i++) GPS_sats[i] = 0;
	for (i = 0; i < MAX_SATELLITES_VIEW; i++) {
		memset(&GPS_sats_view[i],0,sizeof(GPS_Satellite_TypeDef));
		GPS_sats_view[i].SNR = 255;
	}

	GPS_sentences_parsed = 0;
	msg = GPS_FindSentence(GPS_buf,0,FIFO_BUFFER_SIZE); // Find first NMEA sentence
	do {
		if (msg.type != NMEA_BAD) {
			GPS_sentences_parsed++;
			GPS_ParseSentence(GPS_buf,msg);
		}
		msg = GPS_FindSentence(GPS_buf,msg.end,FIFO_BUFFER_SIZE);
	} while (msg.end < GPS_buf_cntr);
	_new_GPS = FALSE;
	GPS_buf_cntr = 0;
}


/////////////////////////////////////////////////////////////////////////


int main(void) {
	SystemCoreClockUpdate(); // Update SystemCoreClock according to Clock Register Values

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
	memset(&GPSData,0,sizeof(GPSData));
	for (i = 0; i < 12; i++) GPS_sats[i] = 0;
	for (i = 0; i < MAX_SATELLITES_VIEW; i++) {
		memset(&GPS_sats_view[i],0,sizeof(GPS_Satellite_TypeDef));
		GPS_sats_view[i].SNR = 255;
	}

	CurData.MinGPSAlt = 0x7FFFFFFF; // LONG_MAX - first time when altitude will be acquired it becomes normal value

	_new_packet = TRUE;
	_new_time   = TRUE;

	_new_GPS = FALSE;
	_DMA_cntr = 0;
	GPS_buf_cntr = 0;

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

	// These values will be stored in EEPROM in future
	_time_GMT_offset = 3;
	WheelCircumference = 206;
	altitude_home = 178;
	LCD_brightness = 50;

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
	PORT.GPIO_PuPd = GPIO_PuPd_NOPULL;
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

	// UART port initialization
	UART2_Init(115200);

	// Configure basic timer TIM7
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE); // Enable TIM7 peripheral
	TIM7->CR1  |= TIM_CR1_ARPE; // Auto-preload enable
	TIM7->PSC   = 1600; // TIM2 prescaler
	TIM7->ARR   = 999; // TIM2 auto reload value
	TIM7->EGR   = 1; // Generate an update event to reload the prescaler value immediately
	TIM7->DIER |= TIM_DIER_UIE; // Enable TIM6 interrupt
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
	UC1701_SetBacklight(LCD_brightness);
	UC1701_Contrast(4,24);
	UC1701_Orientation(scr_normal);
	UC1701_Fill(0x00);
	GUI_DrawBitmap(98,39,30,25,&bmp_bike_man[0]);

	// Boot screen
	UC1701_PutStr5x7(0,0,"Wolk bike computer:",CT_opaque);
	UC1701_PutStr5x7(0,42,"CPU:",CT_opaque);
	i = 24 + UC1701_PutIntF5x7(24,42,SystemCoreClock / 1000,3,CT_opaque);
	UC1701_PutStr5x7(i,42,"MHz",CT_opaque);
	UC1701_Flush();

	UC1701_PutStr5x7(0,56,"STAT:",CT_opaque);
	UC1701_PutStr5x7(30,56,GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_15) == Bit_RESET ? "0" : "1",CT_opaque);

	// Init the RTC and configure it clock to LSE
	UC1701_PutStr5x7(0,8,"LSE:",CT_opaque);
	UC1701_Flush();
	RTC_Config();
	RTC_WaitForSynchro(); // Wait until the RTC Time and Date registers are synchronized with RTC APB clock
	UC1701_PutStr5x7(24,8,"Ok",CT_opaque);
	UC1701_Flush();

	// Configure nRF24L01+
    // Pinout:
    //   PC7  -> CE
    //   PB12 -> CS
    //   PB13 -> SCK
    //   PB15 -> MOSI
    //   PB14 <- MISO
    //   PC6  <- IRQ
	UC1701_PutStr5x7(0,16,"nRF24L01:",CT_opaque);
	UC1701_Flush();
    nRF24_init();
    if (nRF24_Check()) {
    	UC1701_PutStr5x7(54,16,"Ok",CT_opaque);
    	UC1701_Flush();
    	nRF24_RXMode(nRF24_RX_PIPE0,nRF24_RF_CHANNEL,nRF24_DataRate_250kbps,nRF24_CRC_on,
    			     nRF24_CRC_1byte,(uint8_t *)nRF24_RX_Addr,nRF24_RX_Addr_Size,nRF24_RX_PAYLOAD,
    			     nRF24_TXPower_0dBm);
        nRF24_ClearIRQFlags();
        nRF24_FlushRX();
    } else {
    	UC1701_PutStr5x7(54,16,"Fail",CT_opaque);
        UC1701_Flush();
    	while(1);
    }

    // I2C2 port initialization
    UC1701_PutStr5x7(0,24,"BMP180:",CT_opaque);
    UC1701_Flush();
    // I2C fast mode (400kHz)
    if (I2C2_Init(400000) == I2C_SUCCESS) {
        BMP180_Reset(); // Send reset command to BMP180
        Delay_ms(15); // Wait for BMP180 startup time (10ms by datasheet)
        if (BMP180_Check() == BMP180_SUCCESS) {
    		UC1701_PutChar5x7(42,24,'v',CT_opaque);
    		i = BMP180_GetVersion();
    		UC1701_PutInt5x7(48,24,i,CT_opaque);
    		BMP180_ReadCalibration();
    		if (BMP180_GetReadings(&_raw_temp,&_raw_press,BMP180_ADVRES)) {
       			CurData.Temperature = _raw_temp;
    			CurData.MinTemperature = CurData.Temperature;
        		CurData.Pressure = _raw_press;
        		CurData.MinPressure = CurData.Pressure;
        		UC1701_PutTemperature5x7(60,24,CurData.Temperature,CT_opaque);
        		UC1701_PutPressure5x7(60,32,CurData.Pressure,PT_mmHg,CT_opaque);
    		} else {
    			CurData.Temperature = 0;
    			CurData.MinTemperature =  32767;
    			CurData.MaxTemperature = -32767;
    			CurData.Pressure = 0;
    			CurData.MinPressure = 2147483647; // LONG_MAX - it becomes normal when the pressure will be acquired normally first time
    			UC1701_PutStr5x7(60,24,"Readings",CT_opaque);
    			UC1701_PutStr5x7(60,32,"failed",CT_opaque);
    		}
    		_bmp180_present = TRUE;
    	} else {
    		UC1701_PutStr5x7(42,24,"Not present",CT_opaque);
    	}
    } else {
    	UC1701_PutStr5x7(42,24,"I2C timeout",CT_opaque);
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

/*
	// Configure basic timer TIM6 (1000 ticks per second at 32MHz)
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE); // Enable TIM6 peripheral
	TIM6->CR1  |= TIM_CR1_ARPE; // Auto-preload enable
	TIM6->PSC   = 32000; // TIM2 prescaler [ PSC = APB1clock / (PWMfreq * OVFCounter) ]
	TIM6->ARR   = 999; // TIM2 auto reload value -> overflow every second at 32MHz
	TIM6->EGR   = 1; // Generate an update event to reload the prescaler value immediately
	TIM6->DIER |= TIM_DIER_UIE; // Enable TIM6 interrupt
	TIM6->CR1  |= TIM_CR1_CEN; // Counter enable
	// TIM6 IRQ
	NVICInit.NVIC_IRQChannel = TIM6_IRQn;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 0x07; // middle priority
	NVICInit.NVIC_IRQChannelSubPriority = 0x07; // middle priority
//	NVIC_Init(&NVICInit);
*/

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


//	Delay_ms(2500); // Fancy delay

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

	_current_screen = 1;

	ccc = 0;
	bbb = 0;


/////////////////////////////////////////////////////////////////////////
//	Main loop
/////////////////////////////////////////////////////////////////////////

	while(1) {
		if (_idle_time > LCD_BACKLIGHT_TIMEOUT)	UC1701_SetBacklight(0); else UC1701_SetBacklight(LCD_brightness);

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

			// Check if buttons hold (it is unnecessary to do this in IRQ handler)
			for (i = 0; i < 4; i++) {
				if (!(BTN[i].PORT->IDR & BTN[i].PIN)) {
					BTN[i].hold_cntr++;
					if (BTN[i].hold_cntr > 2) BTN[i].state = BTN_Hold;
				}
			}

			_new_time = FALSE;
		}

		if (_new_GPS) {
			ParseGPS();

			if (GPSData.datetime_valid) {
				// Date and time obtained from GPS
				RTC_Time.RTC_Hours   =  GPSData.time / 3600;
				RTC_Time.RTC_Minutes = (GPSData.time / 60) % 60;
				RTC_Time.RTC_Seconds =  GPSData.time % 60;
				i = GPSData.date / 1000000;
				RTC_Date.RTC_Date  = i;
				RTC_Date.RTC_Month = (GPSData.date - (i * 1000000)) / 10000;
				RTC_Date.RTC_Year  = (GPSData.date % 10000) - 2000;
				RTC_AdjustTimeZone(&RTC_Time,&RTC_Date,_time_GMT_offset);
				RTC_SetTime(RTC_Format_BIN,&RTC_Time);
				RTC_SetDate(RTC_Format_BIN,&RTC_Date);
			}

			if (GPSData.fix == 3) {
				// GPS altitude makes sense only with 3D fix
				CurData.GPSAlt = GPSData.altitude;
				if (CurData.GPSAlt > CurData.MaxGPSAlt) CurData.MaxGPSAlt = CurData.GPSAlt;
				if (CurData.GPSAlt < CurData.MinGPSAlt) CurData.MinGPSAlt = CurData.GPSAlt;
			}

			if (GPSData.fix == 2 || GPSData.fix == 3) {
				// GPS speed makes sense only when 2D or 3D position fix
				CurData.GPSSpeed = GPSData.speed;
				if (CurData.GPSSpeed > CurData.MaxGPSSpeed) CurData.MaxGPSSpeed = CurData.GPSSpeed;
			}

			GPSData.datetime_valid = FALSE;
			GPSData.time_valid = FALSE;
		}

		if (BTN[0].cntr > 0) {
			_current_screen++;
			if (_current_screen > 10) _current_screen = 0;
			BTN[0].cntr = 0;
			_idle_time = 0;
		}

		if (BTN[1].cntr > 0) {
			_current_screen--;
			if (_current_screen > 10) _current_screen = 10;
			BTN[1].cntr = 0;
			_idle_time = 0;
		}

		switch(_current_screen) {
		default:
			UC1701_Fill(0x00);
			GUI_DrawSpeed(scr_width - 55,0,CurData.Speed,CurData.AvgSpeed);

			UC1701_VLine(scr_width - 58,0,scr_height - 29,PSet);
			UC1701_HLine(0,scr_width - 1,scr_height - 29,PSet);
			UC1701_VLine(67,scr_height - 29,scr_height - 1,PSet);

			if (_no_signal_time > 10)
				GUI_DrawBitmap(0,0,13,7,&bmp_icon_13x7[52]);
			else
				GUI_DrawBitmap(0,0,13,7,&bmp_icon_13x7[39]);

			if (GPSData.fix != 2 && GPSData.fix != 3)
				GUI_DrawBitmap(16,0,13,7,&bmp_icon_13x7[26]);
			else if (GPSData.fix == 2) GUI_DrawBitmap(16,0,13,7,&bmp_icon_13x7[0]);
			else GUI_DrawBitmap(16,0,13,7,&bmp_icon_13x7[13]);

			UC1701_PutStr5x7(scr_width - 30,scr_height - 27,"CDC",CT_opaque);
			GUI_DrawNumber(scr_width - 38,scr_height - 19,CurData.Cadence,3,0,DS_Small);
			GUI_DrawBitmap(scr_width - 5,scr_height - 19,5,19,&small_signs[15]);

			UC1701_PutStr5x7(3,scr_height - 27,"Ride Time",CT_opaque);
			GUI_DrawRideTime(0,scr_height - 19,CurData.TripTime);

			UC1701_PutStr5x7(0,18,"Alt:",CT_opaque);
			UC1701_PutChar5x7(23 + UC1701_PutInt5x7(23,18,CurData.Altitude,CT_opaque),18,'m',CT_opaque);
			UC1701_PutTemperature5x7(0,26,CurData.Temperature,CT_opaque);
			break;
		case 1:
			UC1701_Fill(0x00);
			GUI_DrawTime(12,16,&RTC_Time,TT_Full,DS_Big);
			UC1701_PutDate5x7(33,scr_height - 7,(RTC_Date.RTC_Date * 1000000) + (RTC_Date.RTC_Month * 10000) + RTC_Date.RTC_Year + 2000,CT_opaque);
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
			UC1701_Fill(0x00);
			UC1701_PutStr5x7(0,0,"Buttons:",CT_opaque);
			for (i = 0; i < 4; i++) {
				UC1701_PutInt5x7(UC1701_PutStr5x7(0,10 + i * 10,"BTN",CT_opaque),10 + i * 10,i + 1,CT_opaque);
				switch(BTN[i].state) {
				case BTN_Hold:
					UC1701_PutStr5x7(29,10 + i * 10,"Hold",CT_opaque);
					break;
				case BTN_Pressed:
					UC1701_PutStr5x7(29,10 + i * 10,"Pressed",CT_opaque);
					break;
				default:
					UC1701_PutStr5x7(29,10 + i * 10,"Released",CT_opaque);
					break;
				}
				UC1701_PutInt5x7(90,10 + i * 10,BTN[i].cntr,CT_opaque);
			}
			break;
		case 7:
			UC1701_Fill(0x00);
			UC1701_PutPressure5x7(0,0,CurData.Pressure,PT_mmHg,CT_opaque);
			UC1701_PutChar5x7(83 + UC1701_PutInt5x7(83,0,altitude_history[0],CT_opaque),0,'m',CT_opaque);
			GUI_DrawGraph(0,8,128,56,&altitude_history[0],GT_line);
			break;
		case 8:
			UC1701_Fill(0x00);
			GUI_NumericSet(LCD_brightness,0,100,5,"Backlight");
			if (BTN[2].cntr > 0 || BTN[2].state == BTN_Hold) {
				if (BTN[2].state == BTN_Hold) BTN[2].cntr = 1;
				LCD_brightness -= BTN[2].cntr * 5;
				if (LCD_brightness > 100) LCD_brightness = 0;
				BTN[2].cntr = 0;
				_idle_time = 0;
			}
			if (BTN[3].cntr > 0 || BTN[3].state == BTN_Hold) {
				if (BTN[3].state == BTN_Hold) BTN[3].cntr = 1;
				LCD_brightness += BTN[3].cntr * 5;
				if (LCD_brightness > 100) LCD_brightness = 100;
				BTN[3].cntr = 0;
				_idle_time = 0;
			}
			UC1701_SetBacklight(LCD_brightness);

			break;
		case 9:
			UC1701_Fill(0x00);

			uint8_t xx = 0;
			uint8_t yy = 0;

			if (BTN[2].cntr > 0 || BTN[2].state == BTN_Hold) {
				bbb += 21;
				if (bbb > FIFO_BUFFER_SIZE) bbb = FIFO_BUFFER_SIZE;
				BTN[2].cntr = 0;
				_idle_time = 0;
			}

			if (BTN[3].cntr > 0 || BTN[3].state == BTN_Hold) {
				bbb -= 21;
				if (bbb > FIFO_BUFFER_SIZE) bbb = 0;
				BTN[3].cntr = 0;
				_idle_time = 0;
			}

			i = bbb;

			do {
				UC1701_PutChar5x7(xx,yy,GPS_buf[i++],CT_opaque);
				xx += 6;
				if (xx > scr_width - 6) {
					xx  = 0;
					yy += 8;
				}
			} while (yy < 56 && i < FIFO_BUFFER_SIZE);

			xx = 0;
			xx += UC1701_PutInt5x7(xx,57,GPS_sentences_parsed,CT_opaque) + 5;
			if (GPSData.fix != 2 && GPSData.fix != 3)
				UC1701_PutStr5x7(xx,57,"NA",CT_opaque);
			else
				UC1701_PutChar5x7(xx + UC1701_PutInt5x7(xx,57,GPSData.fix,CT_opaque),57,'D',CT_opaque);
			xx += 18;
			xx += UC1701_PutInt5x7(xx,57,GPSData.sats_used,CT_opaque);
			UC1701_PutChar5x7(xx,57,'/',CT_opaque);
			xx += UC1701_PutInt5x7(xx + 6,57,GPSData.sats_view,CT_opaque) + 12;
			xx += UC1701_PutStr5x7(xx,57,GPSData.time_valid ? "TV" : "TX",CT_opaque) + 2;
			xx += UC1701_PutStr5x7(xx,57,GPSData.datetime_valid ? "DTV" : "DTX",CT_opaque) + 5;
			xx += UC1701_PutInt5x7(xx,57,_DMA_cntr,CT_opaque) + 5;

			UC1701_HLine(0,scr_width - 1,55,PSet);

			_idle_time = 0;

			break;
		case 10:
			GUI_DrawGPSInfo();
			_idle_time = 0;
			break;
		}

		ccc++;
/*
		i = UC1701_PutInt5x7(3,3,ccc,CT_opaque);
		UC1701_Rect(0,0,i + 4,12,PReset);
		UC1701_Rect(1,1,i + 3,11,PSet);
		UC1701_Rect(2,2,i + 2,10,PReset);
*/

//		UART_SendStr("Sentences: ");
//		UART_SendInt(GPS_sentences_parsed);
//		UART_SendStr("\r\n");

		UC1701_Flush();

		SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk; // Disable SysTick interrupt
	    PWR->CR |= 1 << 2; // Clear the WUF wakeup flag
//	    __WFI();

//		UART_SendStr("MAIN()\r\n");
	}

	// Something bad happens when you reach this point
}
