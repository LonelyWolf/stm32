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

#define FIFO_BUFFER_SIZE              1024  // GPS FIFO and data buffer size

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

int32_t pressure_history[128];              // Last 128 pressure values
uint8_t _pressure_duty_cycle;               // Pressure measurement duty cycle

int32_t altitude_history[128];              // Last 128 altitude values
uint8_t _altitude_duty_cycle;               // Altitude measurement duty cycle
int32_t altitude_home;                      // Home altitude

bool _new_GPS;                              // TRUE if received new GPS packet
bool _USART_timeout;                        // Flag for detect UART receive timeout
uint8_t USART_FIFO[FIFO_BUFFER_SIZE];       // DMA FIFO receive buffer from USART
uint8_t GPS_buf[FIFO_BUFFER_SIZE];          // Buffer with data from EB500
uint16_t GPS_buf_cntr;                      // Number of bytes contained in GPS buffer
NMEASentence_TypeDef msg;
uint8_t GPS_sentences_parsed;               // NMEA sentences parsed

uint32_t i;

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

BTN_TypeDef BTN1;
BTN_TypeDef BTN2;
BTN_TypeDef BTN3;
BTN_TypeDef BTN4;


/////////////////////////////////////////////////////////////////////////


// EXTI[5..9] lines IRQ handler
void EXTI9_5_IRQHandler(void) {
//	uint16_t tim_diff;
	nRF24_RX_PCKT_TypeDef RX_status;

	if (EXTI_GetITStatus(EXTI_Line5) != RESET) {
		// EXTI5 (Button#1)
		if (BTN1_PORT->IDR & BTN1_PIN) {
		    // Button released
/*
			if (TIM6->CNT > BTN1.tim_start) {
				tim_diff = TIM6->CNT - BTN1.tim_start;
			} else {
				tim_diff = (999 - BTN1.tim_start) + TIM6->CNT;
			}
*/
			BTN1.hold_cntr = 0;
			BTN1.cntr++;
			BTN1.state = BTN_Released;
		} else {
			// Button pressed
			BTN1.hold_cntr = 0;
			BTN1.state = BTN_Pressed;
			BTN1.tim_start = TIM6->CNT;
		}

		EXTI_ClearITPendingBit(EXTI_Line5);
	}

	if (EXTI_GetITStatus(EXTI_Line7) != RESET) {
		// EXTI7 (Button#2)
		if (BTN2_PORT->IDR & BTN2_PIN) {
		    // Button released
			BTN2.hold_cntr = 0;
			BTN2.cntr++;
			BTN2.state = BTN_Released;
		} else {
			// Button pressed
			BTN2.hold_cntr = 0;
			BTN2.state = BTN_Pressed;
			BTN2.tim_start = TIM6->CNT;
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
		if (BTN3_PORT->IDR & BTN3_PIN) {
		    // Button released
			BTN3.hold_cntr = 0;
			BTN3.cntr++;
			BTN3.state = BTN_Released;
		} else {
			// Button pressed
			BTN3.hold_cntr = 0;
			BTN3.state = BTN_Pressed;
			BTN3.tim_start = TIM6->CNT;
		}

		EXTI_ClearITPendingBit(EXTI_Line10);
	}

	if (EXTI_GetITStatus(EXTI_Line11) != RESET) {
		// EXTI11 (Button#4)
		if (BTN4_PORT->IDR & BTN4_PIN) {
		    // Button released
			BTN4.hold_cntr = 0;
			BTN4.cntr++;
			BTN4.state = BTN_Released;
		} else {
			// Button pressed
			BTN4.hold_cntr = 0;
			BTN4.state = BTN_Pressed;
			BTN4.tim_start = TIM6->CNT;
		}

		EXTI_ClearITPendingBit(EXTI_Line11);
	}
}

// RTC wakeup IRQ handler
void RTC_WKUP_IRQHandler(void) {
	if (RTC_GetITStatus(RTC_IT_WUT)) {
		// RTC Wakeup interrupt
		RTC_GetTime(RTC_Format_BIN,&RTC_Time);
//		RTC_GetDate(RTC_Format_BIN,&RTC_Date);
		_new_time = TRUE;

		_pressure_duty_cycle++;
		_no_signal_time++;
		if (_no_signal_time > 3600) _no_signal_time = 3600; // Counter overflow protection

		_idle_time++;
		if (_idle_time > LCD_BACKLIGHT_TIMEOUT) {
			UC1701_SetBacklight(0);
		}

		RTC_ClearITPendingBit(RTC_IT_WUT);
		RTC_ClearFlag(RTC_FLAG_WUTF);
		EXTI_ClearITPendingBit(EXTI_Line20);
	}
}

// TIM6 IRQ handler
void TIM6_IRQHandler(void) {
	TIM6->SR = (uint16_t)~TIM_IT_Update; // Clear the TIM6's interrupt pending bit (TIM6 produce only UPDATE IT)

	// Check if buttons pressed
	if (!(BTN1_PORT->IDR & BTN1_PIN)) {
		BTN1.hold_cntr++;
		if (BTN1.hold_cntr > 2) BTN1.state = BTN_Hold;
	}
	if (!(BTN2_PORT->IDR & BTN2_PIN)) {
		BTN2.hold_cntr++;
		if (BTN2.hold_cntr > 2) BTN2.state = BTN_Hold;
	}
	if (!(BTN3_PORT->IDR & BTN3_PIN)) {
		BTN3.hold_cntr++;
		if (BTN3.hold_cntr > 2) BTN3.state = BTN_Hold;
	}
	if (!(BTN4_PORT->IDR & BTN4_PIN)) {
		BTN4.hold_cntr++;
		if (BTN4.hold_cntr > 2) BTN4.state = BTN_Hold;
	}
}

// TIM7 IRQ handler
void TIM7_IRQHandler(void) {
	TIM7->SR = (uint16_t)~TIM_IT_Update; // Clear the TIM6's interrupt pending bit (TIM7 produce only UPDATE IT)

	if (_USART_timeout) {
		// No USART timeout occurred
		UART_PORT->CR1 |= (1<<5); // Enable USART2 RX complete interrupt
		_USART_timeout = FALSE;
	} else {
		// USART timeout occurred
		TIM7->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN)); // Disable TIM7
		GPS_buf_cntr = FIFO_BUFFER_SIZE - (uint16_t)DMA1_Channel6->CNDTR; // Remember how many bytes received in FIFO buffer
		DMA1_Channel6->CCR &= (uint16_t)(~DMA_CCR1_EN); // Disable DMA
		_new_GPS = TRUE;
	}
}

// USART2 IRQ handler
void USART2_IRQHandler(void) {
	if (UART_PORT->SR & USART_IT_RXNE) {
		// Received data read to be read
		_USART_timeout = TRUE;
		USART_ITConfig(UART_PORT,USART_IT_RXNE,DISABLE); // Disable USART2 RX complete interrupt
		if ((DMA1_Channel6->CCR & DMA_CCR1_EN) == RESET) {
			// Byte received, DMA disabled -> enable DMA
			TIM7->CNT = 0;
			TIM7->EGR = 1;
			TIM7->CR1 |= TIM_CR1_CEN; // Enable TIM7
			DMA1_Channel6->CMAR = (uint32_t)&USART_FIFO[0];
			DMA1_Channel6->CNDTR = FIFO_BUFFER_SIZE;
			DMA1_Channel6->CCR |= DMA_CCR1_EN; // Enable DMA
		}
	}
}

// UART2_RX DMA IRQ handler
void DMA1_Channel6_IRQHandler() {
//	memcpy(GPS_buf,&USART_FIFO,GPS_BUFFER_SIZE); // Copy data from FIFO to GPS buffer
//	_new_GPS = TRUE;
//	GPS_buf_cntr++;
	DMA1->IFCR |= DMA_ISR_TCIF6; // Clear DMA1 channel6 transfer complete flag
}

/////////////////////////////////////////////////////////////////////////


// Parse received nRF24L01 data packet
void ParsePacket(void) {
	float tmp,tmp_f,tmp_i;
	int16_t diff_SPD;

	// memcpy doesn't work here due to struct alignments
//	nRF24_Packet.cntr_CDC     = (nRF24_RX_Buf[0] << 8) + nRF24_RX_Buf[1];
	nRF24_Packet.cntr_SPD     = (nRF24_RX_Buf[0] << 8) + nRF24_RX_Buf[1];
	nRF24_Packet.tim_CDC      = (nRF24_RX_Buf[2] << 8) + nRF24_RX_Buf[3];
	nRF24_Packet.tim_SPD      = (nRF24_RX_Buf[4] << 8) + nRF24_RX_Buf[5];
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


/////////////////////////////////////////////////////////////////////////


int main(void) {
	SystemCoreClockUpdate(); // Update SystemCoreClock according to Clock Register Values

/////////////////////////////////////////////////////////////////////////
//  Boot section
/////////////////////////////////////////////////////////////////////////

	// Variables initialization
	LCD_brightness = 50;

	memset(&nRF24_RX_Buf,0,nRF24_RX_PAYLOAD);
	memset(&nRF24_Packet,0,sizeof(nRF24_Packet));
	memset(&CurData,0,sizeof(CurData));
	memset(&pressure_history,0,sizeof(pressure_history));
	memset(&altitude_history,0,sizeof(altitude_history));
	memset(&GPS_buf,0,sizeof(GPS_buf));
	memset(&USART_FIFO,0,sizeof(USART_FIFO));
	memset(&GPSData,0,sizeof(GPSData));
	for (i = 0; i < 12; i++) GPS_sats[i] = 0;
	for (i = 0; i < MAX_SATELLITES_VIEW; i++) {
		memset(&GPS_sats_view[i],0,sizeof(GPS_Satellite_TypeDef));
		GPS_sats_view[i].SNR = 255;
	}

	_new_packet = TRUE;
	_new_time   = TRUE;

	_new_GPS    = FALSE;
	GPS_buf_cntr = 0;
	_USART_timeout = FALSE;

	_bmp180_present = FALSE;
	_pressure_duty_cycle = BMP180_MEASURE_DUTY_CYCLE + 1;
	_altitude_duty_cycle = ALT_MEASURE_DUTY_CYCLE + 1;
	altitude_home = 178;

	_current_screen = 0;

	_prev_cntr_SPD = 0;
	_prev_tim_SPD  = 0;
	_tim_excess    = 0;
	_cntr_cadence  = 0;
	_cadence_accum = 0;

	_no_signal_time = 0;
	_idle_time = 0;

	WheelCircumference = 206;

	memset(&BTN1,0,sizeof(BTN1));
	memset(&BTN2,0,sizeof(BTN2));
	memset(&BTN3,0,sizeof(BTN3));
	memset(&BTN4,0,sizeof(BTN4));

	// Configure some GPIO
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
	PORT.GPIO_Pin  = GPIO_Pin_5;
	GPIO_Init(BTN1_PORT,&PORT);
	// BTN2 pin (PA7)
	PORT.GPIO_Pin  = GPIO_Pin_7;
	GPIO_Init(BTN2_PORT,&PORT);
	// BTN3 pin (PC10)
	PORT.GPIO_Pin  = GPIO_Pin_10;
	GPIO_Init(BTN3_PORT,&PORT);
	// BTN4 pin (PC11)
	PORT.GPIO_Pin  = GPIO_Pin_11;
	GPIO_Init(BTN4_PORT,&PORT);

	// UART port initialization
	UART2_Init(115200);

	// UART DMA configuration
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE); // Enable DMA1 peripheral clock
	DMAInit.DMA_BufferSize = FIFO_BUFFER_SIZE;
	DMAInit.DMA_DIR = DMA_DIR_PeripheralSRC; // Copy from peripheral
	DMAInit.DMA_M2M = DMA_M2M_Disable; // Memory-to-memory disable
	DMAInit.DMA_MemoryBaseAddr = (uint32_t)&USART_FIFO[0]; // Pointer to memory buffer
	DMAInit.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; // Write bytes to memory
	DMAInit.DMA_MemoryInc = DMA_MemoryInc_Enable; // Enable memory counter per write
	DMAInit.DMA_Mode = DMA_Mode_Normal; // Non-circular mode
//	DMAInit.DMA_Mode = DMA_Mode_Circular; // Circular mode
	DMAInit.DMA_PeripheralBaseAddr = (uint32_t)(&UART_PORT->DR); // Pointer to USART_DR register
	DMAInit.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; // Read bytes from peripheral
	DMAInit.DMA_PeripheralInc = DMA_PeripheralInc_Disable; // Do not increment peripheral pointer
	DMAInit.DMA_Priority = DMA_Priority_High; // High priority
	DMA_Init(DMA1_Channel6,&DMAInit); // USART2_RX connected to DMA1_Channel6 (from datasheet table 54)
	USART_DMACmd(UART_PORT,USART_DMAReq_Rx,ENABLE); // Enable DMA for USART2 RX
/*
	DMA_ITConfig(DMA1_Channel6,DMA_IT_TC,ENABLE); // Enable DMA transfer complete interrupt
	NVICInit.NVIC_IRQChannel = DMA1_Channel6_IRQn;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 0x03; // high priority
	NVICInit.NVIC_IRQChannelSubPriority = 0x03; // high priority
	NVIC_Init(&NVICInit);
*/
//	DMA_Cmd(DMA1_Channel6,ENABLE);

	// USART2 IRQ
	USART_ITConfig(UART_PORT,USART_IT_RXNE,ENABLE); // Enable USART2
	NVICInit.NVIC_IRQChannel = USART2_IRQn;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 0x02; // high priority
	NVICInit.NVIC_IRQChannelSubPriority = 0x02; // high priority
	NVIC_Init(&NVICInit);

	// Configure basic timer TIM7
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE); // Enable TIM7 peripheral
	TIM7->CR1  |= TIM_CR1_ARPE; // Auto-preload enable
	TIM7->PSC   = 3200; // TIM2 prescaler
	TIM7->ARR   = 999; // TIM2 auto reload value
	TIM7->EGR   = 1; // Generate an update event to reload the prescaler value immediately
	TIM7->DIER |= TIM_DIER_UIE; // Enable TIM6 interrupt
//	TIM7->CR1  |= TIM_CR1_CEN; // Counter enable
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
    I2C2_Init(400000); // I2C fast mode
    BMP180_Reset(); // Reset BMP180
    Delay_ms(15); // Wait for BMP180 startup time (10ms by datasheet)
	i = BMP180_ReadReg(BMP180_CHIP_ID_REG);
	if (i == 0x55) {
		UC1701_PutChar5x7(42,24,'v',CT_opaque);
		i = BMP180_ReadReg(BMP180_VERSION_REG);
		UC1701_PutInt5x7(48,24,i,CT_opaque);
		_bmp180_present = TRUE;
		BMP180_ReadCalibration();
		CurData.Temperature = BMP180_Calc_RT(BMP180_Read_UT());
		CurData.MinTemperature = CurData.Temperature;
		CurData.Pressure = BMP180_Calc_RP(BMP180_Read_PT(3),3);
		CurData.MinPressure = CurData.Pressure;
		UC1701_PutTemperature5x7(60,24,CurData.Temperature,CT_opaque);
		UC1701_PutPressure5x7(60,32,CurData.Pressure,PT_mmHg,CT_opaque);
	} else {
		UC1701_PutStr5x7(42,24,"Not present",CT_opaque);
		_bmp180_present = FALSE;
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

//	for (i = 0; i < 128; i++) pressure_history[i] = BMP180_Calc_RP(BMP180_Read_PT(0),0);
	pressure_history[0] = BMP180_Calc_RP(BMP180_Read_PT_3AVG(),3);
	altitude_history[0] = BMP180_hPa_to_Altitude(pressure_history[0]);

	_current_screen = 8;

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


/////////////////////////////////////////////////////////////////////////
//	Main loop
/////////////////////////////////////////////////////////////////////////

	while(1) {
//		UC1701_Fill(0x00);

		if (_new_packet) ParsePacket();

		if (_new_time) {
			if (_no_signal_time > 10) {
				CurData.Speed = 0;
				CurData.Cadence = 0;
			}

			if (_bmp180_present) {
				if (_altitude_duty_cycle > ALT_MEASURE_DUTY_CYCLE) {
					CurData.Temperature = BMP180_Calc_RT(BMP180_Read_UT());
					if (CurData.Temperature > CurData.MaxTemperature) CurData.MaxTemperature = CurData.Temperature;
					if (CurData.Temperature < CurData.MinTemperature) CurData.MinTemperature = CurData.Temperature;

					CurData.Pressure = BMP180_Calc_RP(BMP180_Read_PT_3AVG(),3);
					if (CurData.Pressure > CurData.MaxPressure) CurData.MaxPressure = CurData.Pressure;
					if (CurData.Pressure < CurData.MinPressure) CurData.MinPressure = CurData.Pressure;

					CurData.Altitude = BMP180_hPa_to_Altitude(CurData.Pressure);
					if (CurData.Altitude > CurData.MaxAltitude) CurData.MaxAltitude = CurData.Altitude;
					if (CurData.Altitude < CurData.MinAltitude) CurData.MinAltitude = CurData.Altitude;

					memmove(&pressure_history[1],&pressure_history[0],sizeof(pressure_history) - sizeof(*pressure_history));
					pressure_history[0] = CurData.Pressure;

					memmove(&altitude_history[1],&altitude_history[0],sizeof(altitude_history) - sizeof(*altitude_history));
					altitude_history[0] = CurData.Altitude;

					_pressure_duty_cycle = 0;
				}
			}

			_new_time = FALSE;
		}

		if (_new_GPS) {
			// Clear previously parsed GPS data
			memset(&GPSData,0,sizeof(GPSData));
			for (i = 0; i < 12; i++) GPS_sats[i] = 0;
			for (i = 0; i < MAX_SATELLITES_VIEW; i++) {
				memset(&GPS_sats_view[i],0,sizeof(GPS_Satellite_TypeDef));
				GPS_sats_view[i].SNR = 255;
			}

//			memset(&GPS_buf,'#',FIFO_BUFFER_SIZE); // <------------------ for debug purposes
			memcpy(&GPS_buf[0],&USART_FIFO[0],GPS_buf_cntr); // Copy data from FIFO to GPS buffer
			// Find first NMEA sentence
			GPS_sentences_parsed = 0;
			msg = GPS_FindSentence(GPS_buf,0,FIFO_BUFFER_SIZE);
			do {
				if (msg.type != NMEA_BAD) {
					GPS_sentences_parsed++;
					GPS_ParseSentence(GPS_buf,msg);
				}
				msg = GPS_FindSentence(GPS_buf,msg.end,FIFO_BUFFER_SIZE);
			} while (msg.end < FIFO_BUFFER_SIZE);

			_new_GPS = FALSE;
			GPS_buf_cntr = 0;
			memset(&USART_FIFO,'x',FIFO_BUFFER_SIZE);
			USART_ITConfig(UART_PORT,USART_IT_RXNE,ENABLE);
		}

		if (BTN1.cntr > 0) {
			_current_screen++;
			if (_current_screen > 8) _current_screen = 0;
			BTN1.cntr = 0;

			_idle_time = 0;
			UC1701_SetBacklight(LCD_brightness);
		}

		if (BTN2.cntr > 0) {
			_current_screen--;
			if (_current_screen > 8) _current_screen = 8;
			BTN2.cntr = 0;

			_idle_time = 0;
			UC1701_SetBacklight(LCD_brightness);
		}

/*
		if (BTN3.cntr > 0) {
			BTN3.cntr = 0;

			_idle_time = 0;
			UC1701_SetBacklight(LCD_brightness);
		}

		if (BTN4.cntr > 0) {
			BTN4.cntr = 0;

			_idle_time = 0;
			UC1701_SetBacklight(LCD_brightness);
		}
*/
		switch(_current_screen) {
		default:
			UC1701_Fill(0x00);
			GUI_DrawSpeed(scr_width - 55,0,CurData.Speed,CurData.AvgSpeed);

			UC1701_VLine(scr_width - 58,0,scr_height - 29,PSet);
			UC1701_HLine(0,scr_width - 1,scr_height - 29,PSet);
			UC1701_VLine(67,scr_height - 29,scr_height - 1,PSet);

			if (_no_signal_time > 10) {
				GUI_DrawBitmap(0,0,16,16,&bmp_icons_16x16[32]);
			} else {
				GUI_DrawBitmap(0,0,16,16,&bmp_icons_16x16[0]);
			}

			UC1701_PutStr5x7(scr_width - 30,scr_height - 27,"CDC",CT_opaque);
			GUI_DrawNumber(scr_width - 38,scr_height - 19,CurData.Cadence,3,0,DS_Small);
			GUI_DrawBitmap(scr_width - 5,scr_height - 19,5,19,&small_signs[15]);

			UC1701_PutStr5x7(3,scr_height - 27,"Ride Time",CT_opaque);
			GUI_DrawRideTime(0,scr_height - 19,CurData.TripTime);

//			UC1701_PutPressure5x7(0,18,CurData.Pressure,PT_mmHg,CT_opaque);
			UC1701_PutStr5x7(0,18,"Alt:",CT_opaque);
			UC1701_PutChar5x7(23 + UC1701_PutInt5x7(23,18,CurData.Altitude,CT_opaque),18,'m',CT_opaque);
			UC1701_PutTemperature5x7(0,26,CurData.Temperature,CT_opaque);
			break;
		case 1:
			GUI_Screen_CurVal1();
			break;
		case 2:
			GUI_Screen_CurVal2();
			break;
		case 3:
			GUI_Screen_SensorRAW();
			_idle_time = 0;
			break;
		case 4:
			UC1701_Fill(0x00);
			UC1701_PutStr5x7(0,0,"BTN1:",CT_opaque);
			switch(BTN1.state) {
			case BTN_Hold:
				UC1701_PutStr5x7(29,0,"Hold",CT_opaque);
				break;
			case BTN_Pressed:
				UC1701_PutStr5x7(29,0,"Pressed",CT_opaque);
				break;
			default:
				UC1701_PutStr5x7(29,0,"Released",CT_opaque);
				break;
			}
			UC1701_PutInt5x7(90,0,BTN1.cntr,CT_opaque);

			UC1701_PutStr5x7(0,8,"BTN2:",CT_opaque);
			switch(BTN2.state) {
			case BTN_Hold:
				UC1701_PutStr5x7(29,8,"Hold",CT_opaque);
				break;
			case BTN_Pressed:
				UC1701_PutStr5x7(29,8,"Pressed",CT_opaque);
				break;
			default:
				UC1701_PutStr5x7(29,8,"Released",CT_opaque);
				break;
			}
			UC1701_PutInt5x7(90,8,BTN2.cntr,CT_opaque);

			UC1701_PutStr5x7(0,16,"BTN3:",CT_opaque);
			switch(BTN3.state) {
			case BTN_Hold:
				UC1701_PutStr5x7(29,16,"Hold",CT_opaque);
				break;
			case BTN_Pressed:
				UC1701_PutStr5x7(29,16,"Pressed",CT_opaque);
				break;
			default:
				UC1701_PutStr5x7(29,16,"Released",CT_opaque);
				break;
			}
			UC1701_PutInt5x7(90,16,BTN3.cntr,CT_opaque);

			UC1701_PutStr5x7(0,24,"BTN4:",CT_opaque);
			switch(BTN4.state) {
			case BTN_Hold:
				UC1701_PutStr5x7(29,24,"Hold",CT_opaque);
				break;
			case BTN_Pressed:
				UC1701_PutStr5x7(29,24,"Pressed",CT_opaque);
				break;
			default:
				UC1701_PutStr5x7(29,24,"Released",CT_opaque);
				break;
			}
			UC1701_PutInt5x7(90,24,BTN4.cntr,CT_opaque);

			break;
		case 5:
			UC1701_Fill(0x00);
			UC1701_PutPressure5x7(0,0,CurData.Pressure,PT_mmHg,CT_opaque);
			UC1701_PutChar5x7(83 + UC1701_PutInt5x7(83,0,altitude_history[0],CT_opaque),0,'m',CT_opaque);
			GUI_DrawGraph(0,8,128,56,&altitude_history[0],GT_line);
//			_idle_time = 0;

			break;
		case 6:
			UC1701_Fill(0x00);
			GUI_NumericSet(LCD_brightness,0,100,5,"Backlight");

			if (BTN3.cntr > 0 || BTN3.state == BTN_Hold) {
				if (BTN3.state == BTN_Hold) BTN3.cntr = 1;
				LCD_brightness -= BTN3.cntr * 5;
				if (LCD_brightness > 100) LCD_brightness = 0;
				BTN3.cntr = 0;
				_idle_time = 0;
			}

			if (BTN4.cntr > 0 || BTN4.state == BTN_Hold) {
				if (BTN4.state == BTN_Hold) BTN4.cntr = 1;
				LCD_brightness += BTN4.cntr * 5;
				if (LCD_brightness > 100) LCD_brightness = 100;
				BTN4.cntr = 0;
				_idle_time = 0;
			}

			UC1701_SetBacklight(LCD_brightness);

			break;
		case 7:
			UC1701_Fill(0x00);
			UC1701_PutStr5x7(0,57,_new_GPS ? "T" : "F",CT_opaque);
			UC1701_PutInt5x7(10,57,GPS_buf_cntr,CT_opaque);

/*
			uint8_t xx = 0;
			uint8_t yy = 0;
			i = 0;
			do {
				UC1701_PutChar5x7(xx,yy,GPS_buf[i++],CT_opaque);
				xx += 6;
				if (xx > scr_width - 7) {
					xx  = 0;
					yy += 8;
				}
			} while (yy < 8);
*/

			UC1701_PutInt5x7(40,57,GPS_sentences_parsed,CT_opaque);
			UC1701_PutInt5x7(scr_width - 13,57,GPSData.sats_view,CT_opaque);
			UC1701_PutInt5x7(scr_width - 29,57,GPSData.sats_used,CT_opaque);
			if (GPSData.fix != 2 && GPSData.fix != 3)
				UC1701_PutStr5x7(54,57,"NA",CT_opaque);
			else
				UC1701_PutChar5x7(54 + UC1701_PutInt5x7(54,57,GPSData.fix,CT_opaque),57,'D',CT_opaque);

			_idle_time = 0;

			break;
		case 8:
			GUI_DrawGPSInfo();
			_idle_time = 0;
			break;
		}


		UC1701_Flush();
	}

	// Something bad happens if you reach these point
}
