#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_i2c.h>
#include <math.h>

#include <uart.h>
#include <delay.h>


/* I2C to use for communications with TEA5767 */
#define _I2C_PORT 1

#if _I2C_PORT == 1
	#define I2C_PORT         I2C1
	#define I2C_SCL_PIN      GPIO_Pin_6     // PB6
	#define I2C_SDA_PIN      GPIO_Pin_7     // PB7
	#define I2C_GPIO_PORT    GPIOB
	#define I2C_CLOCK        RCC_APB1Periph_I2C1
#elif _I2C_PORT == 2
	#define I2C_PORT         I2C2
	#define I2C_SCL_PIN      GPIO_Pin_10    // PB10
	#define I2C_SDA_PIN      GPIO_Pin_11    // PB11
	#define I2C_GPIO_PORT    GPIOB
	#define I2C_CLOCK        RCC_APB1Periph_I2C2
#endif

/* TEA5767 defines */
#define TEA5767_ADDR                     0xC0 // I2C address (0b1100000 shifted to left for one bit)
/* 1st data byte */
#define TEA5767_MUTE                     0x80 // Mute output
#define TEA5767_SCAN                     0x40 // Scan mode
/* 3rd data byte */
#define TEA5767_SCAN_UP                  0x80 // Scan UP (frequency increase)
#define TEA5767_SCAN_DOWN                0x00 // Scan DOWN (frequency decrease)
#define TEA5767_SCAN_STOP_HIGH           0x60 // Scan stop level: High (ADC = 10)
#define TEA5767_SCAN_STOP_MID            0x40 // Scan stop level: Mid (ADC = 7)
#define TEA5767_SCAN_STOP_LOW            0x20 // Scan stop level: Low (ADC = 5)
#define TEA5767_HI_INJECTION             0x10 // HLSI=1 (HI side injection)
#define TEA5767_LO_INJECTION             0x00 // HLSI=0 (LO side injection)
#define TEA5767_MONO                     0x08 // Force mono mode
#define TEA5767_STEREO                   0x00 // Enable stereo
#define TEA5767_MUTE_RIGHT               0x04 // Mute right channel
#define TEA5767_MUTE_LEFT                0x02 // Mute left channel
/* 4rd data byte */
#define TEA5767_STANDBY                  0x40 // Standby mode
#define TEA5767_JAPAN_BAND               0x20 // Japanese FM band (76MHz to 91MHz)
#define TEA5767_EUROPE_BAND              0x00 // Europe FM band (87.5MHz to 108MHz)
#define TEA5767_XTAL                     0x10 // Clock freq = 32.768kHz
#define TEA5767_SOFT_MUTE                0x08 // Soft mute
#define TEA5767_HCC                      0x04 // High Cut Control on  (HCC bit = 1)
#define TEA5767_HCC_OFF                  0x00 // High Cut Control off (HCC bit = 0)
#define TEA5767_SNC                      0x02 // Stereo Noise Cancelling is on  (SNC bit = 1)
#define TEA5767_SNC_OFF                  0x00 // Stereo Noise Cancelling is off (SNC bit = 0)

uint8_t txbuf[5];                             // This array will be sent to TEA5767 with TEA5767_Write
uint8_t rxbuf[5];                             // Data received from TEA5767 will be placed to this array

typedef struct {
	uint8_t HILO;            /* HILO=1 -> HI and HILO=0 -> LO injection */
	uint8_t Mute;            /* if set nonzero -> MUTE */
	uint8_t Mono;            /* if set nonzero -> force MONO mode */
	uint8_t Band;            /* if set nonzero -> use Japanese band (76..91MHz) */
	uint8_t HCC;             /* if set nonzero -> enable High Cut Control */
	uint8_t SNC;             /* if set nonzero -> enable Stereo Noise Cancelling */
} TEA5767_SettingsTypeDef;

TEA5767_SettingsTypeDef tuner;


uint8_t TEA5767_Init_I2C(uint32_t I2C_Clock_Speed) {
	GPIO_InitTypeDef PORT;

	// Init I2C
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	PORT.GPIO_Pin = I2C_SDA_PIN | I2C_SCL_PIN;
	PORT.GPIO_Mode = GPIO_Mode_AF_OD;
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(I2C_GPIO_PORT,&PORT);

	I2C_InitTypeDef I2CInit;
	RCC_APB1PeriphClockCmd(I2C_CLOCK,ENABLE); // Enable I2C clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);

	I2C_DeInit(I2C_PORT); // I2C reset to initial state
	I2CInit.I2C_Mode = I2C_Mode_I2C; // I2C mode is I2C
	I2CInit.I2C_DutyCycle = I2C_DutyCycle_2; // I2C fast mode duty cycle (WTF is this?)
	I2CInit.I2C_OwnAddress1 = 1; // This device address (7-bit or 10-bit)
	I2CInit.I2C_Ack = I2C_Ack_Disable; // Acknowledgment
	I2CInit.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // choose 7-bit address for acknowledgment
	I2CInit.I2C_ClockSpeed = I2C_Clock_Speed;
	I2C_Cmd(I2C_PORT,ENABLE); // Enable I2C
	I2C_Init(I2C_PORT,&I2CInit); // Configure I2C

	/* WARNING: HANG HERE IF I2C DEVICE IS NOT RESPONDING */
	while (I2C_GetFlagStatus(I2C_PORT,I2C_FLAG_BUSY)); // Wait until I2C free

	return 0;
}

void TEA5767_write(void) {
	uint8_t i;

	I2C_AcknowledgeConfig(I2C_PORT,ENABLE); // Enable I2C acknowledge
	I2C_GenerateSTART(I2C_PORT,ENABLE);
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,TEA5767_ADDR,I2C_Direction_Transmitter); // Send slave address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6

	for (i = 0; i < 5; i++) {
		I2C_SendData(I2C_PORT,txbuf[i]);
		while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	}

	I2C_AcknowledgeConfig(I2C_PORT,DISABLE); // Disable I2C acknowledge
	I2C_GenerateSTOP(I2C_PORT,ENABLE);
}

void TEA5767_read(void) {
	uint8_t i;

	I2C_AcknowledgeConfig(I2C_PORT,ENABLE); // Enable I2C acknowledge
	I2C_GenerateSTART(I2C_PORT,ENABLE);
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,TEA5767_ADDR,I2C_Direction_Receiver); // Send slave address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); // Wait for EV6

	for (i = 0; i < 4; i++) {
		while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
		rxbuf[i] = I2C_ReceiveData(I2C_PORT);
	}

	I2C_AcknowledgeConfig(I2C_PORT,DISABLE); // Disable I2C acknowledgment
	I2C_GenerateSTOP(I2C_PORT,ENABLE); // Send STOP condition

	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	rxbuf[i] = I2C_ReceiveData(I2C_PORT); // Receive last byte
}

// set frequency
// input: freq - frequency in MHz
void TEA5767_SetFreq(uint8_t hilo, float freq) {
	uint16_t div;

	if (hilo == 1)
		div = (freq * 1000000 + 225000) / 8192;
	else
		div = (freq * 1000000 - 225000) / 8192;

	txbuf[0] = (div >> 8) & 0x3f;
	if (tuner.Mute) txbuf[0] |= TEA5767_MUTE;
	txbuf[1] = div & 0xff;
	txbuf[2] = (hilo == 0) ? TEA5767_LO_INJECTION : TEA5767_HI_INJECTION;
	txbuf[3] = TEA5767_XTAL;
	if (tuner.Band) txbuf[3] |= TEA5767_JAPAN_BAND;
	if (tuner.HCC)  txbuf[3] |= TEA5767_HCC;
	if (tuner.SNC)  txbuf[3] |= TEA5767_SNC;
	txbuf[4] = 0x00; // Always PLLREF = 0 and DTC = 0

	TEA5767_write();
}

// return ADC value of last readed status
uint8_t TEA5767_get_ADC(void) {
	return (rxbuf[3] & 0xf0) >> 4;
}

// detect optimal injection
// input: freq - desired frequency in Hz
uint8_t TEA5767_hilo_optimal(float freq) {
	uint8_t ADC_lo, ADC_hi;

	// tune to F=freq+450kHz with HIGH injection
	TEA5767_SetFreq(1,(freq + 450000) / 1000000.0);
	Delay_ms(30); // at least 27ms by datasheet
	TEA5767_read();
	ADC_hi = TEA5767_get_ADC();

	// tune to F=freq-450kHz with HIGH injection
	TEA5767_SetFreq(1,(freq - 450000) / 1000000.0);
	Delay_ms(30); // at least 27ms by datasheet
	TEA5767_read();
	ADC_lo = TEA5767_get_ADC();

	// Do as AppNote says: if (LevelHigh < LevelLow) then HILO=1 else HILO=0
	return (ADC_hi < ADC_lo) ? 1 : 0;
}

// detect optimal injection for specified frequency and set it
// input: freq - frequency in MHz
void TEA5767_set_frequency(float freq) {
	tuner.HILO = TEA5767_hilo_optimal(freq * 1000000.0);
	TEA5767_SetFreq(tuner.HILO,freq);
}

// return 1 if ready flag set
uint8_t TEA5767_get_ready(void) {
	return (rxbuf[0] & 0x80) >> 7;
}

// return 1 if BLR (Band Limit Reach) flag set
uint8_t TEA5767_get_blr(void) {
	return (rxbuf[0] & 0x40) >> 6;
}

// return current frequency from tuner
float TEA5767_get_freq(void) {
	float current_freq;

	TEA5767_read();
	if (tuner.HILO)
		current_freq = ((rxbuf[1] | (rxbuf[0] & 0x3f) << 8) * 8192) - 225000;
	else
		current_freq = ((rxbuf[1] | (rxbuf[0] & 0x3f) << 8) * 8192) + 225000;

	return current_freq;
}

void print_freq(void) {
	float f_f, f_i;

	f_f = modff(floor(TEA5767_get_freq() / 100000.0 + 0.5) / 10,&f_i);
	UART_SendInt((uint32_t)f_i);
	UART_SendChar('.');
	UART_SendInt((uint32_t)(f_f * 10));
	UART_SendStr(" MHz\n");
}

// do scan for station in given direction
// input: scan_dir - scan direction
void TEA5767_scan(uint8_t scan_dir) {
	float cur_freq;
	uint16_t div;

	cur_freq = TEA5767_get_freq();

	UART_SendStr("scan: ");
	print_freq();

	if (scan_dir == TEA5767_SCAN_UP)
		div = (((cur_freq + 98304) / 1000000.0) * 1000000.0 + 225000.0) / 8192;
	else
		div = (((cur_freq - 98304) / 1000000.0) * 1000000.0 + 225000.0) / 8192;

	txbuf[0]  = (div >> 8) & 0x3f;
	if (tuner.Mute) txbuf[0] |= TEA5767_MUTE;
	txbuf[0] |= TEA5767_SCAN;
	txbuf[1]  = div & 0xff;
	txbuf[2]  = (tuner.HILO == 0) ? TEA5767_LO_INJECTION : TEA5767_HI_INJECTION;
	txbuf[2] |= scan_dir;
	txbuf[2] |= TEA5767_SCAN_STOP_HIGH;
	txbuf[3]  = TEA5767_XTAL;
	if (tuner.Band) txbuf[3] |= TEA5767_JAPAN_BAND;
	if (tuner.HCC)  txbuf[3] |= TEA5767_HCC;
	if (tuner.SNC)  txbuf[3] |= TEA5767_SNC;
	txbuf[4]  = 0x00; // Always PLLREF = 0 and DTC = 0

	TEA5767_write();
}

// do automatic search for station
// input: scan_dir - search direction
void TEA5767_auto_scan(uint8_t scan_dir) {
	float cur_freq;

	UART_SendStr("auto_scan ready = ");
	UART_SendInt(TEA5767_get_ready());
	UART_SendChar('\n');
	if (TEA5767_get_ready()) {
		UART_SendStr("auto_scan blr = ");
		UART_SendInt(TEA5767_get_blr());
		UART_SendChar('\n');
		if (TEA5767_get_blr()) {
			// Frequency band limit reached, wrap up
			if (scan_dir == TEA5767_SCAN_UP) {
				UART_SendStr("Frequency wrap 108->87.5 MHz...\n");
				TEA5767_set_frequency(87.5);
				TEA5767_scan(scan_dir);
			} else {
				UART_SendStr("Frequency wrap 87.5->108 MHz...\n");
				TEA5767_set_frequency(108.0);
				TEA5767_scan(scan_dir);
			}
		} else {
			// Start scan from current frequency
			cur_freq = floor(TEA5767_get_freq() / 100000.0 + 0.5) / 10;
			TEA5767_set_frequency(cur_freq);
			UART_SendStr("Auto scan done...\n");
		}
	}
}

void TEA5767_Print(void) {
	UART_SendStr("Ready: ");
	UART_SendInt((rxbuf[0] & 0x80) >> 7);
	UART_SendChar('\n');

	UART_SendStr("BLR: ");
	UART_SendInt((rxbuf[0] & 0x40) >> 6);
	UART_SendChar('\n');

	UART_SendStr("PLL: ");
	UART_SendInt(rxbuf[1] | (rxbuf[0] & 0x3f) << 8);
	UART_SendChar('\n');

	UART_SendStr("Stereo: ");
	UART_SendInt((rxbuf[2] & 0x40) >> 6);
	UART_SendChar('\n');

	UART_SendStr("IF counter: ");
	UART_SendInt(rxbuf[2] & 0x7f);
	UART_SendChar('\n');

	UART_SendStr("ADC: ");
	UART_SendInt((rxbuf[3] & 0xf0) >> 4);
	UART_SendChar('\n');
}



int main(void) {
	UART_Init();
	UART_SendStr("\nSTM32F103RET6 is online.\n");

	UART_SendStr("TEA5767\n");

	TEA5767_Init_I2C(400000);

	// Initial settings for TEA5767 tuner
	tuner.Band = TEA5767_EUROPE_BAND;
	tuner.HCC  = TEA5767_HCC;
	tuner.SNC  = TEA5767_SNC;
	tuner.HILO = TEA5767_HI_INJECTION;
	tuner.Mono = TEA5767_STEREO;
	tuner.Mute = 0;

	TEA5767_set_frequency(91.0);

	TEA5767_scan(TEA5767_SCAN_UP);
	Delay_ms(300);
	while (TEA5767_get_ready() == 0) {
		TEA5767_read();
		print_freq();
		Delay_ms(250);
		TEA5767_read();
		TEA5767_auto_scan(TEA5767_SCAN_UP);
	}

	UART_SendStr("-----------------------\n");
	TEA5767_read();
	TEA5767_Print();
	print_freq();
	UART_SendStr("-----------------------\n");

	while(1);
}
