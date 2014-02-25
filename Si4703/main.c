#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_i2c.h>

#include <uart.h>
#include <delay.h>


/* I2C to use for communications with Si4703 */
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


uint16_t Si4703_REGs[16]; // Si4703 registers


/* Si4703 defines */
#define Si4703_ADDR                     0x20 // I2C address (0b0010000 shifted to left for one bit)
/* Si4703 registers */
#define Si4703_DEVICEID                 0x00 // Device ID
#define Si4703_CHIPID                   0x01 // Chip ID
#define Si4703_POWERCFG                 0x02 // Power configuration
#define Si4703_CHANNEL                  0x03 // Channel
#define Si4703_SYSCONFIG1               0x04 // System configuration #1
#define Si4703_SYSCONFIG2               0x05 // System configuration #2
#define Si4703_SYSCONFIG3               0x06 // System configuration #3
#define Si4703_TEST1                    0x07 // Test 1
#define Si4703_TEST2                    0x08 // Test 2
#define Si4703_BOOT                     0x09 // Boot configuration
#define Si4703_RSSI                     0x0a // Status RSSI
#define Si4703_READCHANNEL              0x0b // Read channel
#define Si4703_RDSA                     0x0c // RDSA
#define Si4703_RDSB                     0x0d // RDSB
#define Si4703_RDSC                     0x0e // RDSC
#define Si4703_RDSD                     0x0f // RDSD
/* Power configuration */
#define Si4703_PWR_DSMUTE               15 // Softmute disable (0 = enable (default); 1 = disable)
#define Si4703_PWR_DMUTE                14 // Mute disable (0 = enable (default); 1 = disable)
#define Si4703_PWR_MONO                 13 // Mono select (0 = stereo (default); 1 = force mono)
#define Si4703_PWR_RDSM                 11 // RDS mode (0 = standard (default); 1 = verbose)
#define Si4703_PWR_SKMODE               10 // Seek mode (0 = wrap band limits and continue (default); 1 = stop at band limit)
#define Si4703_PWR_SEEKUP                9 // Seek direction (0 = down (default); 1 = up)
#define Si4703_PWR_SEEK                  8 // Seek (0 = disable (default); 1 = enable)
#define Si4703_PWR_DISABLE               6 // Powerup disable (0 = enable (default))
#define Si4703_PWR_ENABLE                0 // Powerup enable (0 = enable (default))
/* Channel */
#define Si4703_CH_TUNE                  15 // Tune (0 = disable (default); 1 = enable)
/* System configuration #1 */
#define Si4703_SC1_RDSIEN               15 // RDS interrupt enable (0 = disable (default); 1 = enable)
#define Si4703_SC1_STCIEN               14 // Seek/Tune complete interrupt enable (0 = disable (default); 1 = enable)
#define Si4703_SC1_RDS                  12 // RDS enable (0 = disable (default); 1 = enable)
#define Si4703_SC1_DE                   11 // De-emphasis (0 = 75us, USA (default); 1 = 50us, Europe, Australia, Japan)
#define Si4703_SC1_AGCD                 10 // AGC disable (0 = AGC enable (default); 1 = AGC disable)
/* System configuration #2 */
#define Si4703_SC2_BAND0                 6 // Band select
#define Si4703_SC2_BAND1                 7
#define Si4703_SC2_SPACE0                4 // Channel spacing
#define Si4703_SC2_SPACE1                5
/* System configuration #3 */
#define Si4703_SC3_VOLEXT                8 // Extended volume range (0 = disabled (Default); 1 = enabled (decrease the volume by 28dB))
/* Test 1 */
#define Si4703_T1_XOSCEN                15 // Crystal oscillator enable (0 = disable (Default); 1 = enable)
#define Si4703_T1_WTF                    8 // Datasheet aren't say anything about this, but it's necessary on powerup.
/* Status RSSI */
#define Si4703_RSSI_RDSR                15 // RDSR is ready (0 = No RDS group ready; 1 = New RDS group ready)
#define Si4703_RSSI_STC                 14 // Seek/Tune complete (0 = not complete; 1 = complete)
#define Si4703_RSSI_SFBL                13 // Seek fail/Band limit (0 = Seek successful; 1 = Seek failure/Band limit reached)
#define Si4703_RSSI_AFCRL               12 // AFC fail (0 = AFC not railed; 1 = AFC railed)
#define Si4703_RSSI_RDSS                11 // RDS sync (0 = not synchronized; 1 = decoder synchronized)
#define Si4703_RSSI_ST                   8 // Stereo indicator (0 = mono; 1 = stereo)
/* Some additional constants */
#define Si4703_SEEK_UP                   0 // Seek up (default)
#define Si4703_SEEK_DOWN                 1 // Seek down
#define Si4703_WRAP_ON                   0 // Wrap around band limit enabled (default)
#define Si4703_WRAP_OFF                  1 // Wrap around band limit disabled


// Read all registers from Si4703
void Si4703_Read(void) {
	uint8_t i;
	uint8_t buffer[32]; // 16 of 16-bit registers

	I2C_AcknowledgeConfig(I2C_PORT,ENABLE); // Enable I2C acknowledge
	I2C_GenerateSTART(I2C_PORT,ENABLE); // Send START condition
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,Si4703_ADDR,I2C_Direction_Receiver); // Send slave address for READ
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); // Wait for EV6
	// Si4703 read start from r0Ah register
	for (i = 0x14; ; i++) {
		if (i == 0x20) i = 0x00;
		while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
		buffer[i] = I2C_ReceiveData(I2C_PORT); // Receive byte
		if (i == 0x12) break;
	}
	I2C_AcknowledgeConfig(I2C_PORT,DISABLE); // Disable I2C acknowledgment
	I2C_GenerateSTOP(I2C_PORT,ENABLE); // Send STOP condition
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	buffer[i++] = I2C_ReceiveData(I2C_PORT); // Receive last byte

	for (i = 0; i < 16; i++) {
		Si4703_REGs[i] = (buffer[i<<1] << 8) | buffer[(i<<1)+1];
	}
}

// Write all registers into Si4703
void Si4703_Write(void) {
	uint8_t i;

	I2C_AcknowledgeConfig(I2C_PORT,ENABLE); // Enable I2C acknowledge
	I2C_GenerateSTART(I2C_PORT,ENABLE); // Send START condition
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,Si4703_ADDR,I2C_Direction_Transmitter); // Send slave address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	for (i = 2; i < 8; i++) {
		I2C_SendData(I2C_PORT,Si4703_REGs[i] >> 8); // MSB
		while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
		I2C_SendData(I2C_PORT,Si4703_REGs[i] & 0x00ff); // LSB
		while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	}
	I2C_AcknowledgeConfig(I2C_PORT,DISABLE); // Disable I2C acknowledgment
	I2C_GenerateSTOP(I2C_PORT,ENABLE); // Send STOP condition
}

// Init I2C and select I2C mode for Si4703
uint8_t Si4703_Init_I2C(uint32_t I2C_Clock_Speed) {
	GPIO_InitTypeDef PORT;

	// Enable peripheral clocks for PortB
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	// Set RST(PB5) and SDIO(PB7/SDA) as output
	PORT.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	PORT.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB,&PORT);
	GPIO_WriteBit(GPIOB,GPIO_Pin_5,Bit_RESET); // Pull RST low (Si4703 reset operation)
    GPIO_WriteBit(GPIOB,GPIO_Pin_7,Bit_RESET); // Select 2-wire interface (I2C)
    Delay_ms(1); // Just wait
	GPIO_WriteBit(GPIOB,GPIO_Pin_5,Bit_SET); // Pull RST high (Si4703 normal operation)
    Delay_ms(1); // Give Si4703 some time to rise after reset

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

	// WARNING: THERE WILL HANG IF NO RESPOND FROM I2C DEVICE
	while (I2C_GetFlagStatus(I2C_PORT,I2C_FLAG_BUSY)); // Wait until I2C free

    return 0;
}

// Powerup initialization of Si4703
void Si4703_Init(void) {
	// Si4703 powerup configuration sequence
	Si4703_Read();
	Si4703_REGs[Si4703_TEST1] = (1 << Si4703_T1_XOSCEN)|(1 << Si4703_T1_WTF); // power up crystall
	Si4703_Write();
	Delay_ms(500); // wait for crystal to power up (by AN230 v0.61)

	// Configure tuner beginning conditions
	Si4703_Read();
	Si4703_REGs[Si4703_POWERCFG] = (1<<Si4703_PWR_DMUTE)|(1<<Si4703_PWR_ENABLE);
    Si4703_REGs[Si4703_SYSCONFIG1] |= (1<<Si4703_SC1_RDS); // Enable RDS
    Si4703_REGs[Si4703_SYSCONFIG1] |= (1<<Si4703_SC1_DE); // 50us de-emphasis (must be on for Europe, Australia and Japan)
    Si4703_REGs[Si4703_SYSCONFIG2] &= ~(1<<Si4703_SC2_BAND1)|(1<<Si4703_SC2_BAND0); // 87.5-108MHz (USA,Europe)
    //Si4703_REGs[Si4703_SYSCONFIG2] |= (1<<Si4703_SC2_BAND1)|(1<<Si4703_SC2_BAND0); // 76-108MHz (Japan wide band)
    Si4703_REGs[Si4703_SYSCONFIG2] |= (1<<Si4703_SC2_SPACE0); // 100kHz spacing (Europe)
	Si4703_REGs[Si4703_SYSCONFIG2] &= 0xfff0;
	Si4703_REGs[Si4703_SYSCONFIG2] |= 0x0001; // minimum volume
	//Si4703_REGs[Si4703_SYSCONFIG2] |= 0x0007; // medium volume
	//Si4703_REGs[Si4703_SYSCONFIG2] |= 0x000f; // maximum volume
	//Si4703_REGs[Si4703_SYSCONFIG3] |= (1<<Si4703_SC3_VOLEXT); // Decrease the volume by 28dB
	Si4703_Write();
	Delay_ms(150); // wait for device powerup (110ms from datasheet?)
}

// Tune for channel
// Input:
//   Channel - channel frequency (MHz * 10)
//   87.5MHz  = 875
//   107.9MHz = 1079
void Si4703_SetChannel(int32_t Channel) {
	Si4703_Read();

	Channel *= 10;
	Channel -= ((Si4703_REGs[Si4703_SYSCONFIG2] & ((1<<Si4703_SC2_BAND1) | (1<<Si4703_SC2_BAND0))) == 0) ? 8750 : 7600;
	Channel /= 10;

	Si4703_REGs[Si4703_CHANNEL] &= 0xfe00; // Clear channel frequency from register
	Si4703_REGs[Si4703_CHANNEL] |= Channel; // Set new channel frequency
	Si4703_REGs[Si4703_CHANNEL] |= (1<<Si4703_CH_TUNE); // Set TUNE flag
	Si4703_Write();

	Delay_ms(50); // Gime some time for the Si4703 to tune up

	// Wait for the Si4703 to set STC flag
	UART_SendStr("Tuning ");
    UART_SendInt((Channel+875) / 10); UART_SendChar('.'); UART_SendInt((Channel+875) % 10); UART_SendStr("MHz ");
	while(1) {
		Si4703_Read();
		if ((Si4703_REGs[Si4703_RSSI] & (1<<Si4703_RSSI_STC)) != 0) break; // Tuning complete
		UART_SendChar('.');
	}
	UART_SendStr(" Ok\n");

	Si4703_Read();
	Si4703_REGs[Si4703_CHANNEL] &= ~(1<<Si4703_CH_TUNE); // Clear TUNE flag
	Si4703_Write();
	Delay_ms(1); // <---------------------- Is this necessary?

	// Wait for the Si4703 to clear STC flag
	while(1) {
		Si4703_Read();
		if ((Si4703_REGs[Si4703_RSSI] & (1<<Si4703_RSSI_STC)) == 0) break; // Tuning complete
		UART_SendStr("Waiting STC clear...\n");
	}
}

// Get current tuned channel
// Return:
//   875  = 87.5MHz
//   1079 = 107.9MHz
uint32_t Si4703_GetChannel(void) {
	uint32_t Channel;

	Si4703_Read();
	Channel = Si4703_REGs[Si4703_READCHANNEL] & 0x03ff;
	Channel += ((Si4703_REGs[Si4703_SYSCONFIG2] & ((1<<Si4703_SC2_BAND1) | (1<<Si4703_SC2_BAND0))) == 0) ? 875 : 760;

	return Channel;
}

// Seek for next station
// Input:
//   SeekDirection: SEEK_UP/SEEK_DOWN
//   Wrap: WRAP_ON/WRAP_OFF
// Return:
//   0 - Success
//   1 - Fail
uint32_t Si4703_Seek(uint8_t SeekDirection, uint8_t Wrap) {
	uint32_t freq;
	uint32_t _sfbl;

	Si4703_Read();

	if (Wrap) {
		Si4703_REGs[Si4703_POWERCFG] |=  (1<<Si4703_PWR_SKMODE); // Band wrap on
	} else {
		Si4703_REGs[Si4703_POWERCFG] &= ~(1<<Si4703_PWR_SKMODE); // Band wrap off
	}
	if (SeekDirection) {
		Si4703_REGs[Si4703_POWERCFG] &= ~(1<<Si4703_PWR_SEEKUP); // Seek up
	} else {
		Si4703_REGs[Si4703_POWERCFG] |=  (1<<Si4703_PWR_SEEKUP); // Seek down
	}
	Si4703_REGs[Si4703_POWERCFG] |= (1<<Si4703_PWR_SEEK); // Set seek start bit

	Si4703_Write(); // Start seek

	// Wait for the Si4703 to set STC flag
	UART_SendStr("Seek...\n");
	while(1) {
		Si4703_Read();
		if ((Si4703_REGs[Si4703_RSSI] & (1<<Si4703_RSSI_STC)) != 0) break; // Seek complete
		freq = Si4703_GetChannel();
	    UART_SendStr("  -->"); UART_SendInt(freq / 10); UART_SendChar('.');
	    UART_SendInt(freq % 10); UART_SendStr("MHz\n");
	    Delay_ms(50); // <-- Fancy delay, in real this unnecessary
	}

	Si4703_Read();

	_sfbl = Si4703_REGs[Si4703_RSSI] & (1<<Si4703_RSSI_SFBL); // Store value of SFBL bit
	Si4703_REGs[Si4703_POWERCFG] &= ~(1<<Si4703_PWR_SEEK); // Reset seek bit (it must be done after seek)
	Si4703_Write();
	Delay_ms(1); // <---------------------- Is this necessary?

	// Wait for the Si4703 to clear STC flag
	while(1) {
		Si4703_Read();
		if ((Si4703_REGs[Si4703_RSSI] & (1<<Si4703_RSSI_STC)) == 0) break; // Tuning complete
		UART_SendStr("Waiting STC clear...\n");
	}

	if (_sfbl) {
		UART_SendStr("Seek limit hit.\n");
		return 1;
	}

	UART_SendStr("Seek completed.\n");
	return 0;
}


int main(void) {
	UART_Init();
	UART_SendStr("\nSTM32F103RET6 is online.\n");

	UART_SendStr("Init Si4703 I2C ... "); Si4703_Init_I2C(400000); UART_SendStr("ok\n");
    UART_SendStr("Init Si4703 ... "); Si4703_Init(); UART_SendStr("ok\n");

    Si4703_Read();

	UART_SendStr("DevID="); UART_SendHex16(Si4703_REGs[0]);
	UART_SendStr(" ChipID="); UART_SendHex16(Si4703_REGs[1]);
	UART_SendChar('\n');

//    Si4703_SetChannel(893); // Radio Rocks
//    Si4703_SetChannel(904); // Retro FM
//    Si4703_SetChannel(1011); // Europa plus
//    Si4703_SetChannel(1045); // Nashe radio (RDS)
//    Si4703_SetChannel(1079); // Radio Melodia

	Si4703_SetChannel(1065);

	Si4703_Read();
	uint32_t freq = Si4703_GetChannel();
    UART_SendStr("Freq: "); UART_SendInt(freq / 10); UART_SendChar('.');
    UART_SendInt(freq % 10); UART_SendStr("MHz\n");

    Si4703_Seek(Si4703_SEEK_DOWN,Si4703_WRAP_ON);

    Si4703_Read();
	UART_SendStr((Si4703_REGs[Si4703_RSSI] & (1<<Si4703_RSSI_ST)) == 0 ? "MONO\n" : "STEREO\n");
	UART_SendStr("Signal: "); UART_SendInt(Si4703_REGs[Si4703_RSSI] & 0x007f); UART_SendStr("dBuV\n");
	freq = Si4703_GetChannel();
    UART_SendStr("Freq: "); UART_SendInt(freq / 10); UART_SendChar('.');
    UART_SendInt(freq % 10); UART_SendStr("MHz\n");

	while(1);
}
