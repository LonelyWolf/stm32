#include <stm32l1xx_rcc.h>
#include <stm32l1xx_gpio.h>

#include <string.h>
#include <spi.h>
#include <delay.h>
#include <nRF24.h>


const uint8_t RX_PW_PIPES[6]   = {nRF24_REG_RX_PW_P0, nRF24_REG_RX_PW_P1, nRF24_REG_RX_PW_P2,
		                          nRF24_REG_RX_PW_P3, nRF24_REG_RX_PW_P4, nRF24_REG_RX_PW_P5};
const uint8_t RX_ADDR_PIPES[6] = {nRF24_REG_RX_ADDR_P0, nRF24_REG_RX_ADDR_P1, nRF24_REG_RX_ADDR_P2,
		                          nRF24_REG_RX_ADDR_P3, nRF24_REG_RX_ADDR_P4, nRF24_REG_RX_ADDR_P5};


// nRF24L01 initialization
// note: the SPI peripheral must be initialized before
void nRF24_Init() {
	GPIO_InitTypeDef PORT;

	// Initialize nRF24L01 GPIO
	RCC_AHBPeriphClockCmd(nRF24_PORT_PERIPH,ENABLE); // Enable GPIO peripherals

	// Output push-pull
	PORT.GPIO_Mode  = GPIO_Mode_OUT;
	PORT.GPIO_Speed = GPIO_Speed_40MHz;
	PORT.GPIO_OType = GPIO_OType_PP;
	PORT.GPIO_PuPd  = GPIO_PuPd_UP;

	// Configure CS pin
	PORT.GPIO_Pin = nRF24_CSN_PIN;
	GPIO_Init(nRF24_CSN_PORT,&PORT);

	// Configure CE pin
	PORT.GPIO_Pin = nRF24_CE_PIN;
	GPIO_Init(nRF24_CE_PORT,&PORT);

	// Configure IRQ pin as input without Pull-Up
	// IRQ pin is already have the pull-up inside, so enabling it in MCU will increase
	// power consumption for about 80uA
	PORT.GPIO_Pin  = nRF24_IRQ_PIN;
	PORT.GPIO_Mode = GPIO_Mode_IN;
	PORT.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(nRF24_IRQ_PORT,&PORT);

	nRF24_CSN_H(); // Chip release
	nRF24_CE_L();  // RX/TX disable

	// Clear FIFO and interrupt flags
	nRF24_FlushRX();
	nRF24_FlushTX();
	nRF24_ClearIRQFlags();
}

// Write new value to register
// input:
//   reg - register number
//   value - new value
void nRF24_WriteReg(uint8_t reg, uint8_t value) {
	nRF24_CSN_L();
	SPIx_SendRecv(nRF24_SPI_PORT,reg); // Select register
	SPIx_SendRecv(nRF24_SPI_PORT,value); // Write value to register
	nRF24_CSN_H();
}

// Read nRF24L01 register
// input:
//   reg - register number
// return: register value
uint8_t nRF24_ReadReg(uint8_t reg) {
	uint8_t value;

	nRF24_CSN_L();
	SPIx_SendRecv(nRF24_SPI_PORT,reg & 0x1f); // Select register to read from
	value = SPIx_SendRecv(nRF24_SPI_PORT,nRF24_CMD_NOP); // Read register value
	nRF24_CSN_H();

	return value;
}

// Get data from nRF24L01 into buffer
// input:
//   reg - register number
//   pBuf - pointer to buffer
//   count - bytes count
void nRF24_ReadBuf(uint8_t reg, uint8_t *pBuf, uint8_t count) {
	nRF24_CSN_L();
	SPIx_SendRecv(nRF24_SPI_PORT,reg); // Select register to read from
	while (count--) *pBuf++ = SPIx_SendRecv(nRF24_SPI_PORT,nRF24_CMD_NOP);
	nRF24_CSN_H();
}

// Send buffer to nRF24L01
// input:
//   reg - register number
//   pBuf - pointer to buffer
//   count - bytes count
void nRF24_WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t count) {
	nRF24_CSN_L();
	SPIx_SendRecv(nRF24_SPI_PORT,reg); // Select register
	while (count--) SPIx_SendRecv(nRF24_SPI_PORT,*pBuf++);
	nRF24_CSN_H();
}

// Check if nRF24L01 present (send byte sequence, read it back and compare)
// return:
//   1 - nRF24L01 is online and responding
//   0 - received sequence differs from original
uint8_t nRF24_Check(void) {
    uint8_t rxbuf[5];
    uint8_t *ptr = (uint8_t *)nRF24_TEST_ADDR;
    uint8_t i;

    nRF24_WriteBuf(nRF24_CMD_WREG | nRF24_REG_TX_ADDR,ptr,5); // Write fake TX address
    nRF24_ReadBuf(nRF24_REG_TX_ADDR,rxbuf,5); // Read TX_ADDR register
    for (i = 0; i < 5; i++) if (rxbuf[i] != *ptr++) return 0;

    return 1;
}

// Set nRF24L01 frequency channel
// input:
//   RFChannel - Frequency channel (0..127) (frequency = 2400 + RFChan [MHz])
// Note, what part of the OBSERVER_TX register called "PLOS_CNT" will be cleared!
void nRF24_SetRFChannel(uint8_t RFChannel) {
    nRF24_WriteReg(nRF24_CMD_WREG | nRF24_REG_RF_CH,RFChannel);
}

// Flush nRF24L01 TX FIFO buffer
void nRF24_FlushTX(void) {
	nRF24_WriteReg(nRF24_CMD_FLUSH_TX,0xFF);
}

// Flush nRF24L01 RX FIFO buffer
void nRF24_FlushRX(void) {
	nRF24_WriteReg(nRF24_CMD_FLUSH_RX,0xFF);
}

// Put nRF24L01 in TX mode
// input:
//   RetrCnt - Auto retransmit count on fail of AA (1..15 or 0 for disable)
//   RetrDelay - Auto retransmit delay 250us+(0..15)*250us (0 = 250us, 15 = 4000us)
//   RFChan - Frequency channel (0..127) (frequency = 2400 + RFChan [MHz])
//   DataRate - Set data rate: nRF24_DataRate_1Mbps or nRF24_DataRate_2Mbps
//   TXPower - RF output power (-18dBm, -12dBm, -6dBm, 0dBm)
//   CRCS - CRC encoding scheme (nRF24_CRC_[off | 1byte | 2byte])
//   PWR - power state (nRF24_PWR_Up or nRF24_PWR_Down)
//   TX_Addr - buffer with TX address
//   TX_Addr_Width - size of the TX address (3..5 bytes)
void nRF24_TXMode(uint8_t RetrCnt, uint8_t RetrDelay, uint8_t RFChan, nRF24_DataRate_TypeDef DataRate,
		nRF24_TXPower_TypeDef TXPower, nRF24_CRC_TypeDef CRCS, nRF24_PWR_TypeDef Power, uint8_t *TX_Addr,
		uint8_t TX_Addr_Width) {
    nRF24_CE_L();
    nRF24_WriteReg(nRF24_CMD_WREG | nRF24_REG_SETUP_RETR,((RetrDelay << 4) & 0xf0) | (RetrCnt & 0x0f)); // Auto retransmit settings
    nRF24_WriteReg(nRF24_CMD_WREG | nRF24_REG_RF_SETUP,(uint8_t)DataRate | (uint8_t)TXPower); // Setup register
    nRF24_WriteReg(nRF24_CMD_WREG | nRF24_REG_CONFIG,(uint8_t)CRCS | (uint8_t)Power | nRF24_PRIM_TX); // Config register
    nRF24_SetRFChannel(RFChan); // Set frequency channel (OBSERVER_TX part PLOS_CNT will be cleared)
    nRF24_WriteReg(nRF24_CMD_WREG | nRF24_REG_EN_AA,0x01); // Enable ShockBurst for data pipe 0 to receive ACK packet
	nRF24_WriteReg(nRF24_CMD_WREG | nRF24_REG_SETUP_AW,TX_Addr_Width); // Set address width
    nRF24_WriteBuf(nRF24_CMD_WREG | nRF24_REG_TX_ADDR,TX_Addr,TX_Addr_Width); // Set static TX address
	nRF24_WriteBuf(nRF24_CMD_WREG | nRF24_REG_RX_ADDR_P0,TX_Addr,TX_Addr_Width); // Static RX address on PIPE0 must same as TX address for auto ACK
}

// Put nRF24L01 in RX mode
// input:
//   PIPE - RX data pipe (nRF24_RX_PIPE[0..5])
//   PIPE_AA - auto acknowledgment for data pipe (nRF24_ENAA_P[0..5] or nRF24_ENAA_OFF)
//   RFChan - Frequency channel (0..127) (frequency = 2400 + RFChan [MHz])
//   DataRate - Set data rate (nRF24_DataRate_[250kbps,1Mbps,2Mbps])
//   CRCS - CRC encoding scheme (nRF24_CRC_[off | 1byte | 2byte])
//   RX_Addr - buffer with TX address
//   RX_Addr_Width - size of TX address (3..5 byte)
//   RX_PAYLOAD - receive buffer length
//   TXPower - RF output power for ACK packets (-18dBm, -12dBm, -6dBm, 0dBm)
void nRF24_RXMode(nRF24_RX_PIPE_TypeDef PIPE, nRF24_ENAA_TypeDef PIPE_AA, uint8_t RFChan,
		nRF24_DataRate_TypeDef DataRate, nRF24_CRC_TypeDef CRCS, uint8_t *RX_Addr, uint8_t RX_Addr_Width,
		uint8_t RX_PAYLOAD, nRF24_TXPower_TypeDef TXPower) {
	uint8_t rreg;

	nRF24_CE_L();
	nRF24_ReadReg(nRF24_CMD_NOP); // Dummy read
	rreg = nRF24_ReadReg(nRF24_REG_EN_AA);
	if (PIPE_AA != nRF24_ENAA_OFF) {
		// Enable auto acknowledgment for given data pipe
		rreg |= (uint8_t)PIPE_AA;
	} else {
		// Disable auto acknowledgment for given data pipe
		rreg &= ~(1 << (uint8_t)PIPE);
	}
	nRF24_WriteReg(nRF24_CMD_WREG | nRF24_REG_EN_AA,rreg);
	rreg = nRF24_ReadReg(nRF24_REG_EN_RXADDR);
	nRF24_WriteReg(nRF24_CMD_WREG | nRF24_REG_EN_RXADDR,rreg | (1 << (uint8_t)PIPE)); // Enable given data pipe
	nRF24_WriteReg(nRF24_CMD_WREG | RX_PW_PIPES[(uint8_t)PIPE],RX_PAYLOAD); // Set RX payload length
    nRF24_WriteReg(nRF24_CMD_WREG | nRF24_REG_RF_SETUP,(uint8_t)DataRate | (uint8_t)TXPower); // SETUP register
    nRF24_WriteReg(nRF24_CMD_WREG | nRF24_REG_CONFIG,(uint8_t)CRCS | nRF24_PWR_Up | nRF24_PRIM_RX); // Config register
    nRF24_SetRFChannel(RFChan); // Set frequency channel
    nRF24_WriteReg(nRF24_CMD_WREG | nRF24_REG_SETUP_AW,RX_Addr_Width - 2); // Set of address widths (common for all data pipes)
    nRF24_WriteBuf(nRF24_CMD_WREG | RX_ADDR_PIPES[(uint8_t)PIPE],RX_Addr,RX_Addr_Width); // Set static RX address for given data pipe
    nRF24_ClearIRQFlags();
	nRF24_FlushRX();
	nRF24_CE_H(); // RX mode
}

// Send data packet
// input:
//   pBuf - buffer with data to send
//   TX_PAYLOAD - buffer size
// return:
//   nRF24_TX_XXX values
nRF24_TX_PCKT_TypeDef nRF24_TXPacket(uint8_t * pBuf, uint8_t TX_PAYLOAD) {
    uint8_t status;
    uint32_t wait = nRF24_WAIT_TIMEOUT;

    // Release CE pin (in case if it still high)
    nRF24_CE_L();
    // Transfer data from specified buffer to the TX FIFO
    nRF24_WriteBuf(nRF24_CMD_W_TX_PAYLOAD,pBuf,TX_PAYLOAD);
    // CE pin high => Start transmit (must hold pin at least 10us)
    nRF24_CE_H();
    // Wait for and IRQ from nRF24L01
    while ((nRF24_IRQ_PORT->IDR & nRF24_IRQ_PIN) && --wait);
    if (!wait) return nRF24_TX_TIMEOUT;
    // Release CE pin
    nRF24_CE_L();

    // Read the status register
    status = nRF24_ReadReg(nRF24_REG_STATUS);
    // Clear pending IRQ flags
    nRF24_WriteReg(nRF24_CMD_WREG | nRF24_REG_STATUS,status | 0x70);
    if (status & nRF24_MASK_MAX_RT) {
        // Auto retransmit counter exceeds the programmed maximum limit. FIFO is not removed.
    	nRF24_FlushTX();

    	return nRF24_TX_MAXRT;
    };
    if (status & nRF24_MASK_TX_DS) {
        // Transmit successful
//    	nRF24_FlushTX(); // Flush TX FIFO buffer (is it really must be here?)

    	return nRF24_TX_SUCCESS;
    }

    // Some banana happens
	nRF24_FlushTX();
	nRF24_ClearIRQFlags();

    return nRF24_TX_ERROR;
}

// Receive data packet
// input:
//   pBuf - buffer for received data
//   RX_PAYLOAD - buffer size
// return:
//   nRF24_RX_PCKT_PIPE[0..5] - packet received from specific data pipe
//   nRF24_RX_PCKT_ERROR - RX_DR bit was not set
//   nRF24_RX_PCKT_EMPTY - RX FIFO is empty
nRF24_RX_PCKT_TypeDef nRF24_RXPacket(uint8_t * pBuf, uint8_t RX_PAYLOAD) {
	uint8_t status;
	nRF24_RX_PCKT_TypeDef result = nRF24_RX_PCKT_ERROR;

	status = nRF24_ReadReg(nRF24_REG_STATUS); // Read the status register
	if (status & nRF24_MASK_RX_DR) {
    	// RX_DR bit set (Data ready RX FIFO interrupt)
		result = (nRF24_RX_PCKT_TypeDef)((status & 0x0e) > 1); // Pipe number
		if ((uint8_t)result < 6) {
			// Read received payload from RX FIFO buffer
			nRF24_ReadBuf(nRF24_CMD_R_RX_PAYLOAD,pBuf,RX_PAYLOAD);
			// Clear pending IRQ flags
			nRF24_WriteReg(nRF24_CMD_WREG | nRF24_REG_STATUS,status | 0x70);
			// Check if RX FIFO is empty and flush it if not
			status = nRF24_ReadReg(nRF24_REG_FIFO_STATUS);
	    	if (status & nRF24_FIFO_RX_EMPTY) nRF24_FlushRX();

	    	return result; // Data pipe number
		} else {
	    	// RX FIFO is empty
	    	return nRF24_RX_PCKT_EMPTY;
		}
	}

	// Some banana happens
	nRF24_FlushRX(); // Flush the RX FIFO buffer
	nRF24_ClearIRQFlags();

	return result;
}

// Clear pending IRQ flags
void nRF24_ClearIRQFlags(void) {
	uint8_t status;

    status = nRF24_ReadReg(nRF24_REG_STATUS);
	nRF24_WriteReg(nRF24_CMD_WREG | nRF24_REG_STATUS,status | 0x70);
}

// Put nRF24 in Power Down mode
void nRF24_PowerDown(void) {
    uint8_t conf;

    nRF24_CE_L(); // CE pin to low
    conf  = nRF24_ReadReg(nRF24_REG_CONFIG);
    conf &= ~(1<<1); // Clear PWR_UP bit
    nRF24_WriteReg(nRF24_CMD_WREG | nRF24_REG_CONFIG,conf); // Go Power down mode
}

// Wake nRF24 from Power Down mode
// note: with external crystal it wake to Standby-I mode within 1.5ms
void nRF24_Wake(void) {
    uint8_t conf;

    conf = nRF24_ReadReg(nRF24_REG_CONFIG) | (1<<1); // Set PWR_UP bit
    nRF24_WriteReg(nRF24_CMD_WREG | nRF24_REG_CONFIG,conf); // Wake-up
}

// Configure RF output power in TX mode
// input:
//   TXPower - RF output power (-18dBm, -12dBm, -6dBm, 0dBm)
void nRF24_SetTXPower(nRF24_TXPower_TypeDef TXPower) {
    uint8_t rf_setup;

    rf_setup  = nRF24_ReadReg(nRF24_REG_RF_SETUP);
    rf_setup &= 0xf9; // Clear RF_PWR bits
    nRF24_WriteReg(nRF24_CMD_WREG | nRF24_REG_RF_SETUP,rf_setup | (uint8_t)TXPower);
}
