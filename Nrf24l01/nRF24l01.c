#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_spi.h>

#include <nRF24L01.h>
#include <delay.h>


uint8_t nRF24_RX_addr[nRF24_RX_ADDR_WIDTH] = {'W','o','l','k','T'};
uint8_t nRF24_TX_addr[nRF24_TX_ADDR_WIDTH] = {'W','o','l','k','T'};


// SPI initialization with given prescaler
void nRF24_SPI_Init(uint16_t prescaler) {
	SPI_InitTypeDef SPI;
	SPI.SPI_Mode = SPI_Mode_Master;
	SPI.SPI_BaudRatePrescaler = prescaler;
	SPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI.SPI_CPOL = SPI_CPOL_Low;
	SPI.SPI_CPHA = SPI_CPHA_1Edge;
	SPI.SPI_CRCPolynomial = 7;
	SPI.SPI_DataSize = SPI_DataSize_8b;
	SPI.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI.SPI_NSS = SPI_NSS_Soft;
	SPI_Init(SPI_PORT,&SPI);

	// NSS must be set to '1' due to NSS_Soft settings (otherwise it will be Multimaster mode).
	SPI_NSSInternalSoftwareConfig(SPI_PORT,SPI_NSSInternalSoft_Set);
}

// GPIO and SPI initialization
void nRF24_init() {
#if _SPI_PORT == 1
	// SPI1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1 | RCC_APB2Periph_GPIOA,ENABLE);
#elif _SPI_PORT == 2
	// SPI2
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);
#elif _SPI_PORT == 3
	// SPI3
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); // Disable JTAG for use PB3
#endif

	GPIO_InitTypeDef PORT;
	// Configure SPI pins
	PORT.GPIO_Speed = GPIO_Speed_50MHz;
	PORT.GPIO_Pin = SPI_SCK_PIN | SPI_MISO_PIN | SPI_MOSI_PIN;
	PORT.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(SPI_GPIO_PORT,&PORT);
	// Configure CS pin as output with Push-Pull
	PORT.GPIO_Pin = SPI_CS_PIN;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(SPI_GPIO_PORT,&PORT);
	// Configure CE pin as output with Push-Pull
	PORT.GPIO_Pin = nRF24_CE_PIN;
	PORT.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(nRF24_CE_PORT,&PORT);
	// Configure IRQ pin as input with Pull-Up
	PORT.GPIO_Pin = nRF24_IRQ_PIN;
	PORT.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(nRF24_IRQ_PORT,&PORT);

	nRF24_SPI_Init(SPI_BaudRatePrescaler_2); // Which SPI speed do we need?
	SPI_Cmd(SPI_PORT,ENABLE);

	CSN_H();
	CE_L();
}

// Send/Receive data to nRF24L01 via SPI
// input:
//   data - byte to send
// output: received byte from nRF24L01
uint8_t nRF24_ReadWrite(uint8_t data) {
	while (SPI_I2S_GetFlagStatus(SPI_PORT,SPI_I2S_FLAG_TXE) == RESET); // Wait while DR register is not empty
	SPI_I2S_SendData(SPI_PORT,data); // Send byte to SPI
	while (SPI_I2S_GetFlagStatus(SPI_PORT,SPI_I2S_FLAG_RXNE) == RESET); // Wait to receive byte
	return SPI_I2S_ReceiveData(SPI_PORT); // Read byte from SPI bus
}

// Write new value to register
// input:
//   reg - register number
//   value - new value
// output: nRF24L01 status
uint8_t nRF24_RWReg(uint8_t reg, uint8_t value) {
	uint8_t status;

	CSN_L();
	status = nRF24_ReadWrite(reg); // Select register
	nRF24_ReadWrite(value); // Write value to register
	CSN_H();

	return status;
}

// Read nRF24L01 register
// input:
//   reg - register number
// output: register value
uint8_t nRF24_ReadReg(uint8_t reg) {
	uint8_t value;

	CSN_L();
	nRF24_ReadWrite(reg);
	value = nRF24_ReadWrite(0x00);
	CSN_H();

	return value;
}

// Get data from nRF24L01 into buffer
// input:
//   reg - register number
//   pBuf - pointer to buffer
//   count - bytes count
// output: nRF24L01 status
uint8_t nRF24_ReadBuf(uint8_t reg, uint8_t *pBuf, uint8_t count) {
	uint8_t status,i;

	CSN_L();
	status = nRF24_ReadWrite(reg);
	for (i = 0; i < count; i++) pBuf[i] = nRF24_ReadWrite(0);
	CSN_L();

	return status;
}

// Send buffer to nRF24L01
// input:
//   reg - register number
//   pBuf - pointer to buffer
//   count - bytes count
// output: nRF24L01 status
uint8_t nRF24_WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t count) {
	uint8_t status,i;

	CSN_L();
	status = nRF24_ReadWrite(reg);
	for (i = 0; i < count; i++) nRF24_ReadWrite(*pBuf++);
	CSN_H();

	return status;
}

// Check if nRF24L01 present (send byte sequence, read it back and compare)
// return:
//   0 - looks like an nRF24L01 is online
//   1 - received sequence differs from original
uint8_t nRF24_Check(void) {
	uint8_t txbuf[5] = { 0xA9,0xA9,0xA9,0xA9,0xA9 };
	uint8_t rxbuf[5];
	uint8_t i;

	nRF24_WriteBuf(nRF24_CMD_WREG | nRF24_REG_TX_ADDR,txbuf,5); // Write fake TX address
    nRF24_ReadBuf(nRF24_REG_TX_ADDR,rxbuf,5); // Try to read TX_ADDR register
    for (i = 0; i < 5; i++) if (rxbuf[i] != txbuf[i]) return 1;

    return 0;
}

// Put nRF24L01 in RX mode
void nRF24_RXMode(uint8_t RX_PAYLOAD) {
	CE_L();
	nRF24_WriteBuf(nRF24_CMD_WREG | nRF24_REG_RX_ADDR_P0,nRF24_RX_addr,nRF24_RX_ADDR_WIDTH); // Set static RX address
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_EN_AA,0x01); // Enable ShockBurst for data pipe 0
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_EN_RXADDR,0x01); // Enable data pipe 0
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_RF_CH,0x6E); // Set frequency channel 110 (2.510MHz)
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_RX_PW_P0,RX_PAYLOAD); // Set RX payload length (10 bytes)
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_RF_SETUP,0x06); // Setup: 1Mbps, 0dBm, LNA off
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_CONFIG,0x0F); // Config: CRC on (2 bytes), Power UP, RX/TX ctl = PRX
	CE_H();
	// Delay_us(10); // Hold CE high at least 10us
}

// Put nRF24L01 in TX mode
void nRF24_TXMode(void) {
	CE_L();
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_CONFIG,0x02); // Config: Power UP
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_EN_AA,0x01); // Enable ShockBurst for data pipe 0
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_SETUP_RETR,0x1A); // Auto retransmit: wait 500us, 10 retries
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_RF_CH,0x6E); // Set frequency channel 110 (2.510MHz)
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_RF_SETUP,0x06); // Setup: 1Mbps, 0dBm, LNA off
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_CONFIG,0x0E); // Config: CRC on (2 bytes), Power UP, RX/TX ctl = PTX
}

// Check if data is available for reading
// return:
//   0 -> no data
//   1 -> RX_DR is set or some bytes present in FIFO
uint8_t nRF24_DataReady(void) {
    uint8_t status;

    status = nRF24_ReadReg(nRF24_REG_STATUS);
    if (status & nRF24_MASK_RX_DR) return 1;

    // Checking RX_DR isn't good enough, there's can be some data in FIFO
    status = nRF24_ReadReg(nRF24_REG_FIFO_STATUS);

    return (status & nRF24_FIFO_RX_EMPTY) ? 0 : 1;
}

uint8_t nRF24_RXPacket(uint8_t* pBuf, uint8_t RX_PAYLOAD) {
	uint8_t status;

    status = nRF24_ReadReg(nRF24_REG_STATUS); // Read status register
    if (status & nRF24_MASK_RX_DR) {
    	if ((status & 0x0E) == 0) {
    		// pipe 0
    		nRF24_ReadBuf(nRF24_CMD_R_RX_PAYLOAD,pBuf,RX_PAYLOAD); // read received payload from RX FIFO buffer
    	}
		nRF24_ReadWrite(nRF24_CMD_FLUSH_RX); // Flush RX FIFO buffer
		nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_STATUS,status | 0x70); // Clear RX_DR, TX_DS, MAX_RT flags
	    //return nRF24_MASK_RX_DR;
	    return status;
    }

    // Some banana happens
	nRF24_ReadWrite(nRF24_CMD_FLUSH_RX); // Flush RX FIFO buffer
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_STATUS,status | 0x70); // Clear RX_DR, TX_DS, MAX_RT flags
    return status;
}

// Send data packet
// input:
//   pBuf - buffer with data to send
// return:
//   nRF24_MASK_MAX_RT - if transmit failed with maximum auto retransmit count
//   nRF24_MAX_TX_DS - if transmit succeed
//   contents of STATUS register otherwise
uint8_t nRF24_TXPacket(uint8_t * pBuf, uint8_t TX_PAYLOAD) {
    uint8_t status;

    CE_L();
    nRF24_WriteBuf(nRF24_CMD_WREG | nRF24_REG_TX_ADDR,nRF24_TX_addr,nRF24_TX_ADDR_WIDTH); // Set static TX address
    nRF24_WriteBuf(nRF24_CMD_WREG | nRF24_REG_RX_ADDR_P0,nRF24_RX_addr,nRF24_RX_ADDR_WIDTH); // Set static RX address for auto ack
    nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_EN_AA,0x01); // Enable auto acknowledgement for data pipe 0
    nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_SETUP_RETR,0x1A); // Automatic retransmission: wait 500us, 10 retries
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_RF_CH,0x6E); // Set frequency channel 110 (2.510MHz)
    nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_RF_SETUP,0x07); // Setup: 1Mbps, 0dBm, LNA on
    nRF24_WriteBuf(nRF24_CMD_W_TX_PAYLOAD,pBuf,TX_PAYLOAD); // Write specified buffer to FIFO
    nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_CONFIG,0x0E); // Config: CRC on (2 bytes), Power UP, RX/TX ctl = PTX
    CE_H(); // CE pin high => Start transmit
    // Delay_us(10); // Must hold CE at least 10us
    //while(PB_IDR_bit.IDR2 != 0); // Wait for IRQ from nRF24L01
    CE_L();
    status = nRF24_ReadReg(nRF24_REG_STATUS); // Read status register
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_STATUS,status | 0x70); // Clear RX_DR, TX_DS, MAX_RT flags
    if (status & nRF24_MASK_MAX_RT) {
        // Auto retransmit counter exceeds the programmed maximum limit. FIFO is not removed.
        nRF24_RWReg(nRF24_CMD_FLUSH_TX,0xFF); // Flush TX FIFO buffer
        return nRF24_MASK_MAX_RT;
    };
    if (status & nRF24_MASK_TX_DS) {
        // Transmit ok
        nRF24_RWReg(nRF24_CMD_FLUSH_TX,0xFF); // Flush TX FIFO buffer
        return nRF24_MASK_TX_DS;
    }

    // Some banana happens
    return status;
}

// Clear all IRQ flags
void nRF24_ClearIRQFlags(void) {
	uint8_t status;

    status = nRF24_ReadReg(nRF24_REG_STATUS);
	nRF24_RWReg(nRF24_CMD_WREG | nRF24_REG_STATUS,status | 0x70); // Clear RX_DR, TX_DS, MAX_RT flags
}
