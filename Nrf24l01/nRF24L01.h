/* Which SPI use */
#define _SPI_PORT 2

#if _SPI_PORT == 1
	#define SPI_PORT      SPI1
	#define SPI_SCK_PIN   GPIO_Pin_5     // PA5
	#define SPI_MISO_PIN  GPIO_Pin_6     // PA6
	#define SPI_MOSI_PIN  GPIO_Pin_7     // PA7
	#define SPI_CS_PIN    GPIO_Pin_4     // PA4
	#define SPI_GPIO_PORT GPIOA
#elif _SPI_PORT == 2
	#define SPI_PORT      SPI2
	#define SPI_SCK_PIN   GPIO_Pin_13    // PB13
	#define SPI_MISO_PIN  GPIO_Pin_14    // PB14
	#define SPI_MOSI_PIN  GPIO_Pin_15    // PB15
	#define SPI_CS_PIN    GPIO_Pin_12    // PB12
	#define SPI_GPIO_PORT GPIOB
#elif _SPI_PORT == 3
	#define SPI_PORT      SPI3
	#define SPI_SCK_PIN   GPIO_Pin_3     // PB3  (JTDO)
	#define SPI_MISO_PIN  GPIO_Pin_4     // PB4  (NJTRST)
	#define SPI_MOSI_PIN  GPIO_Pin_5     // PB5
	#define SPI_CS_PIN    GPIO_Pin_6     // PB6
	#define SPI_GPIO_PORT GPIOB
#endif

// nRF24L01 CE (Chip Enable) pin
#define nRF24_CE_PORT     GPIOB
#define nRF24_CE_PIN      GPIO_Pin_11    // PB11

// nRF24L01 IRQ pin
#define nRF24_IRQ_PORT    GPIOB
#define nRF24_IRQ_PIN     GPIO_Pin_10    // PB10

// Chip Enable Activates RX or TX mode
#define CE_L() GPIO_ResetBits(nRF24_CE_PORT,nRF24_CE_PIN)
#define CE_H() GPIO_SetBits(nRF24_CE_PORT,nRF24_CE_PIN)

// SPI Chip Select
#define CSN_L() GPIO_ResetBits(SPI_GPIO_PORT,SPI_CS_PIN)
#define CSN_H() GPIO_SetBits(SPI_GPIO_PORT,SPI_CS_PIN)


/* nRF24L0 commands */
#define nRF24_CMD_RREG             0x00  // R_REGISTER -> Read command and status registers
#define nRF24_CMD_WREG             0x20  // W_REGISTER -> Write command and status registers
#define nRF24_CMD_R_RX_PAYLOAD     0x61  // R_RX_PAYLOAD -> Read RX payload
#define nRF24_CMD_W_TX_PAYLOAD     0xA0  // W_TX_PAYLOAD -> Write TX payload
#define nRF24_CMD_FLUSH_TX         0xE1  // FLUSH_TX -> Flush TX FIFO
#define nRF24_CMD_FLUSH_RX         0xE2  // FLUSH_RX -> Flush RX FIFO
#define nRF24_CMD_REUSE_TX_PL      0xE3  // REUSE_TX_PL -> Reuse last transmitted payload
#define nRF24_CMD_NOP              0xFF  // No operation (to read status register)

/* nRF24L0 registers */
#define nRF24_REG_CONFIG           0x00  // Configuration register
#define nRF24_REG_EN_AA            0x01  // Enable "Auto acknowledgment"
#define nRF24_REG_EN_RXADDR        0x02  // Enable RX addresses
#define nRF24_REG_SETUP_AW         0x03  // Setup of address widths
#define nRF24_REG_SETUP_RETR       0x04  // Setup of automatic retranslation
#define nRF24_REG_RF_CH            0x05  // RF channel
#define nRF24_REG_RF_SETUP         0x06  // RF setup register
#define nRF24_REG_STATUS           0x07  // Status register
#define nRF24_REG_OBSERVE_TX       0x08  // Transmit observe register
#define nRF24_REG_CD               0x09  // Carrier detect
#define nRF24_REG_RX_ADDR_P0       0x0A  // Receive address data pipe 0
#define nRF24_REG_RX_ADDR_P1       0x0B  // Receive address data pipe 1
#define nRF24_REG_RX_ADDR_P2       0x0C  // Receive address data pipe 2
#define nRF24_REG_RX_ADDR_P3       0x0D  // Receive address data pipe 3
#define nRF24_REG_RX_ADDR_P4       0x0E  // Receive address data pipe 4
#define nRF24_REG_RX_ADDR_P5       0x0F  // Receive address data pipe 5
#define nRF24_REG_TX_ADDR          0x10  // Transmit address
#define nRF24_REG_RX_PW_P0         0x11  // Number of bytes in RX payload id data pipe 0
#define nRF24_REG_RX_PW_P1         0x12  // Number of bytes in RX payload id data pipe 1
#define nRF24_REG_RX_PW_P2         0x13  // Number of bytes in RX payload id data pipe 2
#define nRF24_REG_RX_PW_P3         0x14  // Number of bytes in RX payload id data pipe 3
#define nRF24_REG_RX_PW_P4         0x15  // Number of bytes in RX payload id data pipe 4
#define nRF24_REG_RX_PW_P5         0x16  // Number of bytes in RX payload id data pipe 5
#define nRF24_REG_FIFO_STATUS      0x17  // FIFO status register
#define nRF24_REG_DYNPD            0x1C  // Enable dynamic payload length
#define nRF24_REG_FEATURE          0x1D  // Feature register

/* nRF24L0 bits */
#define nRF24_MASK_RX_DR           0x40  // Mask interrupt caused by RX_DR
#define nRF24_MASK_TX_DS           0x20  // Mask interrupt caused by TX_DS
#define nRF24_MASK_MAX_RT          0x10  // Mask interrupt caused by MAX_RT
#define nRF24_FIFO_RX_EMPTY        0x01  // RX FIFO empty flag
#define nRF24_FIFO_RX_FULL         0x02  // RX FIFO full flag

/* Some constants */
#define nRF24_RX_ADDR_WIDTH        5    // nRF24 RX address width
#define nRF24_TX_ADDR_WIDTH        5    // nRF24 TX address width


/* Variables */
extern uint8_t nRF24_RX_addr[nRF24_RX_ADDR_WIDTH];
extern uint8_t nRF24_TX_addr[nRF24_TX_ADDR_WIDTH];


/* Function prototypes */
void nRF24_init();

uint8_t nRF24_RWReg(uint8_t reg, uint8_t value);
uint8_t nRF24_ReadReg(uint8_t reg);
uint8_t nRF24_ReadBuf(uint8_t reg, uint8_t *pBuf, uint8_t count);
uint8_t nRF24_WriteBuf(uint8_t reg, uint8_t *pBuf, uint8_t count);

uint8_t nRF24_Check(void);

void nRF24_RXMode(uint8_t RX_PAYLOAD);
void nRF24_TXMode(void);
uint8_t nRF24_DataReady(void);
uint8_t nRF24_TXPacket(uint8_t * pBuf, uint8_t TX_PAYLOAD);
uint8_t nRF24_RXPacket(uint8_t* pBuf, uint8_t RX_PAYLOAD);
void nRF24_ClearIRQFlags(void);
