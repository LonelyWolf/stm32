/* I2C to use for communications with LM75 */
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

/* LM75 defines */
#define LM75_ADDR                     0x90 // LM75 address

/* LM75 registers */
#define LM75_REG_TEMP                 0x00 // Temperature
#define LM75_REG_CONF                 0x01 // Configuration
#define LM75_REG_THYS                 0x02 // Hysteresis
#define LM75_REG_TOS                  0x03 // Overtemperature shutdown


uint8_t LM75_Init(uint32_t SPI_Clock_Speed);

void LM75_WriteReg(uint8_t reg, uint16_t value);
uint16_t LM75_ReadReg(uint8_t reg);
uint8_t LM75_ReadConf(void);
void LM75_WriteConf(uint8_t value);

void LM75_Shutdown(FunctionalState newstate);
int16_t LM75_Temperature(void);
