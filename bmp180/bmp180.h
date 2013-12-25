/* I2C to use for communications with BMP180 */
#define _I2C_PORT 2

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


/* BMP180 defines */
#define BMP180_ADDR                     0xEE // BMP180 address
/* BMP180 registers */
#define BMP180_PROM_START_ADDR          0xAA // E2PROM calibration data start register
#define BMP180_PROM_DATA_LEN            22   // E2PROM length
#define BMP180_CHIP_ID_REG              0xD0 // Chip ID
#define BMP180_VERSION_REG              0xD1 // Version
#define BMP180_CTRL_MEAS_REG            0xF4 // Measurements control (OSS[7.6], SCO[5], CTL[4.0]
#define BMP180_ADC_OUT_MSB_REG          0xF6 // ADC out MSB  [7:0]
#define BMP180_ADC_OUT_LSB_REG          0xF7 // ADC out LSB  [7:0]
#define BMP180_ADC_OUT_XLSB_REG         0xF8 // ADC out XLSB [7:3]
#define BMP180_SOFT_RESET_REG           0xE0 // Soft reset control
/* BMP180 control values */
#define BMP180_T_MEASURE                0x2E // temperature measurement
#define BMP180_P0_MEASURE               0x34 // pressure measurement (OSS=0, 4.5ms)
#define BMP180_P1_MEASURE               0x74 // pressure measurement (OSS=1, 7.5ms)
#define BMP180_P2_MEASURE               0xB4 // pressure measurement (OSS=2, 13.5ms)
#define BMP180_P3_MEASURE               0xF4 // pressure measurement (OSS=3, 25.5ms)
/* BMP180 Pressure calculation constants */
#define BMP180_PARAM_MG                 3038
#define BMP180_PARAM_MH                -7357
#define BMP180_PARAM_MI                 3791


/* Calibration parameters structure */
typedef struct {
	int16_t AC1;
	int16_t AC2;
	int16_t AC3;
	uint16_t AC4;
	uint16_t AC5;
	uint16_t AC6;
	int16_t B1;
	int16_t B2;
	int16_t MB;
	int16_t MC;
	int16_t MD;
	int32_t B5;
} BMP180_Calibration_TypeDef;


/* Calibration parameters from E2PROM of BMP180 */
BMP180_Calibration_TypeDef BMP180_Calibration;


uint8_t BMP180_Init(uint32_t SPI_Clock_Speed);
void BMP180_Reset();

uint8_t BMP180_WriteReg(uint8_t reg, uint8_t value);
uint8_t BMP180_ReadReg(uint8_t reg);

void BMP180_ReadCalibration(void);

uint16_t BMP180_Read_UT(void);
uint32_t BMP180_Read_PT(uint8_t oss);
int16_t BMP180_Calc_RT(uint16_t UT);
int32_t BMP180_Calc_RP(uint32_t UP, uint8_t oss);

int32_t BMP180_kpa_to_mmhg(int32_t Pa);

