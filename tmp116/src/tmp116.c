#include "tmp116.h"


// TMP116 register definitions
#define REG_TEMPERATURE               ((uint8_t)0x00) // Stores the output of the most recent conversion
#define REG_CONFIGURATION             ((uint8_t)0x01) // Chip configuration
#define REG_LIM_HIGH                  ((uint8_t)0x02) // High limit for comparison with temperature result
#define REG_LIM_LOW                   ((uint8_t)0x03) // Low limit for comparison with temperature result
#define REG_EEPROM_CTL                ((uint8_t)0x04) // Chip EEPROM functions control and status
#define REG_EEPROM_DATA1              ((uint8_t)0x05) // Used as a scratch pad by the customer to store general purpose data
#define REG_EEPROM_DATA2              ((uint8_t)0x06) // Used as a scratch pad by the customer to store general purpose data
#define REG_EEPROM_DATA3              ((uint8_t)0x07) // Used as a scratch pad by the customer to store general purpose data
#define REG_EEPROM_DATA4              ((uint8_t)0x08) // Used as a scratch pad by the customer to store general purpose data
#define REG_DEVICE_ID                 ((uint8_t)0x0F) // Indicates the device ID

// Bit mask of CONFIGURATION register definitions
#define MODE_POS                      10 // MOD[11:10]
#define MODE_MSK                      ((uint16_t)0x03)

#define AVG_POS                       5 // AVG[6:5]
#define AVG_MSK                       ((uint16_t)0x03)

#define CONV_POS                      7 // CONV[9:7]
#define CONV_MSK                      ((uint16_t)0x07)

#define ALERT_HIGH_BIT                15 // Set when the conversion result is higher than a high limit
#define ALERT_LOW_BIT                 14 // Set when the conversion result is lower than a low limit
#define DATA_READY_BIT                13 // Indicates conversion completed and temperature register can be read
#define TA_BIT                        4  // Therm/Alert mode select
#define POL_BIT                       3  // Alert pin polarity
#define DRA_BIT                       2  // Alert pin select
#define RESET_BIT                     1  // Software reset trigger

#define RO_MSK                        ((uint16_t)0xF003) // Read-only bits of CONFIGURATION register

// Bit mask of EEPROM register definitions
#define EEPROM_UNLOCK_BIT             15 // EEPROM is unlocked for programming when this bit is set
#define EEPROM_BUSY_BIT               14 // Indicates that the EEPROM is busy during programming or power-up


// Macros for converting raw sensor readings to temperature and vice versa
// NOTE: temperatures are represented in centidegree of Celsius,
//       i.e. the value '3660' means 36.60C
// NOTE: the 1 bit of temperature readings is 0.0078125C, thus
//       the magic number 128 is derived from 1 / 0.0078125
#define RAW2TEMP(__RAW__) (int16_t)(((int16_t)__RAW__ * 100) / 128)
#define TEMP2RAW(__TMP__) (int16_t)(((int16_t)__TMP__ * 128) / 100)

// Macros for manipulating bits in a register value
#define __SET_BITS(__VALUE__, __NAME__, __DATA__) \
	(uint16_t)((__VALUE__ & ~(__NAME__##_MSK << __NAME__##_POS)) | \
		(__DATA__ << __NAME__##_POS))
#define __SET_BIT(__VALUE__, __NAME__) \
	(uint16_t)(__VALUE__ | (1U << __NAME__##_BIT))
#define __CLR_BIT(__VALUE__, __NAME__) \
	(uint16_t)(__VALUE__ & ~(1U << __NAME__##_BIT))
#define __GET_BIT(__VALUE__, __NAME__) \
	((__VALUE__ & (1U << __NAME__##_BIT)) == (1U << __NAME__##_BIT))


// Cached configuration register
static uint16_t config;


// Writes a new value to the TMP116 register
// input:
//   addr - register address
//   value - new register value
static void __reg_write(uint8_t addr, uint16_t value) {
#if defined(__GNUC__)
	value = __builtin_bswap16(value);
	I2C_Transmit(TMP116_I2C_PORT, &addr, 1U, TMP116_ADDR, I2C_TX_NOSTOP | I2C_TX_CONT);
	I2C_Transmit(TMP116_I2C_PORT, (uint8_t *)&value, sizeof(value), TMP116_ADDR, I2C_TX_NOSTART | I2C_TX_STOP);
#else
	// In case the __builtin_bswap16() intrinsic is not available
	uint8_t data[3];

	data[0] = addr;
	data[1] = (uint8_t)(value >> 8);
	data[2] = (uint8_t)(value & 0xFF);
	I2C_Transmit(TMP116_I2C_PORT, data, sizeof(data), TMP116_ADDR, I2C_TX_STOP);
#endif
}

// Reads a value of the TMP116 register
// input:
//   addr - register address
// return:
//   register value
static uint16_t __reg_read(uint8_t addr) {
	union {
		uint16_t u16;
		uint8_t u8[2];
	} data = {0};

	I2C_Transmit(TMP116_I2C_PORT, &addr, sizeof(addr), TMP116_ADDR, I2C_TX_NOSTOP);
	I2C_Receive(TMP116_I2C_PORT, (uint8_t *)&data, sizeof(data), TMP116_ADDR);

#if defined(__GNUC__)
	return __builtin_bswap16(data.u16);
#else
	// In case the __builtin_bswap16() intrinsic is not available
	return (uint16_t)((data.u8[0] << 8) | data.u8[1]);
#endif
}

#if (TMP116_RESET)

// Issues I2C general-call command
static void __general_call(uint8_t cmd) {
#if 1
	I2C_GeneralCall(TMP116_I2C_PORT, cmd);
#else
	// 0x00 is the I2C general-call address
	I2C_Transmit(TMP116_I2C_PORT, &cmd, sizeof(cmd), 0x00, I2C_TX_STOP);
#endif
}

// Calls software reset of the chip
void TMP116_SoftReset(void) {
	__general_call(0x06); // 0x06 is the I2C software reset command (0b00000110)
}

#endif // TMP116_RESET

// Initializes internal configuration
void TMP116_Init(void) {
	config = (uint16_t)(TMP116_GetStatus() & ~RO_MSK);
}

// Configures temperature conversion mode of the sensor
// input:
//   mode - conversion mode, one of TMP116_MOD_xx values
void TMP116_SetMode(TMP116_MODE_t mode) {
	__reg_write(REG_CONFIGURATION, config = __SET_BITS(config, MODE, mode));
}

// Configures averaging mode of the sensor
// input:
//   avg - averaging mode, one of TMP116_AVG_xx values
void TMP116_SetAvg(TMP116_AVG_t avg) {
	__reg_write(REG_CONFIGURATION, config = __SET_BITS(config, AVG, avg));
}

// Configures conversion cycle (for CC mode only)
// input:
//   conv - conversion cycle, one of TMP116_CONV_xx values
void TMP116_SetConv(TMP116_CONV_t conv) {
	__reg_write(REG_CONFIGURATION, config = __SET_BITS(config, CONV, conv));
}

// Configures temperature conversion
// input:
// NOTE: it is a composite product of the functions SetMode(), SetAvg() and SetConv()
//   mode - conversion mode, one of TMP116_MOD_xx values
//   avg - averaging mode, one of TMP116_AVG_xx values
//   conv - conversion cycle, one of TMP116_CONV_xx values
void TMP116_SetConfig(TMP116_MODE_t mode, TMP116_AVG_t avg, TMP116_CONV_t conv) {
	config = __SET_BITS(config, MODE, mode);
	config = __SET_BITS(config, AVG, avg);
	config = __SET_BITS(config, CONV, conv);
	__reg_write(REG_CONFIGURATION, config);
}

// Configures Therm/Alert mode
// input:
//   state - thermal/alert mode, one of TMP116_BIT_xx values:
//     TMP116_BIT_SET - therm mode
//     TMP116_BIT_RESET - alert mode
void TMP116_SetModeTA(TMP116_BIT_t state) {
	config &= (uint16_t)(~(1U << TA_BIT));
	config |= (uint16_t)(state << TA_BIT);
	__reg_write(REG_CONFIGURATION, config);
}

// Configures ALERT pin output
// input:
//   state - ALERT pin output, one of TMP116_BIT_xx values:
//     TMP116_BIT_SET - pin reflects the status of the data ready flag
//     TMP116_BIT_RESET - pin reflects the status of the alert flags
void TMP116_SetAlertPin(TMP116_BIT_t state) {
	config &= (uint16_t)(~(1U << POL_BIT));
	config |= (uint16_t)(state << POL_BIT);
	__reg_write(REG_CONFIGURATION, config);
}

// Configures ALERT pin polarity
// input:
//   state - ALERT pin polarity, one of TMP116_BIT_xx values:
//     TMP116_BIT_SET - active high
//     TMP116_BIT_RESET - active low
void TMP116_SetAlertPolarity(TMP116_BIT_t state) {
	config &= (uint16_t)(~(1U << DRA_BIT));
	config |= (uint16_t)(state << DRA_BIT);
	__reg_write(REG_CONFIGURATION, config);
}

// Configures new high limit value
// input:
//   limit - new value in centidegrees of Celsius
void TMP116_SetLimitHigh(int16_t limit) {
	__reg_write(REG_LIM_HIGH, (uint16_t)(TEMP2RAW(limit)));
}

// Configures new low limit value
// input:
//   limit - new value in centidegrees of Celsius
void TMP116_SetLimitLow(int16_t limit) {
	__reg_write(REG_LIM_LOW, (uint16_t)(TEMP2RAW(limit)));
}

// Reads value of CONFIGURATION register of the sensor into internal variable
// return: value of CONFIGURATION register
uint16_t TMP116_GetStatus(void) {
	return config = __reg_read(REG_CONFIGURATION);
}

// Returns device ID
// return: ID value
uint16_t TMP116_GetID(void) {
	return __reg_read(REG_DEVICE_ID);
}

// Retrieves temperature readings from the sensor
// return: temperature in centidegrees of Celsius
int16_t TMP116_GetTemp(void) {
	return RAW2TEMP(__reg_read(REG_TEMPERATURE));
}

// Retrieves high limit temperature from the sensor
// return: temperature in centidegrees of Celsius
int16_t TMP116_GetLimitHigh(void) {
	return RAW2TEMP(__reg_read(REG_LIM_HIGH));
}

// Retrieves low limit temperature from the sensor
// return: temperature in centidegrees of Celsius
int16_t TMP116_GetLimitLow(void) {
	return RAW2TEMP(__reg_read(REG_LIM_LOW));
}

// Checks if the conversion is complete and temperature can be read
// return: nonzero value if data is ready
uint16_t TMP116_IsDataReady(void) {
	return __GET_BIT(config, DATA_READY);
}

// Checks if High alert flag is set
// return: nonzero value when flag is set
uint16_t TMP116_IsAlertHigh(void) {
	return __GET_BIT(config, ALERT_HIGH);
}

// Checks if Low alert flag is set
// return: nonzero value when flag is set
uint16_t TMP116_IsAlertLow(void) {
	return __GET_BIT(config, ALERT_LOW);
}

// Checks if EEPROM busy flag is set (EEPROM busy during programming or power-up)
// return: nonzero value when flag is set
uint16_t TMP116_IsEEPROMBusy(void) {
	return __GET_BIT(__reg_read(REG_EEPROM_CTL), EEPROM_BUSY);
}

// Unlocks EEPROM for programming
void TMP116_EEPROMUnlock(void) {
	__reg_write(REG_EEPROM_CTL, __SET_BIT(__reg_read(REG_EEPROM_CTL), EEPROM_UNLOCK));
}

// Locks EEPROM for programming
void TMP116_EEPROMLock(void) {
	__reg_write(REG_EEPROM_CTL, __CLR_BIT(__reg_read(REG_EEPROM_CTL), EEPROM_UNLOCK));
}

// Reads a value of the EEPROM register
// input:
//   reg - number of the EEPROM register, one of TMP116_EEPROM_xx values
uint16_t TMP116_EEPROMRead(TMP116_EEPROM_t reg) {
	return __reg_read(REG_EEPROM_DATA1 + reg - 1U);
}

// Writes a value to the EEPROM register
// input:
//   reg - number of the EEPROM register, one of TMP116_EEPROM_xx values
//   value - new value of register
// NOTE: writes data to EEPROM
void TMP116_EEPROMWrite(TMP116_EEPROM_t reg, uint16_t value) {
	__reg_write(REG_EEPROM_DATA1 + reg - 1U, value);
}

// Writes the current configuration to the chip EEPROM
// NOTE: the chip should already be configured to the desired state
// NOTE: writes data to EEPROM
void TMP116_WriteConfig(void) {
	__reg_write(REG_CONFIGURATION, __reg_read(REG_CONFIGURATION) & (uint16_t)(~RO_MSK));
}

// Writes the current high limit to the chip EEPROM
// NOTE: the high limit should already be configured to desired value
// NOTE: writes data to EEPROM
void TMP116_WriteLimitHigh(void) {
	__reg_write(REG_LIM_HIGH, __reg_read(REG_LIM_HIGH));
}

// Writes the current low limit to the chip EEPROM
// NOTE: the high limit should already be configured to desired value
// NOTE: writes data to EEPROM
void TMP116_WriteLimitLow(void) {
	__reg_write(REG_LIM_LOW, __reg_read(REG_LIM_LOW));
}


#if 0

// Dumps the chip configuration in human-readable format

#include "usart.h"

#define printf(...) USART_printf(USART1, __VA_ARGS__)

#define __SR__(__STATEMENT__) ((__STATEMENT__) ? "SET" : "RESET")
#define __LH__(__STATEMENT__) ((__STATEMENT__) ? "HIGH" : "LOW")

static const uint8_t __averaging__[] = { 1, 8, 32, 64 };

static void __print_temp__(uint16_t raw) {
	int16_t temp;
	uint8_t sign;

	temp = RAW2TEMP(raw);
	if (temp < 0) {
		temp = 0 - temp;
		sign = '-';
	} else {
		sign = '+';
	}
	printf("%c%u.%02uC", sign, temp / 100, temp % 100);
}

void TMP116_DUMP(void) {
	uint16_t reg, num;

	reg = __reg_read(REG_DEVICE_ID);
	printf("DEVICE ID: 0x%04X [%016b]\r\n", reg, reg);

	reg = __reg_read(REG_CONFIGURATION);
	printf("CONFIGURATION: 0x%04X [%016b]\r\n", reg, reg);
	printf(" HI alert: %s\r\n", __SR__(reg & (1U << 15)));
	printf(" LO alert: %s\r\n", __SR__(reg & (1U << 14)));
	printf(" data ready: %s\r\n", __SR__(reg & (1U << 13)));
	printf(" EEPROM busy: %s\r\n", __SR__(reg & (1U << 12)));
	printf(" working mode: ");
	switch ((reg >> 10) & 3U) {
		case 0x00:
		case 0x02: printf("CC {continuous}\r\n"); break;
		case 0x01: printf("SD {shutdown}\r\n"); break;
		case 0x03: printf("OS {one-shot}\r\n"); break;
		default: printf("???\r\n"); break;
	}
	printf(" conversion: %u\r\n", ((reg >> 7) & 7U));
	printf(" averaging: %u\r\n", __averaging__[((reg >> 5) & 3U)]);
	printf(" T/nA: %s mode\r\n", (reg & (1U << 4)) ? "therm" : "alert");
	printf(" ALERT polarity: %s\r\n", __LH__(reg & (1U << 3)));
	printf(" DR/Alert: %s\r\n", (reg & (1U << 2)) ? "data ready flag" : "alert flags");

	reg = __reg_read(REG_LIM_HIGH);
	printf("LIM_HIGH: 0x%04X [%016b] = ", reg, reg);
	__print_temp__(reg);
	printf("\r\n");

	reg = __reg_read(REG_LIM_LOW);
	printf("LIM_LOW:  0x%04X [%016b] = ", reg, reg);
	__print_temp__(reg);
	printf("\r\n");

	reg = __reg_read(REG_EEPROM_CTL);
	printf("EEPROM: 0x%04X [%016b] %s, %slocked [",
			reg,
			reg,
			((reg >> 14) & 0x01) ? "busy" : "ready",
			((reg >> 15) & 0x01) ? "un" : ""
		);
	for (num = TMP116_EEPROM_1; num <= TMP116_EEPROM_4; num++) {
		printf("0x%04X%s", TMP116_EEPROMRead(num), (num == TMP116_EEPROM_4) ? "]\r\n" : ":");
	}

	reg = __reg_read(REG_TEMPERATURE);
	printf("TEMPERATURE: 0x%04X [%016b] = ", reg, reg);
	__print_temp__(reg);
	printf("\r\n");
}

#endif
