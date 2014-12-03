// Define to prevent recursive inclusion -------------------------------------
#ifndef __BMC050_H
#define __BMC050_H


// BMC050 HAL
#define BMC050_I2C_PORT                           I2C2 // I2C port where the BMC050 connected

// BMC050 I2C related
// All possible I2C device addresses
#define BMC050_ACC_ADDR_G                         0x18 // BMC050 accelerometer I2C address (SDO->GND)
#define BMC050_ACC_ADDR_V                         0x19 // BMC050 accelerometer I2C address (SDO->VDDIO)
#define BMC050_MAG_ADDR_GG                        0x10 // BMC050 magnetometer I2C address (CSB2->GND; SDO->GND)
#define BMC050_MAG_ADDR_GV                        0x11 // BMC050 magnetometer I2C address (CSB2->GND; SDO->VDDIO)
#define BMC050_MAG_ADDR_VG                        0x12 // BMC050 magnetometer I2C address (CSB2->VDDIO; SDO->GND)
#define BMC050_MAG_ADDR_VV                        0x13 // BMC050 magnetometer I2C address (CSB2->VDDIO; SDO->VDDIO)

// Accelerometer address
#define BMC050_ACC_ADDR                           BMC050_ACC_ADDR_G << 1

// Magnetometer address
#define BMC050_MAG_ADDR                           BMC050_MAG_ADDR_GG << 1

// BMC050 accelerometer chip ID
#define BMC050_ACC_ID                             0x03

// BMC050 magnetometer chip ID
#define BMC050_MAG_ID                             0x32

// --- BMC050 registers ---
// Accelerometer
#define BMC050_REG_ACC_WHO_AM_I                   0x00 // Chip ID
#define BMC050_REG_ACC_OUT_XL                     0x02 // Accelerometer X-axis LSB part
#define BMC050_REG_ACC_OUT_XH                     0x03 // Accelerometer X-axis MSB part
#define BMC050_REG_ACC_OUT_YL                     0x04 // Accelerometer Y-axis LSB part
#define BMC050_REG_ACC_OUT_YH                     0x05 // Accelerometer Y-axis MSB part
#define BMC050_REG_ACC_OUT_ZL                     0x06 // Accelerometer Z-axis LSB part
#define BMC050_REG_ACC_OUT_ZH                     0x07 // Accelerometer Z-axis MSB part
#define BMC050_REG_ACC_OUT_TEMP                   0x08 // Chip temperature value
#define BMC050_REG_ACC_STATUS_IRQL                0x09 // Accelerometer interrupt status LSB part
#define BMC050_REG_ACC_STATUS_IRQH                0x0a // Accelerometer interrupt status MSB part
#define BMC050_REG_ACC_TS_IRQ                     0x0b // Tap and slope interrupt status
#define BMC050_REG_ACC_FO_IRQ                     0x0c // Flat and orientation interrupt status
#define BMC050_REG_ACC_G_RANGE                    0x0f // G-range selection
#define BMC050_REG_ACC_BANDWIDTH                  0x10 // Bandwidth selection for filtered acceleration data
#define BMC050_REG_ACC_POWER_MODE                 0x11 // Configuration of the power modes
#define BMC050_REG_ACC_FILTER                     0x13 // Setting for the configuration of the acceleration data acquisition
                                                       // and the data output format
#define BMC050_REG_ACC_SOFT_RESET                 0x14 // Register to perform software reset of the sensor
#define BMC050_REG_ACC_IRQ1                       0x16 // Accelerometer interrupt settings register #1
#define BMC050_REG_ACC_IRQ2                       0x17 // Accelerometer interrupt settings register #2
#define BMC050_REG_ACC_INT_MAP1                   0x19 // Accelerometer interrupt mapping register #1
#define BMC050_REG_ACC_INT_MAP2                   0x1a // Accelerometer interrupt mapping register #2
#define BMC050_REG_ACC_INT_MAP3                   0x1b // Accelerometer interrupt mapping register #3
#define BMC050_REG_ACC_INT_CONFIG                 0x20 // Accelerometer INT# pins configuration
#define BMC050_REG_ACC_IRQ_MODE                   0x21 // Interrupt reset and mode selection
#define BMC050_REG_ACC_SLOPE_SAMPLES              0x27 // Accelerometer number of samples for slope interrupt
#define BMC050_REG_ACC_SLOPE_THRESHOLD            0x28 // Accelerometer slope interrupt threshold
#define BMC050_REG_ACC_IF_CONFIG                  0x34 // Accelerometer interface configuration

// Magnetometer
#define BMC050_REG_MAG_WHO_AM_I                   0x40


// --- BMC050 register bits ---
#define BMC050_ACC_BW_SHADOW                      0x40 // Disable shadowing procedure
#define BMC050_ACC_SOFT_RESET                     0xb6 // Software reset value
// Accelerometer power mode
#define BMC050_ACC_SUSPEND                        0x80 // Suspend
#define BMC050_ACC_LOWPOWER                       0x40 // Low power mode


// Accelerometer full scale
typedef enum {
	ACC_FS_2G  = 0x03, // 2G range  (3.91mg/LSB)
	ACC_FS_4G  = 0x05, // 4G range  (7.81mg/LSB)
	ACC_FS_8G  = 0x08, // 8G range  (15.62mg/LSB)
	ACC_FS_16G = 0x0c  // 16G range (31.25mg/LSB)
} BMC050_ACC_FS_TypeDef;

// Accelerometer sleep phase duration
typedef enum {
	ACC_SLEEP_0R5  = 0x00, // 0.5ms  (100.5uA)
	ACC_SLEEP_1    = 0x0c, // 1ms    (78.8uA)
	ACC_SLEEP_2    = 0x0e, // 2ms    (55.0uA)
	ACC_SLEEP_4    = 0x10, // 4ms    (34.5uA)
	ACC_SLEEP_6    = 0x12, // 6ms    (25.2uA)
	ACC_SLEEP_10   = 0x14, // 10ms   (16.4uA)
	ACC_SLEEP_25   = 0x16, // 25ms   (7.4uA)
	ACC_SLEEP_50   = 0x18, // 50ms   (4.0uA)
	ACC_SLEEP_100  = 0x1a, // 100ms  (2.3uA)
	ACC_SLEEP_500  = 0x1c, // 500ms  (0.9uA)
	ACC_SLEEP_1000 = 0x1e  // 1s     (0.7uA)
} BMC050_ACC_Sleep_TypeDef;

// Accelerometer bandwidth select
typedef enum {
	ACC_BW8    = 0x08, // 7.81Hz  (64ms)
	ACC_BW16   = 0x09, // 15.63Hz (32ms)
	ACC_BW31   = 0x0a, // 31.25Hz (16ms)
	ACC_BW63   = 0x0b, // 62.5Hz  (8ms)
	ACC_BW125  = 0x0c, // 125Hz   (4ms)
	ACC_BW250  = 0x0d, // 250Hz   (2ms)
	ACC_BW500  = 0x0e, // 500Hz   (1ms)
	ACC_BW1000 = 0x0f, // 1000Hz  (0.5ms)
	ACC_BW2000 = 0x80  // 2000Hz  (0.25ms) - unfiltered acceleration data
} BMC050_ACC_BW_TypeDef;

// Accelerometer interrupts enable
typedef enum {
	ACC_IE_DISABLE = 0x0000, // All interrupts disabled
	ACC_IE_FLAT    = 0x8000, // Flat interrupt
	ACC_IE_ORIENT  = 0x4000, // Orientation interrupt
	ACC_IE_STAP    = 0x2000, // Single tap interrupt
	ACC_IE_DTAP    = 0x1000, // Double tap interrupt
	ACC_IE_SLOPEZ  = 0x0400, // Slope interrupt for Z-axis
	ACC_IE_SLOPEY  = 0x0200, // Slope interrupt for Y-axis
	ACC_IE_SLOPEX  = 0x0100, // Slope interrupt for X-axis
	ACC_IE_DATA    = 0x0010, // New data interrupt
	ACC_IE_LG      = 0x0008, // Log-G interrupt
	ACC_IE_HGZ     = 0x0004, // High-G interrupt for Z-axis
	ACC_IE_HGY     = 0x0002, // High-G interrupt for Y-axis
	ACC_IE_HGX     = 0x0001  // High-G interrupt for X-axis
} BMC050_ACC_IE_TypeDef;

// Accelerometer interrupts status
typedef enum {
	ACC_IRQ_NONE   = 0x0000, // No interrupts
	ACC_IRQ_DATA   = 0x8000, // New data IRQ
	ACC_IRQ_FLAT   = 0x0080, // Flat IRQ
	ACC_IRQ_ORIENT = 0x0040, // Orientation IRQ
	ACC_IRQ_STAP   = 0x0020, // Single tap IRQ
	ACC_IRQ_DTAP   = 0x0010, // Double tap IRQ
	ACC_IRQ_SLOPE  = 0x0004, // Slope IRQ
	ACC_IRQ_HG     = 0x0002, // High-G IRQ
	ACC_IRQ_LG     = 0x0001  // Low-G IRQ
} BMC050_ACC_IRQ_TypeDef;

// Accelerometer tap and slope interrupt status bits
typedef enum {
	ACC_TS_TAPNEG   = 0x80, // Negative 1-st tap is triggered tap interrupt
	ACC_TS_TAPZ     = 0x40, // Z-axis triggered tap interrupt
	ACC_TS_TAPY     = 0x20, // Y-axis triggered tap interrupt
	ACC_TS_TAPX     = 0x10, // X-axis triggered tap interrupt
	ACC_TS_SLOPENEG = 0x08, // Negative slope is triggered slope interrupt
	ACC_TS_SLOPEZ   = 0x04, // Z-axis triggered slope interrupt
	ACC_TS_SLOPEY   = 0x02, // Y-axis triggered slope interrupt
	ACC_TS_SLOPEX   = 0x01, // X-axis triggered slope interrupt
} BMC050_ACC_TS_TypeDef;

// Accelerometer flat and orientation interrupt status bits
typedef enum {
	ACC_FO_FLAT      = 0x80, // Flat condition fulfilled
	ACC_FO_DOWNWARD  = 0x40, // Orientation of Z-axis: downward looking
	ACC_FO_PUPRIGHT  = 0x00, // Orientation of XY-plane: portrait upright
	ACC_FO_PUPDOWN   = 0x10, // Orientation of XY-plane: portrait upside-down
	AFF_FO_LANDLEFT  = 0x20, // Orientation of XY-plane: landscape left
	AFF_FO_LANDRIGHT = 0x30, // Orientation of XY-plane: landscape right
	ACC_FO_SLOPENEG  = 0x08, // Negative slope triggered interrupt
	ACC_FO_HGZ       = 0x04, // Z-axis triggered high-G interrupt
	ACC_FO_HGY       = 0x02, // Y-axis triggered high-G interrupt
	ACC_FO_HGX       = 0x01  // X-axis triggered high-G interrupt
} BMC050_ACC_FO_TypeDef;

// Accelerometer interrupt mode
typedef enum {
	ACC_IM_RESET     = 0x80, // Reset all latched interrupts
	ACC_IM_NOLATCH   = 0x00, // Non-latched interrupts
	ACC_IM_500us     = 0x09, // Temporary latch for 500us
	ACC_IM_1ms       = 0x0b, // Temporary latch for 1ms
	ACC_IM_12ms      = 0x0c, // Temporary latch for 12.5ms
	ACC_IM_25ms      = 0x0d, // Temporary latch for 25ms
	ACC_IM_50ms      = 0x0e, // Temporary latch for 50ms
	ACC_IM_250ms     = 0x01, // Temporary latch for 250ms
	ACC_IM_500ms     = 0x02, // Temporary latch for 500ms
	ACC_IM_1s        = 0x03, // Temporary latch for 1s
	ACC_IM_2s        = 0x04, // Temporary latch for 2s
	ACC_IM_4s        = 0x05, // Temporary latch for 4s
	ACC_IM_8s        = 0x06, // Temporary latch for 8s
	ACC_IM_LATCH     = 0x0f  // Latched interrupts
} BMC050_ACC_IM_TypeDef;

// Accelerometer interface configuration
typedef enum {
	ACC_IF_WDT_OFF   = 0x00, // I2C watchdog timer disabled
	ACC_IF_WDT_1ms   = 0x04, // I2C watchdog timer period 1ms
	ACC_IF_WDT_50ms  = 0x06  // I2C watchdog timer period 50ms
} BMC050_ACC_IF_TypeDef;

// Accelerometer INT# pins configuration
typedef enum {
	ACC_INT1_OD   = 0x02, // INT1 pin is open drain
	ACC_INT1_PP   = 0x00, // INT1 pin is push-pull
	ACC_INT1_LOW  = 0x00, // INT1 pin active level low
	ACC_INT1_HIGH = 0x01, // INT1 pin active level high
	ACC_INT2_OD   = 0x08, // INT2 pin is open drain
	ACC_INT2_PP   = 0x00, // INT2 pin is push-pull
	ACC_INT2_LOW  = 0x00, // INT2 pin active level low
	ACC_INT2_HIGH = 0x04, // INT2 pin active level high
} BMC050_ACC_IntConfig_TypeDef;

// Accelerometer INT# pins interrupts mapping
typedef enum {
	ACC_IM1_FLAT   = 0x800000, // Flat interrupt to INT1 pin
	ACC_IM1_ORIENT = 0x400000, // Orientation interrupt to INT1 pin
	ACC_IM1_STAP   = 0x200000, // Single tap interrupt to INT1 pin
	ACC_IM1_DTAP   = 0x100000, // Double tap interrupt to INT1 pin
	ACC_IM1_SLOPE  = 0x040000, // Slope interrupt to INT1 pin
	ACC_IM1_HIGHG  = 0x020000, // High-G interrupt to INT1 pin
	ACC_IM1_LOWG   = 0x010000, // Low-G interrupt to INT1 pin
	ACC_IM1_DATA   = 0x000100, // New data interrupt to INT1 pin
	ACC_IM2_FLAT   = 0x000080, // Flat interrupt to INT2 pin
	ACC_IM2_ORIENT = 0x000040, // Orientation interrupt to INT2 pin
	ACC_IM2_STAP   = 0x000020, // Single tap interrupt to INT2 pin
	ACC_IM2_DTAP   = 0x000010, // Double tap interrupt to INT2 pin
	ACC_IM2_SLOPE  = 0x000004, // Slope interrupt to INT2 pin
	ACC_IM2_HIGHG  = 0x000002, // High-G interrupt to INT2 pin
	ACC_IM2_LOWG   = 0x000001, // Low-G interrupt to INT2 pin
	ACC_IM2_DATA   = 0x008000, // New data interrupt to INT2 pin
} BMC050_ACC_INtMapping_TypeDef;


void BMC050_Init(void);

uint8_t BMC050_ACC_GetDeviceID(void);
int16_t BMC050_ReadTemp(void);
void BMC050_ACC_SetRange(BMC050_ACC_FS_TypeDef range);
void BMC050_ACC_SetBandwidth(BMC050_ACC_BW_TypeDef BW);
void BMC050_ACC_SoftReset(void);
void BMC050_ACC_PwrNormal(void);
void BMC050_ACC_Suspend(void);
void BMC050_ACC_LowPower(BMC050_ACC_Sleep_TypeDef sleep_duration);
int16_t BMC050_ACC_GetX(void);
int16_t BMC050_ACC_GetY(void);
int16_t BMC050_ACC_GetZ(void);
void BMC050_ACC_GetXYZ(int16_t *X, int16_t *Y, int16_t *Z);
void BMC050_ACC_SetIRQ(BMC050_ACC_IE_TypeDef irqs);
BMC050_ACC_IRQ_TypeDef BMC050_ACC_GetIRQStatus(void);
void BMC050_ACC_SetIRQMode(BMC050_ACC_IM_TypeDef mode);
void BMC050_ACC_ConfigSlopeIRQ(uint8_t nSamples, uint8_t threshold);
uint8_t BMC050_ACC_GetTSIRQ(void);
void BMC050_ACC_InterfaceConfig(BMC050_ACC_IF_TypeDef mode);
void BMC050_ACC_IntPinConfig(BMC050_ACC_IntConfig_TypeDef mode);
void BMC050_ACC_IntPinMap(BMC050_ACC_INtMapping_TypeDef map);

uint8_t BMC050_MAG_GetDeviceID(void);

#endif // __BMC050_H
