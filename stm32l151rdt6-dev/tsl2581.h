// Define to prevent recursive inclusion -------------------------------------
#ifndef __TSL2581_H
#define __TSL2581_H


// TSL2581 HAL
#define TSL2581_I2C_PORT                       I2C1 // I2C port where the TSL2581 connected

// TSL2581 I2C related
// All possible I2C device addresses
#define TSL2581_ADDR_G                         0x29 // TSL2581 I2C address (ADDR_SEL->GND)
#define TSL2581_ADDR_V                         0x49 // TSL2581 I2C address (ADDR_SEL->VDDIO)
#define TSL2581_ADDR_F                         0x39 // TSL2581 I2C address (ADDR_SEL floating)

// TSL2581 current address
#define TSL2581_ADDR                           TSL2581_ADDR_G << 1

// TSL2581 registers
#define TSL2581_CONTROL                        0x00 // Control of basic functions
#define TSL2581_TIMING                         0x01 // Integration time/gain control
#define TSL2581_INTTERUPT                      0x02 // Interrupt control
#define TSL2581_TLL                            0x03 // Low byte of low interrupt threshold
#define TSL2581_TLH                            0x04 // High byte of low interrupt threshold
#define TSL2581_THL                            0x05 // Low byte of high interrupt threshold
#define TSL2581_THH                            0x06 // High byte of high interrupt threshold
#define TSL2581_ANALOG                         0x07 // Analog control register
#define TSL2581_ID                             0x12 // Part number / Rev ID
#define TSL2581_DATA0L                         0x14 // ADC channel 0 low data register
#define TSL2581_DATA0H                         0x15 // ADC channel 0 high data register
#define TSL2581_DATA1L                         0x16 // ADC channel 1 low data register
#define TSL2581_DATA1H                         0x17 // ADC channel 1 high data register
#define TSL2581_TIMERL                         0x18 // Manual integration timer low register
#define TSL2581_TIMERH                         0x19 // Manual integration timer high register
#define TSL2581_ID2                            0x1e // TSL258x ID

// TSL2581 command register masks
#define TSL2581_CMD_REG                        0x80 // Select command register
#define TSL2581_CMD_INC                        0x20 // Auto-increment protocol transaction
#define TSL2581_CMD_SPECIAL                    0x60 // Special function

// TSL2581 control register masks
#define TSL2581_CTL_ADC                        0x02 // ADC on/off
#define TSL2581_CTL_POWER                      0x01 // Power on/off


// Analog gain control values
enum {
	TSL2581_GAIN1   = 0, // x1
	TSL2581_GAIN8   = 1, // x8
	TSL2581_GAIN16  = 2, // x16
	TSL2581_GAIN111 = 3  // x111
};


// For Lux calculation (from TAOS datasheet)
// Scale factors
#define TSL2581_LUX_SCALE           16 // scale by 2^16
#define TSL2581_RATIO_SCALE          9 // scale ratio by 2^9
// Integration time scaling factors
#define TSL2581_CH_SCALE            16 // scale channel values by 2^16
#define TSL2581_NOM_INTEG_CYCLE    148 // Nominal 400 ms integration. See Timing Register
// Gain scaling factors
#define TSL2581_CH0GAIN128X        107 // 128X gain scalar for Ch0
#define TSL2581_CH1GAIN128X        115 // 128X gain scalar for Ch1
// FN Package coefficients
// For Ch1/Ch0=0.00 to 0.30: Lux=0.130*Ch0−0.240*Ch1
// For Ch1/Ch0=0.30 to 0.38: Lux=0.1649*Ch0−0.3562*Ch1
// For Ch1/Ch0=0.38 to 0.45: Lux=0.0974*Ch0−0.1786*Ch1
// For Ch1/Ch0=0.45 to 0.54: Lux=0.062*Ch0−0.10*Ch1
// For Ch1/Ch0>0.54        : Lux/Ch0=0
#define TSL2581_K1C             0x009A // 0.30 * 2^RATIO_SCALE
#define TSL2581_B1C             0x2148 // 0.130 * 2^LUX_SCALE
#define TSL2581_M1C             0x3d71 // 0.240 * 2^LUX_SCALE
#define TSL2581_K2C             0x00c3 // 0.38 * 2^RATIO_SCALE
#define TSL2581_B2C             0x2a37 // 0.1649 * 2^LUX_SCALE
#define TSL2581_M2C             0x5b30 // 0.3562 * 2^LUX_SCALE
#define TSL2581_K3C             0x00e6 // 0.45 * 2^RATIO_SCALE
#define TSL2581_B3C             0x18ef // 0.0974 * 2^LUX_SCALE
#define TSL2581_M3C             0x2db9 // 0.1786 * 2^LUX_SCALE
#define TSL2581_K4C             0x0114 // 0.54 * 2^RATIO_SCALE
#define TSL2581_B4C             0x0fdf // 0.062 * 2^LUX_SCALE
#define TSL2581_M4C             0x199a // 0.10 * 2^LUX_SCALE
#define TSL2581_K5C             0x0114 // 0.54 * 2^RATIO_SCALE
#define TSL2581_B5C             0x0000 // 0.00000 * 2^LUX_SCALE
#define TSL2581_M5C             0x0000 // 0.00000 * 2^LUX_SCALE


// Public variables
extern uint8_t TSL2581_gain; // Current gain value
extern uint8_t TSL2581_int_cycles; // Current integration cycles value


void TSL2581_Init(void);
void TSL2581_PowerOn(void);
void TSL2581_PowerOff(void);
void TSL2581_ADCOn(void);
void TSL2581_ADCOff(void);
uint16_t TSL2581_GetData0(void);
uint16_t TSL2581_GetData1(void);
void TSL2581_SetTime(uint8_t time);
void TSL2581_SetGain(uint8_t gain);
uint8_t TSL2581_GetDeviceID(void);
uint32_t TSL2581_LuxCalc(uint16_t d0, uint16_t d1);

#endif // __TSL2581_H
