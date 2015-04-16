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
	TSL2581_GAIN1   = 0, // 1x
	TSL2581_GAIN8   = 1, // 8x
	TSL2581_GAIN16  = 2, // 16x
	TSL2581_GAIN111 = 3  // 111x
};


void TSL2581_Init(void);
void TSL2581_PowerOn(void);
void TSL2581_PowerOff(void);
void TSL2581_ADCOn(void);
void TSL2581_SetTime(uint8_t time);
uint16_t TSL2581_GetData0(void);
uint16_t TSL2581_GetData1(void);
void TSL2581_SetGain(uint8_t gain);
uint8_t TSL2581_GetDeviceID(void);

#endif // __TSL2581_H
