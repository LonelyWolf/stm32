// Define to prevent recursive inclusion -------------------------------------
#ifndef __WOLK_H
#define __WOLK_H


#include <stm32l1xx_rcc.h>


// Defines to font pointer for a bit compact code
#define fnt5x7              &Font5x7
#define fnt7x10             &Font7x10


// nRF24L01 working parameters
#define nRF24_RX_Addr                 "WBC" // RX address for nRF24L01
#define nRF24_RX_Addr_Size                3 // RX address size
//#define nRF24_RF_CHANNEL                 90 // nRF24L01 channel (90CH = 2490MHz)
#define nRF24_RF_CHANNEL                115 // nRF24L01 channel (115CH = 2515MHz)
#define nRF24_RX_PAYLOAD                 11 // nRF24L01 payload length


// Buttons HAL
#define BTN0_PORT     GPIOB
#define BTN0_PIN      GPIO_Pin_10
#define BTN0_PERIPH   RCC_AHBPeriph_GPIOB
#define BTN0_EXTI     (1 << 10)
#define BTN0_EXTI_N   EXTI15_10_IRQn

#define BTN1_PORT     GPIOB
#define BTN1_PIN      GPIO_Pin_11
#define BTN1_PERIPH   RCC_AHBPeriph_GPIOB
#define BTN1_EXTI     (1 << 11)
#define BTN1_EXTI_N   EXTI15_10_IRQn

#define BTN2_PORT     GPIOA
#define BTN2_PIN      GPIO_Pin_15
#define BTN2_PERIPH   RCC_AHBPeriph_GPIOA
#define BTN2_EXTI     (1 << 15)
#define BTN2_EXTI_N   EXTI15_10_IRQn

#define BTN3_PORT     GPIOA
#define BTN3_PIN      GPIO_Pin_0
#define BTN3_PERIPH   RCC_AHBPeriph_GPIOA
#define BTN3_EXTI     (1 << 0)
#define BTN3_EXTI_N   EXTI0_IRQn

#define BTN_UP        0
#define BTN_DOWN      1
#define BTN_ENTER     2
#define BTN_ESCAPE    3


// NULL declaration
#ifndef NULL
#define NULL  ((void *)0)
#endif

// Simple boolean
#ifndef BOOL
#define BOOL
typedef enum {
	FALSE = 0,
	TRUE  = !FALSE
} bool;
#endif

// Button state
typedef enum {
	BTN_Released = 0,
	BTN_Pressed  = 1,
	BTN_Hold     = 2
} BTN_StateTypeDef;

// Structure for data packet received from sensor
typedef struct {
	uint16_t cntr_SPD;  // SPD impulses counter
	uint16_t tim_SPD;   // SPD interval
	uint16_t tim_CDC;   // CDC interval
	uint16_t vrefint;   // VrefInt of sensor MCU
	uint16_t cntr_wake; // Wake counter
	uint8_t  CRC8;      // 8-bit CRC
} nRF24_Packet_TypeDef;

// Structure for current data
typedef struct {
	uint16_t Speed;            // Current speed (km/h * 10)
	uint16_t MaxSpeed;         // Trip maximum speed (km/h * 10)
	uint16_t AvgSpeed;         // Trip average speed (km/h * 10)
	uint16_t Cadence;          // Current cadence (RPM)
	uint16_t MaxCadence;       // Trip maximum cadence (RPM)
	uint16_t AvgCadence;       // Trip average cadence (RPM)
	uint32_t TripDist;         // Trip distance (centimeters)
	uint32_t Odometer;         // Total travel distance (centimeters)
	uint32_t TripTime;         // Trip time (seconds)
	int16_t  Temperature;      // Current temperature (Celsius degree)
	int16_t  MaxTemperature;   // Maximum temperature (Celsius degree)
	int16_t  MinTemperature;   // Minimum temperature (Celsius degree)
	int32_t  Pressure;         // Current air pressure (Pa)
	int32_t  MaxPressure;      // Maximum air pressure (Pa)
	int32_t  MinPressure;      // Minimum air pressure (Pa)
	int16_t  Altitude;         // Current altitude (m)
	int16_t  MaxAltitude;      // Maximum altitude (m)
	int16_t  MinAltitude;      // Minimum altitude (m)
	int32_t  GPSAlt;           // Current GPS altitude (m)
	int32_t  MaxGPSAlt;        // Maximum GPS altitude (m)
	int32_t  MinGPSAlt;        // Minimum GPS altitude (m)
	uint32_t GPSSpeed;         // Current GPS speed (km/h * 100)
	uint32_t MaxGPSSpeed;      // Maximum GPS speed (km/h * 100)
	uint32_t dbg_cntr_diff;    // Debug: last diff_SPD value
	uint32_t dbg_prev_cntr;    // Debug: last _prev_cntr_SPD value
} Cur_Data_TypeDef;

// Button structure
typedef struct {
	GPIO_TypeDef *PORT;       // Button GPIO port
	uint16_t EXTIn;           // Button EXTI line
	uint16_t PIN;             // Button pin
	uint8_t cntr;             // Number of button clicks
	BTN_StateTypeDef state;   // Button state
	uint8_t hold_cntr;        // Button pressed counter (how long it's pushed)
} BTN_TypeDef;

// Parameters which stored in EEPROM (structure must be aligned to 4 byte)
typedef struct {
	int16_t  altitude_home;       // Home altitude (m)
	uint8_t  GMT_offset;          // GMT time offset (hours)
	uint8_t  LCD_brightness;      // LCD brightness (percents 0..100)
	uint16_t WheelCircumference;  // Wheel circumference (cm)
	uint8_t  LCD_timeout;         // LCD backlight timeout (seconds)
} Settings_TypeDef;


extern nRF24_Packet_TypeDef nRF24_Packet;          // nRF24L01 last received packet
extern Cur_Data_TypeDef CurData;                   // Current data (Speed, Cadence, etc.)
extern BTN_TypeDef BTN[4];                         // Buttons
extern Settings_TypeDef Settings;                  // Settings which stored in EEPROM
extern bool _screensaver;                          // TRUE if screensaver active or must be activated
extern uint32_t _time_scr_timeout;                 // Timeout for screensaver start
extern uint32_t _time_no_signal;                   // Time since last packet received (seconds)
extern uint32_t _time_idle;                        // Time from last user event (button press)


// Embedded internal reference voltage calibration value
// Factory measured Vrefint at Vdda = 3V (+/- 5mV)
#ifdef STM32L151RD
#define VREFINT_CAL           ((uint16_t *)(uint32_t)0x1ff800f8) // STM32L15xxD
#else
#define VREFINT_CAL           ((uint16_t *)(uint32_t)0x1ff80078) // STM32L15xx6/8/B
#endif


// Function prototypes
void SleepWait(void);
void SleepStop(void);
void SleepStandby(void);

uint32_t stringlen(const char *str);
uint8_t numlen(int32_t num);
uint8_t numlenu(uint32_t num);
uint8_t CRC8_CCITT(uint8_t *buf, uint8_t len);

void WaitForKeyPress(bool Sleep, bool *WaitFlag, uint32_t Timeout);
void ClearKeys(void);

#endif // __WOLK_H
