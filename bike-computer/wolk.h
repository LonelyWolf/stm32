// Define to prevent recursive inclusion -------------------------------------
#ifndef __WOLK_H
#define __WOLK_H


// nRF24L01 working parameters
#define nRF24_RX_Addr                 "WBC" // RX address for nRF24L01
#define nRF24_RX_Addr_Size                3 // RX address size
#define nRF24_RF_CHANNEL                 90 // nRF24L01 channel (90CH = 2490MHz)
#define nRF24_RX_PAYLOAD                 17 // nRF24L01 payload length


// Buttons
#define      BTN1_PORT   GPIOA
#define      BTN1_PIN    GPIO_Pin_5
#define      BTN2_PORT   GPIOA
#define      BTN2_PIN    GPIO_Pin_7
#define      BTN3_PORT   GPIOC
#define      BTN3_PIN    GPIO_Pin_10
#define      BTN4_PORT   GPIOC
#define      BTN4_PIN    GPIO_Pin_11

#define      BTN_UP      0
#define      BTN_DOWN    1
#define      BTN_ENTER   2
#define      BTN_ESCAPE  3

// Just boolean
typedef enum {
	FALSE = 0,
	TRUE  = !FALSE
} bool;

// Button state
typedef enum {
	BTN_Released = 0,
	BTN_Pressed  = 1,
	BTN_Hold     = 2
} BTN_StateTypeDef;

// Structure for data packet received from sensor
typedef struct {
	uint16_t cntr_SPD;         // SPD impulses counter
	uint16_t tim_CDC;          // CDC interval
	uint16_t tim_SPD;          // SPD interval
	uint16_t vrefint;          // VrefInt of sensor MCU
	uint8_t  observe_TX;       // Previous value of the OBSERVE_TX register
	uint16_t cntr_wake;        // Wakeups counter
	uint16_t packets_lost;     // Lost packets counter
	uint16_t ride_time;        // SPD interval from last delivered packet
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
} Cur_Data_TypeDef;

// Button structure
typedef struct {
	GPIO_TypeDef *PORT;       // Button GPIO port
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
} Settings_TypeDef;


extern nRF24_Packet_TypeDef nRF24_Packet;          // nRF24L01 last received packet
extern Cur_Data_TypeDef CurData;                   // Current data (Speed, Cadence, etc.)
extern BTN_TypeDef BTN[4];                         // Buttons
extern Settings_TypeDef Settings;                  // Settings which stored in EEPROM


// Embedded internal reference voltage calibration value
// Factory measured Vrefint at Vdda = 3V (+/- 5mV)
#define VREFINT_CAL           ((uint16_t *)(uint32_t)0x1ff80078)


// Function prototypes
void SleepWait(void);

uint32_t atos_len(uint8_t *buf, uint8_t len);
int32_t atos_char(uint8_t *buf, uint16_t *pos);
uint32_t stringlen(const char *str);
uint8_t numlen(int32_t num);

void ReadSettings_EEPROM(void);
void SaveSettings_EEPROM(void);

void WaitForKeyPress(bool Sleep, bool *WaitFlag);
void ClearKeys(void);

#endif // __WOLK_H
