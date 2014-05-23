// Define to prevent recursive inclusion -------------------------------------
#ifndef __WOLK_H
#define __WOLK_H

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

// Struct for data packet from sensor
typedef struct {
	uint16_t cntr_SPD;         // SPD impulses counter
	uint16_t tim_CDC;          // CDC interval
	uint16_t tim_SPD;          // SPD interval
	uint16_t vrefint;          // VrefInt of sensor MCU
	uint8_t  observe_TX;       // Previous value of the OBSERVE_TX register
	uint16_t cntr_wake;        // Wakeups counter
	uint16_t packets_lost;     // Lost packets counter
	uint8_t  tx_power;         // TX power
	uint16_t ride_time;        // SPD interval from last delivered packet
} nRF24_Packet_TypeDef;

// Struct for current data
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
	int32_t  Altitude;         // Current altitude (m)
	int32_t  MaxAltitude;      // Maximum altitude (m)
	int32_t  MinAltitude;      // Minimum altitude (m)
} Cur_Data_TypeDef;

typedef struct {
	uint8_t cntr;             // Number of button clicks
	BTN_StateTypeDef state;   // Button state
	uint16_t tim_start;       // Timer counter when button was pressed
	uint8_t hold_cntr;        // Button pressed counter (how long it's pushed)
} BTN_TypeDef;



// nRF24L01 working parameters
#define nRF24_RX_Addr               "WolkS" // TX address for nRF24L01
#define nRF24_RX_Addr_Size                5 // TX address size
#define nRF24_RF_CHANNEL                 90 // nRF24L01 channel (90ch = 2490MHz)
#define nRF24_RX_PAYLOAD                 17 // nRF24L01 payload length


static char * const nRF24_TX_POWERS[] = {"-18dBm","-12dBm","-6dBm","0dBm"};


extern nRF24_Packet_TypeDef nRF24_Packet;          // nRF24L01 last received packet
extern Cur_Data_TypeDef CurData;                   // Current data (Speed, Cadence, etc.)


// Embedded internal reference voltage calibration value
// Factory measured at Vdda = 3V (+/- 5mV)
#define VREFINT_CAL                     ((uint16_t *) (uint32_t)0x1ff80078)


// Function prototypes
uint16_t atos_len(uint8_t *buf, uint8_t len);
int32_t atos_char(uint8_t *buf, uint16_t *pos);

#endif // __WOLK_H
