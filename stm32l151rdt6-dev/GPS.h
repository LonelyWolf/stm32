// Define to prevent recursive inclusion -------------------------------------
#ifndef __GPS_H
#define __GPS_H


// GPS HAL
#define GPS_USART_PORT                  USART2 // Port connected to the GPS

// Size constants
#define GPS_BUFFER_SIZE                 1024  // Size of GPS buffer
#define MAX_SATELLITES_VIEW             12    // Maximum number of satellites in view to handle

// GPS commands
// MTK test packet (MTK should respond with "$PMTK001,0,3*30")
#define PMTK_TEST                       "$PMTK000*"
// Enable all supported NMEA sentences
#define PMTK_SET_NMEA_OUTPUT_ALLDATA    "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,1,0*"
// Efficient NMEA sentences (no GLL and VTG sentences)
#define PMTK_SET_NMEA_OUTPUT_EFFICIENT  "$PMTK314,0,1,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,1,0*"
// Set NMEA baud rate 115200bps
#define PMTK_SET_NMEA_BAUDRATE_115200   "$PMTK251,115200*"
// Set NMEA baud rate 38400bps
#define PMTK_SET_NMEA_BAUDRATE_38400    "$PMTK251,38400*"
// Set NMEA baud rate 9600bps
#define PMTK_SET_NMEA_BAUDRATE_9600     "$PMTK251,9600*"
// Hot start the GPS module
#define PMTK_CMD_HOT_START              "$PMTK101*"
// Enter standby mode
#define PMTK_CMD_STANDBY_MODE           "$PMTK161,0*"
// Enable AIC multi-tone Active Interference Cancellation in cost of +1mA consumption
#define PMTK_SET_AIC_ENABLED            "$PMTK286,1*"
// Disable the speed threshold for static navigation (MT333x)
// that mode useful for high speed applications
#define PMTK_API_SET_STATIC_NAV_THD_OFF "$PMTK386,0*"
// Enable EASY function (MT333x)
#define PMTK_EASY_ENABLE                "$PMTK869,1,1*"
// Disable EASY function (MT333x)
#define PMTK_EASY_DISABLE               "$PMTK869,1,0*"
// Disable periodic mode
#define PMTK_SET_PERIODIC_MODE_NORMAL   "$PMTK225,0*"

// Factor for translating PDOP to accuracy in meters
// This very rough value representing GPS horizontal position accuracy
#define GPS_DOP_FACTOR                  5

// Simple boolean
#ifndef BOOL
#define BOOL
typedef enum {
	FALSE = 0,
	TRUE  = !FALSE
} bool;
#endif


// NMEA sentence type
enum {
	// GPS sentences
	NMEA_GPGLL,
	NMEA_GPRMC,
	NMEA_GPVTG,
	NMEA_GPGGA,
	NMEA_GPGSA,
	NMEA_GPGSV,
	NMEA_GPZDA,
	// MTK sentences
	NMEA_PMTK001,
	NMEA_PMTK010,
	NMEA_PMTK011,
	// Status sentences
	NMEA_INVALID,      // Sentence validation failed
	NMEA_UNKNOWN,      // Unsupported sentence found
	NMEA_NOTFOUND      // No sentence has been found
};

// NMEA sentence
typedef struct {
	uint8_t *start; // Pointer to the first byte of sentence
	uint8_t *data;  // Pointer to the first term of sentence
	uint8_t *end;   // Pointer to the last byte of sentence
	uint8_t  type;  // Sentence type
} NMEASentence_TypeDef;

// Structure to hold NMEA time
typedef struct {
	uint8_t  Hours;      // 0..23
	uint8_t  Minutes;    // 0..59
	uint8_t  Seconds;    // 0..59
} NMEATime;

// Structure to hold NMEA date
typedef struct {
	uint8_t  Date;       // 1..31
	uint8_t  Month;      // 1..12
	uint16_t Year;       // e.g. 2015
} NMEADate;

// Structure to hold all parsed GPS data
typedef struct {
	int32_t  latitude;            // Latitude (microdegrees), e.g. 89123456 = 89.123456 degrees
	uint8_t  latitude_char;       // Latitude N/S indicator (X if no valid data)
	int32_t  longitude;           // Longitude (microdegrees), e.g. 179123456 = 179.123456 degrees
	uint8_t  longitude_char;      // Longitude E/W indicator (X if no valid data)
	uint32_t speed_k;             // Speed over ground (Knots, 1 knot = 1.85200km/h)
	uint32_t speed;               // Speed over ground (km/h)
	uint32_t course;              // Track angle relative to North (Degrees)
	uint32_t PDOP;                // Dilution of precision
	uint32_t HDOP;                // Horizontal dilution of precision
	uint32_t VDOP;                // Vertical dilution of precision
	uint32_t accuracy;            // Position accuracy (meters) [value of '500' represents 5.00m]
	uint8_t  sats_used;           // Satellites used for fix
	uint8_t  sats_view;           // Satellites in view
	int32_t  altitude;            // Mean-sea-level altitude (meters)
	int32_t  geoid_separation;    // Geoid-to-ellipsoid separation (meters)
	uint8_t  fix;                 // Fix indicator (1 = fix not available, 2 = 2D fix, 3 = 3D fix)
	NMEATime fix_time;            // Time of fix
	NMEADate fix_date;            // Date of fix
	NMEATime time;                // UTC time
	NMEADate date;                // UTC date
	bool     datetime_valid;      // Have valid date and time
	uint8_t  fix_quality;         // Position fix indicator:
	                              //   0 = invalid
	                              //   1 = GPS fix (SPS)
	                              //   2 = DGPS fix
	                              //   3 = PPS fix (Encrypted military signal)
	                              //   4 = RTK (Real Time Kinematic)
	                              //   5 = Floating RTK
	                              //   6 = Estimated (dead reckoning) (2.3 feature)
	                              //   7 = Manual input mode
	                              //   8 = Simulation mode
	uint8_t  mode;                // Mode indicator:
	                              //   A = Autonomous
	                              //   D = Differential(DGPS)
	                              //   E = Estimated(DR),
	                              //   R = Coarse position
	                              //   S = Simulator
	                              //   N = Data not valid
	uint32_t dgps_age;            // Time since last DGPS update (seconds)
	uint32_t dgps_id;             // DGPS station ID number
	bool     valid;               // GPS status: TRUE if data valid
} GPS_Data_TypeDef;

// Structure describes the GPS satellite
typedef struct {
	uint8_t  PRN;                 // Satellite PRN number
	uint8_t  elevation;           // Elevation, degrees (max 90)
	uint16_t azimuth;             // Azimuth, degrees from true north (0..359)
	uint8_t  SNR;                 // SNR, dB (0..99, 255 when not tracking)
	bool     used;                // TRUE if satellite used in location fix
} GPS_Satellite_TypeDef;

// PMTK sentences parse result
typedef struct {
	bool     PMTK_BOOT;           // TRUE when "$PMTK011,MTKGPS*08" sentence parsed
	uint8_t  PMTK010;             // Last parsed $PMTK010 sentence:
	                              //   0 = unknown
	                              //   1 = startup
	                              //   2 = notification for the host aiding EPO
	                              //   3 = notification for the transition to normal mode is successfully done
	uint16_t PMTK001_CMD;         // Cmd field from last parsed $PMTK001 sentence
	uint8_t  PMTK001_FLAG;        // Flag field from last parsed $PMTK001 sentence:
	                              //   0 = invalid packet
	                              //   1 = unsupported packet type
	                              //   2 = valid packet, but action failed
	                              //   3 = valid packet, action succeeded
} GPS_PMTK_TypeDef;


// Public variables
extern GPS_Data_TypeDef GPSData;                   // Parsed GPS information
extern bool GPS_new_data;                          // TRUE if received new GPS packet
extern bool GPS_parsed;                            // TRUE if GPS packets was parsed
extern uint16_t GPS_buf_cntr;                      // Number of actual bytes in GPS buffer
extern uint8_t GPS_sentences_parsed;               // Parsed NMEA sentences counter
extern uint8_t GPS_sentences_unknown;              // Unsupported NMEA sentences counter
extern uint8_t GPS_sentences_invalid;              // Invalid NMEA sentences counter
extern uint8_t GPS_buf[];                          // Buffer for GPS data
extern uint8_t GPS_sats[];                         // IDs of satellites used in position fix
extern GPS_Satellite_TypeDef GPS_sats_view[];      // Information about satellites in view


// Function prototypes
void GPS_Send(char *cmd);
void GPS_InitData(void);
void GPS_CheckUsedSats(void);
void GPS_ParseBuf(uint8_t *buf, uint32_t length);
void GPS_Init(void);

#endif // __GPS_H
