// Define to prevent recursive inclusion -------------------------------------
#ifndef __NMEA_H
#define __NMEA_H


// Size constants
#define MAX_SATELLITES_VIEW             12 // Maximum number of satellites in view to handle

// Factor for translating PDOP to accuracy in meters
// This is very rough value representing GPS horizontal position accuracy
#define GPS_DOP_FACTOR                  5


// Simple boolean
#ifndef BOOL
#define BOOL
typedef enum {
	FALSE = 0,
	TRUE  = !FALSE
} bool;
#endif


// Types of NMEA sentences
enum {
	// Status sentences
	NMEA_NOTFOUND = 1,      // No sentence has been found
	NMEA_UNKNOWN,           // Unsupported sentence found
	NMEA_INVALID,           // Sentence validation failed
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
	NMEA_PMTK011
};

// Structure describes NMEA sentence in the data buffer
typedef struct {
	uint8_t *start; // Pointer to the first byte of sentence
	uint8_t *data;  // Pointer to the first term of sentence
	uint8_t *end;   // Pointer to the last byte of sentence
	uint8_t  type;  // Sentence type
} NMEASentence_TypeDef;

// Structure to hold NMEA time
typedef struct {
	uint8_t  Hours;      // Hours (0..23)
	uint8_t  Minutes;    // Minutes (0..59)
	uint8_t  Seconds;    // Seconds (0..59)
} NMEATime;

// Structure to hold NMEA date
typedef struct {
	uint8_t  Date;       // Day of month (1..31)
	uint8_t  Month;      // Month (1..12)
	uint16_t Year;       // Year four-digit number (e.g. 2015)
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

// Structure describes the satellite parameters
typedef struct {
	uint8_t  PRN;                 // Satellite PRN number
	uint8_t  elevation;           // Elevation, degrees (max 90)
	uint16_t azimuth;             // Azimuth, degrees from true north (0..359)
	uint8_t  SNR;                 // SNR, dB (0..99, 255 when not tracking)
	bool     used;                // TRUE if satellite used in location fix
} NMEA_Sat_TypeDef;

// Structure to hold data from PMTK sentences
typedef struct {
	bool     PMTK_BOOT;           // TRUE when "$PMTK011,MTKGPS*08" sentence parsed
	uint8_t  PMTK010;             // Last parsed $PMTK010 sentence:
	                              //   0 = unknown
	                              //   1 = startup
	                              //   2 = notification for the host aiding EPO
	                              //   3 = notification for the transition to normal mode is successfully done
	uint16_t PMTK001_CMD;         // CMD field from last parsed $PMTK001 sentence
	uint8_t  PMTK001_FLAG;        // FLAG field from last parsed $PMTK001 sentence:
	                              //   0 = invalid packet
	                              //   1 = unsupported packet type
	                              //   2 = valid packet, but action failed
	                              //   3 = valid packet, action succeeded
} NMEA_PMTK_TypeDef;


// Public variables
// Structures for parsed data
extern GPS_Data_TypeDef GPSData;                   // Data from GNSS sentences
extern NMEA_PMTK_TypeDef PMTKData;                 // Data from PMTK sentences

// Sentences counters
extern uint8_t NMEA_sentences_parsed;              // Parsed
extern uint8_t NMEA_sentences_unknown;             // Unsupported
extern uint8_t NMEA_sentences_invalid;             // Invalid

// Information about satellites in view
extern NMEA_Sat_TypeDef GPS_sats_view[];


// Function prototypes
uint8_t NMEA_CalcCRC(char *str);
void NMEA_InitData(void);
void NMEA_CheckUsedSats(void);
void NMEA_ParseBuf(uint8_t *buf, uint16_t *length);

#endif // __NMEA_H
