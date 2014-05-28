// Define to prevent recursive inclusion -------------------------------------
#ifndef __GPS_H
#define __GPS_H


#define MAX_SATELLITES_VIEW   16  // Maximum number of satellites in view to handle


typedef enum {
	NMEA_BAD = 0,
	NMEA_GLL = 1,
	NMEA_RMC = 2,
	NMEA_VTG = 3,
	NMEA_GGA = 4,
	NMEA_GSA = 5,
	NMEA_GSV = 6,
	NMEA_ZDA = 7
} NMEASentenceType_TypeDef;


typedef struct {
	uint16_t start;
	uint16_t end;
	NMEASentenceType_TypeDef type;
} NMEASentence_TypeDef;

typedef struct {
	uint8_t  latitude_degree;     // Latitude degrees
	uint32_t latitude_seconds;    // Latitude seconds
	uint8_t  latitude_char;       // Latitude N/S indicator (X if no valid data)
	uint8_t  longitude_degree;    // Longitude degrees
	uint32_t longitude_seconds;   // Longitude seconds
	uint8_t  longitude_char;      // Longitude E/W indicator (X if no valid data)
	int32_t  speed_k;             // Speed over ground (Knots, 1 knot = 1.825km/h)
	int32_t  speed;               // Speed over ground (km/h)
	int32_t  course;              // Track angle relative to North (Degrees)
	uint16_t PDOP;                // Dilution of precision
	uint16_t HDOP;                // Horizontal dilution of precision
	uint16_t VDOP;                // Vertical dilution of precision
	uint8_t  sats_used;           // Satellites used for fix
	uint8_t  sats_view;           // Satellites in view
	int32_t  altitude;            // Mean-sea-level altitude (meters)
	int32_t  geoid_separation;    // Geoid-to-ellipsoid separation (meters)
	uint8_t  fix;                 // Fix indicator (1=Fix not available, 2=2D fix, 3=3D fix)
	uint32_t fix_time;            // Time of fix (seconds from midnight)
	uint32_t fix_date;            // Date of fix (DDMMYYYY)
	uint8_t  fix_quality;         // Position fix indicator: (http://www.gpsinformation.org/dale/nmea.htm#GGA)
                                  //   0 = invalid
                                  //   1 = GPS fix (SPS)
                                  //   2 = DGPS fix
                                  //   3 = PPS fix
                                  //   4 = Real Time Kinematic
                                  //   5 = Float RTK
                                  //   6 = estimated (dead reckoning) (2.3 feature)
                                  //   7 = Manual input mode
                                  //   8 = Simulation mode
	uint32_t time;                // UTC Time (seconds from midnight)
	uint32_t date;                // Date (DDMMYYYY)
	uint8_t  mode;                // Mode indicator (A=Autonomous, D=Differential(DGPS), E=Estimated(DR),
	                              // R=Coarse pos., S=Simulator, N=Data not valid)
	bool     time_valid;          // Have valid time
	bool     datetime_valid;      // Have valid date and time
	bool     valid;               // GPS status: TRUE if data valid
} GPS_Data_TypeDef;

typedef struct {
	uint8_t  PRN;                 // Satellite PRN number
	uint8_t  elevation;           // Elevation, degrees (max 90)
	uint16_t azimuth;             // Azimuth, degrees from true noth (0..359)
	uint8_t  SNR;                 // SNR, dB (0..99, 255 when not tracking)
} GPS_Satellite_TypeDef;


// Public variables
extern GPS_Data_TypeDef GPSData;  // Parsed GPS data
extern uint8_t GPS_sats[];        // IDs of satellites used in position fix
extern GPS_Satellite_TypeDef GPS_sats_view[];   // Information about satellites in view


// Function prototypes
uint8_t GPS_CRC(char *str);
void GPS_SendCommand(char *cmd);
NMEASentence_TypeDef GPS_FindSentence(uint8_t *buf, uint16_t start, uint16_t buf_size);
void GPS_ParseSentence(uint8_t *buf, NMEASentence_TypeDef Sentence);

#endif // __GPS_H
