// Define to prevent recursive inclusion -------------------------------------
#ifndef __GPS_H
#define __GPS_H


#define GPS_BUFFER_SIZE     1024  // Size of GPS buffer
#define MAX_SATELLITES_VIEW   12  // Maximum number of satellites in view to handle

#define PMTK_TEST                       "$PMTK000*" // MTK test packet (MTK should respond with "$PMTK001,0,3*30")
#define PMTK_SET_NMEA_OUTPUT_ALLDATA    "$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,1,0*" // All supported NMEA sentences
#define PMTK_SET_NMEA_OUTPUT_EFFICIENT  "$PMTK314,0,1,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,1,0*" // Efficient NMEA sentences (no GLL and VTG sentences)
#define PMTK_SET_NMEA_BAUDRATE_115200   "$PMTK251,115200*" // Set NMEA baudrate to 115200bps
#define PMTK_CMD_HOT_START              "$PMTK101*" // Hot start the GPS module
#define PMTK_CMD_STANDBY_MODE           "$PMTK161,0*" // Enter standby mode
#define PMTK_SET_AIC_ENABLED            "$PMTK286,1*" // Enable AIC multi-tone Active Interference Cancellation
                                                      // in cost of 1mA consumption
#define PMTK_API_SET_STATIC_NAV_THD_OFF "$PMTK386,0*" // Disable the speed threshold for static navigation (MT333x)
                                                      // this mode is useful for high speed applications
#define PMTK_EASY_ENABLE                "$PMTK869,1,1*" // Enable EASY function (MT333x)
#define PMTK_EASY_DISABLE               "$PMTK869,1,0*" // Disable EASY function (MT333x)
#define PMTK_SET_PERIODIC_MODE_NORMAL   "$PMTK225,0*" // Disable periodic mode

#define GPS_DOP_FACTOR                  5 // Factor for translating PDOP to accuracy in meters
                                          // This very rough value representing GPS horizontal position accuracy


#define GPS_USART_PORT                 USART2 // Port connected to the GPS


typedef enum {
	NMEA_BAD     = 0x00,
	NMEA_GLL     = 0x01,
	NMEA_RMC     = 0x02,
	NMEA_VTG     = 0x03,
	NMEA_GGA     = 0x04,
	NMEA_GSA     = 0x05,
	NMEA_GSV     = 0x06,
	NMEA_ZDA     = 0x07,
	NMEA_PMTK001 = 0x08,
	NMEA_PMTK010 = 0x09,
	NMEA_PMTK011 = 0x0a
} NMEASentenceType_TypeDef;


typedef struct {
	uint16_t start;
	uint16_t end;
	NMEASentenceType_TypeDef type;
} NMEASentence_TypeDef;

typedef struct {
	uint32_t latitude;            // Latitude (degrees)
	uint8_t  latitude_char;       // Latitude N/S indicator (X if no valid data)
	uint32_t longitude;           // Longitude (degrees)
	uint8_t  longitude_char;      // Longitude E/W indicator (X if no valid data)
	uint32_t speed_k;             // Speed over ground (Knots, 1 knot = 1.825km/h)
	uint32_t speed;               // Speed over ground (km/h)
	uint32_t course;              // Track angle relative to North (Degrees)
	uint32_t PDOP;                // Dilution of precision
	uint32_t HDOP;                // Horizontal dilution of precision
	uint32_t VDOP;                // Vertical dilution of precision
	uint32_t accuracy;            // Position accuracy (meters) [500 means 5.00m]
	uint8_t  sats_used;           // Satellites used for fix
	uint8_t  sats_view;           // Satellites in view
	int32_t  altitude;            // Mean-sea-level altitude (meters)
	int32_t  geoid_separation;    // Geoid-to-ellipsoid separation (meters)
	uint8_t  fix;                 // Fix indicator (1=Fix not available, 2=2D fix, 3=3D fix)
	uint32_t fix_time;            // Time of fix (seconds from midnight)
	uint32_t fix_date;            // Date of fix (DDMMYYYY)
	uint8_t  fix_quality;         // Position fix indicator:
                                  //   0 = invalid
                                  //   1 = GPS fix (SPS)
                                  //   2 = DGPS fix
                                  //   3 = PPS fix (Encrypted military signal)
                                  //   4 = Real Time Kinematic
                                  //   5 = Floating RTK
                                  //   6 = Estimated (dead reckoning) (2.3 feature)
                                  //   7 = Manual input mode
                                  //   8 = Simulation mode
	uint32_t time;                // UTC Time (seconds from midnight)
	uint32_t date;                // UTC Date (DDMMYYYY)
	uint8_t  mode;                // Mode indicator (A=Autonomous, D=Differential(DGPS), E=Estimated(DR),
	                              // R=Coarse pos., S=Simulator, N=Data not valid)
	uint32_t dgps_age;            // Time since last DGPS update (seconds)
	uint32_t dgps_id;             // DGPS station ID number
	bool     datetime_valid;      // Have valid date and time
	bool     valid;               // GPS status: TRUE if data valid
} GPS_Data_TypeDef;

typedef struct {
	uint8_t  PRN;                 // Satellite PRN number
	uint8_t  elevation;           // Elevation, degrees (max 90)
	uint16_t azimuth;             // Azimuth, degrees from true noth (0..359)
	uint8_t  SNR;                 // SNR, dB (0..99, 255 when not tracking)
	bool     used;                // TRUE if satellite used in location fix
} GPS_Satellite_TypeDef;

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
extern NMEASentence_TypeDef GPS_msg;               // NMEA sentence position
extern uint8_t GPS_sentences_parsed;               // Parsed NMEA sentences counter
extern uint8_t GPS_sentences_unknown;              // Found unknown NMEA sentences counter
extern uint8_t GPS_buf[GPS_BUFFER_SIZE];           // Buffer with data from GPS
extern GPS_Data_TypeDef GPSData;                   // Parsed GPS data
extern uint8_t GPS_sats[];                         // IDs of satellites used in position fix
extern GPS_Satellite_TypeDef GPS_sats_view[];      // Information about satellites in view
extern GPS_PMTK_TypeDef GPS_PMTK;                  // PMTK messages result


// Function prototypes
uint8_t GPS_CRC(char *str);
void GPS_SendCommand(char *cmd);
void GPS_FindSentence(NMEASentence_TypeDef *msg, uint8_t *buf, uint16_t start, uint16_t buf_size);
void GPS_ParseSentence(uint8_t *buf, NMEASentence_TypeDef *Sentence);
void GPS_InitData(void);
void GPS_CheckUsedSats(void);
void GPS_Init(void);

#endif // __GPS_H
