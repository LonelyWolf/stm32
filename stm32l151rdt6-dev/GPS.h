#ifndef __GPS_H
#define __GPS_H


// GPS HAL
#define GPS_USART_PORT                  USART2 // Port connected to the GPS

// Size constants
#define GPS_BUFFER_SIZE                 1024  // Size of GPS buffer


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


// Public variables
extern uint16_t GPS_buf_cntr;                      // Number of actual bytes in GPS buffer
extern uint8_t GPS_buf[];                          // Buffer for GPS data
extern bool GPS_new_data;                          // TRUE if received new GPS packet
extern bool GPS_parsed;                            // TRUE if GPS packets was parsed


// Function prototypes
void GPS_Send(char *cmd);
void GPS_Init(void);

#endif // __GPS_H
