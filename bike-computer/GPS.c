#include <stm32l1xx_rcc.h>
#include <string.h> // For memset

#include <uart.h>
#include <wolk.h>
#include <GPS.h>


GPS_Data_TypeDef GPSData;                   // Parsed GPS information
bool GPS_new_data;                          // TRUE if received new GPS packet
uint16_t GPS_buf_cntr;                      // Number of actual bytes in GPS buffer
NMEASentence_TypeDef GPS_msg;               // NMEA sentence position
uint8_t GPS_sentences_parsed;               // Parsed NMEA sentences counter
uint8_t GPS_sentences_unknown;               // Found unknown NMEA sentences counter
uint8_t GPS_buf[GPS_BUFFER_SIZE];           // Buffer with data from GPS
uint8_t GPS_sats[12];                       // IDs of satellites used in position fix
// Information about satellites in view (can be increased if receiver able handle more)
GPS_Satellite_TypeDef GPS_sats_view[MAX_SATELLITES_VIEW];


// Calculates ten raised to the given power
// input:
//   exp - exponent value
// return: ten in raised to the exponent
uint32_t pwr10(uint32_t exp) {
	uint32_t result = 1;

	while (exp--) result *= 10;

	return result;
}

// Calculate CRC of NMEA command
// input:
//   str - pointer to the string with command
// return:
//   checksum of command
// note: command must begin with '$' char and end with '*' or zero symbol
uint8_t GPS_CRC(char *str) {
	uint8_t result = 0;

	if (*str++ == '$') while (*str != '*' && *str != '\0') result ^= *str++;

	return result;
}

// Send NMEA command
// input:
//   cmd - pointer to string with command
// note: command must begin with '$' char and end with '*'
void GPS_SendCommand(char *cmd) {
	uint8_t cmd_CRC;

	cmd_CRC = GPS_CRC(cmd);
	UART_SendStr(cmd);
	UART_SendChar(HEX_CHARS[cmd_CRC >> 4]);
	UART_SendChar(HEX_CHARS[cmd_CRC & 0x0f]);
	UART_SendChar('\r');
	UART_SendChar('\n');
}

// Find NMEA sentence in buffer
// input:
//   buf - pointer to buffer with NMEA packet
//   start - position in buffer to start search
//   buf_size - size of buffer
// return: position of sentence beginning or 0 if no sentence found
NMEASentence_TypeDef GPS_FindSentence(uint8_t *buf, uint16_t start, uint16_t buf_size) {
	uint16_t pos = start;
	uint32_t hdr;
	NMEASentence_TypeDef result;

	result.start = start;
	result.end   = buf_size;
	result.type  = NMEA_BAD;

	do {
		if (buf[pos] == '$' && buf[pos + 1] == 'G' && buf[pos + 2] == 'P') {
			result.start = pos + 3;

			// Find end of sentence
			if (pos + 6 > buf_size) return result; // sentence doesn't have ending

			pos += 4;
			do {
				if (buf[pos] == '\r' && buf[pos + 1] == '\n') {
					result.end = pos + 1;

					// sentence have ending, now determine type of sentence
					hdr = (buf[result.start] << 16) |
							(buf[result.start + 1] << 8) |
							(buf[result.start + 2]);
					if (hdr == 0x00474c4c) result.type = NMEA_GLL;
					if (hdr == 0x00524d43) result.type = NMEA_RMC;
 					if (hdr == 0x00565447) result.type = NMEA_VTG;
					if (hdr == 0x00474741) result.type = NMEA_GGA;
					if (hdr == 0x00475341) result.type = NMEA_GSA;
					if (hdr == 0x00475356) result.type = NMEA_GSV;
					if (hdr == 0x005a4441) result.type = NMEA_ZDA;

					return result;
				}
				pos++;
			} while (pos < buf_size - 2);

			return result;
		}
		pos++;
	} while (pos < buf_size - 3);

	return result;
}

// Find next field in GPS sentence
// input:
//   buf - pointer to the data buffer
// return: buffer offset of next field
uint16_t GPS_NextField(uint8_t *buf) {
	uint16_t pos = 1;

	while (*buf != ',' && *buf++ != '*') pos++;

	return pos;
}

// Parse float value from a GPS sentence
// input:
//   buf - pointer to the data buffer
//   value - pointer to parsed value (float represented as integer, e.g. 1234.567 -> 1234567)
// return: number of parsed bytes
uint16_t GPS_ParseFloat(uint8_t *buf, uint32_t *value) {
	uint16_t pos = 0;
	uint32_t ip;
	uint16_t len;

	if (*buf == ',' || *buf == '*') {
		*value = 0;
		return 1;
	}
	ip = atos_char(&buf[pos],&pos); // integer part
	if (buf[pos - 1] == '.') {
		// fractional part
		len = pos;
		*value  = atos_char(&buf[pos],&pos);
		*value += ip * pwr10(pos - len - 1);
	} else {
		// this value is not float
		*value = ip;
	}

	return pos;
}

// Parse latitude or longitude coordinate
// input:
//   buf - pointer to the data buffer
//   len - length of the degrees value (2 for latitude, 3 for longitude)
//   value - pointer to the coordinate variable
//   char_value - pointer to the coordinate character variable
// return: number of parsed bytes
uint16_t GPS_ParseCoordinate(uint8_t *buf, uint8_t len, uint32_t *value, uint8_t *char_value) {
	uint16_t pos = 0;
	uint32_t coord_minutes;
	int16_t f_len; // fractional part length

	// Coordinate
	if (buf[pos] != ',') {
		// '10000' determines length of the fractional part in result
		// e.g. 10000 means 4 fractional digits
		*value = atos_len(&buf[pos],len) * 10000; // degrees
		pos += len;
		f_len = pos;
		pos += GPS_ParseFloat(&buf[pos],&coord_minutes); // minutes
		f_len = pos - f_len - 4; // fractional part length
		if (f_len > 0) {
			// Float calculations, slow
			*value += (uint32_t)((coord_minutes / (pwr10(f_len) * 60.0)) * 10000);
		} else {
			// Are you serious? Floating part is mandatory!
			*value += coord_minutes * 100;
		}
	} else pos++;

	// Coordinate character
	if (buf[pos] != ',') {
		*char_value = buf[pos];
		pos += 2;
	} else {
		*char_value = 'X';
		pos++;
	}

	return pos;
}

// Parse one satellite from $GPGSV sentence
// input:
//   buf - pointer to the data buffer
//   set_num - satellite number in GPS_sats_view[]
// return: number of parsed bytes
uint16_t GPS_ParseSatelliteInView(uint8_t *buf, uint8_t sat_num) {
	uint16_t pos = 0;

	// Satellite PRN number
	if (buf[pos] != ',') {
		GPS_sats_view[sat_num].PRN = atos_len(&buf[pos],2);
		pos += 3;
	} else {
		GPS_sats_view[sat_num].PRN = 0;
		pos++;
	}

	// Satellite elevation
	if (buf[pos] != ',') {
		GPS_sats_view[sat_num].elevation = atos_len(&buf[pos],2);
		pos += 3;
	} else {
		GPS_sats_view[sat_num].elevation = 0;
		pos++;
	}

	// Satellite azimuth
	if (buf[pos] != ',') {
		GPS_sats_view[sat_num].azimuth = atos_len(&buf[pos],3);
		pos += 4;
	} else {
		GPS_sats_view[sat_num].azimuth = 0;
		pos++;
	}

	// Satellite SNR
	if (buf[pos] != ',' && buf[pos] != '*') {
		GPS_sats_view[sat_num].SNR = atos_len(&buf[pos],2);
		pos += 3;
	} else {
		GPS_sats_view[sat_num].SNR = 255; // Satellite not tracked
		pos++;
	}

	// This must be set after all GPS sentences parsed
	GPS_sats_view[sat_num].used = FALSE;

	return pos;
}

// Parse time from NMEA sentence
// input:
//   buf - pointer to the data buffer
//   time - pointer to variable where time will be stored
// return: number of parsed bytes
uint16_t GPS_ParseTime(uint8_t *buf, uint32_t *time) {
	uint16_t pos = 0;

	// Parse time
	*time  = atos_len(&buf[pos],2) * 3600;
	pos += 2;
	*time += atos_len(&buf[pos],2) * 60;
	pos += 2;
	*time += atos_len(&buf[pos],2);
	pos += 3;
	pos += GPS_NextField(&buf[pos]); // Ignore milliseconds
	GPSData.time_valid = TRUE;

	return pos;
}

// Parse NMEA sentence
// input:
//   buf - pointer to the data buffer
//   Sentence - pointer to the structure with NMEA sentence parameters
void GPS_ParseSentence(uint8_t *buf, NMEASentence_TypeDef *Sentence) {
	uint16_t pos = Sentence->start + 4;
	uint8_t i;
	uint8_t GSV_msg;   // GSV sentence number
	uint8_t GSV_sats;  // Total number of satellites in view

	switch (Sentence->type) {
	case NMEA_RMC:
		// $GPRMC - Recommended minimum specific GPS/Transit data

		// Time of fix
		if (buf[pos] != ',') pos += GPS_ParseTime(&buf[pos],&GPSData.fix_time); else pos++;

		// Valid data marker
		GPSData.valid = FALSE;
		if (buf[pos] != ',') {
			if (buf[pos] == 'A') GPSData.valid = TRUE;
			pos += 2;
		} else pos++;

		// Latitude
		pos += GPS_ParseCoordinate(&buf[pos],2,&GPSData.latitude,&GPSData.latitude_char);

		// Longitude
		pos += GPS_ParseCoordinate(&buf[pos],3,&GPSData.longitude,&GPSData.longitude_char);

		// Horizontal speed (in knots)
		pos += GPS_ParseFloat(&buf[pos],&GPSData.speed_k);

		// Course
		pos += GPS_ParseFloat(&buf[pos],&GPSData.course);

		// Date of fix
		if (buf[pos] != ',') {
			GPSData.fix_date  = atos_len(&buf[pos],2) * 1000000;
			GPSData.fix_date += atos_len(&buf[pos + 2],2) * 100000;
			GPSData.fix_date += atos_len(&buf[pos + 4],2) + 2000;
			pos += 6;
		} pos++;

		// Magnetic variation
		// ignore this (mostly not supported by GPS receivers)
		pos += GPS_NextField(&buf[pos]);

		// Magnetic variation direction
		// ignore this (mostly not supported by GPS receivers)
		pos += GPS_NextField(&buf[pos]);

		// Mode indicator (NMEA 0183 v3.0 or never)
		if (buf[pos] != ',' && buf[pos] != '*') GPSData.mode = buf[pos];

		break; // NMEA_RMC
	case NMEA_GLL:
		// $GPGLL - Geographic position, latitude / longitude

		// Latitude
		pos += GPS_ParseCoordinate(&buf[pos],2,&GPSData.latitude,&GPSData.latitude_char);

		// Longitude
		pos += GPS_ParseCoordinate(&buf[pos],3,&GPSData.longitude,&GPSData.longitude_char);

		// Time of fix
		if (buf[pos] != ',') pos += GPS_ParseTime(&buf[pos],&GPSData.fix_time); else pos++;

		// Valid data marker
		GPSData.valid = FALSE;
		if (buf[pos] != ',') {
			if (buf[pos] == 'A') GPSData.valid = TRUE;
			pos += 2;
		} else pos++;

		// Mode indicator
		if (buf[pos] != ',') GPSData.mode = buf[pos]; else GPSData.mode = 'N';

		break; // NMEA_GLL
	case NMEA_ZDA:
		// $GPZDA - Date & Time

		// Time
		if (buf[pos] != ',') pos += GPS_ParseTime(&buf[pos],&GPSData.time); else pos++;

		// Date: day
		if (buf[pos] != ',') {
			GPSData.date = atos_len(&buf[pos],2) * 1000000;
			pos += 3;
		} else {
			GPSData.date = 1000000;
			pos++;
		}

		// Date: month
		if (buf[pos] != ',') {
			GPSData.date += atos_len(&buf[pos],2) * 10000;
			pos += 3;
		} else {
			GPSData.date += 10000;
			pos++;
		}

		// Date: year
		if (buf[pos] != ',') {
			GPSData.date += atos_len(&buf[pos],4);
		} else GPSData.date += 2013;

		// Local time zone offset
		// sad but true: this feature mostly not supported by GPS receivers

		// Check for year, if it less than 2014, the date from the GPS receiver is not valid
		if (GPSData.date % 10000 > 2013) GPSData.datetime_valid = TRUE;

		break; // NMEA_ZDA
	case NMEA_VTG:
		// $GPVTG - Course over ground and ground speed

		// Course (heading relative to true north)
		pos += GPS_ParseFloat(&buf[pos],&GPSData.course);

		// Field with 'T' letter - "track made good is relative to true north"
		// ignore it
		pos += GPS_NextField(&buf[pos]);

		// Field with course relative to magnetic north
		// mostly not supported by GPS receivers, ignore it
		pos += GPS_NextField(&buf[pos]);

		// Field with 'M' letter - "track made good is relative to magnetic north"
		// mostly not supported by GPS receivers, ignore it
		pos += GPS_NextField(&buf[pos]);

		// Speed over ground in knots
		pos += GPS_ParseFloat(&buf[pos],&GPSData.speed_k);

		// Field with 'N' - speed over ground measured in knots, ignore it
		pos += GPS_NextField(&buf[pos]);

		// Speed over ground in km/h
		pos += GPS_ParseFloat(&buf[pos],&GPSData.speed);

		// Field with 'K' - speed over ground measured in km/h, ignore it
		pos += GPS_NextField(&buf[pos]);

		// Mode indicator (NMEA 0183 v3.0 or later)
		if (buf[pos] != ',' && buf[pos] != '*') GPSData.mode = buf[pos]; else GPSData.mode = 'N';

		break; // NMEA_VTG
	case NMEA_GGA:
		// $GPGGA - GPS fix data

		// Time
		if (buf[pos] != ',') pos += GPS_ParseTime(&buf[pos],&GPSData.time); else pos++;

		// Latitude
		pos += GPS_ParseCoordinate(&buf[pos],2,&GPSData.latitude,&GPSData.latitude_char);

		// Longitude
		pos += GPS_ParseCoordinate(&buf[pos],3,&GPSData.longitude,&GPSData.longitude_char);

		// Position fix indicator
		if (buf[pos] != ',') {
			GPSData.fix_quality = buf[pos] - '0';
			pos += 2;
		} else pos++;

		// Satellites used
		if (buf[pos] != ',') GPSData.sats_used = atos_char(&buf[pos],&pos); else pos++;

		// HDOP - horizontal dilution of precision
		pos += GPS_ParseFloat(&buf[pos],&GPSData.HDOP);

		// MSL Altitude (mean-sea-level)
		if (buf[pos] != ',') {
			// This value can be negative
			// Only integer part, fractional is useless
			GPSData.altitude = atos_char(&buf[pos],&pos);
			pos += GPS_NextField(&buf[pos]);
/*
			GPSData.altitude  = atos_char(&buf[pos],&pos) * 1000;
			if (GPSData.altitude >= 0)
				GPSData.altitude += atos_len(&buf[pos],3);
			else
				GPSData.altitude -= atos_len(&buf[pos],3);
			pos += 4;
*/
		} else pos++;

		// Altitude measurement units
		// ignore this field and assume what units is meters
		buf += GPS_NextField(&buf[pos]);

		// Geoid-to-ellipsoid separation (Ellipsoid altitude = MSL altitude + Geoid separation)
		if (buf[pos] != ',') {
			// This value can be negative
			GPSData.geoid_separation = atos_char(&buf[pos],&pos) * 1000;
			if (GPSData.geoid_separation >= 0)
				GPSData.geoid_separation += atos_char(&buf[pos],&pos);
			else
				GPSData.geoid_separation -= atos_char(&buf[pos],&pos);
		} else pos++;

		// Time since last DGPS update
		if (buf[pos] != ',') {
			GPSData.dgps_age = atos_char(&buf[pos],&pos);
		} else pos++;

		// DGPS station ID
		if (buf[pos] != ',' && buf[pos] != '*') {
			GPSData.dgps_id = atos_char(&buf[pos],&pos);
		}

		break; // NMEA_GGA
	case NMEA_GSA:
		// $GPGSA - GPS DOP and active satellites

		// Satellite acquisition mode (M = manually forced 2D or 3D, A = automatic switch between 2D and 3D)
		// ignore this field
		pos += GPS_NextField(&buf[pos]);

		// Position mode (1=fix not available, 2=2D fix, 3=3D fix)
		if (buf[pos] != ',') {
			GPSData.fix = buf[pos] - '0';
			pos += 2;
		} else {
			GPSData.fix = 1;
			pos++;
		}

		// IDs of satellites used in position fix (12 fields)
		for (i = 0; i < 12; i++) {
			if (buf[pos] != ',') {
				GPS_sats[i] = atos_len(&buf[pos],2);
				pos += 3;
			} else {
				GPS_sats[i] = 0;
				pos++;
			}
		}

		// PDOP - position dilution
		pos += GPS_ParseFloat(&buf[pos],&GPSData.PDOP);

		// HDOP - horizontal position dilution
		pos += GPS_ParseFloat(&buf[pos],&GPSData.HDOP);

		// VDOP - vertical position dilution
		pos += GPS_ParseFloat(&buf[pos],&GPSData.VDOP);

		break; // NMEA_GSA
	case NMEA_GSV:
		// $GPGSV - GPS Satellites in view

		// Field with "total number of GSV sentences in this cycle"
		// ignore it
		pos += GPS_NextField(&buf[pos]);

		// GSV sentence number
		if (buf[pos] != ',') {
			GSV_msg = atos_len(&buf[pos],1);
			pos += 2;
		} else pos++;

		// Total number of satellites in view
		if (buf[pos] != ',') {
			GSV_sats = atos_len(&buf[pos],2);
			pos += 3;
		} else pos++;

		GPSData.sats_view = GSV_sats;

		// Parse no more than 12 satellites in view
		uint8_t sat_num = (GSV_msg - 1) * 4;
		if (GSV_sats != 0 && sat_num < MAX_SATELLITES_VIEW) {
			// 4 satellites per sentence
			pos += GPS_ParseSatelliteInView(&buf[pos],sat_num++);
			pos += GPS_ParseSatelliteInView(&buf[pos],sat_num++);
			pos += GPS_ParseSatelliteInView(&buf[pos],sat_num++);
			pos += GPS_ParseSatelliteInView(&buf[pos],sat_num++);
		}

		break; // NMEA_GSV
	default:
		// Unknown NMEA sentence
		break;
	}
}

// Initialize GPSData variable
void GPS_InitData(void) {
	uint32_t i;

	memset(&GPSData,0,sizeof(GPSData));
	for (i = 0; i < 12; i++) GPS_sats[i] = 0;
	for (i = 0; i < MAX_SATELLITES_VIEW; i++) {
		memset(&GPS_sats_view[i],0,sizeof(GPS_Satellite_TypeDef));
		GPS_sats_view[i].SNR = 255;
	}
	GPSData.longitude_char = 'X';
	GPSData.latitude_char  = 'X';
	GPSData.mode = 'N';

	GPS_sentences_parsed = 0;
	GPS_sentences_unknown = 0;

	memset(&GPS_msg,0,sizeof(GPS_msg));
}

// Check which satellites in view used in location fix
void GPS_CheckUsedSats(void) {
	uint32_t i,j;

	for (i = 0; i < GPSData.sats_view; i++) {
		GPS_sats_view[i].used = FALSE;
		for (j = 0; GPSData.sats_used; j++) {
			if (GPS_sats[j] == GPS_sats_view[i].PRN) {
				GPS_sats_view[i].used = TRUE;
				break;
			}
		}
	}
}
