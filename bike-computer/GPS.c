#include <stm32l1xx_rcc.h>

#include <uart.h>
#include <wolk.h>
#include <GPS.h>


GPS_Data_TypeDef GPSData;                   // Parsed GPS information
uint8_t GPS_sats[12];                       // IDs of satellites used in position fix
// Information about satellites in view (can be increased if receiver able handle more)
GPS_Satellite_TypeDef GPS_sats_view[MAX_SATELLITES_VIEW];


// Calculate CRC for NMEA command
// input:
//   str - pointer to string with command
//         command must begin with '$' char and end with '*'
// output:
//   checksum of command
uint8_t GPS_CRC(char *str) {
	uint8_t str_CRC = 0;
	uint16_t i = 1;

	while (str[i] != '*') str_CRC ^= str[i++];

	return str_CRC;
}

// Send NMEA command
// input:
//   cmd - pointer to string with command
//         command must begin with '$' char and end with '*'
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
// output:
//   position of sentence beginning or 0 if no sentence found
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

// Parse latitude and longitude
uint16_t GPS_ParseCoordinates(uint8_t *buf) {
	uint16_t pos = 0;

	// Latitude
	if (buf[pos] != ',') {
		GPSData.latitude_degree = atos_len(&buf[pos],2);
		pos += 2;
		GPSData.latitude_seconds  = atos_char(&buf[pos],&pos) * 1000000;
		GPSData.latitude_seconds += atos_char(&buf[pos],&pos);
	} else pos++;
	// Latitude char
	if (buf[pos] != ',') {
		GPSData.latitude_char = buf[pos];
		pos += 2;
	} else {
		GPSData.latitude_char = 'X';
		pos++;
	}

	// Longitude
	if (buf[pos] != ',') {
		GPSData.longitude_degree = atos_len(&buf[pos],3);
		pos += 3;
		GPSData.longitude_seconds  = atos_char(&buf[pos],&pos) * 1000000;
		GPSData.longitude_seconds += atos_char(&buf[pos],&pos);
	} else pos++;
	// Longitude char
	if (buf[pos] != ',') {
		GPSData.longitude_char = buf[pos];
		pos += 2;
	} else {
		GPSData.longitude_char = 'X';
		pos++;
	}

	return pos;
}

// Parse one satellite from $GPGSV sentence
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
	if (buf[pos] != ',') {
		GPS_sats_view[sat_num].SNR = atos_len(&buf[pos],2);
		pos += 3;
	} else {
		GPS_sats_view[sat_num].SNR = 255; // Satellite not tracked
		pos++;
	}

	return pos;
}

void GPS_ParseSentence(uint8_t *buf, NMEASentence_TypeDef Sentence) {
	uint16_t pos = Sentence.start + 4;
	uint8_t i;
	uint8_t GSV_msg;   // GSV sentence number
	uint8_t GSV_sats;  // Total number of satellites in view

	GPSData.time_valid = FALSE;
	GPSData.datetime_valid = FALSE;

	switch (Sentence.type) {
	case NMEA_RMC:
		// $GPRMC - Recommended minimum specific GPS/Transit data

		// Time of fix
		if (buf[pos] != ',') {
			GPSData.fix_time  = atos_len(&buf[pos],2) * 3600;
			GPSData.fix_time += atos_len(&buf[pos + 2],2) * 60;
			GPSData.fix_time += atos_len(&buf[pos + 4],2);
			pos += 11;
		} else pos++;

		// Valid data marker
		if (buf[pos] != ',') {
			GPSData.valid = (buf[pos] == 'A') ? TRUE : FALSE;
			pos += 2;
		} else {
			GPSData.valid = FALSE;
			pos++;
		}

		// Latitude + Longitude
		pos += GPS_ParseCoordinates(&buf[pos]);

		// Horizontal speed (in knots)
		if (buf[pos] != ',') {
			GPSData.speed_k  = atos_char(&buf[pos],&pos) * 100;
			GPSData.speed_k += atos_char(&buf[pos],&pos);
		} else {
			GPSData.speed_k = 0;
			pos++;
		}

		// Course
		if (buf[pos] != ',') {
			GPSData.course  = atos_char(&buf[pos],&pos) * 100;
			GPSData.course += atos_char(&buf[pos],&pos);
		} else {
			GPSData.course = 0;
			pos++;
		}

		// Date of fix
		if (buf[pos] != ',') {
			GPSData.fix_date  = atos_len(&buf[pos],2) * 1000000;
			GPSData.fix_date += atos_len(&buf[pos + 2],2) * 100000;
			GPSData.fix_date += atos_len(&buf[pos + 4],2) + 2000;
		};
		break; // NMEA_RMC
	case NMEA_GLL:
		// $GPGLL - Geographic position, latitude / longitude

		// Latitude + Longitude
		pos += GPS_ParseCoordinates(&buf[pos]);

		// Time of fix
		if (buf[pos] != ',') {
			GPSData.fix_time  = atos_len(&buf[pos],2) * 3600;
			GPSData.fix_time += atos_len(&buf[pos + 2],2) * 60;
			GPSData.fix_time += atos_len(&buf[pos + 4],2);
			pos += 11;
		} else pos++;

		// Valid data marker
		if (buf[pos] != ',') {
			GPSData.valid = (buf[pos] == 'A') ? TRUE : FALSE;
			pos += 2;
		} else {
			GPSData.valid = FALSE;
			pos++;
		}

		// Mode indicator
		if (buf[pos] != ',') GPSData.mode = buf[pos]; else GPSData.mode = 'N';

		break; // NMEA_GLL
	case NMEA_ZDA:
		// $GPZDA - Date & Time

		// Time
		if (buf[pos] != ',') {
			GPSData.time  = atos_len(&buf[pos],2) * 3600;
			GPSData.time += atos_len(&buf[pos + 2],2) * 60;
			GPSData.time += atos_len(&buf[pos + 4],2);
			GPSData.time_valid = TRUE;
			pos += 11;
		} else {
			GPSData.time = 0;
			pos++;
		}

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
		} else GPSData.date += 2014;

		if (GPSData.time != 0 && GPSData.date != 01012014) GPSData.datetime_valid = TRUE;

		// Local time zone offset
		// ..... (not supported by EB-500)

		break; // NMEA_ZDA
	case NMEA_VTG:
		// $GPVTG - Course over ground and ground speed

		// Course (heading relative to true north)
		if (buf[pos] != ',') {
			GPSData.course  = atos_char(&buf[pos],&pos) * 100;
			GPSData.course += atos_len(&buf[pos],2);
			pos += 9;
		} else pos += 6;
		// Skip field with 'T' letter - "track made good is relative to true north"
		// Skip field with course relative to magnetic north (not supported)
		// Skip field with 'N' letter - "track made good is relative to magnetic north"

		// Speed in knots
		if (buf[pos] != ',') {
			GPSData.speed_k  = atos_char(&buf[pos],&pos) * 100;
			GPSData.speed_k += atos_len(&buf[pos],2);
			pos += 5;
		} else {
			GPSData.speed_k = 0;
			pos += 4;
		}
		// Skip field with 'N' - speed over ground measured in knots

		// Speed in km/h
		if (buf[pos] != ',') {
			GPSData.speed  = atos_char(&buf[pos],&pos) * 100;
			GPSData.speed += atos_len(&buf[pos],2);
			pos += 5;
		}
		// Skip field with 'K' - speed over ground measured in km/h

		// Mode indicator
		if (buf[pos] != ',') GPSData.mode = buf[pos]; else GPSData.mode = 'N';

		break; // NMEA_VTG
	case NMEA_GGA:
		// $GPGGA - GPS fixed data

		// Time
		if (buf[pos] != ',') {
			GPSData.time  = atos_len(&buf[pos],2) * 3600;
			GPSData.time += atos_len(&buf[pos + 2],2) * 60;
			GPSData.time += atos_len(&buf[pos + 4],2);
			GPSData.time_valid = TRUE;
			pos += 11;
		} else {
			GPSData.time = 0;
			GPSData.time_valid = FALSE;
			pos++;
		}

		// Latitude + Longitude
		pos += GPS_ParseCoordinates(&buf[pos]);

		// Position fix indicator
		if (buf[pos] != ',') {
			GPSData.fix_quality = buf[pos] - '0';
			pos += 2;
		} else {
			GPSData.fix_quality = 0;
			pos++;
		}

		// Satellites used
		if (buf[pos] != ',') {
			GPSData.sats_used = atos_char(&buf[pos],&pos);
		} else {
			GPSData.sats_used = 0;
			pos++;
		}

		// HDOP - horizontal dilution of precision
		if (buf[pos] != ',') {
			GPSData.HDOP  = atos_char(&buf[pos],&pos) * 100;
			GPSData.HDOP += atos_char(&buf[pos],&pos);
		} else {
			GPSData.HDOP = 0;
			pos++;
		}

		// MSL Altitude (mean-sea-level)
		if (buf[pos] != ',') {
			// Only integer part, fractional is useless
			GPSData.altitude  = atos_char(&buf[pos],&pos);
			pos += 4;
/*
			GPSData.altitude  = atos_char(&buf[pos],&pos) * 1000;
			// This value can be negative
			if (GPSData.altitude >= 0)
				GPSData.altitude += atos_len(&buf[pos],3);
			else
				GPSData.altitude -= atos_len(&buf[pos],3);
			pos += 4;
*/
		} else {
			GPSData.altitude = 0;
			pos++;
		}
		// Altitude measurement units
		if (buf[pos] != ',') pos += 2; else pos++; // skip this field

		// Geoid-to-ellipsoid separation (Ellipsoid altitude = MSL altitude + Geoid separation)
		if (buf[pos] != ',') {
			// This value can be negative
			GPSData.geoid_separation = atos_char(&buf[pos],&pos) * 1000;
			if (GPSData.geoid_separation >= 0)
				GPSData.geoid_separation += atos_char(&buf[pos],&pos);
			else
				GPSData.geoid_separation -= atos_char(&buf[pos],&pos);
		} else {
			GPSData.geoid_separation = 0;
			pos++;
		}

		break; // NMEA_GGA
	case NMEA_GSA:
		// $GPGSA - GPS DOP and active satellites

		// Satellite acquisition mode (M = manually force 2D or 3D, A = automatic swich between 2D and 3D)
		if (buf[pos] != ',') pos += 2; else pos++; // Skip this field

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
		if (buf[pos] != ',') {
			GPSData.PDOP  = atos_char(&buf[pos],&pos) * 100;
			GPSData.PDOP += atos_len(&buf[pos],2);
			pos += 3;
		} else {
			GPSData.PDOP = 0;
			pos++;
		}

		// HDOP - horizontal position dilution
		if (buf[pos] != ',') {
			GPSData.HDOP  = atos_char(&buf[pos],&pos) * 100;
			GPSData.HDOP += atos_len(&buf[pos],2);
			pos += 3;
		} else {
			GPSData.HDOP = 0;
			pos++;
		}

		// VDOP - vertical position dilution
		if (buf[pos] != ',') {
			GPSData.VDOP  = atos_char(&buf[pos],&pos) * 100;
			GPSData.VDOP += atos_len(&buf[pos],2);
			pos += 3;
		} else {
			GPSData.VDOP = 0;
			pos++;
		}

		break; // NMEA_GSA
	case NMEA_GSV:
		// $GPGSV - GPS Satellites in view

		// Skip total number of GSV sentences in this cycle field
		pos += 2;

		// GSV sentence number
		if (buf[pos] != ',') {
			GSV_msg = atos_len(&buf[pos],1);
			pos += 2;
		} else {
			GSV_msg = 0; // This should not happen
			pos++;
		}

		// Total number of satellites in view
		if (buf[pos] != ',') {
			GSV_sats = atos_len(&buf[pos],2);
			pos += 3;
		} else {
			GSV_sats = 0;
			pos++;
		}
		GPSData.sats_view = GSV_sats;

		// Parse no more than 12 satellites in view
		uint8_t sat_num = (GSV_msg - 1) * 4;
		if (GSV_sats != 0 && sat_num < MAX_SATELLITES_VIEW) {
			// 4 satellites in one sentence
			pos += GPS_ParseSatelliteInView(&buf[pos],sat_num++);
			pos += GPS_ParseSatelliteInView(&buf[pos],sat_num++);
			pos += GPS_ParseSatelliteInView(&buf[pos],sat_num++);
			pos += GPS_ParseSatelliteInView(&buf[pos],sat_num++);
		}

		break; // NMEA_GSV
	default:
		// Some banana
		break;
	}
}
