#include <stm32l1xx_rcc.h>
#include <string.h> // For memset

#include "uart.h"
#include "GPS.h"




// FIXME: remove this after debugging
// Debug output
#include <stdio.h>
#include "VCP.h"
#pragma GCC diagnostic ignored "-Wformat"




GPS_Data_TypeDef GPSData;                   // Parsed GPS information
GPS_PMTK_TypeDef GPS_PMTK;                  // PMTK messages result
bool GPS_new_data;                          // TRUE if a new GPS packet received
bool GPS_parsed;                            // TRUE if GPS data was parsed
uint16_t GPS_buf_cntr;                      // Number of actual bytes in GPS buffer
uint8_t GPS_sentences_parsed;               // Parsed NMEA sentences counter
uint8_t GPS_sentences_unknown;              // Unsupported NMEA sentences counter
uint8_t GPS_sentences_invalid;              // Invalid NMEA sentences counter
uint8_t GPS_buf[GPS_BUFFER_SIZE];           // Buffer with data from GPS
uint8_t GPS_sats[12];                       // IDs of satellites used in position fix
// Information about satellites in view (can be increased if receiver is able to handle more)
GPS_Satellite_TypeDef GPS_sats_view[MAX_SATELLITES_VIEW];




// Parse a number in data buffer with specified length
// input:
//   buf - pointer to the pointer to the data buffer
//   len - number of digits to parse
// return: parsed value
// note: only positive values can be parsed
// note: buf pointer will point to the next byte after parsed number
uint32_t atoi_len(uint8_t **buf, uint8_t len) {
	uint32_t value = 0;

	do {
		if ((**buf < '0') || (**buf > '9')) return 0; // character is not a digit -> error
		value *= 10;
		value += *((*buf)++) - '0';
	} while (--len);

	return value;
}

// Parse a (potentially negative) number in data buffer with unknown length
// input:
//   buf - pointer to the pointer to the data buffer
// return: parsed value
// note: buf will point to the next byte after parsed value or to the next term if
//       a byte after parsed number is ','
// note: the function will parse the buffer contents until it hits a non-digit char,
//       therefore in case of long number the int32_t value can be overflowed
int32_t atoi_chr(uint8_t **buf) {
	int32_t neg = 1;
	int32_t value = 0;

	// Check if a number is negative
	if (**buf == '-') {
		neg = -1;
		(*buf)++;
	}

	// Parse buffer until first non-digit character, no overflow check for 'value',
	// therefore 32-bit value will overflow if number length is more than 10 digits
	while ((**buf > '0' - 1) && (**buf < '9' + 1)) {
		value *= 10;
		value += *((*buf)++) - '0';
	}

	// Shift the pointer to the next term if it points to the "," symbol
	if (**buf == ',') (*buf)++;

	return neg * value;
}

// Convert two HEX characters into 8-bit binary
// input:
//   buf - pointer to the data buffer
// return: binary value
uint8_t atoi_hex(uint8_t *buf) {
	uint8_t result,tmp;

	// If byte contains letter, it will be converted to lowercase, in case of digit the byte will remain intact
	tmp = (*buf++ | 0x20);
	// Add 0xD0 is same as subtract 0x30, same is for 0xA9
	result = (tmp <= '9') ? (tmp + 0xD0) : (tmp + 0xA9);
	// This is high nibble
	result <<= 4;
	// Repeat the same for low nibble
	tmp = (*buf | 0x20);
	result += (tmp <= '9') ? (tmp + 0xD0) : (tmp + 0xA9);

	return result;
}

// Parse float value from a GPS sentence
// input:
//   buf - pointer to the pointer to the data buffer
// return: parsed value
// note: buf will point to the next byte after parsed value or to the next term if
//       a byte after parsed number is ','
// note: float will be represented as integer, e.g. string '1234.567' will be parsed to 1234567
int32_t atoi_flt(uint8_t **buf) {
	int32_t value;
	int32_t neg = 1;

	// Parse integer part of a number
	value = atoi_chr(buf);
	if (value < 0) {
		value *= -1;
		neg = -1;
	}

	// Parse fractional part if it present
	if (*((*buf)++) == '.') {
		while ((**buf > '0' - 1) && (**buf < '9' + 1)) {
			value *= 10;
			value += *((*buf)++) - '0';
		}
	}

	// Shift the pointer to the next term if it points to the "," symbol
	if (**buf == ',') (*buf)++;

	return neg * value;
}

// Calculate the CRC value of NMEA sentence
// input:
//   str - pointer to the buffer containing sentence
// return: checksum of sentence in HEX format
// note: input sentence should begin with a '$' and end with '*' or a zero byte
uint8_t GPS_CalcCRC(char *str) {
	uint8_t result = 0;

	if (*str++ == '$') while ((*str != '*') && (*str != '\0')) result ^= *str++;

	return result;
}

// Send NMEA sentence
// input:
//   cmd - pointer to the buffer containing sentence
// note: sentence must begin with '$' char and end with '*'
void GPS_Send(char *cmd) {
	uint8_t cmd_CRC;

	cmd_CRC = GPS_CalcCRC(cmd);
	UART_SendStr(GPS_USART_PORT,cmd);
	UART_SendChar(GPS_USART_PORT,HEX_CHARS[cmd_CRC >> 4]);
	UART_SendChar(GPS_USART_PORT,HEX_CHARS[cmd_CRC & 0x0f]);
	UART_SendChar(GPS_USART_PORT,'\r');
	UART_SendChar(GPS_USART_PORT,'\n');

	// Wait for an USART transmit complete
	while (!(GPS_USART_PORT->SR & USART_SR_TC));
}

// Find end of the NMEA sentence
// input:
//   buf - pointer to a byte in the data buffer from where search will start
//   buf_end - pointer to the last byte where search must stop
// return: pointer to the last byte of NMEA sentence
// note: the result pointer will point to the last byte of a sentence or to the end
//       of the data buffer in case of sentence is incomplete
uint8_t * GPS_FindTail(uint8_t *buf, uint8_t *buf_end) {
	do {
		if (*buf++ == '\r') {
			if (*buf == '\n') {
				break;
			}
		}
	} while (buf < buf_end);

	return buf;
}

// Find next term of NMEA sentence
// input:
//   buf - pointer to pointer to the data buffer
// note: buf pointer will point to the next term or to the sentence end ('*' character)
void GPS_NextTerm(uint8_t **buf) {
	// Find next term or sentence end
	while ((**buf != ',') && (**buf != '*')) (*buf)++;

	// Point to the first character of next term
	if (**buf == ',') (*buf)++;
}

// Find NMEA sentence in buffer
// input:
//   msg - pointer to the NMEASentence structure
//   buf_start - pointer to the data buffer where search will start
//   buf_end - pointer to the end of the data buffer
// note: function modifies the msg variable
void GPS_FindSentence(NMEASentence_TypeDef *Sentence, uint8_t *buf_start, uint8_t *buf_end) {
	uint32_t *ptr = (uint32_t *)buf_start;
	uint32_t hdr;

	// Populate the sentence structure with initial values
	Sentence->start = (uint8_t *)ptr;
	Sentence->data  = Sentence->start;
	Sentence->end   = buf_end;
	Sentence->type   = NMEA_NOTFOUND;

	// Further parsing does not make sense if there is no space left in buffer for NMEA sentence
	if (buf_end - (uint8_t *)ptr < 10) return;

	do {
		hdr = *ptr << 8;
		if (hdr == 0x50472400) {
			// $GPxxx sentence - GPS

			// Point to the first byte of sentence
			Sentence->start = (uint8_t *)ptr;

			// Point to the 4th byte of sentence
			ptr = (uint32_t *)((uint8_t *)ptr + 3);
			hdr = *ptr << 8;

			// Point to the first term of sentence
			Sentence->data = (uint8_t *)ptr + 4;

			// Find sentence tail
			Sentence->end = GPS_FindTail(Sentence->start,buf_end);

			// Determine a sentence type
			switch (hdr) {
				case 0x4c4c4700:
					Sentence->type = NMEA_GPGLL;
					return;
				case 0x434d5200:
					Sentence->type = NMEA_GPRMC;
					return;
				case 0x47545600:
					Sentence->type = NMEA_GPVTG;
					return;
				case 0x41474700:
					Sentence->type = NMEA_GPGGA;
					return;
				case 0x41534700:
					Sentence->type = NMEA_GPGSA;
					return;
				case 0x56534700:
					Sentence->type = NMEA_GPGSV;
					return;
				case 0x41445a00:
					Sentence->type = NMEA_GPZDA;
					return;
				default:
					// Unsupported GPS sentence
					Sentence->type = NMEA_UNKNOWN;
					return;
			}
		} else if (hdr == 0x4e472400) {
			// $GNxxx sentence - GLONASS + GPS

			// Unsupported GLONASS + GPS sentence
			Sentence->type = NMEA_UNKNOWN;

			return;
		} else if (hdr == 0x4c472400) {
			// $GLxxx sentence - GLONASS

			// Unsupported GLONASS sentence
			Sentence->type = NMEA_UNKNOWN;

			return;
		} else if (hdr == 0x4d502400) {
			// $PMTKxxx sentence

			// Point to the first byte of sentence
			Sentence->start = (uint8_t *)ptr;

			// Point to the 5th byte of sentence
			ptr = (uint32_t *)(((uint8_t *)ptr) + 4);
			hdr = *ptr;

			// Point to the first term of sentence
			Sentence->data = (uint8_t *)ptr + 5;

			// Find sentence tail
			Sentence->end = GPS_FindTail(Sentence->start,buf_end);

			switch (hdr) {
				case 0x3130304b:
					Sentence->type = NMEA_PMTK001;
					return;
				case 0x3031304b:
					Sentence->type = NMEA_PMTK010;
					return;
				case 0x3131304b:
					Sentence->type = NMEA_PMTK011;
					return;
				default:
					// Unsupported MTK sentence
					Sentence->type = NMEA_UNKNOWN;
					return;
			}
		}

		// Proceed to next byte in the buffer
		ptr = (uint32_t *)((uint8_t *)ptr + 1);
	} while (ptr < (uint32_t *)buf_end);
}

// Parse latitude or longitude term
// input:
//   buf - pointer to the pointer to the data buffer
//   deg_len - length of the degrees value (2 for latitude, 3 for longitude)
//   value - pointer to the coordinate variable
//   char_value - pointer to the coordinate character variable
// note: buf will point to the next term
void GPS_ParseLatLon(uint8_t **buf, uint8_t deg_len, int32_t *value, uint8_t *char_value) {
	uint32_t f_deg = 0; // coordinate degrees fractional part
	uint8_t  f_len = 0; // fractional part length

	// Coordinate
	if (**buf != ',') {
		// Degrees
		*value = atoi_len(buf,deg_len) * 60;

		// Minutes integer part
		*value += atoi_len(buf,2);

		// Skip decimal dot
		(*buf)++;

		// Minutes fractional part, it length deends on GPS receiver
		// Parse no more than 4 digits (~22cm precision?)
		while (f_len++ < 4) {
			f_deg *= 10;
			f_deg += *((*buf)++) - '0';
			if ((**buf < '0') || (**buf > '9')) break; // character is not a digit -> end of number
		}

		// If a fractional part consists less than 4 digits, need to adjust a
		// fractional part and scale factor up to 4 digits
		if (f_len < 4) {
			while (f_len++ < 4) f_deg *= 10;
		}

		// If a fractional part consists more than 4 digits, need to scan up to the end of the number
		if (**buf != ',') {
			while ((**buf > '0' - 1) && (**buf < '9' + 1)) (*buf)++;
		}

		// Point to next NMEA term
		(*buf)++;

		// Calculate a 'micro-degrees' value
		// value of '10000' - the scaling factor, deending on the length of the fractional part
		*value = (((*value * 10000) + f_deg) * 10) / 6;
	} else {
		// Point to next NMEA term
		(*buf)++;

		// No coordinates in sentence, bail out
		*value = 0;
	}

	// Coordinate character
	if (**buf != ',') {
		*char_value = **buf;
		*buf += 2;

		// In case of 'S' latitude or 'W' longitude the degrees value must be negative
		if ((*char_value == 'W') || (*char_value == 'S')) *value *= -1;
	} else {
		*char_value = 'X';
		(*buf)++;
	}
}

// Parse one satellite from $GPGSV sentence
// input:
//   buf - pointer to the pointer to the data buffer
//   satellite - pointer to the GPS_Satellite_TypeDef structure
// note: buf will point to the next term of sentence
void GPS_ParseSatInView(uint8_t **buf, GPS_Satellite_TypeDef *satellite) {
	// Satellite PRN number
	satellite->PRN = (**buf != ',') ? atoi_len(buf,2) : 0;
	(*buf)++;

	// Satellite elevation
	satellite->elevation = (**buf != ',') ? atoi_len(buf,2) : 0;
	(*buf)++;

	// Satellite azimuth
	satellite->azimuth = (**buf != ',') ? atoi_len(buf,3) : 0;
	(*buf)++;

	// Satellite SNR (255 = satellite is not tracked)
	satellite->SNR = (**buf != ',') ? atoi_len(buf,2) : 255;
	(*buf)++;

	// This must be set after all of NMEA sentences parsed (call GPS_CheckUsedSats)
	satellite->used = FALSE;
}

// Parse time from NMEA sentence (format: HHMMSS.XXX)
// input:
//   buf - pointer to the data buffer
//   time - pointer to variable where time will be stored
// note: the buf pointer will point to the next term
void GPS_ParseTime(uint8_t **buf, NMEATime *time) {
	if (**buf != ',') {
		// Hours
		time->Hours   = atoi_len(buf,2);

		// Minutes
		time->Minutes = atoi_len(buf,2);

		// Seconds
		time->Seconds = atoi_len(buf,2);

		// ... Milliseconds are ignored
		GPS_NextTerm(buf);
	} else {
		(*buf)++;
	}
}

// Parse NMEA sentence
// input:
//   buf - pointer to the data buffer
//   Sentence - pointer to the structure with NMEA sentence parameters
void GPS_ParseSentence(NMEASentence_TypeDef *sentence) {
	uint8_t *ptr = sentence->data;
	uint32_t tmp;

	switch (sentence->type) {

		// GPS sentences
		case NMEA_GPRMC:
			// $GPRMC - Recommended minimum specific GPS/Transit data

			// Time of fix
			GPS_ParseTime(&ptr,&GPSData.fix_time);

			// Valid data marker (A=active or V=void)
			GPSData.valid = FALSE;
			if (*ptr != ',') {
				if (*ptr++ == 'A') GPSData.valid = TRUE;
			}
			ptr++;

			// Latitude
			GPS_ParseLatLon(&ptr,2,&GPSData.latitude,&GPSData.latitude_char);

			// Longitude
			GPS_ParseLatLon(&ptr,3,&GPSData.longitude,&GPSData.longitude_char);

			// Horizontal speed (in knots)
			GPSData.speed_k = atoi_flt(&ptr);
			// Convert speed in knots to speed in km/h
			if ((GPSData.speed == 0) && (GPSData.speed_k != 0)) GPSData.speed = (GPSData.speed_k * 1852) / 1000;

			// Course
			GPSData.course = atoi_flt(&ptr);

			// Date of fix
			if (*ptr != ',') {
				// Day
				GPSData.fix_date.Date  = atoi_len(&ptr,2);

				// Month
				GPSData.fix_date.Month = atoi_len(&ptr,2);

				// Year (two digits)
				GPSData.fix_date.Year  = atoi_len(&ptr,2);
				// Some receivers report date year as 70 or 80 when their internal clock has
				// not yet synchronized with the satellites
				// Yep, this trick wouldn't work after 2069 year ^_^
				if (GPSData.fix_date.Year > 69) {
					// Assume what year is less than 2000
					GPSData.fix_date.Year += 1900;
				} else {
					// Assume what year is greater than 2000
					// Copy fix_date to date and fix_time to time in case of the $GPZDA sentence are disabled
					GPSData.fix_date.Year += 2000;
					GPSData.date = GPSData.fix_date;
					GPSData.time = GPSData.fix_time;
					GPSData.datetime_valid = TRUE;
				}
			}
			ptr++;

			// Magnetic variation
			// ignore this term (mostly not supported by GPS receivers)
			GPS_NextTerm(&ptr);

			// Magnetic variation direction
			// ignore this term (mostly not supported by GPS receivers)
			GPS_NextTerm(&ptr);

			// Mode indicator (NMEA 0183 v2.3 or never)
			if ((*ptr != ',') && (*ptr != '*')) GPSData.mode = *ptr;

			break; // NMEA_GPRMC
		case NMEA_GPGLL:
			// $GPGLL - Geographic position, latitude / longitude

			// Latitude
			GPS_ParseLatLon(&ptr,2,&GPSData.latitude,&GPSData.latitude_char);

			// Longitude
			GPS_ParseLatLon(&ptr,3,&GPSData.longitude,&GPSData.longitude_char);

			// Time of fix
			GPS_ParseTime(&ptr,&GPSData.fix_time);

			// Valid data marker
			GPSData.valid = FALSE;
			if (*ptr != ',') {
				if (*ptr++ == 'A') GPSData.valid = TRUE;
			}

			// Mode indicator (NMEA 0183 v2.3 or never)
			if ((*ptr != ',') && (*ptr != '*')) GPSData.mode = *ptr;

			break; // NMEA_GPGLL
		case NMEA_GPZDA:
			// $GPZDA - Date & Time

			// Time
			GPS_ParseTime(&ptr,&GPSData.time);

			// Date: day
			GPSData.date.Date  = (*ptr != ',') ? atoi_len(&ptr,2) : 01;
			ptr++;

			// Date: month
			GPSData.date.Month = (*ptr != ',') ? atoi_len(&ptr,2) : 01;
			ptr++;

			// Date: year
			GPSData.date.Year  = (*ptr != ',') ? atoi_len(&ptr,4) : 1980;
			ptr++;

			// Local time zone offset term
			// sad but true: this feature mostly not supported by GPS receivers

			// Check for year, if it less than 2014, the date from the GPS receiver is not valid
			GPSData.datetime_valid = (GPSData.date.Year > 2013);

			break; // NMEA_GPZDA
		case NMEA_GPVTG:
			// $GPVTG - Course over ground and ground speed

			// Course (heading relative to true north)
			GPSData.course = atoi_flt(&ptr);

			// Skip term: 'T' letter - "track made good is relative to true north"
			GPS_NextTerm(&ptr);

			// Skip term: course relative to magnetic north
			GPS_NextTerm(&ptr);

			// Skip term: 'M' letter - "track made good is relative to magnetic north"
			GPS_NextTerm(&ptr);

			// Speed over ground in knots
			GPSData.speed_k = atoi_flt(&ptr);
			// Convert speed in knots to speed in km/h
			if ((GPSData.speed == 0) && (GPSData.speed_k != 0)) GPSData.speed = (GPSData.speed_k * 1852) / 1000;

			// Skip term: 'N' letter - speed over ground measured in knots
			GPS_NextTerm(&ptr);

			// Speed over ground in km/h
			GPSData.speed = atoi_flt(&ptr);

			// Skip term: 'K' letter - speed over ground measured in km/h
			GPS_NextTerm(&ptr);

			// Mode indicator (NMEA 0183 v2.3 or never)
			if ((*ptr != ',') && (*ptr != '*')) GPSData.mode = *ptr;

			break; // NMEA_GPVTG
		case NMEA_GPGGA:
			// $GPGGA - GPS fix data

			// Time
			GPS_ParseTime(&ptr,&GPSData.fix_time);

			// Latitude
			GPS_ParseLatLon(&ptr,2,&GPSData.latitude,&GPSData.latitude_char);

			// Longitude
			GPS_ParseLatLon(&ptr,3,&GPSData.longitude,&GPSData.longitude_char);

			// Position fix indicator
			GPSData.fix_quality = (*ptr != ',') ? *ptr++ - '0' : 0;
			ptr++;

			// Satellites used
			GPSData.sats_used = atoi_chr(&ptr);

			// HDOP - horizontal dilution of precision
			GPSData.HDOP = atoi_flt(&ptr);

			// MSL (mean-sea-level) altitude, can be negative
			if (*ptr != ',') {
				// Get only integer part, fractional is useless
				GPSData.altitude = atoi_chr(&ptr);
				GPS_NextTerm(&ptr);
			} else ptr++;

			// MSL measurement units, ignore this term and assume what units is meters
			GPS_NextTerm(&ptr);

			// Geoid-to-ellipsoid separation (ellipsoid altitude = MSL altitude + geoid separation)
			// Value can be negative
			GPSData.geoid_separation = atoi_flt(&ptr);

			// Geoid-to-ellipsoid separation measurement units, ignore this term and assume what units is meters
			GPS_NextTerm(&ptr);

			// Time since last DGPS update
			GPSData.dgps_age = atoi_chr(&ptr);

			// DGPS station ID
			if ((*ptr != ',') && (*ptr != '*')) GPSData.dgps_id = atoi_chr(&ptr);

			break; // NMEA_GPGGA
		case NMEA_GPGSA:
			// $GPGSA - GPS DOP and active satellites

			// Satellite acquisition mode (M = manually forced 2D or 3D, A = automatic switch between 2D and 3D)
			// Skip this term
			GPS_NextTerm(&ptr);

			// Position mode (1 = fix not available, 2 = 2D fix, 3 = 3D fix)
			GPSData.fix = (*ptr != ',') ? *ptr++ - '0' : 1;
			ptr++;

			// IDs of satellites used in position fix (12 terms per sentence)
			tmp = 0;
			do {
				GPS_sats[tmp] = (*ptr != ',') ? atoi_len(&ptr,2) : 0;
				ptr++;
			} while (++tmp < 12);

			// PDOP - position dilution, in theory this thing must be equal to SQRT(HDOP^2 + VDOP^)
			GPSData.PDOP = atoi_flt(&ptr);

			// HDOP - horizontal position dilution
			GPSData.HDOP = atoi_flt(&ptr);

			// VDOP - vertical position dilution
			GPSData.VDOP = atoi_flt(&ptr);

			// Calculate some human-friendly value for GPS accuracy
			GPSData.accuracy = GPSData.PDOP * GPS_DOP_FACTOR;

			break; // NMEA_GPGSA
		case NMEA_GPGSV:
			// $GPGSV - GPS Satellites in view

			// Total number of GSV sentences in this cycle, skip this term
			GPS_NextTerm(&ptr);

			// GSV sentence number
			tmp = (*ptr != ',') ? (*ptr++ - '0' - 1) << 2 : 0;
			ptr++;

			// Total number of satellites in view
			GPSData.sats_view = (*ptr != ',') ? atoi_len(&ptr,2) : 0;
			ptr++;

			// Parse no more than 12 satellites in view
			if (GPSData.sats_view && (tmp < MAX_SATELLITES_VIEW)) {
				if (*(ptr - 1) != '*') GPS_ParseSatInView(&ptr,&GPS_sats_view[tmp++]);
				if (*(ptr - 1) != '*') GPS_ParseSatInView(&ptr,&GPS_sats_view[tmp++]);
				if (*(ptr - 1) != '*') GPS_ParseSatInView(&ptr,&GPS_sats_view[tmp++]);
				if (*(ptr - 1) != '*') GPS_ParseSatInView(&ptr,&GPS_sats_view[tmp++]);
			}

			break; // NMEA_GPGSV

		// MTK sentences
		case NMEA_PMTK001:
			// $PMTK001 - PMTK_ACK

			GPS_PMTK.PMTK001_CMD = atoi_chr(&ptr);
			GPS_PMTK.PMTK001_FLAG = (*ptr != ',') ? *ptr - '0' : 0;

			break; // NMEA_PMTK001
		case NMEA_PMTK010:
			// $PMTK010 - PMTK_SYS_MSG

			GPS_PMTK.PMTK010 = atoi_chr(&ptr);

			break; // NMEA_PMTK010
		case NMEA_PMTK011:
			// $PMTK011 - PMTK_BOOT

			// Check if term value is 'MTKGPS'
			if (*ptr != ',') {
				GPS_PMTK.PMTK_BOOT = FALSE;
				tmp = *(uint32_t *)ptr;
				if (tmp == 0x474b544d) {
					ptr += 4;
					tmp = *(uint32_t *)ptr << 16;
					GPS_PMTK.PMTK_BOOT = (tmp == 0x53500000);
				}
			}

			break; // NMEA_PMTK011

		// Unsupported NMEA sentence
		default:
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

	GPS_sentences_parsed  = 0;
	GPS_sentences_unknown = 0;
	GPS_sentences_invalid = 0;

	memset(&GPS_PMTK,0,sizeof(GPS_PMTK));
}

// Check which satellites in view is used in location fix
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

// Parse the GPS data buffer
// input:
//   buf - pointer to the buffer with GPS data
//   length - length of the data buffer
void GPS_ParseBuf(uint8_t *buf, uint32_t length) {
	NMEASentence_TypeDef sentence;
	uint8_t *buf_end = buf + length; // Pointer to the last significant byte in GPS buffer

	// Clear previously parsed GPS data
	GPS_InitData();

	// Find all sentences and parse known
	while (buf < buf_end) {
		GPS_FindSentence(&sentence,buf,buf_end);
		if (sentence.type != NMEA_NOTFOUND) {
			// Validate a sentence by CRC check
			if (atoi_hex(sentence.end - 3) == GPS_CalcCRC((char *)sentence.start)) {
				// Sentence validation passed
				if (sentence.type != NMEA_UNKNOWN) {
					// Supported sentence found -> parse it
					GPS_ParseSentence(&sentence);
					GPS_sentences_parsed++;
				} else {
					// Unsupported sentence found -> skip it
					GPS_sentences_unknown++;
				}
			} else {
				// Sentence validation failed
				sentence.type = NMEA_INVALID;
				GPS_sentences_invalid++;
			}
		}

		// Move the pointer to the byte following the last parsed
		buf = sentence.end + 1;
	}

	// Reset the new GPS data flag (data were parsed)
	GPS_new_data = FALSE;

	// Reset the GPS buffer counter
	GPS_buf_cntr = 0;

	// Set flag indicating what GPS data was parsed
	GPS_parsed = TRUE;
}

// Initialize the GPS module
void GPS_Init(void) {
	uint32_t wait;
	uint32_t baud = 9600;
	uint32_t BC; // FIXME: debug remove this
	uint32_t trials = 5;

	// Reset all GPS related variables
	GPS_InitData();

	// After first power-on with no backup the Quectel L80 baud rate will be 9600bps
	// After power-on with backup the baud rate remains same as it was before power-off
	// What the hell to do with this shit?

	// Set USART baud rate to 9600pbs and wait some time for a NMEA sentence
	// It must be "$PMTK011,MTKGPS" followed by "$PMTK010,001"
	// In case of timeout we decide what there are no GPS receiver?
	UARTx_SetSpeed(GPS_USART_PORT,baud);

	while (trials--) {
		wait = 0x00300000; // Magic number, about 1.5s on 32MHz CPU
		while (!GPS_new_data && --wait);
		if (wait) {
			// No timeout, USART IDLE frame detected

			// FIXME: Output data for debug purposes
			BC = GPS_buf_cntr;
			VCP_SendBuf(GPS_buf,GPS_buf_cntr);

			// Parse data from GPS receiver
			GPS_ParseBuf(GPS_buf,GPS_buf_cntr);

			// FIXME: Output data for debug purposes
			printf("\r\nPMTK: BOOT=%s PMTK010=%u CMD=%u FLAG=%u | B=%u SC=%u/%u/%u [BR=%u] %X\r\n",
					(GPS_PMTK.PMTK_BOOT) ? "TRUE" : "FALSE",
					GPS_PMTK.PMTK010,
					GPS_PMTK.PMTK001_CMD,
					GPS_PMTK.PMTK001_FLAG,
					BC,
					GPS_sentences_parsed,
					GPS_sentences_unknown,
					GPS_sentences_invalid,
					baud,
					wait
					);

			if (baud == 9600) {
				if (GPS_sentences_parsed) {
					// Known NMEA sentences were detected on 9600 baud rate
					// Send command to the GPS receiver to switch to higher baud rate
					// And do this only after GPS receiver finish the boot sequence
					if ((!GPS_PMTK.PMTK_BOOT) && (GPS_PMTK.PMTK010 != 2)) {
						GPS_Send(PMTK_SET_NMEA_BAUDRATE_38400);
						baud = 38400;
						UARTx_SetSpeed(GPS_USART_PORT,baud);
					}
				} else {
					// Known NMEA sentences were not detected, set USART baud rate to 38400 and try again
					baud = 38400;
					UARTx_SetSpeed(GPS_USART_PORT,baud);
				}
			}

			if (GPS_sentences_parsed && (baud == 38400)) {
				// Known NMEA sentences were detected on 38400 baud rate, thats's all
				break;
			}
		} else {
			// GPS timeout
			baud = 0;
			break;
		}

		printf("---------------------------------------------\r\n");
	}

	// There is no result after several trials, count this as no GPS present
	if (!trials) baud = 0;

	if (baud) {
		// FIXME: here must be a little delay, before sending PMTK commands!
		printf("--->>> It's time to configure <<<---\r\n");

		// Looks like an initialization completed, configure the GPS receiver
		GPS_Send(PMTK_SET_NMEA_OUTPUT_EFFICIENT); // Efficient sentences only
		GPS_Send(PMTK_SET_AIC_ENABLED); // Enable AIC (enabled by default)
		GPS_Send(PMTK_API_SET_STATIC_NAV_THD_OFF); // Disable speed threshold
		GPS_Send(PMTK_EASY_ENABLE); // Enable EASY (for MT3339)
		GPS_Send(PMTK_SET_PERIODIC_MODE_NORMAL); // Disable periodic mode

		// FIXME: just for debug, remove this
		trials = 4;
		while (trials--) {
			wait = 0x00300000; // Magic number, about 1.5s on 32MHz CPU
			while (!GPS_new_data && --wait);
			if (wait) {
				// No timeout, USART IDLE frame detected

				// Output data for debug purposes
				BC = GPS_buf_cntr;
				VCP_SendBuf(GPS_buf,GPS_buf_cntr);

				// Parse data from GPS receiver
				GPS_ParseBuf(GPS_buf,GPS_buf_cntr);

				// Output data for debug purposes
				printf("\r\nPMTK: BOOT=%s PMTK010=%u CMD=%u FLAG=%u | B=%u SC=%u/%u/%u [BR=%u] %X\r\n",
						(GPS_PMTK.PMTK_BOOT) ? "TRUE" : "FALSE",
						GPS_PMTK.PMTK010,
						GPS_PMTK.PMTK001_CMD,
						GPS_PMTK.PMTK001_FLAG,
						BC,
						GPS_sentences_parsed,
						GPS_sentences_unknown,
						GPS_sentences_invalid,
						baud,
						wait
						);
			}
		}

	} else {
		// No proper communication with GPS receiver
		printf("GPS_Init timeout\r\n");
	}

	// FIXME: return some value here, no VOID
}
