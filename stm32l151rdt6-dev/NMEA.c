#include <stm32l1xx_rcc.h>
#include <string.h> // For memset

#include "NMEA.h"




// Structures for parsed data
GPS_Data_TypeDef GPSData;                   // Data from GNSS sentences
NMEA_PMTK_TypeDef PMTKData;                 // Data from PMTK sentences

// Sentences counters
uint8_t NMEA_sentences_parsed;              // Parsed
uint8_t NMEA_sentences_unknown;             // Unsupported
uint8_t NMEA_sentences_invalid;             // Invalid

// Satellites information
uint8_t NMEA_sats_fix[12];                      // IDs of satellites used in position fix
// Information about satellites in view (can be increased if receiver is able to handle more)
NMEA_Sat_TypeDef GPS_sats_view[MAX_SATELLITES_VIEW];




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
uint8_t NMEA_CalcCRC(char *str) {
	uint8_t result = 0;

	if (*str++ == '$') while ((*str != '*') && (*str != '\0')) result ^= *str++;

	return result;
}

// Find end of the NMEA sentence
// input:
//   buf - pointer to a byte in the data buffer from where search will start
//   buf_end - pointer to the last byte where search must stop
// return: pointer to the last byte of NMEA sentence
// note: the result pointer will point to the last byte of a sentence or to the end
//       of the data buffer in case of sentence is incomplete
uint8_t * NMEA_FindTail(uint8_t *buf, uint8_t *buf_end) {
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
void NMEA_NextTerm(uint8_t **buf) {
	// Find next term or sentence end
	while ((**buf != ',') && (**buf != '*')) (*buf)++;

	// Point to the first character of next term
	if (**buf == ',') (*buf)++;
}

// Find NMEA sentence in buffer
// input:
//   sentence - pointer to the structure describing NMEA sentence
//   buf_start - pointer to the data buffer where search will start
//   buf_end - pointer to the end of the data buffer
// note: function modifies the 'sentence' structure
void NMEA_FindSentence(NMEASentence_TypeDef *sentence, uint8_t *buf_start, uint8_t *buf_end) {
	uint32_t *ptr = (uint32_t *)buf_start;
	uint32_t hdr;

	// Populate the sentence structure with initial values
	sentence->start = (uint8_t *)ptr;
	sentence->data  = sentence->start;
	sentence->end   = buf_end;
	sentence->type  = NMEA_NOTFOUND;

	// Further parsing does not make sense if there is no space left in buffer for NMEA sentence
	if (buf_end - (uint8_t *)ptr < 10) return;

	do {
		hdr = *ptr << 8;
		if (hdr == 0x50472400) {
			// $GPxxx sentences - GPS

			// Point to the first byte of sentence
			sentence->start = (uint8_t *)ptr;

			// Point to the 4th byte of sentence
			ptr = (uint32_t *)((uint8_t *)ptr + 3);
			hdr = *ptr << 8;

			// Point to the first term of sentence
			sentence->data = (uint8_t *)ptr + 4;

			// Find sentence tail
			sentence->end = NMEA_FindTail(sentence->start,buf_end);

			// Determine a sentence type
			switch (hdr) {
				case 0x4c4c4700:
					sentence->type = NMEA_GPGLL;
					return;
				case 0x434d5200:
					sentence->type = NMEA_GPRMC;
					return;
				case 0x47545600:
					sentence->type = NMEA_GPVTG;
					return;
				case 0x41474700:
					sentence->type = NMEA_GPGGA;
					return;
				case 0x41534700:
					sentence->type = NMEA_GPGSA;
					return;
				case 0x56534700:
					sentence->type = NMEA_GPGSV;
					return;
				case 0x41445a00:
					sentence->type = NMEA_GPZDA;
					return;
				default:
					// Unsupported GPS sentence
					sentence->type = NMEA_UNKNOWN;
					return;
			}
		} else if (hdr == 0x4e472400) {
			// $GNxxx sentences - GLONASS + GPS

			// Unsupported GLONASS + GPS sentence
			sentence->type = NMEA_UNKNOWN;

			return;
		} else if (hdr == 0x4c472400) {
			// $GLxxx sentences - GLONASS

			// Unsupported GLONASS sentence
			sentence->type = NMEA_UNKNOWN;

			return;
		} else if (hdr == 0x4d502400) {
			// $PMTKxxx sentences - proprietary MTK

			// Point to the first byte of sentence
			sentence->start = (uint8_t *)ptr;

			// Point to the 5th byte of sentence
			ptr = (uint32_t *)(((uint8_t *)ptr) + 4);
			hdr = *ptr;

			// Point to the first term of sentence
			sentence->data = (uint8_t *)ptr + 5;

			// Find sentence tail
			sentence->end = NMEA_FindTail(sentence->start,buf_end);

			// Determine a sentence type
			switch (hdr) {
				case 0x3130304b:
					sentence->type = NMEA_PMTK001;
					return;
				case 0x3031304b:
					sentence->type = NMEA_PMTK010;
					return;
				case 0x3131304b:
					sentence->type = NMEA_PMTK011;
					return;
				default:
					// Unsupported MTK sentence
					sentence->type = NMEA_UNKNOWN;
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
void NMEA_ParseLatLon(uint8_t **buf, uint8_t deg_len, int32_t *value, uint8_t *char_value) {
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
//   satellite - pointer to the structure describing the satellite parameters
// note: buf will point to the next term of sentence
void NMEA_ParseSatsInView(uint8_t **buf, NMEA_Sat_TypeDef *satellite) {
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

	// This must be set after all of NMEA sentences parsed (call NMEA_CheckUsedSats)
	satellite->used = FALSE;
}

// Parse time from NMEA sentence (format: HHMMSS.XXX)
// input:
//   buf - pointer to the pointer to the data buffer
//   time - pointer to structure where time will be stored
// note: the buf pointer will point to the next term
void NMEA_ParseTime(uint8_t **buf, NMEATime *time) {
	if (**buf != ',') {
		// Hours
		time->Hours   = atoi_len(buf,2);

		// Minutes
		time->Minutes = atoi_len(buf,2);

		// Seconds
		time->Seconds = atoi_len(buf,2);

		// ... Milliseconds are ignored
		NMEA_NextTerm(buf);
	} else {
		(*buf)++;
	}
}

// Parse NMEA sentence
// input:
//   sentence - pointer to the structure describing sentence
void NMEA_ParseSentence(NMEASentence_TypeDef *sentence) {
	uint8_t *ptr = sentence->data;
	uint32_t tmp;

	switch (sentence->type) {

		// GPS sentences
		case NMEA_GPRMC:
			// $GPRMC - Recommended minimum specific GPS/Transit data

			// Time of fix
			NMEA_ParseTime(&ptr,&GPSData.fix_time);

			// Valid data marker (A=active or V=void)
			GPSData.valid = FALSE;
			if (*ptr != ',') {
				if (*ptr++ == 'A') GPSData.valid = TRUE;
			}
			ptr++;

			// Latitude
			NMEA_ParseLatLon(&ptr,2,&GPSData.latitude,&GPSData.latitude_char);

			// Longitude
			NMEA_ParseLatLon(&ptr,3,&GPSData.longitude,&GPSData.longitude_char);

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
			NMEA_NextTerm(&ptr);

			// Magnetic variation direction
			// ignore this term (mostly not supported by GPS receivers)
			NMEA_NextTerm(&ptr);

			// Mode indicator (NMEA 0183 v2.3 or never)
			if ((*ptr != ',') && (*ptr != '*')) GPSData.mode = *ptr;

			break; // NMEA_GPRMC
		case NMEA_GPGLL:
			// $GPGLL - Geographic position, latitude / longitude

			// Latitude
			NMEA_ParseLatLon(&ptr,2,&GPSData.latitude,&GPSData.latitude_char);

			// Longitude
			NMEA_ParseLatLon(&ptr,3,&GPSData.longitude,&GPSData.longitude_char);

			// Time of fix
			NMEA_ParseTime(&ptr,&GPSData.fix_time);

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
			NMEA_ParseTime(&ptr,&GPSData.time);

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
			NMEA_NextTerm(&ptr);

			// Skip term: course relative to magnetic north
			NMEA_NextTerm(&ptr);

			// Skip term: 'M' letter - "track made good is relative to magnetic north"
			NMEA_NextTerm(&ptr);

			// Speed over ground in knots
			GPSData.speed_k = atoi_flt(&ptr);
			// Convert speed in knots to speed in km/h
			if ((GPSData.speed == 0) && (GPSData.speed_k != 0)) GPSData.speed = (GPSData.speed_k * 1852) / 1000;

			// Skip term: 'N' letter - speed over ground measured in knots
			NMEA_NextTerm(&ptr);

			// Speed over ground in km/h
			GPSData.speed = atoi_flt(&ptr);

			// Skip term: 'K' letter - speed over ground measured in km/h
			NMEA_NextTerm(&ptr);

			// Mode indicator (NMEA 0183 v2.3 or never)
			if ((*ptr != ',') && (*ptr != '*')) GPSData.mode = *ptr;

			break; // NMEA_GPVTG
		case NMEA_GPGGA:
			// $GPGGA - GPS fix data

			// Time
			NMEA_ParseTime(&ptr,&GPSData.fix_time);

			// Latitude
			NMEA_ParseLatLon(&ptr,2,&GPSData.latitude,&GPSData.latitude_char);

			// Longitude
			NMEA_ParseLatLon(&ptr,3,&GPSData.longitude,&GPSData.longitude_char);

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
				NMEA_NextTerm(&ptr);
			} else ptr++;

			// MSL measurement units, ignore this term and assume what units is meters
			NMEA_NextTerm(&ptr);

			// Geoid-to-ellipsoid separation (ellipsoid altitude = MSL altitude + geoid separation)
			// Value can be negative
			GPSData.geoid_separation = atoi_flt(&ptr);

			// Geoid-to-ellipsoid separation measurement units, ignore this term and assume what units is meters
			NMEA_NextTerm(&ptr);

			// Time since last DGPS update
			GPSData.dgps_age = atoi_chr(&ptr);

			// DGPS station ID
			if ((*ptr != ',') && (*ptr != '*')) GPSData.dgps_id = atoi_chr(&ptr);

			break; // NMEA_GPGGA
		case NMEA_GPGSA:
			// $GPGSA - GPS DOP and active satellites

			// Satellite acquisition mode (M = manually forced 2D or 3D, A = automatic switch between 2D and 3D)
			// Skip this term
			NMEA_NextTerm(&ptr);

			// Position mode (1 = fix not available, 2 = 2D fix, 3 = 3D fix)
			GPSData.fix = (*ptr != ',') ? *ptr++ - '0' : 1;
			ptr++;

			// IDs of satellites used in position fix (12 terms per sentence)
			tmp = 0;
			do {
				NMEA_sats_fix[tmp] = (*ptr != ',') ? atoi_len(&ptr,2) : 0;
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
			NMEA_NextTerm(&ptr);

			// GSV sentence number
			tmp = (*ptr != ',') ? (*ptr++ - '0' - 1) << 2 : 0;
			ptr++;

			// Total number of satellites in view
			GPSData.sats_view = (*ptr != ',') ? atoi_len(&ptr,2) : 0;
			ptr++;

			// Parse no more than 12 satellites in view
			if (GPSData.sats_view && (tmp < MAX_SATELLITES_VIEW)) {
				if (*(ptr - 1) != '*') NMEA_ParseSatsInView(&ptr,&GPS_sats_view[tmp++]);
				if (*(ptr - 1) != '*') NMEA_ParseSatsInView(&ptr,&GPS_sats_view[tmp++]);
				if (*(ptr - 1) != '*') NMEA_ParseSatsInView(&ptr,&GPS_sats_view[tmp++]);
				if (*(ptr - 1) != '*') NMEA_ParseSatsInView(&ptr,&GPS_sats_view[tmp++]);
			}

			break; // NMEA_GPGSV

		// MTK sentences
		case NMEA_PMTK001:
			// $PMTK001 - PMTK_ACK

			PMTKData.PMTK001_CMD = atoi_chr(&ptr);
			PMTKData.PMTK001_FLAG = (*ptr != ',') ? *ptr - '0' : 0;

			break; // NMEA_PMTK001
		case NMEA_PMTK010:
			// $PMTK010 - PMTK_SYS_MSG

			PMTKData.PMTK010 = atoi_chr(&ptr);

			break; // NMEA_PMTK010
		case NMEA_PMTK011:
			// $PMTK011 - PMTK_BOOT

			// Check if term value is 'MTKGPS'
			if (*ptr != ',') {
				PMTKData.PMTK_BOOT = FALSE;
				tmp = *(uint32_t *)ptr;
				if (tmp == 0x474b544d) {
					ptr += 4;
					tmp = *(uint32_t *)ptr << 16;
					PMTKData.PMTK_BOOT = (tmp == 0x53500000);
				}
			}

			break; // NMEA_PMTK011

		// Unsupported NMEA sentence
		default:
			break;
	}
}

// Initialize variables
void NMEA_InitData(void) {
	uint32_t i;

	// Clear parsed data
	memset(&GPSData,0,sizeof(GPSData));
	memset(&PMTKData,0,sizeof(PMTKData));

	// Clear satellites information
	for (i = 0; i < 12; i++) NMEA_sats_fix[i] = 0;
	for (i = 0; i < MAX_SATELLITES_VIEW; i++) {
		memset(&GPS_sats_view[i],0,sizeof(NMEA_Sat_TypeDef));
		GPS_sats_view[i].SNR = 255;
	}

	// Some non-zero initial values
	GPSData.longitude_char = 'X';
	GPSData.latitude_char  = 'X';
	GPSData.mode  = 'N';
	GPSData.valid = FALSE;

	// Clear counters
	NMEA_sentences_parsed  = 0;
	NMEA_sentences_unknown = 0;
	NMEA_sentences_invalid = 0;
}

// Check which satellites in view is used in location fix
void NMEA_CheckUsedSats(void) {
	uint32_t i;
	uint32_t j;

	for (i = 0; i < GPSData.sats_view; i++) {
		GPS_sats_view[i].used = FALSE;
		for (j = 0; GPSData.sats_used; j++) {
			if (NMEA_sats_fix[j] == GPS_sats_view[i].PRN) {
				GPS_sats_view[i].used = TRUE;
				break;
			}
		}
	}
}

// Parse NMEA sentences in specified data buffer
// input:
//   buf - pointer to the buffer with GPS data
//   length - pointer to the variable with number of bytes in the data buffer
void NMEA_ParseBuf(uint8_t *buf, uint16_t *length) {
	NMEASentence_TypeDef sentence;
	uint8_t *buf_end = buf + *length; // Pointer to the last significant byte in GPS buffer

	// Clear previously parsed GPS data
	NMEA_InitData();

	// Find all sentences and parse known
	while (buf < buf_end) {
		NMEA_FindSentence(&sentence,buf,buf_end);
		if (sentence.type != NMEA_NOTFOUND) {
			// Validate a sentence by CRC check
			if (atoi_hex(sentence.end - 3) == NMEA_CalcCRC((char *)sentence.start)) {
				// Sentence validation passed
				if (sentence.type != NMEA_UNKNOWN) {
					// Supported sentence found -> parse it
					NMEA_ParseSentence(&sentence);
					NMEA_sentences_parsed++;
				} else {
					// Unsupported sentence found -> skip it
					NMEA_sentences_unknown++;
				}
			} else {
				// Sentence validation failed
				sentence.type = NMEA_INVALID;
				NMEA_sentences_invalid++;
			}
		}

		// Move the pointer to the byte following the last parsed
		buf = sentence.end + 1;
	}

	// Reset the GPS buffer counter
	*length = 0;
}
