#include <stm32l1xx_rcc.h>
#include <stm32l1xx_rtc.h>

#include <wolk.h>
#include <uc1701.h>
#include <GUI.h>
#include <GPS.h>

#include <resources.h>
#include <math.h>


// Callback function for change display brightness settings
void callback_Brightness(int32_t param) {
	UC1701_SetBacklight(param);
}

// Draw bitmap
// input:
//   X, Y - top left corner coordinates of bitmap
//   W, H - width and height of bitmap in pixels
//   pBMP - pointer to array containing bitmap
void GUI_DrawBitmap(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, const uint8_t* pBMP) {
	uint8_t i,j;

	for (i = 0; i < W; i++) {
		for (j = 0; j < H; j++) {
			if ((pBMP[i + (j / 8) * W] >> (j % 8)) & 0x01) {
				SetPixel(X + i, Y + j);
			} else {
				ResetPixel(X + i, Y + j);
			}
		}
	}
}

// Big digit
void GUI_BigDig(uint8_t X, uint8_t Y, uint8_t digit) {
	GUI_DrawBitmap(X,Y,14,34,&big_digits[(digit * 70)]);
}

// Middle digit
void GUI_MidDig(uint8_t X, uint8_t Y, uint8_t digit) {
	GUI_DrawBitmap(X,Y,10,22,&mid_digits[(digit * 30)]);
}

// Small digit
void GUI_SmallDig(uint8_t X, uint8_t Y, uint8_t digit) {
	GUI_DrawBitmap(X,Y,9,19,&small_digits[(digit * 27)]);
}

// Draw number with big digits
// input:
//   X - left horizontal coordinate of number
//       if this value is negative -> number right horizontal coordinate
//   Y - top vertical coordinate of number
//       if this value is negative -> number bottom vertical coordinate
//   number - number to display
//   decimals - number of decimal digits (0..4)
//   DigitSize - type of digit (DS_Big, DS_Mid, DS_Small)
void GUI_DrawNumber(int8_t X, int8_t Y, int32_t Number, uint8_t Decimals,
		DigitSize_TypeDef DigitSize) {
	uint8_t i = 0;
	int32_t num = Number;
	uint8_t xx = X;
	uint8_t dig_w,dig_h; // Digit width and height
	uint8_t dig_int; // Interval between digits
	uint8_t dec_w,dec_h; // Decimal point width and height
	const uint8_t *dig_offset; // Offset to digit bitmap
	uint16_t dig_len; // Length of digit bitmap
	uint8_t buf[12];
	uint8_t strLen; // String length
	uint8_t neg; // Negative flag
	uint8_t minus_w,minus_h; // Width and height of minus sign
	uint8_t minus_t; // Top coordinate of minus sign
	uint8_t num_width; // Number width

	switch(DigitSize) {
	case DS_Mid:
		dig_w = 10;
		dig_h = 22;
		dig_len = 30;
		dig_offset = mid_digits;
		dig_int = 1;
		dec_w = 2;
		dec_h = 3;
		minus_w = 5;
		minus_h = 2;
		minus_t = 9;
		break;
	case DS_Small:
		dig_w = 9;
		dig_h = 19;
		dig_len = 27;
		dig_offset = small_digits;
		dig_int = 1;
		dec_w = 2;
		dec_h = 3;
		minus_w = 4;
		minus_h = 2;
		minus_t = 8;
		break;
	default:
		dig_w = 14;
		dig_h = 34;
		dig_len = 70;
		dig_offset = big_digits;
		dig_int = 2;
		dec_w = 3;
		dec_h = 4;
		minus_w = 6;
		minus_h = 4;
		minus_t = 15;
		break;
	}

	if (num < 0) {
		neg  = 1;
		num *= -1;
	} else neg = 0;

	do buf[i++] = num % 10; while ((num /= 10) > 0);
	strLen = i;
	if (strLen <= Decimals) {
		for (i = strLen; i <= Decimals; i++) buf[i] = 0;
		strLen = Decimals + 1;
	}

	num_width = (dig_w * strLen) + (dig_int * (strLen - 1));
	if (Decimals) num_width += dec_w + dig_int + 1;
	if (neg) num_width += minus_w + dig_int + 1;

	// If X negative -> adjust number left horizontal coordinate
	if (X < 0) xx = (-1 * X) - num_width + 1;

	// If Y negative -> adjust number top horizontal coordinate
	if (Y < 0) Y = (-1 * Y) - dig_h + 1;

	// Draw minus only if number is non zero
	if (neg && Number) {
		FillRect(xx,Y + minus_t,xx + minus_w,Y + minus_t + minus_h,PSet);
		xx += minus_w + dig_int + 1;
	}

	// Draw number
	for (i = 0; i < strLen; i++) {
		GUI_DrawBitmap(xx,Y,dig_w,dig_h,&dig_offset[dig_len * buf[strLen - i - 1]]);
		xx += dig_w + dig_int;
		if (strLen - i - 1 == Decimals && Decimals) {
			// Decimal point
			FillRect(xx,Y + dig_h - dec_h,xx + dec_w,Y + dig_h - 1,PSet);
			xx += dec_w + dig_int + 1;
		}
	}
}

// Draw time
// input:
//   X, Y - left top corner coordinates
//   RTC_Time - number to display (0..65535)
//   TimeType - time format (TT_full or TT_short)
//   DigitSize - type of digit (DS_Big, DS_Mid, DS_Small)
// Small size: 71 x 18 | 45 x 18
// Mid size:   77 x 21 | 50 x 21
// Big size:  104 x 34 | 67 x 34
void GUI_DrawTime(uint8_t X, uint8_t Y, RTC_TimeTypeDef *RTC_Time, TimeType_TypeDef TimeType, DigitSize_TypeDef DigitSize) {
	uint8_t dig1,dig2;
	uint8_t dig_w;

	switch(DigitSize) {
	case DS_Mid:
		dig_w = 12;
		break;
	case DS_Small:
		dig_w = 11;
		break;
	default:
		dig_w = 16;
		break;
	}

	// Hours
	dig1 = RTC_Time->RTC_Hours % 10;
	dig2 = RTC_Time->RTC_Hours / 10;
	switch(DigitSize) {
	case DS_Mid:
		GUI_MidDig(X,Y,dig2);
		GUI_MidDig(X + dig_w,Y,dig1);
		X += dig_w * 2;
		FillRect(X,Y + 4,X + 1,Y + 8,PSet);
		FillRect(X,Y + 13,X + 1,Y + 17,PSet);
		break;
	case DS_Small:
		GUI_SmallDig(X,Y,dig2);
		GUI_SmallDig(X + dig_w,Y,dig1);
		X += dig_w * 2;
		FillRect(X,Y + 3,X + 1,Y + 7,PSet);
		FillRect(X,Y + 11,X + 1,Y + 15,PSet);
		break;
	default:
		GUI_BigDig(X,Y,dig2);
		GUI_BigDig(X + dig_w,Y,dig1);
		X += dig_w * 2;
		FillRect(X,Y + 8,X + 2,Y + 11,PSet);
		FillRect(X,Y + 22,X + 2,Y + 25,PSet);
		break;
	}

	// Minutes
	X += 4;
	dig1 = RTC_Time->RTC_Minutes % 10;
	dig2 = RTC_Time->RTC_Minutes / 10;
	switch(DigitSize) {
	case DS_Mid:
		GUI_MidDig(X,Y,dig2);
		GUI_MidDig(X + dig_w,Y,dig1);
		if (TimeType == TT_Full) {
			X += dig_w * 2;
			FillRect(X,Y + 4,X + 1,Y + 8,PSet);
			FillRect(X,Y + 13,X + 1,Y + 17,PSet);
		}
		break;
	case DS_Small:
		GUI_SmallDig(X,Y,dig2);
		GUI_SmallDig(X + dig_w,Y,dig1);
		if (TimeType == TT_Full) {
			X += dig_w * 2;
			FillRect(X,Y + 3,X + 1,Y + 7,PSet);
			FillRect(X,Y + 11,X + 1,Y + 15,PSet);
		}
		break;
	default:
		X++;
		GUI_BigDig(X,Y,dig2);
		GUI_BigDig(X + dig_w,Y,dig1);
		if (TimeType == TT_Full) {
			X += dig_w * 2;
			FillRect(X,Y + 8,X + 2,Y + 11,PSet);
			FillRect(X,Y + 22,X + 2,Y + 25,PSet);
		}
		break;
	}

	if (TimeType == TT_Short) return;

	// Seconds
	X += 4;
	dig1 = RTC_Time->RTC_Seconds % 10;
	dig2 = RTC_Time->RTC_Seconds / 10;
	switch(DigitSize) {
	case DS_Mid:
		GUI_MidDig(X,Y,dig2);
		GUI_MidDig(X + dig_w,Y,dig1);
		X += dig_w * 2;
		break;
	case DS_Small:
		GUI_SmallDig(X,Y,dig2);
		GUI_SmallDig(X + dig_w,Y,dig1);
		X += dig_w * 2;
		break;
	default:
		X++;
		GUI_BigDig(X,Y,dig2);
		GUI_BigDig(X + dig_w,Y,dig1);
		X += dig_w * 2;
		break;
	}
}

// Screen with RAW data packet
void GUI_Screen_SensorRAW(void) {
	uint8_t X,Y;

	// Frame
	UC1701_Fill(0x00);
	Rect(0,4,scr_width - 1,scr_height - 1,PSet);
	FillRect(3,0,scr_width - 4,8,PSet);
	PutStr5x7(19,1,"Sensor RAW data",CT_transp_inv);
	// Cadence data
	X = 5; Y = 10;
	X += PutStr5x7(X,Y,"CDC:",CT_transp) - 1;
	PutInt5x7(X,Y,nRF24_Packet.tim_CDC,CT_transp);
	// Speed data
	X = 5; Y += 9;
	X += PutStr5x7(X,Y,"SPD c:",CT_transp) - 1;
	X += PutInt5x7(X,Y,nRF24_Packet.cntr_SPD,CT_transp) + 8;
	X += PutStr5x7(X,Y,"t:",CT_transp) - 1;
	PutInt5x7(X,Y,nRF24_Packet.tim_SPD,CT_transp);
	// Packets lost
	X = 5; Y += 9;
	X += PutStr5x7(X,Y,"P.Lost:",CT_transp) - 1;
	X += PutInt5x7(X,Y,nRF24_Packet.packets_lost,CT_transp) + 5;
	// OBSERVER_TX
	X += PutStr5x7(X,Y,"OTX:",CT_transp) - 1;
	X += PutHex5x7(X,Y,nRF24_Packet.observe_TX,CT_transp);
	// Wakeups
	X = 5; Y += 9;
	X += PutStr5x7(X,Y,"Wake:",CT_transp) - 1;
	PutInt5x7(X,Y,nRF24_Packet.cntr_wake,CT_transp);
	X = 76;
	X += PutStr5x7(X,Y,"RT:",CT_transp) - 1;
	PutInt5x7(X,Y,nRF24_Packet.ride_time,CT_transp);
	// Battery
	X = 5; Y += 9;
	X += PutStr5x7(X,Y,"Battery:",CT_transp) - 1;
	PutChar5x7(X + PutIntF5x7(X,Y,nRF24_Packet.vrefint,2,CT_transp),Y,'V',CT_transp);
}

// Screen with current values (trip data)
void GUI_Screen_CurVal1(void) {
	uint8_t X,Y;

	// Frame
	UC1701_Fill(0x00);
	Rect(0,4,scr_width - 1,scr_height - 1,PSet);
	FillRect(3,0,scr_width - 4,8,PSet);
	PutStr5x7(22,1,"Current values",CT_transp_inv);

	X = 7; Y = 10;
	PutStr5x7(X,Y,"SPD:",CT_transp);
	PutIntF5x7(X + 23,Y,CurData.Speed,1,CT_transp);
	X += 54;
	PutStr5x7(X,Y,"CDC:",CT_transp);
	PutInt5x7(X + 23,Y,CurData.Cadence,CT_transp);
	X = 7; Y += 9;
	PutStr5x7(X,Y,"A.S:",CT_transp);
	PutIntF5x7(X + 23,Y,CurData.AvgSpeed,1,CT_transp);
	X += 54;
	PutStr5x7(X,Y,"A.C:",CT_transp);
	PutInt5x7(X + 23,Y,CurData.AvgCadence,CT_transp);
	X = 7; Y += 9;
	PutStr5x7(X,Y,"M.S:",CT_transp);
	PutIntF5x7(X + 23,Y,CurData.MaxSpeed,1,CT_transp);
	X += 54;
	PutStr5x7(X,Y,"M.C:",CT_transp);
	PutInt5x7(X + 23,Y,CurData.MaxCadence,CT_transp);
	X = 7; Y += 9;
	PutStr5x7(X,Y,"T.D:",CT_transp);
	PutInt5x7(X + 23,Y,CurData.TripDist,CT_transp);
	X = 7; Y += 9;
	PutStr5x7(X,Y,"Odo:",CT_transp);
	PutInt5x7(X + 23,Y,CurData.Odometer,CT_transp);
	X = 7; Y += 9;
	PutStr5x7(X,Y,"Time",CT_transp);
	GUI_PutTimeSec5x7(X + 26,Y,CurData.TripTime,CT_opaque);
}

// Screen with current values (BMP180 values)
void GUI_Screen_CurVal2(void) {
	uint8_t X,Y;

	// Frame
	UC1701_Fill(0x00);
	Rect(0,4,scr_width - 1,scr_height - 1,PSet);
	FillRect(3,0,scr_width - 4,8,PSet);
	PutStr5x7(46,1,"BMP180",CT_transp_inv);

	X = 4; Y = 10;
	X += PutStr5x7(X,Y,"Temperature:",CT_transp) - 1;
	GUI_PutTemperature5x7(X,Y,CurData.Temperature,CT_transp);
	X = 4; Y += 9;
	X += PutStr5x7(X,Y,"Min:",CT_transp) - 1;
	GUI_PutTemperature5x7(X,Y,CurData.MinTemperature,CT_transp);
	X = 67;
	X += PutStr5x7(X,Y,"Max:",CT_transp) - 1;
	GUI_PutTemperature5x7(X,Y,CurData.MaxTemperature,CT_transp);

	HLine(1,scr_width - 2,Y + 10,PSet);

	X = 4; Y += 14;
	X += PutStr5x7(X,Y,"Pressure:",CT_transp) - 1;
	GUI_PutPressure5x7(X,Y,CurData.Pressure,PT_mmHg,CT_transp);
	X = 4; Y += 9;
	X += PutStr5x7(X,Y,"Min:",CT_transp) - 1;
	PutIntF5x7(X,Y,CurData.MinPressure * 75 / 1000,1,CT_transp);
	X = 67;
	X += PutStr5x7(X,Y,"Max:",CT_transp) - 1;
	PutIntF5x7(X,Y,CurData.MaxPressure * 75 / 1000,1,CT_transp);
}

// Screen with current values (GPS values)
void GUI_Screen_CurVal3(void) {
	uint8_t X,Y;

	// Frame
	UC1701_Fill(0x00);
	Rect(0,4,scr_width - 1,scr_height - 1,PSet);
	FillRect(3,0,scr_width - 4,8,PSet);
	PutStr5x7(36,1,"GPS stats",CT_transp_inv);

	X = 4; Y = 10;
	X += PutStr5x7(X,Y,"Speed:",CT_transp) - 1;
	X += PutIntF5x7(X,Y,CurData.GPSSpeed,2,CT_transp);
	PutStr5x7(X,Y,"km/h",CT_transp);
	X = 4; Y += 9;
	X += PutStr5x7(X,Y,"Max:",CT_transp) - 1;
	X += PutIntF5x7(X,Y,CurData.MaxGPSSpeed,2,CT_transp);
	PutStr5x7(X,Y,"km/h",CT_transp);

	HLine(1,scr_width - 2,Y + 10,PSet);

	X = 4; Y += 14;
	X += PutStr5x7(X,Y,"Altitude:",CT_transp) - 1;
	X += PutInt5x7(X,Y,CurData.GPSAlt,CT_transp);
	PutChar5x7(X,Y,'m',CT_transp);
	X = 4; Y += 9;
	X += PutStr5x7(X,Y,"Min:",CT_transp) - 1;
	X += PutInt5x7(X,Y,CurData.MinGPSAlt,CT_transp);
	PutChar5x7(X,Y,'m',CT_transp);
	X = 4; Y += 9;
	X += PutStr5x7(X,Y,"Max:",CT_transp) - 1;
	X += PutInt5x7(X,Y,CurData.MaxGPSAlt,CT_transp);
	PutChar5x7(X,Y,'m',CT_transp);
}

// Screen with satellites in view bar graph
// input:
//   WaitForKey - function pointer to WaitForKeyPress function
// note: if WaitForKey are NULL - just draw screen and return
void GUI_Screen_GPSSatsView(funcPtrKeyPress_TypeDef WaitForKey) {
	uint8_t i;
	uint8_t X;
	uint8_t Y;
	uint8_t sats;
	uint8_t max_SNR;

	do {
		ClearKeys();

		sats = GPSData.sats_view;
		max_SNR = 40;

		// Used satellites can be no more than 12 (from $GPGGA)
		if (sats > 12) sats = 12;

		// Find maximal SNR
		for (i = 0; i < sats; i++) {
			if (GPS_sats_view[i].SNR > max_SNR && GPS_sats_view[i].SNR != 255) max_SNR = GPS_sats_view[i].SNR;
		}

		UC1701_Fill(0x00);

		// Draw scale
		HLine(0,scr_width - 1,50,PSet);
		VLine(scr_width - 9,0,63,PSet);
		for (i = 0; i < 5; i++) {
			PutIntULZ3x5(scr_width - 7,44 - (i * 11),i * (max_SNR / 4),2);
			SetPixel(scr_width - 10,i * 11 + 2);
		}
		for (i = 0; i < 11; i++) VLine(8 + (i * 10),51,63,PSet);

		if (!GPSData.sats_view) {
			// Just for decoration
			X = PutStr5x7(12,21,"No satellites",CT_opaque);
			for (i = 0; i < 12; i++) PutIntULZ3x5(i * 10,52,0,2);
		} else {
			// Satellites SNR graphs
			for (i = 0; i < sats; i++) {
				X = i * 10;
				if (GPS_sats_view[i].SNR != 255) {
					Y = 48 - ((GPS_sats_view[i].SNR * 48) / max_SNR);
					if (GPS_sats_view[i].used) FillRect(X,Y,X + 6,50,PSet); else Rect(X,Y,X + 6,50,PSet);
				}
				PutIntULZ3x5(X,52,GPS_sats_view[i].PRN,2);
				if (GPS_sats_view[i].SNR != 255) PutIntULZ3x5(X,59,GPS_sats_view[i].SNR,2);
			}
		}

		UC1701_Flush();

		if (WaitForKey) WaitForKey(TRUE,&GPS_new_data); else return;
		GPS_new_data = FALSE;
	} while (!BTN[BTN_ESCAPE].cntr);

	ClearKeys();
	UC1701_Fill(0x00);
}

// Screen with RAW GPS values
// input:
//   WaitForKey - function pointer to WaitForKeyPress function
// note: if WaitForKey are NULL - just draw screen and return
void GUI_Screen_GPSInfo(funcPtrKeyPress_TypeDef WaitForKey) {
	uint8_t X,Y;

	do {
		ClearKeys();

		// Frame
		UC1701_Fill(0x00);
		Rect(0,0,scr_width - 1,scr_height - 1,PSet);

		X = 4; Y = 4;
		X += GUI_PutTimeSec5x7(X,Y,GPSData.time,CT_opaque) + 5;
		X += GUI_PutDate5x7(X,Y,GPSData.date,CT_opaque);

		X = 4; Y += 8;
		X += PutStr5x7(X,Y,"Fix:",CT_opaque) - 1;
		if (GPSData.fix != 2 && GPSData.fix != 3)
			X += PutStr5x7(X,Y,"NA",CT_opaque) + 5;
		else {
			PutInt5x7(X,Y,GPSData.fix,CT_opaque);
			PutChar5x7(X + 6,Y,'D',CT_opaque);
			X += 17;
		}
		X += PutStr5x7(X,Y,"Qlty:",CT_opaque) - 1;
		PutChar5x7(X,Y,GPSData.fix_quality + '0',CT_opaque);
		X += 11;
		X += PutStr5x7(X,Y,"Mode:",CT_opaque) - 1;
		PutChar5x7(X,Y,GPSData.mode,CT_opaque);

		X = 4; Y += 8;
		GUI_PutCoord5x7(X,Y,GPSData.latitude_degree,GPSData.latitude_seconds,GPSData.latitude_char,CT_opaque);
		X = 67;
		GUI_PutCoord5x7(X,Y,GPSData.longitude_degree,GPSData.longitude_seconds,GPSData.longitude_char,CT_opaque);

		X = 4; Y += 8;
		X += PutStr5x7(X,Y,"Alt:",CT_opaque) - 1;
		X += PutInt5x7(X,Y,GPSData.altitude,CT_opaque) + 5;
		X += PutStr5x7(X,Y,"Spd:",CT_opaque) - 1;
		X += PutIntF5x7(X,Y,GPSData.speed,2,CT_opaque);

		X = 4; Y += 8;
		X += PutStr5x7(X,Y,"Crs:",CT_opaque) - 1;
		X += PutIntF5x7(X,Y,GPSData.course,2,CT_opaque) + 5;
		X += PutStr5x7(X,Y,"Sat:",CT_opaque) - 1;
		X += PutInt5x7(X,Y,GPSData.sats_used,CT_opaque);
		PutChar5x7(X,Y,'/',CT_opaque);
		X += 6;
		X += PutInt5x7(X,Y,GPSData.sats_view,CT_opaque);

		X = 4; Y += 8;
		X += PutStr5x7(X,Y,"PDOP:",CT_opaque) - 1;
		X += PutIntF5x7(X,Y,GPSData.PDOP,2,CT_opaque) + 5;

		X = 4; Y += 8;
		X += PutStr5x7(X,Y,"HDOP:",CT_opaque) - 1;
		X += PutIntF5x7(X,Y,GPSData.HDOP,2,CT_opaque) + 5;
		X += PutStr5x7(X,Y,"VDOP:",CT_opaque) - 1;
		X += PutIntF5x7(X,Y,GPSData.VDOP,2,CT_opaque) + 5;

		UC1701_Flush();

		if (WaitForKey) WaitForKey(TRUE,&GPS_new_data); else return;
		GPS_new_data = FALSE;
	} while (!BTN[BTN_ESCAPE].cntr);

	ClearKeys();
	UC1701_Fill(0x00);
}

// Display byte buffer with scroll
// input:
//   pBuf - pointer to byte buffer
//   BufSize - size of buffer
//   UpdateFlag - pointer to bool variable which contains flag to refresh buffer content
void GUI_Screen_Buffer(uint8_t *pBuf, uint16_t BufSize, bool *UpdateFlag, funcPtrKeyPress_TypeDef WaitForKey) {
	uint8_t X,Y;
	uint16_t pos = 0;
	uint16_t i;
	uint8_t chars_per_line;

	chars_per_line = scr_width / 6;

	do {
		UC1701_Fill(0x00);

		X = 0; Y = 0;
		i = pos;
		do {
			PutChar5x7(X,Y,pBuf[i++],CT_opaque);
			X += 6;
			if (X > scr_width - 6) {
				X  = 0;
				Y += 8;
			}
		} while (Y < scr_height - 8 && i < BufSize);
		HLine(0,scr_width - 1,scr_height - 7,PSet);

		X = PutIntULZ3x5(0,scr_height - 5,pos,0) + 10;
		PutIntULZ3x5(X,scr_height - 5,BufSize,0);

		UC1701_Flush();

		// Wait for key press
		if (WaitForKey) WaitForKey(TRUE,UpdateFlag);
		*UpdateFlag = FALSE;

		// "Down" key
		if (BTN[BTN_DOWN].cntr > 0 || BTN[BTN_DOWN].state == BTN_Hold) {
			if (pos + i < BufSize) {
				pos += chars_per_line;
				if (pos > BufSize) pos = BufSize;
			}
			BTN[BTN_DOWN].cntr = 0;
		}

		// "Up" key
		if (BTN[BTN_UP].cntr > 0 || BTN[BTN_UP].state == BTN_Hold) {
			pos -= chars_per_line;
			if (pos > BufSize) pos = 0;
			BTN[BTN_UP].cntr = 0;
		}

		// "Enter" key
		if (BTN[BTN_ENTER].cntr > 0 || BTN[BTN_ENTER].state == BTN_Hold) {
			pos = 0;
			BTN[BTN_ENTER].cntr = 0;
		}
	} while (!BTN[BTN_ESCAPE].cntr);

	ClearKeys();
	UC1701_Fill(0x00);
}

// Draw speed with 'km/h' mark and AVG speed pace indicator
// input:
//   X, Y - left top corner coordinates
//   speed - speed value (max 999 -> 99.9 km/h)
//   avg - average speed value for pace indicator
// Size: 54 x 33
void GUI_DrawSpeed(int8_t X, int8_t Y, uint32_t speed, uint32_t avg) {
	FillRect(X,Y,X + 54,Y + 33,PReset);
//	GUI_DrawNumber(X,Y,speed / 10,2,0,DS_Big);
	GUI_DrawNumber(-(X + 29),Y,speed / 10,0,DS_Big);
	FillRect(X + 31,Y + 31,X + 33,Y + 33,PSet);
	GUI_SmallDig(X + 35,Y + 15,speed % 10);
	GUI_DrawBitmap(X + 34,Y + 3,20,10,&bmp_kmph[0]);
	if (speed > 0) {
		if (avg <= speed) {
			// Up pace arrow
			GUI_DrawBitmap(X + 46,Y + 18,9,13,&bmp_pace_arrow[0]);
		} else {
			// Down pace arrow
			GUI_DrawBitmap(X + 46,Y + 18,9,13,&bmp_pace_arrow[18]);
		}
	}
}

// Draw ride time (in seconds)
// input:
//   X, Y - left top corner coordinates
//   time - time in seconds (max value 359999 -> 99:59:59)
void GUI_DrawRideTime(uint8_t X, uint8_t Y, uint32_t time) {
	uint16_t hours;
	uint8_t minutes,seconds;

	hours   = time / 3600;
	minutes = (time / 60) % 60;
	seconds = time % 60;
	if (hours > 99) {
		hours   = 99;
		minutes = 59;
		seconds = 59;
	}

	// Hours
	GUI_SmallDig(X,Y,hours / 10);
	GUI_SmallDig(X + 10,Y,hours % 10);
	X += 20;
	FillRect(X,Y + 3,X + 1,Y + 7,PSet);
	FillRect(X,Y + 11,X + 1,Y + 15,PSet);
	X += 3;

	// Minutes
	GUI_SmallDig(X,Y,minutes / 10);
	GUI_SmallDig(X + 10,Y,minutes % 10);
	X += 20;
	FillRect(X,Y + 3,X + 1,Y + 7,PSet);
	FillRect(X,Y + 11,X + 1,Y + 15,PSet);
	X += 3;

	// Seconds
	GUI_SmallDig(X,Y,seconds / 10);
	GUI_SmallDig(X + 10,Y,seconds % 10);
}

// Draw graph
// input:
//   X, Y - top left corner coordinates
//   W, H - width and height of graph
//   data - pointer to array with graph values (must be same or less size of graph width)
//   GraphType - type of graph (GT_dot, GT_fill, GT_line)
void GUI_DrawGraph(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, const int16_t* data, GraphType_TypeDef GraphType) {
	uint8_t i,bY,pY,YY,offset;
	int16_t min,max;
	float mY;

	// Find maximal and minimal values
	min = data[0]; max = data[0];
	for (i = 0; i < W; i++) {
		if (data[i] < min) min = data[i];
		if (data[i] > max) max = data[i];
	}
	// Scale ratio
	mY = (H - 1) / (float)(max - min);
	if (mY > H / 10) {
		mY = H / 10;
		offset = (H / 2) - ((max - min) * mY / 2);
	} else offset = 0;
	// Bottom line of the graph
	bY = Y + H - 1;

	// Draw graph
	if (GraphType == GT_dot) {
		// Dots
		for(i = 0; i < W; i++) SetPixel(X + i,bY - (uint16_t)((data[i] - min) * mY) - offset);
	} else if (GraphType == GT_line) {
		// Continuous line
		pY = bY - (uint16_t)((data[0] - min) * mY) - offset;
		for(i = 1; i < W; i++) {
			YY = bY - (uint16_t)((data[i] - min) * mY) - offset;
			Line(X + i - 1,pY,X + i,YY);
			pY = YY;
		}
	} else {
		// Filled graph
		for(i = 0; i < W; i++) VLine(X + i,bY - (uint16_t)((data[i] - min) * mY) - offset,bY,PSet);
	}
}

// Put coordinates in format "Ndd.xxxxxx"
uint8_t GUI_PutCoord5x7(uint8_t X, uint8_t Y, uint8_t degree, uint32_t seconds, char ch, CharType_TypeDef CharType) {
	uint8_t pX = X;

	PutChar5x7(X,Y,ch,CharType);
	X += 6;
	X += PutIntF5x7(X,Y,(degree * 1000000) + (seconds / 60),6,CharType);

	return X - pX;
}

// Draw time with standard 5x7 font
// input:
//   X,Y - top left coordinates of text
//   time - time in seconds (max value = 3599999 seconds or 999:59:59)
//   CharType - character drawing style
uint8_t GUI_PutTimeSec5x7(uint8_t X, uint8_t Y, uint32_t time, CharType_TypeDef CharType) {
	uint8_t pX = X;
	uint16_t hours;
	uint8_t minutes,seconds;

	hours   = time / 3600;
	minutes = (time / 60) % 60;
	seconds = time % 60;
	if (hours > 999) {
		hours   = 999;
		minutes = 59;
		seconds = 59;
	}

	if (hours < 10) {
		PutChar5x7(X,Y,'0',CharType);
		X += 6;
	}
	PutInt5x7(X,Y,hours,CharType);
	if (hours > 99) X += 6;
	if (hours > 9)  X += 11; else X += 5;
	PutChar5x7(X,Y,':',CharType);
	X += 4;

	if (minutes < 10) {
		PutChar5x7(X,Y,'0',CharType);
		X += 6;
	}
	PutInt5x7(X,Y,minutes,CharType);
	if (minutes > 9) X += 11; else X += 5;
	PutChar5x7(X,Y,':',CharType);
	X += 4;

	if (seconds < 10) {
		PutChar5x7(X,Y,'0',CharType);
		X += 6;
	}
	X += PutInt5x7(X,Y,seconds,CharType);

	return X - pX;
}

// Draw date with standard 5x7 font
// input:
//   X,Y - top left coordinates of text
//   date - date in format DDMMYYYY
//   CharType - character drawing style
uint8_t GUI_PutDate5x7(uint8_t X, uint8_t Y, uint32_t date, CharType_TypeDef CharType) {
	uint8_t pX = X;
	uint16_t dig;

	// Day
	dig = date / 1000000;
	X += PutIntLZ5x7(X,Y,dig,2,CharType);
	PutChar5x7(X,Y,'.',CharType);
	X += 5;

	// Month
	dig = (date - (dig * 1000000)) / 10000;
	X += PutIntLZ5x7(X,Y,dig,2,CharType);
	PutChar5x7(X,Y,'.',CharType);
	X += 5;

	// Year
	dig = date % 10000;
	X += PutIntLZ5x7(X,Y,dig,4,CharType);

	return X - pX;
}

// Print pressure with 'mmHg'
// input:
//   X,Y - top left coordinates of text
//   pressure - pressure value in Pa
//   PressureType - what units pressure should be displayed (PT_hPa, PT_mmHg)
//   CharType - character drawing style
uint8_t GUI_PutPressure5x7(uint8_t X, uint8_t Y, int32_t pressure, PressureType_TypeDef PressureType,
		CharType_TypeDef CharType) {
	uint8_t pX = X;

	if (PressureType == PT_mmHg) {
		pressure = pressure * 75 / 1000;
		X += PutIntF5x7(X,Y,pressure,1,CharType);
		X += PutStr5x7(X,Y,"mmHg",CharType);
	} else {
		X += PutIntF5x7(X,Y,pressure,2,CharType);
		X += PutStr5x7(X,Y,"hPa",CharType);
	}

	return X - pX;
}

// Print temperature value with Celsius sign
// input:
//   X,Y - top left coordinates of text
//   temperature - temperature value in Celsius degree
//   CharType - character drawing style
uint8_t GUI_PutTemperature5x7(uint8_t X, uint8_t Y, int32_t temperature, CharType_TypeDef CharType) {
	uint8_t pX = X;

	if (temperature < 0) {
		HLine(X,X + 2,Y + 3,PSet);
		temperature *= -1;
		X += 4;
	}
	X += PutInt5x7(X,Y,temperature / 10,CharType);

	// Decimal point
	Rect(X,Y + 5,X + 1,Y + 6,PSet);
	X += 3;

	// Temperature fractional
	X += PutInt5x7(X,Y,temperature % 10,CharType);

	// Celsius degree sign
	HLine(X + 1,X + 2,Y,PSet);
	HLine(X + 1,X + 2,Y + 3,PSet);
	VLine(X,Y + 1,Y + 2,PSet);
	VLine(X + 3,Y + 1,Y + 2,PSet);
	PutChar5x7(X + 5,Y,'C',CharType);

	return X - pX + 10;
}

// Draw vertical menu with scrolling
// input:
//   X,Y - top left menu corner coordinates
//   W,H - width and height of menu
//   Menu - pointer to Menu_TypeDef structure, containing menu
//   StartPos - start menu position
//   WaitForKey - function pointer to WaitForKeyPress function
// return:
//   selected menu position or 0xff if "Escape" key pressed
uint8_t GUI_Menu(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, const Menu_TypeDef *Menu,
		uint8_t StartPos, funcPtrKeyPress_TypeDef WaitForKey) {
	uint8_t i;
	int8_t scrPos; // Cursor position related to screen
	int8_t scrOfs; // Menu scroll (number of first displayed item)
	uint8_t xx,yy;
	uint8_t Cr; // Right side of menu cursor
	uint8_t miYoffs; // First menu item vertical offset
	uint8_t sbTH; // Height of scrollbar thumb
	uint8_t sbH; // Height of scrollbar
	uint8_t sbTY; // Vertical position of scrollbar thumb
	uint8_t scrItems; // Visible menu items on screen

	// Compute locals for less further calculations
	Cr = X + W - 3;
	scrItems = (H - 4) / MenuItemHeight;
	if (scrItems < Menu->NumItems) {
		sbTH = ((H - 5) * scrItems ) / Menu->NumItems;
		if (sbTH < 2) sbTH = 2;
		Cr -= 4;
		sbH = H - 5;
	} else {
		sbTH = 0;
		sbH  = 0;
	}
	if (scrItems > Menu->NumItems) scrItems = Menu->NumItems;
	miYoffs = ((H - 6 - (scrItems * MenuItemHeight)) / 2) - 1;

	if (StartPos > Menu->NumItems) StartPos = 0;
	if (StartPos >= scrItems) {
		if (StartPos <= Menu->NumItems - scrItems) {
			scrPos = (scrItems * StartPos) / Menu->NumItems;
			scrOfs = StartPos - scrPos;
		} else {
			scrOfs = Menu->NumItems - scrItems;
			scrPos = StartPos - scrOfs;
		}
	} else {
		scrPos = StartPos;
		scrOfs = 0;
	}

	// Menu frame
	Rect(X,Y,X + W - 1,Y + H - 1,PSet);

	while(1) {
		ClearKeys();

		// Clear menu background
		FillRect(X + 1,Y + 1,X + W - 2,Y + H - 2,PReset);

		// Menu items
		yy = Y + 4 + miYoffs;
		for (i = scrOfs; i < scrOfs + scrItems; i++) {
			xx = X + 5;
			// If MenuAlign given something except MA_left - it will be centered
			if (Menu->MenuAlign == MA_left) {
				sbTY = 0;
			} else {
				sbTY = ((Cr - X - 4) / 2) - (stringlen(Menu->Items[i].ItemName) * 3);
			}
			if (i - scrOfs == scrPos) {
				// Menu item under cursor
				FillRect(X + 2,yy,Cr,yy + MenuItemHeight - 1,PSet);
				PutStr5x7(xx + sbTY,yy + 1,Menu->Items[i].ItemName,CT_transp_inv);
			} else {
				// Menu item
				PutStr5x7(xx + sbTY,yy + 1,Menu->Items[i].ItemName,CT_opaque);
			}
			yy += MenuItemHeight;
		}

		// Scrollbar
		if (sbH > 0) {
			sbTY = Y + 2 + (((scrPos + scrOfs) * (sbH - sbTH)) / (Menu->NumItems - 1));
			VLine(X + W - 4,Y + 2,Y + 2 + sbH,PSet);
			VLine(X + W - 5,sbTY,sbTY + sbTH,PSet);
			VLine(X + W - 3,sbTY,sbTY + sbTH,PSet);
		}

		UC1701_Flush();

		// Wait for key press
		if (WaitForKey) WaitForKey(TRUE,NULL);

		// Up button
		if (BTN[BTN_UP].cntr || BTN[BTN_UP].state == BTN_Hold) {
			scrPos--;
			if (scrPos < 0) {
				scrPos = 0;
				scrOfs--;
				if (scrOfs < 0) {
					if (Menu->MenuScroll == MS_normal) {
						scrOfs = 0;
						scrPos = 0;
					} else {
						scrOfs = Menu->NumItems - scrItems;
						scrPos = scrItems - 1;
					}
				}
			}
		}

		// Down button
		if (BTN[BTN_DOWN].cntr || BTN[BTN_DOWN].state == BTN_Hold) {
			scrPos++;
			if (scrPos > scrItems - 1) {
				scrPos = scrItems - 1;
				scrOfs++;
				if (scrOfs > Menu->NumItems - scrItems) {
					if (Menu->MenuScroll == MS_normal) {
						scrPos = scrItems - 1;
						scrOfs = Menu->NumItems - scrItems;
					} else {
						scrPos = 0;
						scrOfs = 0;
					}
				}
			}
		}

		// "Enter" button
		if (BTN[BTN_ENTER].cntr) {
			ClearKeys();
			return scrPos + scrOfs;
		}

		// "Escape" button
		if (BTN[BTN_ESCAPE].cntr) {
			ClearKeys();
			return 0xff;
		}
	}
}

// Adjust numeric value with up/down buttons
// input:
//   X - left horizontal coordinate of dialog
//       if this value is negative -> dialog right horizontal coordinate
//   Y - top vertical coordinate of dialog
//       if this value is negative -> dialog bottom vertical coordinate
//   W - width of dialog (zero for auto size)
//   H - height of dialog (zero for auto size)
//   Value - start numeric value
//   Min - minimum value
//   Max - maximum value
//   Step - step change of numeric value
//   unit - unit to add to value
//   CallBack - pointer to function, which will be called on every Value change
//   WaitForKey - function pointer to WaitForKeyPress function
void GUI_NumericScroll(int8_t X, int8_t Y, uint8_t W, uint8_t H, int32_t *Value,
		int32_t Min, int32_t Max, int32_t Step, char *unit, funcPtrParam_TypeDef CallBack,
		funcPtrKeyPress_TypeDef WaitForKey) {
	uint8_t i;
	uint8_t vY; // Numeric value vertical coordinate
	uint8_t mid; // Middle horizontal position
	int32_t val = *Value; // Value to change
	uint8_t frame_width = W;
	uint8_t frame_height = H;

	// Check range
	if (val > Max || val < Min) val = Min + ((Max - Min) >> 1);

	// Calculate dialog width
	if (W == 0) {
		i = stringlen(unit) + numlen(Min);
		mid = stringlen(unit) + numlen(Max);
		if (i > mid) mid = i;
		frame_width = (mid * 6) + 3;
	}

	// Calculate dialog height
	if (H == 0) frame_height = 29;

	// If X negative -> adjust dialog left horizontal coordinate
	if (X < 0) X = (-1 * X) - frame_width;

	// If Y negative -> adjust dialog top horizontal coordinate
	if (Y < 0) Y = (-1 * Y) - frame_height + 1;

	// Middle of dialog
	vY = Y + ((frame_height - 4) / 2) - 1;
	mid = X + (frame_width / 2) + 1; // Middle of the dialog window

	// Dialog frame
	Rect(X,Y,X + frame_width,Y + frame_height - 1,PSet);

	while(1) {
		ClearKeys();

		// Call a callback function
		if (CallBack) CallBack(val);

		// Clear dialog background
		FillRect(X + 1,Y + 1,X + frame_width - 1,Y + frame_height - 2,PReset);

		// Draw up/down arrows
		for (i = 6; i > 0; i --) {
			if (val + Step <= Max) HLine(mid - i,mid + i - 1,vY + i - 10,PSet);
			if (val - Step >= Min) HLine(mid - i,mid + i - 1,vY - i + 16,PSet);
		}

		// Draw value
		i = mid - ((stringlen(unit) + numlen(val)) * 3);
		FillRect(X + 1,vY - 2,X + frame_width - 1,vY + 8,PSet);
		i += PutInt5x7(i,vY,val,CT_transp_inv) + 1;
		PutStr5x7(i,vY,unit,CT_transp_inv);
		UC1701_Flush();

		// Wait for key press
		if (WaitForKey) WaitForKey(TRUE,NULL);

		// Up button
		if (BTN[BTN_UP].cntr || BTN[BTN_UP].state == BTN_Hold) {
			val += Step;
			if (val > Max) val = Max;
			BTN[BTN_UP].cntr = 0;
		}

		// Down button
		if (BTN[BTN_DOWN].cntr || BTN[BTN_DOWN].state == BTN_Hold) {
			val -= Step;
			if (val < Min) val = Min;
			BTN[BTN_DOWN].cntr = 0;
		}

		// "Enter" button
		if (BTN[BTN_ENTER].cntr) {
			*Value = val;
			if (CallBack) CallBack(*Value);
			ClearKeys();
			return;
		}

		// "Escape" button
		if (BTN[BTN_ESCAPE].cntr) {
			// Restore old value by calling callback function with original value
			if (CallBack) CallBack(*Value);
			ClearKeys();
			return;
		}
	}
}

// Main menu with sub menus
void GUI_MainMenu(void) {
	uint8_t mnu_sel;
	uint8_t mnu_sub_sel;
	int32_t mnu_val;

	ClearKeys();
	mnu_sel = 0;
	do {
		mnu_sel = GUI_Menu(0,0,scr_width,scr_height,&mnuMain,mnu_sel,WaitForKeyPress);
		mnu_sub_sel = 0;
		switch (mnu_sel) {
		case 0:
			// Statistics
			do {
				UC1701_Fill(0x00);
				PutStr5x7(0,0,"Statistics...",CT_opaque);
				mnu_sub_sel = GUI_Menu(10,10,scr_width - 20,scr_height - 20,&mnuStatistics,mnu_sub_sel,WaitForKeyPress);
			} while (mnu_sub_sel != 0xff);
			break;
		case 1:
			// GPS
			do {
				UC1701_Fill(0x00);
				PutStr5x7(0,0,"GPS...",CT_opaque);
				mnu_sub_sel = GUI_Menu(10,10,scr_width - 20,scr_height - 20,&mnuGPS,mnu_sub_sel,WaitForKeyPress);
				if (mnu_sub_sel != 0xff) switch (mnu_sub_sel) {
					case 0:
						GUI_Screen_GPSSatsView(WaitForKeyPress);
						break;
					case 1:
						GUI_Screen_GPSInfo(WaitForKeyPress);
						break;
					case 2:
						GUI_Screen_Buffer(GPS_buf,GPS_BUFFER_SIZE,&GPS_new_data,WaitForKeyPress);
						break;
					default:
						break;
				}
			} while (mnu_sub_sel != 0xff);
			break;
		case 2:
			// Settings
			do {
				UC1701_Fill(0x00);
				PutStr5x7(0,0,"Settings...",CT_opaque);
				mnu_sub_sel = GUI_Menu(10,10,scr_width - 20,scr_height - 20,&mnuSettings,mnu_sub_sel,WaitForKeyPress);
				if (mnu_sub_sel != 0xff) switch (mnu_sub_sel) {
					case 0:
						mnu_val = Settings.LCD_brightness;
						GUI_NumericScroll(-100,10,0,0,&mnu_val,0,100,5,"%",callback_Brightness,WaitForKeyPress);
						Settings.LCD_brightness = (uint8_t)mnu_val;
						break;
					case 1:
						mnu_val = Settings.WheelCircumference;
						GUI_NumericScroll(-100,10,0,0,&mnu_val,150,300,1,"cm",NULL,WaitForKeyPress);
						Settings.WheelCircumference = (uint16_t)mnu_val;
						break;
					case 2:
						mnu_val = Settings.GMT_offset;
						GUI_NumericScroll(-100,10,0,0,&mnu_val,-12,13,1,"hr",NULL,WaitForKeyPress);
						Settings.GMT_offset = (int8_t)mnu_val;
						break;
					case 3:
						mnu_val = Settings.altitude_home;
						GUI_NumericScroll(-100,10,0,0,&mnu_val,-1000,9999,1,"m",NULL,WaitForKeyPress);
						Settings.altitude_home = (int16_t)mnu_val;
						break;
					default:
						break;
				}
				SaveSettings_EEPROM();
			} while (mnu_sub_sel != 0xff);
			break;
		default:
			break;
		}
	} while (mnu_sel != 0xff);

	ClearKeys();
}
