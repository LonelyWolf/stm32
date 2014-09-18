#include <stm32l1xx_rcc.h>
#include <stm32l1xx_rtc.h>

#include <wolk.h>
#include <uc1701.h>
#include <GUI.h>
#include <GPS.h>
#include <RTC.h>
#include <log.h>
#include <EEPROM.h>
#include <beeper.h> // FIXME: just for debug, remove it later

#include <font5x7.h>
#include <font7x10.h>
#include <resources.h>


bool GUI_refresh;                     // Flag to refresh GUI
bool GUI_new_BMP180;                  // BMP180 data updated


// Callback function for change display brightness settings
void callback_Brightness(int32_t param) {
	Settings.LCD_brightness = substDisplayBrightness.Items[param].subst_val;
	UC1701_SetBacklight(Settings.LCD_brightness);
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
void GUI_Screen_SensorRAW(funcPtrKeyPress_TypeDef WaitForKey) {
	uint8_t X,Y;

	do {
		// Frame
		UC1701_Fill(0x00);
		Rect(0,4,scr_width - 1,scr_height - 1,PSet);
		HLine(4,scr_width - 5,4,PReset);
		PutStr(19,1,"Sensor RAW data",fnt5x7);
		InvertRect(4,0,scr_width - 8,9);
		// Cadence data
		X = 5; Y = 10;
		X += PutStr(X,Y,"CDC tim:",fnt5x7) - 1;
		PutInt(X,Y,nRF24_Packet.tim_CDC,fnt5x7);
		// Speed data
		X = 5; Y += 9;
		X += PutStr(X,Y,"SPD tim:",fnt5x7) - 1;
		PutInt(X,Y,nRF24_Packet.tim_SPD,fnt5x7);
		X = 5; Y += 9;
		X += PutStr(X,Y,"SPD cntr:",fnt5x7) - 1;
		PutInt(X,Y,nRF24_Packet.cntr_SPD,fnt5x7);
		// Wake-ups
		X = 5; Y += 9;
		X += PutStr(X,Y,"Wake:",fnt5x7) - 1;
		PutInt(X,Y,nRF24_Packet.cntr_wake,fnt5x7);
		// Battery
		X = 5; Y += 9;
		X += PutStr(X,Y,"Battery:",fnt5x7) - 1;
		PutChar(X + PutIntF(X,Y,nRF24_Packet.vrefint,2,fnt5x7),Y,'V',fnt5x7);
		UC1701_Flush();

		if (WaitForKey) WaitForKey(TRUE,&GUI_refresh,GUI_TIMEOUT); else return;
		if (_time_idle > GUI_TIMEOUT) BTN[BTN_ESCAPE].cntr++;
		GUI_refresh = FALSE;
		if (!BTN[BTN_ESCAPE].cntr) ClearKeys();
	} while (!BTN[BTN_ESCAPE].cntr);

	BTN[BTN_ESCAPE].cntr = 0;
	UC1701_Fill(0x00);
}

// Screen with current values (trip data)
void GUI_Screen_CurVal1(funcPtrKeyPress_TypeDef WaitForKey) {
	uint8_t X,Y;

	do {
		// Frame
		UC1701_Fill(0x00);
		Rect(0,4,scr_width - 1,scr_height - 1,PSet);
		HLine(4,scr_width - 5,4,PReset);
		PutStr(18,1,"Trip statistics",fnt5x7);
		InvertRect(4,0,scr_width - 8,9);

		X = 4; Y = 10;
		X += PutStr(X,Y,"SPD:",fnt5x7) - 1;
		PutIntF(X,Y,CurData.Speed,1,fnt5x7);
		X = 63;
		X += PutStr(X,Y,"CDC:",fnt5x7) - 1;
		PutInt(X,Y,CurData.Cadence,fnt5x7);

		X = 4; Y += 9;
		X += PutStr(X,Y,"A.S:",fnt5x7) - 1;
		PutIntF(X,Y,CurData.AvgSpeed,1,fnt5x7);
		X = 63;
		X += PutStr(X,Y,"A.C:",fnt5x7) - 1;
		PutInt(X,Y,CurData.AvgCadence,fnt5x7);
		X = 4; Y += 9;
		X += PutStr(X,Y,"M.S:",fnt5x7) - 1;
		PutIntF(X,Y,CurData.MaxSpeed,1,fnt5x7);
		X = 63;
		X += PutStr(X,Y,"M.C:",fnt5x7) - 1;
		PutInt(X,Y,CurData.MaxCadence,fnt5x7);
		X = 4; Y += 9;
		X += PutStr(X,Y,"T.D:",fnt5x7) - 1;
		X += PutIntF(X,Y,CurData.TripDist / 100,3,fnt5x7);
		PutStr(X,Y,"km",fnt5x7);
		X = 4; Y += 9;
		X += PutStr(X,Y,"Odo:",fnt5x7) - 1;
		X += PutIntF(X,Y,CurData.Odometer / 10000,1,fnt5x7);
		PutStr(X,Y,"km",fnt5x7);
		X = 4; Y += 9;
		X += PutStr(X,Y,"Time:",fnt5x7) - 1;
		GUI_PutTimeSec(X,Y,CurData.TripTime,fnt5x7);
		UC1701_Flush();

		if (WaitForKey) WaitForKey(TRUE,&GUI_refresh,GUI_TIMEOUT); else return;
		if (_time_idle > GUI_TIMEOUT) BTN[BTN_ESCAPE].cntr++;
		GUI_refresh = FALSE;
		if (!BTN[BTN_ESCAPE].cntr) ClearKeys();
	} while (!BTN[BTN_ESCAPE].cntr);

	BTN[BTN_ESCAPE].cntr = 0;
	UC1701_Fill(0x00);
}

// Screen with current values (BMP180 values)
void GUI_Screen_CurVal2(funcPtrKeyPress_TypeDef WaitForKey) {
	uint8_t X,Y;

	do {
		// Frame
		UC1701_Fill(0x00);
		Rect(0,4,scr_width - 1,scr_height - 1,PSet);
		HLine(4,scr_width - 5,4,PReset);
		PutStr(24,1,"BMP180 values",fnt5x7);
		InvertRect(4,0,scr_width - 8,9);

		X = 4; Y = 10;
		X += PutStr(X,Y,"Temperature:",fnt5x7) - 1;
		GUI_PutTemperature(X,Y,CurData.Temperature,fnt5x7);
		X = 4; Y += 9;
		X += PutStr(X,Y,"Min:",fnt5x7) - 1;
		GUI_PutTemperature(X,Y,CurData.MinTemperature,fnt5x7);
		X = 67;
		X += PutStr(X,Y,"Max:",fnt5x7) - 1;
		GUI_PutTemperature(X,Y,CurData.MaxTemperature,fnt5x7);

		HLine(1,scr_width - 2,Y + 10,PSet);

		X = 4; Y += 14;
		X += PutStr(X,Y,"Pressure:",fnt5x7) - 1;
		GUI_PutPressure(X,Y,CurData.Pressure,PT_mmHg,fnt5x7);
		X = 4; Y += 9;
		X += PutStr(X,Y,"Min:",fnt5x7) - 1;
		PutIntF(X,Y,CurData.MinPressure * 75 / 1000,1,fnt5x7);
		X = 67;
		X += PutStr(X,Y,"Max:",fnt5x7) - 1;
		PutIntF(X,Y,CurData.MaxPressure * 75 / 1000,1,fnt5x7);

		UC1701_Flush();

		if (WaitForKey) WaitForKey(TRUE,&GUI_new_BMP180,GUI_TIMEOUT); else return;
		if (_time_idle > GUI_TIMEOUT) BTN[BTN_ESCAPE].cntr++;
		GUI_new_BMP180 = FALSE;
		if (!BTN[BTN_ESCAPE].cntr) ClearKeys();
	} while (!BTN[BTN_ESCAPE].cntr);

	BTN[BTN_ESCAPE].cntr = 0;
	UC1701_Fill(0x00);
}

// Screen with current values (GPS values)
void GUI_Screen_CurVal3(funcPtrKeyPress_TypeDef WaitForKey) {
	uint8_t X,Y;

	do {
		// Frame
		UC1701_Fill(0x00);
		Rect(0,4,scr_width - 1,scr_height - 1,PSet);
		HLine(4,scr_width - 5,4,PReset);
		PutStr(9,1,"GPS current values",fnt5x7);
		InvertRect(4,0,scr_width - 8,9);

		X = 4; Y = 10;
		X += PutStr(X,Y,"Speed:",fnt5x7) - 1;
		X += PutIntF(X,Y,CurData.GPSSpeed,2,fnt5x7);
		PutStr(X,Y,"km/h",fnt5x7);
		X = 4; Y += 9;
		X += PutStr(X,Y,"Max:",fnt5x7) - 1;
		X += PutIntF(X,Y,CurData.MaxGPSSpeed,2,fnt5x7);
		PutStr(X,Y,"km/h",fnt5x7);

		HLine(1,scr_width - 2,Y + 10,PSet);

		X = 4; Y += 14;
		X += PutStr(X,Y,"Altitude:",fnt5x7) - 1;
		X += PutInt(X,Y,CurData.GPSAlt,fnt5x7);
		PutChar(X,Y,'m',fnt5x7);
		X = 4; Y += 9;
		X += PutStr(X,Y,"Min:",fnt5x7) - 1;
		X += PutInt(X,Y,CurData.MinGPSAlt,fnt5x7);
		PutChar(X,Y,'m',fnt5x7);
		X = 4; Y += 9;
		X += PutStr(X,Y,"Max:",fnt5x7) - 1;
		X += PutInt(X,Y,CurData.MaxGPSAlt,fnt5x7);
		PutChar(X,Y,'m',fnt5x7);

		UC1701_Flush();

		if (WaitForKey) WaitForKey(TRUE,&GPS_new_data,GUI_TIMEOUT); else return;
		if (_time_idle > GUI_TIMEOUT) BTN[BTN_ESCAPE].cntr++;
		GPS_new_data = FALSE;
		if (!BTN[BTN_ESCAPE].cntr) ClearKeys();
	} while (!BTN[BTN_ESCAPE].cntr);

	BTN[BTN_ESCAPE].cntr = 0;
	UC1701_Fill(0x00);
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
			X = PutStr(12,21,"No satellites",fnt5x7);
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

		if (WaitForKey) WaitForKey(TRUE,&GPS_new_data,GUI_TIMEOUT); else return;
		if (_time_idle > GUI_TIMEOUT) BTN[BTN_ESCAPE].cntr++;
		GPS_new_data = FALSE;
		if (!BTN[BTN_ESCAPE].cntr) ClearKeys();
	} while (!BTN[BTN_ESCAPE].cntr);

	BTN[BTN_ESCAPE].cntr = 0;
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
		X += GUI_PutTimeSec(X,Y,GPSData.time,fnt5x7) + 5;
		GUI_PutDate(X,Y,GPSData.date,fnt5x7);

		X = 4; Y += 8;
		X += PutStr(X,Y,"Fix:",fnt5x7) - 1;
		if (GPSData.fix != 2 && GPSData.fix != 3)
			X += PutStr(X,Y,"NA",fnt5x7) + 5;
		else {
			X += PutInt(X,Y,GPSData.fix,fnt5x7);
			X += PutChar(X,Y,'D',fnt5x7);
			X += 5;
		}
		X += PutStr(X,Y,"Qlty:",fnt5x7) - 1;
		X += PutChar(X,Y,GPSData.fix_quality + '0',fnt5x7);
		X += 5;
		X += PutStr(X,Y,"Mode:",fnt5x7) - 1;
		PutChar(X,Y,GPSData.mode,fnt5x7);

		Y += 8;
		GUI_PutCoord(4,Y,GPSData.latitude,GPSData.latitude_char,fnt5x7);
		GUI_PutCoord(67,Y,GPSData.longitude,GPSData.longitude_char,fnt5x7);

		X = 4; Y += 8;
		X += PutStr(X,Y,"Alt:",fnt5x7) - 1;
		X += PutInt(X,Y,GPSData.altitude,fnt5x7) + 5;
		X += PutStr(X,Y,"Spd:",fnt5x7) - 1;
		PutIntF(X,Y,GPSData.speed,2,fnt5x7);

		X = 4; Y += 8;
		X += PutStr(X,Y,"Crs:",fnt5x7) - 1;
		X += PutIntF(X,Y,GPSData.course,2,fnt5x7) + 5;
		X += PutStr(X,Y,"Sat:",fnt5x7) - 1;
		X += PutInt(X,Y,GPSData.sats_used,fnt5x7);
		X += PutChar(X,Y,'/',fnt5x7);
		PutInt(X,Y,GPSData.sats_view,fnt5x7);

		X = 4; Y += 8;
		X += PutStr(X,Y,"PDOP:",fnt5x7) - 1;
		X += PutIntF(X,Y,GPSData.PDOP,2,fnt5x7) + 3;
		X += PutInt(X,Y,GPS_sentences_parsed,fnt5x7) + 2;
		X += PutInt(X,Y,GPS_sentences_unknown,fnt5x7) + 3;
		X += PutInt(X,Y,GPSData.dgps_age,fnt5x7) + 2;
		PutInt(X,Y,GPSData.dgps_id,fnt5x7);

/*
		Since PDOP = sqrt(pow(HDOP,2) + pow(VDOP,2)) there is no sense to output these values
		X = 4; Y += 8;
		X += PutStr(X,Y,"HDOP:",fnt5x7) - 1;
		X += PutIntF(X,Y,GPSData.HDOP,2,fnt5x7) + 5;
		X += PutStr(X,Y,"VDOP:",fnt5x7) - 1;
		X += PutIntF(X,Y,GPSData.VDOP,2,fnt5x7) + 5;
*/

		X = 4; Y += 8;
		X += PutStr(X,Y,"Accuracy:",fnt5x7) - 1;
		X += PutIntU(X,Y,GPSData.accuracy / 100,fnt5x7);
		PutChar(X,Y,'m',fnt5x7);

		UC1701_Flush();

		if (WaitForKey) WaitForKey(TRUE,&GPS_new_data,GUI_TIMEOUT); else return;
		if (_time_idle > GUI_TIMEOUT) BTN[BTN_ESCAPE].cntr++;
		GPS_new_data = FALSE;
		if (!BTN[BTN_ESCAPE].cntr) ClearKeys();
	} while (!BTN[BTN_ESCAPE].cntr);

	BTN[BTN_ESCAPE].cntr = 0;
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
			X += PutChar(X,Y,pBuf[i++],fnt5x7);
			if (X > scr_width - 6) {
				X  = 0;
				Y += 8;
			}
		} while (Y < scr_height - 8 && i < BufSize);
		HLine(0,scr_width - 1,scr_height - 7,PSet);

		X = PutIntULZ3x5(0,scr_height - 5,pos,0) + 10;
		X += PutIntULZ3x5(X,scr_height - 5,pos + i,0) + 10;
		PutIntULZ3x5(X,scr_height - 5,BufSize,0);

		UC1701_Flush();

		// Wait for key press
		if (WaitForKey) WaitForKey(TRUE,UpdateFlag,GUI_TIMEOUT);
		if (_time_idle > GUI_TIMEOUT) BTN[BTN_ESCAPE].cntr++;
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

	BTN[BTN_ESCAPE].cntr = 0;
	UC1701_Fill(0x00);
}

// Draw speed with 'km/h' mark and AVG speed pace indicator
// input:
//   X, Y - left top corner coordinates
//   speed - speed value (max 999 -> 99.9 km/h)
//   avg - average speed value for pace indicator
// Size: 54 x 33
void GUI_DrawSpeed(int8_t X, int8_t Y, uint32_t speed, uint32_t avg) {
	if (speed > 999) speed = 999;
	FillRect(X,Y,X + 54,Y + 33,PReset);
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

	hours = time / 3600;
	if (hours > 99) {
		hours   = 99;
		minutes = 59;
		seconds = 59;
	} else {
		minutes = (time / 60) % 60;
		seconds = time % 60;
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
//   data - pointer to array with graph values
//   GraphType - type of graph (GT_dot, GT_fill, GT_line)
// note: values array must be equal to or less than graph width
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

// Put coordinates in format "Cdd.xxxx"
// input:
//   X,Y - top left coordinates of the text
//   coord - coordinate in degrees
//   ch - coordinate character
//   Font - pointer to font
uint8_t GUI_PutCoord(uint8_t X, uint8_t Y, uint32_t coord, char ch, const Font_TypeDef *Font) {
	uint8_t pX = X;

	X += PutChar(X,Y,ch,Font);
	X += PutIntF(X,Y,coord,6,Font);

	return X - pX;
}

// Draw time
// input:
//   X,Y - top left coordinates of the text
//   time - time in seconds (max value = 3599999 seconds or 999:59:59)
//   Font - pointer to font
uint8_t GUI_PutTimeSec(uint8_t X, uint8_t Y, uint32_t time, const Font_TypeDef *Font) {
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

	if (hours < 10)	X += PutChar(X,Y,'0',Font);
	X += PutInt(X,Y,hours,Font);
	X += PutChar(X,Y,':',Font) - 1;

	if (minutes < 10) X += PutChar(X,Y,'0',Font);
	X += PutInt(X,Y,minutes,Font);
	X += PutChar(X,Y,':',Font) - 1;

	if (seconds < 10) X += PutChar(X,Y,'0',Font);
	X += PutInt(X,Y,seconds,Font);

	return X - pX;
}

// Draw date
// input:
//   X,Y - top left coordinates of text
//   date - date in format DDMMYYYY
//   Font - pointer to font
uint8_t GUI_PutDate(uint8_t X, uint8_t Y, uint32_t date, const Font_TypeDef *Font) {
	uint8_t pX = X;
	uint16_t dig;

	// Day
	dig = date / 1000000;
	X += PutIntLZ(X,Y,dig,2,Font) - 1;
	X += PutChar(X,Y,'.',Font) - 1;

	// Month
	dig = (date - (dig * 1000000)) / 10000;
	X += PutIntLZ(X,Y,dig,2,Font) - 1;
	X += PutChar(X,Y,'.',Font) - 1;

	// Year
	dig = date % 10000;
	X += PutIntLZ(X,Y,dig,4,Font);

	return X - pX;
}

// Print pressure with 'mmHg'
// input:
//   X,Y - top left coordinates of text
//   pressure - pressure value in Pa
//   PressureType - what units pressure should be displayed (PT_hPa, PT_mmHg)
//   Font - pointer to font
uint8_t GUI_PutPressure(uint8_t X, uint8_t Y, int32_t pressure, PressureType_TypeDef PressureType,
		const Font_TypeDef *Font) {
	uint8_t pX = X;

	if (PressureType == PT_mmHg) {
		pressure = pressure * 75 / 1000;
		X += PutIntF(X,Y,pressure,1,Font);
		X += PutStr(X,Y,"mmHg",Font);
	} else {
		X += PutIntF(X,Y,pressure,2,Font);
		X += PutStr(X,Y,"hPa",Font);
	}

	return X - pX;
}

// Print temperature value with Celsius sign
// input:
//   X,Y - top left coordinates of text
//   temperature - temperature value in Celsius degree
//   Font - pointer to font
uint8_t GUI_PutTemperature(uint8_t X, uint8_t Y, int32_t temperature, const Font_TypeDef *Font) {
	uint8_t pX = X;

	if (temperature < 0) {
		HLine(X,X + 2,Y + 3,PSet);
		temperature *= -1;
		X += 4;
	}
	X += PutInt(X,Y,temperature / 10,Font);

	// Decimal point
	Rect(X,Y + Font->font_Height - 2,X + 1,Y + Font->font_Height - 1,PSet);
	X += 3;

	// Temperature fractional
	X += PutInt(X,Y,temperature % 10,Font);

	// Celsius degree sign
	HLine(X + 1,X + 2,Y,PSet);
	HLine(X + 1,X + 2,Y + 3,PSet);
	VLine(X,Y + 1,Y + 2,PSet);
	VLine(X + 3,Y + 1,Y + 2,PSet);
	PutChar(X + 5,Y,'C',Font);

	return X - pX + 10;
}

// Draw vertical menu with scrolling
// input:
//   X,Y - top left menu corner coordinates
//   W,H - width and height of menu
//   Font - pointer to font
//   Menu - pointer to Menu_TypeDef structure, containing menu
//   StartPos - start menu position
//   WaitForKey - function pointer to WaitForKeyPress function
// return:
//   selected menu position or 0xff if "Escape" key pressed
uint8_t GUI_Menu(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, const Font_TypeDef *Font,
		MenuFrame_TypeDef MenuFrame, const Menu_TypeDef *Menu, uint8_t StartPos,
		funcPtrKeyPress_TypeDef WaitForKey) {
	uint8_t i;
	int8_t scrPos; // Cursor position related to screen
	int8_t scrOfs; // Menu scroll (number of first displayed item)
	uint8_t yy;
	uint8_t Cr; // Right side of the menu cursor
	uint8_t Cl; // Left side of the menu cursor
	uint8_t miYoffs; // First menu item vertical offset
	uint8_t sbTH; // Height of scrollbar thumb
	uint8_t sbH; // Height of scrollbar
	uint8_t sbTY; // Vertical position of the scrollbar thumb
	uint8_t scrItems; // Visible menu items
	uint8_t MIH; // Menu item height

	// Menu frame
	switch (MenuFrame) {
		case MF_rect:
			Rect(X,Y,X + W - 1,Y + H - 1,PSet);
			X += 1;
			Y += 1;
			W -= 2;
			H -= 2;
			break;
		case MF_top_bottom:
			HLine(X,X + W - 1,Y,PSet);
			HLine(X,X + W - 1,Y + H - 1,PSet);
			Y += 1;
			H -= 2;
		default:
			// No frame
			break;
	}

	Cl = X;
	Cr = Cl + W - 1;
	if (MenuFrame == MF_rect) {
		Cl++;
		Cr--;
	}

	MIH = Font->font_Height + 2;
	scrItems = (H - 3) / MIH;
	if (scrItems < Menu->NumItems) {
		sbTH = ((H - 3) * scrItems ) / Menu->NumItems;
		if (sbTH < 2) sbTH = 2;
		Cr -= 4;
		sbH = H - 3;
	} else {
		sbTH = 0;
		sbH  = 0;
	}
	if (scrItems > Menu->NumItems) scrItems = Menu->NumItems;
	miYoffs = ((H - 6 - (scrItems * MIH)) / 2) - 1;

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

	while(1) {
		ClearKeys();

		// Clear menu background
		FillRect(X,Y,X + W - 1,Y + H - 1,PReset);

		// Draw scrollbar
		if (sbH > 0) {
			sbTY = Y + 1 + (((scrPos + scrOfs) * (sbH - sbTH)) / (Menu->NumItems - 1));
			VLine(Cr + 3,Y + 1,Y + 1 + sbH,PSet);
			VLine(Cr + 4,sbTY,sbTY + sbTH,PSet);
			VLine(Cr + 2,sbTY,sbTY + sbTH,PSet);
		}

		// Draw menu items
		yy = Y + 4 + miYoffs;
		for (i = scrOfs; i < scrOfs + scrItems; i++) {
			// If MenuAlign given something other except MA_left - it will be centered
			if (Menu->MenuAlign == MA_left) {
				sbTY = 1;
			} else {
				sbTY = ((Cr - Cl + 1) / 2) - ((stringlen(Menu->Items[i].ItemName) * (Font->font_Width + 1)) / 2);
			}
			PutStr(Cl + sbTY,yy + 1,Menu->Items[i].ItemName,Font);
			if (i - scrOfs == scrPos) InvertRect(Cl,yy,Cr - Cl + 1,MIH);
			yy += MIH;
		}
		UC1701_Flush();

		// Wait for key press
		if (WaitForKey) WaitForKey(FALSE,NULL,GUI_MENU_TIMEOUT);
		if (_time_idle >= GUI_MENU_TIMEOUT) return 0xff;
		_time_idle = 0;

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
//   Subst - pointer to array with substitute values (NULL if none)
//   CallBack - pointer to function, which will be called on every Value change
//   WaitForKey - function pointer to WaitForKeyPress function
// note: when Subst pointer is not NULL, Min, Max, Step and unit parameters are ignored
void GUI_NumericScroll(int8_t X, int8_t Y, uint8_t W, uint8_t H, const Font_TypeDef *Font,
		int32_t *Value, int32_t Min, int32_t Max, int32_t Step,	char *unit,
		const Subst_TypeDef *Subst, funcPtrParam_TypeDef CallBack, funcPtrKeyPress_TypeDef WaitForKey) {
	uint8_t i;
	uint8_t mX; // Middle horizontal position
	uint8_t mY; // Middle vertical position
	uint8_t mT; // Text horizontal position
	int32_t val = *Value; // Value to change
	int32_t sV; // Unchanged value
	uint8_t frame_width = W;
	uint8_t frame_height = H;

	// Calculate dialog width
	if (W == 0) {
		mX = stringlen(unit);
		if (Subst) {
			mY = 0;
			for (i = 0; i < Subst->NumItems; i++) {
				mT = stringlen(Subst->Items[i].subst_str);
				if (mT > mY) mY = mT;
			}
		} else {
			mY = numlen(Min) + mX;
			mT = numlen(Max) + mX;
			if (mT > mY) mY = mT;
		}
		frame_width = (mY * (Font->font_Width + 1)) + 3;
	}

	// Check range
	if (Subst) {
		Min = 0;
		Max = Subst->NumItems - 1;
		Step = 1;
		for (i = 0; i < Subst->NumItems; i++) if (val == Subst->Items[i].subst_val) {
			val = i;
			break;
		}
	}
	sV = val;
	if (val > Max || val < Min) val = Min + ((Max - Min) >> 1);

	// Calculate dialog height
	if (H == 0) frame_height = 20 + Font->font_Height;

	// If X negative -> adjust dialog left horizontal coordinate
	if (X < 0) X = (-1 * X) - frame_width + 1;

	// If Y negative -> adjust dialog top horizontal coordinate
	if (Y < 0) Y = (-1 * Y) - frame_height + 1;

	// Middle of dialog frame
	mY = Y + (frame_height / 2);
	mX = X + (frame_width / 2);

	// Vertical text position
	mT = mY - (Font->font_Height / 2);

	// Dialog frame
	Rect(X,Y,X + frame_width - 1,Y + frame_height - 1,PSet);

	while(1) {
		ClearKeys();

		// Clear dialog background
		FillRect(X + 1,Y + 1,X + frame_width - 2,Y + frame_height - 2,PReset);

		// Draw up/down arrows
		for (i = 6; i > 0; i --) {
			if (val + Step <= Max) HLine(mX - i,mX + i - 1,Y + i + 1,PSet);
			if (val - Step >= Min) HLine(mX - i,mX + i - 1,Y + frame_height - i - 2,PSet);
		}

		// Draw value
		if (Subst) {
			i = mX - (stringlen(Subst->Items[val].subst_str) * ((Font->font_Width + 1) / 2)) + 1;
		    PutStr(i,mT,Subst->Items[val].subst_str,Font);
		} else {
			i = mX - ((stringlen(unit) + numlen(val)) * ((Font->font_Width + 1) / 2)) + 1;
			i += PutInt(i,mT,val,Font);
			PutStr(i,mT,unit,Font);
		}
		InvertRect(X + 1,Y + 9,frame_width - 2,frame_height - 18);
		UC1701_Flush();

		// Execute a callback function
		if (CallBack) CallBack(val);

		// Wait for key press
		if (WaitForKey) WaitForKey(FALSE,NULL,GUI_MENU_TIMEOUT);
		if (_time_idle >= GUI_MENU_TIMEOUT) {
			// Execute callback function with original value
			if (CallBack) CallBack(sV);
			*Value = sV;
			return;
		}
		_time_idle = 0;

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
			// Execute callback function with original value
			if (CallBack) CallBack(sV);
			*Value = sV;
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
	uint32_t i;
	uint32_t log_num;

	ClearKeys();
	mnu_sel = 0;
	do {
		mnu_sel = GUI_Menu(0,0,scr_width,scr_height,fnt7x10,MF_none,
				&mnuMain,mnu_sel,WaitForKeyPress);
		mnu_sub_sel = 0;
		switch (mnu_sel) {
		case 0:
			// Statistics
			do {
				UC1701_Fill(0x00);
				PutStr(11,1,"Statistics...",fnt7x10);
				InvertRect(0,0,scr_width,12);
				mnu_sub_sel = GUI_Menu(5,11,scr_width - 10,scr_height - 11,fnt7x10,MF_rect,
						&mnuStatistics,mnu_sub_sel,WaitForKeyPress);
				if (mnu_sub_sel != 0xff) switch (mnu_sub_sel) {
					case 0:
						// Current trip values
						GUI_Screen_CurVal1(WaitForKeyPress);
						break;
					case 1:
						// Sensor RAW values
						GUI_Screen_SensorRAW(WaitForKeyPress);
						break;
					case 2:
						// BMP180 current values
						GUI_Screen_CurVal2(WaitForKeyPress);
						break;
					case 3:
						// GPS current values
						GUI_Screen_CurVal3(WaitForKeyPress);
						break;
					default:
						break;
				}
				if (_time_idle >= GUI_TIMEOUT) mnu_sel = mnu_sub_sel = 0xff;
			} while (mnu_sub_sel != 0xff);
			break;
		case 1:
			// GPS
			do {
				UC1701_Fill(0x00);
				PutStr(39,1,"GPS...",fnt7x10);
				InvertRect(0,0,scr_width,12);
				mnu_sub_sel = GUI_Menu(10,11,scr_width - 20,scr_height - 18,fnt7x10,MF_rect,
						&mnuGPS,mnu_sub_sel,WaitForKeyPress);
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
				if (_time_idle >= GUI_TIMEOUT) mnu_sel = mnu_sub_sel = 0xff;
			} while (mnu_sub_sel != 0xff);
			break;
		case 2:
			// Settings
			do {
				UC1701_Fill(0x00);
				PutStr(19,1,"Settings...",fnt7x10);
				InvertRect(0,0,scr_width,12);
				mnu_sub_sel = GUI_Menu(10,11,scr_width - 20,scr_height - 11,fnt7x10,MF_rect,
						&mnuSettings,mnu_sub_sel,WaitForKeyPress);
				if (mnu_sub_sel != 0xff) switch (mnu_sub_sel) {
					case 0:
						do {
							UC1701_Fill(0x00);
							PutStr(19,1,"Display...",fnt7x10);
							InvertRect(0,0,scr_width,12);
							mnu_sub_sel = GUI_Menu(2,11,scr_width - 4,scr_height - 22,fnt7x10,MF_rect,
									&mnuDisplay,mnu_sub_sel,WaitForKeyPress);
							if (mnu_sub_sel != 0xff) switch (mnu_sub_sel) {
								case 0:
									mnu_val = Settings.LCD_brightness;
									GUI_NumericScroll(-100,10,0,0,fnt7x10,&mnu_val,0,0,0,NULL,&substDisplayBrightness,callback_Brightness,WaitForKeyPress);
									Settings.LCD_brightness = substDisplayBrightness.Items[mnu_val].subst_val;
									break;
								case 1:
									mnu_val = Settings.LCD_timeout;
									GUI_NumericScroll(-100,10,0,0,fnt7x10,&mnu_val,0,0,0,NULL,&substDisplayTimeout,NULL,WaitForKeyPress);
									Settings.LCD_timeout = substDisplayTimeout.Items[mnu_val].subst_val;
									break;
								case 2:
									break;
								default:
									break;
							}
						} while (mnu_sub_sel != 0xff);
						mnu_sub_sel = 0;
						break;
					case 1:
						mnu_val = Settings.WheelCircumference;
						GUI_NumericScroll(-100,10,0,0,fnt7x10,&mnu_val,100,300,1,"cm",NULL,NULL,WaitForKeyPress);
						Settings.WheelCircumference = (uint16_t)mnu_val;
						break;
					case 2:
						mnu_val = Settings.GMT_offset;
						GUI_NumericScroll(-100,10,0,0,fnt7x10,&mnu_val,-12,13,1,"hr",NULL,NULL,WaitForKeyPress);
						Settings.GMT_offset = (int8_t)mnu_val;
						break;
					case 3:
						mnu_val = Settings.altitude_home;
						GUI_NumericScroll(-100,10,0,0,fnt7x10,&mnu_val,-1000,9999,1,"m",NULL,NULL,WaitForKeyPress);
						Settings.altitude_home = (int16_t)mnu_val;
						break;
					default:
						break;
				}
				SaveBuffer_EEPROM(DATA_EEPROM_START_ADDR,(uint32_t *)&Settings,sizeof(Settings));
			} while (mnu_sub_sel != 0xff);
			break;
		case 3:
			// Logging
			if (_SD_present) {
				// Show this menu only if SD card present
				do {
					UC1701_Fill(0x00);
					PutStr(23,1,"Logging...",fnt7x10);
					InvertRect(0,0,scr_width,12);
					mnu_sub_sel = GUI_Menu(10,11,scr_width - 20,scr_height - 18,fnt7x10,MF_rect,
							&mnuLogging,mnu_sub_sel,WaitForKeyPress);
					if (mnu_sub_sel != 0xff) switch (mnu_sub_sel) {
						case 0:
							if (!_logging) {
								log_num = 0;
								i = LOG_NewFile(&log_num);
								if (i == LOG_OK) {
									_logging = TRUE;
									// Write header
									LOG_WriteStr("WBC log #");
									LOG_WriteInt(log_num);
									LOG_WriteStr(" (start ");
									LOG_WriteDate(RTC_Date.RTC_Date,RTC_Date.RTC_Month,RTC_Date.RTC_Year);
									LOG_WriteStr(" ");
									LOG_WriteTime(RTC_Time.RTC_Hours,RTC_Time.RTC_Minutes,RTC_Time.RTC_Seconds);
									LOG_WriteStr(")\r\nDate;Time;Wake;SPD.c;_prev_cntr_SPD;dbg_cntr_diff;dbg_prev_cntr;Speed;Cadence;_cdc0;_cdc1;_cdc2;_cdc3;_cdc4;_cdc_avg;Odometer;");
									LOG_WriteStr("DateTime;lat;lon;ele;GPS_spd;Course;pdop;vdop;hdop;fix;sat;Vref;Temperature;Pressure\r\n");
								}
								UC1701_Fill(0x00);
								PutStr(0,0,"RES:",fnt7x10);
								PutIntU(32,0,i,fnt7x10);
								PutStr(0,12,"NUM:",fnt7x10);
								PutIntU(32,12,log_num,fnt7x10);
								PutStr(0,40,_logging ? "TRUE" : "FALSE",fnt7x10);
								UC1701_Flush();
								Delay_ms(2000);
								mnu_sub_sel = 0xff;
								mnu_sel = 0xff;
							} else {
								BEEPER_Enable(4321,10);
							}
							break;
						case 1:
							if (_logging) {
								LOG_FileSync();
							} else {
								BEEPER_Enable(4321,10);
							}
							mnu_sub_sel = 0xff;
							break;
						case 2:
							if (_logging) {
								// Write file ending
								LOG_WriteStr("WBC log end (");
								LOG_WriteDate(RTC_Date.RTC_Date,RTC_Date.RTC_Month,RTC_Date.RTC_Year);
								LOG_WriteStr(" ");
								LOG_WriteTime(RTC_Time.RTC_Hours,RTC_Time.RTC_Minutes,RTC_Time.RTC_Seconds);
								LOG_WriteStr(")\r\n");
								LOG_FileSync();
							} else {
								BEEPER_Enable(4321,10);
							}
							_logging = FALSE;
							mnu_sub_sel = 0xff;
							break;
						default:
							break;
					}
					if (_time_idle >= GUI_TIMEOUT) mnu_sel = mnu_sub_sel = 0xff;
				} while (mnu_sub_sel != 0xff);
			} else {
				BEEPER_Enable(4321,10);
				mnu_sub_sel = 0xff;
			}
			break;
		case 4:
			// Debug
			do {
				UC1701_Fill(0x00);
				PutStr(31,1,"Debug...",fnt7x10);
				InvertRect(0,0,scr_width,12);
				mnu_sub_sel = GUI_Menu(0,12,scr_width,scr_height - 13,fnt7x10,MF_none,
						&mnuDebug,mnu_sub_sel,WaitForKeyPress);
				if (mnu_sub_sel != 0xff) switch (mnu_sub_sel) {
					case 0:
						UC1701_Init();
						UC1701_Contrast(4,24);
						UC1701_Orientation(scr_normal);
						UC1701_SetBacklight(Settings.LCD_brightness);
						break;
					case 1:
						RTC_SetWakeUp(10);
						GUI_ScreenSaver();
			 			RTC_SetWakeUp(1);
						break;
					case 2:
						BEEPER_PlayTones(tones_SMB);
						break;
					case 3:
						GPS_SendCommand(PMTK_CMD_HOT_START); // GPS hot start
						break;
					case 4:
						GPS_SendCommand(PMTK_EASY_ENABLE); // GPS EASY enable
						break;
					case 5:
						GPS_SendCommand(PMTK_EASY_DISABLE); // GPS EASY disable
						break;
					case 9:
						// Initiate SYSRESETREQ signal to reboot the system
						NVIC_SystemReset();
						break;
					default:
						break;
				}
			} while (mnu_sub_sel != 0xff);
		default:
			break;
		}
		if (_time_idle >= GUI_MENU_TIMEOUT) mnu_sel = 0xff;
	} while (mnu_sel != 0xff);

	ClearKeys();
}

// Show screensaver
void GUI_ScreenSaver(void) {
	bool key_pressed = FALSE;
	uint32_t itts = 0;

	do {
		UC1701_Fill(0x00);
//		GUI_DrawTime(30,11,&RTC_Time,TT_Short,DS_Big);
		GUI_DrawTime(12,11,&RTC_Time,TT_Full,DS_Big);
		GUI_PutDate(26,scr_height - 12,(RTC_Date.RTC_Date * 1000000) + (RTC_Date.RTC_Month * 10000) +
				RTC_Date.RTC_Year + 2000,fnt7x10);
//		GUI_PutDate(33,scr_height - 12,(RTC_Date.RTC_Date * 1000000) + (RTC_Date.RTC_Month * 10000) +
//				RTC_Date.RTC_Year + 2000,fnt5x7);
		PutInt(1,1,itts,fnt5x7);
		UC1701_Flush();
		SleepStop(); // Enter STOP mode (deep sleep)
		key_pressed = BTN[0].cntr || BTN[1].cntr || BTN[2].cntr || BTN[3].cntr ||
				BTN[0].state == BTN_Hold || BTN[1].state == BTN_Hold ||
				BTN[2].state == BTN_Hold || BTN[3].state == BTN_Hold;
 		itts++;
	} while (!key_pressed);
	ClearKeys();
}
