#include <stm32l1xx_rcc.h>
#include <stm32l1xx_rtc.h>

#include <wolk.h>
#include <uc1701.h>
#include <GUI.h>
#include <GPS.h>

#include <resources.h>


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
				UC1701_SetPixel(X + i, Y + j);
			} else {
				UC1701_ResetPixel(X + i, Y + j);
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

// Draw number
// input:
//   X, Y - left top corner coordinates
//   number - number to display (0..99999)
//   digits - number of digits to draw ([1..5] if no decimal point, [2..5] with decimal point)
//   decimals - number of decimal digits (0..4)
//   DigitSize - type of digit (DS_Big, DS_Mid, DS_Small)
void GUI_DrawNumber(uint8_t X, uint8_t Y, uint32_t number, uint8_t digits, uint8_t decimals, DigitSize_TypeDef DigitSize) {
	uint8_t dig1,dig2,dig3,dig4,dig5;
	uint8_t dig_w,dig_h;
	uint8_t width;
	uint16_t tmp;

	// Some foolproof
	if (decimals > 4) decimals = 4;
	if (number > 99999) number = 99999;

	dig5 = number % 10;
	tmp  = number / 10;
	dig4 = tmp % 10;
	tmp  = tmp / 10;
	dig3 = tmp % 10;
	tmp  = tmp / 10;
	dig2 = tmp % 10;
	tmp  = tmp / 10;
	dig1 = tmp % 10;

	switch(DigitSize) {
	case DS_Mid:
		width = 63;
		dig_h = 22;
		dig_w = 12;
		break;
	case DS_Small:
		width = 58;
		dig_h = 19;
		dig_w = 11;
		break;
	default:
		width = 83;
		dig_h = 34;
		dig_w = 16;
		break;
	}

	// Maximum width with decimal point and measurement unit
	X += width - (5 - digits) * dig_w;
	if (!decimals) X -= 3;

	// Fifth digit
	X -= dig_w;
	switch(DigitSize) {
	case DS_Mid:
		GUI_MidDig(X,Y,dig5);
		break;
	case DS_Small:
		GUI_SmallDig(X,Y,dig5);
		break;
	default:
		GUI_BigDig(X,Y,dig5);
		break;
	}
	// Decimal point
	if (decimals == 1) {
		X -= 3;
		UC1701_FillRect(X - 1,Y + dig_h - 3,X + 1,Y + dig_h - 1,PSet);
	}
	if ((decimals < 1) && (number < 10)) return;

	// Fourth digit
	X -= dig_w;
	switch(DigitSize) {
	case DS_Mid:
		GUI_MidDig(X,Y,dig4);
		break;
	case DS_Small:
		GUI_SmallDig(X,Y,dig4);
		break;
	default:
		GUI_BigDig(X,Y,dig4);
		break;
	}
	// Decimal point
	if (decimals == 2) {
		X -= 3;
		UC1701_FillRect(X - 1,Y + dig_h - 3,X + 1,Y + dig_h - 1,PSet);
	}
	if ((decimals < 2) && (number < 100)) return;

	// Third digit
	X -= dig_w;
	switch(DigitSize) {
	case DS_Mid:
		GUI_MidDig(X,Y,dig3);
		break;
	case DS_Small:
		GUI_SmallDig(X,Y,dig3);
		break;
	default:
		GUI_BigDig(X,Y,dig3);
		break;
	}
	// Decimal point
	if (decimals == 3) {
		X -= 3;
		UC1701_FillRect(X - 1,Y + dig_h - 3,X + 1,Y + dig_h - 1,PSet);
	}
	if ((decimals < 3) && (number < 1000)) return;

	// Second digit
	X -= dig_w;
	switch(DigitSize) {
	case DS_Mid:
		GUI_MidDig(X,Y,dig2);
		break;
	case DS_Small:
		GUI_SmallDig(X,Y,dig2);
		break;
	default:
		GUI_BigDig(X,Y,dig2);
		break;
	}
	// Decimal point
	if (decimals == 4) {
		X -= 3;
		UC1701_FillRect(X - 1,Y + dig_h - 3,X + 1,Y + dig_h - 1,PSet);
	}
	if ((decimals < 4) && (number < 10000)) return;

	// First digit
	X -= dig_w;
	switch(DigitSize) {
	case DS_Mid:
		GUI_MidDig(X,Y,dig1);
		break;
	case DS_Small:
		GUI_SmallDig(X,Y,dig1);
		break;
	default:
		GUI_BigDig(X,Y,dig1);
		break;
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
		UC1701_FillRect(X,Y + 4,X + 1,Y + 8,PSet);
		UC1701_FillRect(X,Y + 13,X + 1,Y + 17,PSet);
		break;
	case DS_Small:
		GUI_SmallDig(X,Y,dig2);
		GUI_SmallDig(X + dig_w,Y,dig1);
		X += dig_w * 2;
		UC1701_FillRect(X,Y + 3,X + 1,Y + 7,PSet);
		UC1701_FillRect(X,Y + 11,X + 1,Y + 15,PSet);
		break;
	default:
		GUI_BigDig(X,Y,dig2);
		GUI_BigDig(X + dig_w,Y,dig1);
		X += dig_w * 2;
		UC1701_FillRect(X,Y + 8,X + 2,Y + 11,PSet);
		UC1701_FillRect(X,Y + 22,X + 2,Y + 25,PSet);
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
			UC1701_FillRect(X,Y + 4,X + 1,Y + 8,PSet);
			UC1701_FillRect(X,Y + 13,X + 1,Y + 17,PSet);
		}
		break;
	case DS_Small:
		GUI_SmallDig(X,Y,dig2);
		GUI_SmallDig(X + dig_w,Y,dig1);
		if (TimeType == TT_Full) {
			X += dig_w * 2;
			UC1701_FillRect(X,Y + 3,X + 1,Y + 7,PSet);
			UC1701_FillRect(X,Y + 11,X + 1,Y + 15,PSet);
		}
		break;
	default:
		X++;
		GUI_BigDig(X,Y,dig2);
		GUI_BigDig(X + dig_w,Y,dig1);
		if (TimeType == TT_Full) {
			X += dig_w * 2;
			UC1701_FillRect(X,Y + 8,X + 2,Y + 11,PSet);
			UC1701_FillRect(X,Y + 22,X + 2,Y + 25,PSet);
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
	UC1701_Rect(0,4,scr_width - 1,scr_height - 1,PSet);
	UC1701_FillRect(3,0,scr_width - 4,8,PSet);
	UC1701_PutStr5x7(19,1,"Sensor RAW data",CT_transp_inv);
	// Cadence data
	X = 5; Y = 10;
	X += UC1701_PutStr5x7(X,Y,"CDC:",CT_transp) - 1;
	UC1701_PutInt5x7(X,Y,nRF24_Packet.tim_CDC,CT_transp);
	// Speed data
	X = 5; Y += 9;
	X += UC1701_PutStr5x7(X,Y,"SPD c:",CT_transp) - 1;
	X += UC1701_PutInt5x7(X,Y,nRF24_Packet.cntr_SPD,CT_transp) + 8;
	X += UC1701_PutStr5x7(X,Y,"t:",CT_transp) - 1;
	UC1701_PutInt5x7(X,Y,nRF24_Packet.tim_SPD,CT_transp);
	// Packets lost
	X = 5; Y += 9;
	X += UC1701_PutStr5x7(X,Y,"P.Lost:",CT_transp) - 1;
	X += UC1701_PutInt5x7(X,Y,nRF24_Packet.packets_lost,CT_transp) + 5;
	// OBSERVER_TX
	X += UC1701_PutStr5x7(X,Y,"OTX:",CT_transp) - 1;
	X += UC1701_PutHex5x7(X,Y,nRF24_Packet.observe_TX,CT_transp);
	// TX power
	X = 5; Y += 9;
	X += UC1701_PutStr5x7(X,Y,"TX.pwr:",CT_transp) - 1;
	UC1701_PutStr5x7(X,Y,nRF24_TX_POWERS[nRF24_Packet.tx_power],CT_transp);
	// Wakeups
	X = 5; Y += 9;
	X += UC1701_PutStr5x7(X,Y,"Wake:",CT_transp) - 1;
	UC1701_PutInt5x7(X,Y,nRF24_Packet.cntr_wake,CT_transp);
	X = 76;
	X += UC1701_PutStr5x7(X,Y,"RT:",CT_transp) - 1;
	UC1701_PutInt5x7(X,Y,nRF24_Packet.ride_time,CT_transp);
	// Battery
	X = 5; Y += 9;
	X += UC1701_PutStr5x7(X,Y,"Battery:",CT_transp) - 1;
	UC1701_PutChar5x7(X + UC1701_PutIntF5x7(X,Y,nRF24_Packet.vrefint,2,CT_transp),Y,'V',CT_transp);
}

// Screen with current values (trip data)
void GUI_Screen_CurVal1(void) {
	uint8_t X,Y;

	// Frame
	UC1701_Fill(0x00);
	UC1701_Rect(0,4,scr_width - 1,scr_height - 1,PSet);
	UC1701_FillRect(3,0,scr_width - 4,8,PSet);
	UC1701_PutStr5x7(22,1,"Current values",CT_transp_inv);

	X = 7; Y = 10;
	UC1701_PutStr5x7(X,Y,"SPD:",CT_transp);
	UC1701_PutIntF5x7(X + 23,Y,CurData.Speed,1,CT_transp);
	X += 54;
	UC1701_PutStr5x7(X,Y,"CDC:",CT_transp);
	UC1701_PutInt5x7(X + 23,Y,CurData.Cadence,CT_transp);
	X = 7; Y += 9;
	UC1701_PutStr5x7(X,Y,"A.S:",CT_transp);
	UC1701_PutIntF5x7(X + 23,Y,CurData.AvgSpeed,1,CT_transp);
	X += 54;
	UC1701_PutStr5x7(X,Y,"A.C:",CT_transp);
	UC1701_PutInt5x7(X + 23,Y,CurData.AvgCadence,CT_transp);
	X = 7; Y += 9;
	UC1701_PutStr5x7(X,Y,"M.S:",CT_transp);
	UC1701_PutIntF5x7(X + 23,Y,CurData.MaxSpeed,1,CT_transp);
	X += 54;
	UC1701_PutStr5x7(X,Y,"M.C:",CT_transp);
	UC1701_PutInt5x7(X + 23,Y,CurData.MaxCadence,CT_transp);
	X = 7; Y += 9;
	UC1701_PutStr5x7(X,Y,"T.D:",CT_transp);
	UC1701_PutInt5x7(X + 23,Y,CurData.TripDist,CT_transp);
	X = 7; Y += 9;
	UC1701_PutStr5x7(X,Y,"Odo:",CT_transp);
	UC1701_PutInt5x7(X + 23,Y,CurData.Odometer,CT_transp);
	X = 7; Y += 9;
	UC1701_PutStr5x7(X,Y,"Time",CT_transp);
	UC1701_PutTimeSec5x7(X + 26,Y,CurData.TripTime,CT_opaque);
}

// Screen with current values (BMP180 values)
void GUI_Screen_CurVal2(void) {
	uint8_t X,Y;

	// Frame
	UC1701_Fill(0x00);
	UC1701_Rect(0,4,scr_width - 1,scr_height - 1,PSet);
	UC1701_FillRect(3,0,scr_width - 4,8,PSet);
	UC1701_PutStr5x7(46,1,"BMP180",CT_transp_inv);

	X = 4; Y = 10;
	X += UC1701_PutStr5x7(X,Y,"Temperature:",CT_transp) - 1;
	UC1701_PutTemperature5x7(X,Y,CurData.Temperature,CT_transp);
	X = 4; Y += 9;
	X += UC1701_PutStr5x7(X,Y,"Min:",CT_transp) - 1;
	UC1701_PutTemperature5x7(X,Y,CurData.MinTemperature,CT_transp);
	X = 67;
	X += UC1701_PutStr5x7(X,Y,"Max:",CT_transp) - 1;
	UC1701_PutTemperature5x7(X,Y,CurData.MaxTemperature,CT_transp);

	UC1701_HLine(1,scr_width - 2,Y + 10,PSet);

	X = 4; Y += 14;
	X += UC1701_PutStr5x7(X,Y,"Pressure:",CT_transp) - 1;
	UC1701_PutPressure5x7(X,Y,CurData.Pressure,PT_mmHg,CT_transp);
	X = 4; Y += 9;
	X += UC1701_PutStr5x7(X,Y,"Min:",CT_transp) - 1;
	UC1701_PutIntF5x7(X,Y,CurData.MinPressure * 75 / 1000,1,CT_transp);
	X = 67;
	X += UC1701_PutStr5x7(X,Y,"Max:",CT_transp) - 1;
	UC1701_PutIntF5x7(X,Y,CurData.MaxPressure * 75 / 1000,1,CT_transp);
}

// Screen with current values (GPS values)
void GUI_Screen_CurVal3(void) {
	uint8_t X,Y;

	// Frame
	UC1701_Fill(0x00);
	UC1701_Rect(0,4,scr_width - 1,scr_height - 1,PSet);
	UC1701_FillRect(3,0,scr_width - 4,8,PSet);
	UC1701_PutStr5x7(36,1,"GPS stats",CT_transp_inv);

	X = 4; Y = 10;
	X += UC1701_PutStr5x7(X,Y,"Speed:",CT_transp) - 1;
	X += UC1701_PutIntF5x7(X,Y,CurData.GPSSpeed,2,CT_transp);
	UC1701_PutStr5x7(X,Y,"km/h",CT_transp);
	X = 4; Y += 9;
	X += UC1701_PutStr5x7(X,Y,"Max:",CT_transp) - 1;
	X += UC1701_PutIntF5x7(X,Y,CurData.MaxGPSSpeed,2,CT_transp);
	UC1701_PutStr5x7(X,Y,"km/h",CT_transp);

	UC1701_HLine(1,scr_width - 2,Y + 10,PSet);

	X = 4; Y += 14;
	X += UC1701_PutStr5x7(X,Y,"Altitude:",CT_transp) - 1;
	X += UC1701_PutInt5x7(X,Y,CurData.GPSAlt,CT_transp);
	UC1701_PutChar5x7(X,Y,'m',CT_transp);
	X = 4; Y += 9;
	X += UC1701_PutStr5x7(X,Y,"Min:",CT_transp) - 1;
	X += UC1701_PutInt5x7(X,Y,CurData.MinGPSAlt,CT_transp);
	UC1701_PutChar5x7(X,Y,'m',CT_transp);
	X = 4; Y += 9;
	X += UC1701_PutStr5x7(X,Y,"Max:",CT_transp) - 1;
	X += UC1701_PutInt5x7(X,Y,CurData.MaxGPSAlt,CT_transp);
	UC1701_PutChar5x7(X,Y,'m',CT_transp);
}

// Draw speed with 'km/h' mark and AVG speed pace indicator
// input:
//   X, Y - left top corner coordinates
//   speed - speed value (max 999 -> 99.9 km/h)
//   avg - average speed value for pace indicator
// Size: 54 x 33
void GUI_DrawSpeed(uint8_t X, uint8_t Y, uint32_t speed, uint32_t avg) {
	UC1701_FillRect(X,Y,X + 54,Y + 33,PReset);
	GUI_DrawNumber(X,Y,speed / 10,2,0,DS_Big);
	UC1701_FillRect(X + 31,Y + 31,X + 33,Y + 33,PSet);
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

// Draw big cadence value with 'RPM' mark
// input:
//   X, Y - left top corner coordinates
//   cadence - cadence value (max 999)
void GUI_DrawCadence(uint8_t X, uint8_t Y, uint32_t cadence) {
	UC1701_FillRect(X,Y,X + 53,Y + 33,PReset);
	GUI_DrawNumber(X,Y,cadence,3,0,DS_Big);
	GUI_DrawBitmap(X + 49,Y + 8,5,19,&small_signs[15]);
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
	UC1701_FillRect(X,Y + 3,X + 1,Y + 7,PSet);
	UC1701_FillRect(X,Y + 11,X + 1,Y + 15,PSet);
	X += 3;

	// Minutes
	GUI_SmallDig(X,Y,minutes / 10);
	GUI_SmallDig(X + 10,Y,minutes % 10);
	X += 20;
	UC1701_FillRect(X,Y + 3,X + 1,Y + 7,PSet);
	UC1701_FillRect(X,Y + 11,X + 1,Y + 15,PSet);
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
void GUI_DrawGraph(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, const int32_t* data, GraphType_TypeDef GraphType) {
	uint8_t i,bY,pY,YY,offset;
	int32_t min,max;
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
		for(i = 0; i < W; i++) UC1701_SetPixel(X + i,bY - (uint32_t)((data[i] - min) * mY) - offset);
	} else if (GraphType == GT_line) {
		// Continuous line
		pY = bY - (uint32_t)((data[0] - min) * mY) - offset;
		for(i = 1; i < W; i++) {
			YY = bY - (uint32_t)((data[i] - min) * mY) - offset;
			UC1701_Line(X + i - 1,pY,X + i,YY);
			pY = YY;
		}
	} else {
		// Filled graph
		for(i = 0; i < W; i++) UC1701_VLine(X + i,bY - (uint32_t)((data[i] - min) * mY) - offset,bY,PSet);
	}
}

// Draw menu
void GUI_Menu(uint8_t X, uint8_t Y, uint8_t W, uint8_t H, char * Title) {
	UC1701_FillRect(X,Y,X + W - 1,Y + H - 1,PReset);
	UC1701_Rect(X + 2,Y + 4,X + W - 3,Y + H - 3,PSet);
	UC1701_FillRect(X + 5,Y,X + W - 6,Y + 8,PSet);
	UC1701_PutStr5x7(X + 8,Y + 1,Title,CT_opaque_inv);
}

// Change some number interface
void GUI_NumericSet(int32_t Value, int32_t Min, int32_t Max, int32_t Step, char * Title) {
	UC1701_Fill(0x55);
	UC1701_FillRect(10,5,scr_width - 11,scr_height - 5,PReset);
	UC1701_Rect(11,6,scr_width - 12,scr_height - 6,PSet);
	UC1701_FillRect(11,6,scr_width - 12,14,PSet);
	UC1701_PutStr5x7(20,7,Title,CT_opaque_inv);

	GUI_DrawNumber(40,30,Value,3,0,DS_Small);
//	UC1701_PutInt5x7(15,25,Value,CT_opaque);
}

// Put coordinates in format "Ndd.xxxxxx"
//void GUI_PutCoord5x7(uint8_t X, uint8_t Y, uint8_t degree, uint8_t min_i, uint32_t min_f, char ch, CharType_TypeDef CharType) {
void GUI_PutCoord5x7(uint8_t X, uint8_t Y, uint8_t degree, uint32_t seconds, char ch, CharType_TypeDef CharType) {
	UC1701_PutChar5x7(X,Y,ch,CharType);
	UC1701_PutIntF5x7(X + 6,Y,(degree * 1000000) + (seconds / 60),6,CharType);
}

void GUI_DrawGPSInfo(void) {
	uint8_t X,Y;

	UC1701_Fill(0x00);

	X = 0; Y = 0;
	X += UC1701_PutTimeSec5x7(X,Y,GPSData.time,CT_opaque) + 5;
	X += UC1701_PutDate5x7(X,Y,GPSData.date,CT_opaque);

	X = 0; Y += 8;
	X += UC1701_PutStr5x7(X,Y,"Fix:",CT_opaque) - 1;
	if (GPSData.fix != 2 && GPSData.fix != 3)
		X += UC1701_PutStr5x7(X,Y,"NA",CT_opaque) + 5;
	else {
		UC1701_PutInt5x7(X,Y,GPSData.fix,CT_opaque);
		UC1701_PutChar5x7(X + 6,Y,'D',CT_opaque);
		X += 17;
	}
	X += UC1701_PutStr5x7(X,Y,"Qlty:",CT_opaque) - 1;
	UC1701_PutChar5x7(X,Y,GPSData.fix_quality + '0',CT_opaque);
	X += 11;
	X += UC1701_PutStr5x7(X,Y,"Mode:",CT_opaque) - 1;
	UC1701_PutChar5x7(X,Y,GPSData.mode,CT_opaque);

	X = 0; Y += 8;
	GUI_PutCoord5x7(X,Y,GPSData.latitude_degree,GPSData.latitude_seconds,GPSData.latitude_char,CT_opaque);
	X = 63;
	GUI_PutCoord5x7(X,Y,GPSData.longitude_degree,GPSData.longitude_seconds,GPSData.longitude_char,CT_opaque);

	X = 0; Y += 8;
	X += UC1701_PutStr5x7(X,Y,"Alt:",CT_opaque) - 1;
	X += UC1701_PutInt5x7(X,Y,GPSData.altitude,CT_opaque) + 5;
	X += UC1701_PutStr5x7(X,Y,"Spd:",CT_opaque) - 1;
	X += UC1701_PutIntF5x7(X,Y,GPSData.speed,2,CT_opaque);

	X = 0; Y += 8;
	X += UC1701_PutStr5x7(X,Y,"Crs:",CT_opaque) - 1;
	X += UC1701_PutIntF5x7(X,Y,GPSData.course,2,CT_opaque) + 5;
	X += UC1701_PutStr5x7(X,Y,"Sat:",CT_opaque) - 1;
	X += UC1701_PutInt5x7(X,Y,GPSData.sats_used,CT_opaque);
	UC1701_PutChar5x7(X,Y,'/',CT_opaque);
	X += 6;
	X += UC1701_PutInt5x7(X,Y,GPSData.sats_view,CT_opaque);

	X = 0; Y += 8;
	X += UC1701_PutStr5x7(X,Y,"PDOP:",CT_opaque) - 1;
	X += UC1701_PutIntF5x7(X,Y,GPSData.PDOP,2,CT_opaque) + 5;

	X = 0; Y += 8;
	X += UC1701_PutStr5x7(X,Y,"HDOP:",CT_opaque) - 1;
	X += UC1701_PutIntF5x7(X,Y,GPSData.HDOP,2,CT_opaque) + 5;
	X += UC1701_PutStr5x7(X,Y,"VDOP:",CT_opaque) - 1;
	X += UC1701_PutIntF5x7(X,Y,GPSData.VDOP,2,CT_opaque) + 5;
}
