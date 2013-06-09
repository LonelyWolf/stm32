// RTC
typedef struct {
	unsigned char  hour;
	unsigned char  min;
	unsigned char  sec;
} RTC_Time;


uint32_t TimeToRTC(RTC_Time *time);
void RTCToTime(uint32_t cnt, RTC_Time *time);
void RTC_Init(void);
