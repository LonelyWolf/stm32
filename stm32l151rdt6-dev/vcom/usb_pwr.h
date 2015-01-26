// Define to prevent recursive inclusion
#ifndef __USB_PWR_H
#define __USB_PWR_H


// Exported types

// USB resume state machine states
typedef enum _RESUME_STATE {
	RESUME_EXTERNAL,
	RESUME_INTERNAL,
	RESUME_LATER,
	RESUME_WAIT,
	RESUME_START,
	RESUME_ON,
	RESUME_OFF,
	RESUME_ESOF
} RESUME_STATE;

// USB device state
typedef enum _DEVICE_STATE {
	UNCONNECTED,
	ATTACHED,
	POWERED,
	SUSPENDED,
	ADDRESSED,
	CONFIGURED
} DEVICE_STATE;


// External variables
extern __IO DEVICE_STATE bDeviceState; // USB device status
extern __IO bool fSuspendEnabled; // true when suspend is possible


// Exported functions
void Suspend(void);
void Resume_Init(void);
void Resume(RESUME_STATE eResumeSetVal);
RESULT PowerOn(void);
RESULT PowerOff(void);

#endif  // __USB_PWR_H
