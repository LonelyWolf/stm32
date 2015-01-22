// Define to prevent recursive inclusion
#ifndef __USB_PROP_H
#define __USB_PROP_H


// Exported types
typedef struct {
	uint32_t bitrate;
	uint8_t format;
	uint8_t paritytype;
	uint8_t datatype;
} LINE_CODING;


// Exported defines
#define USBdev_GetConfiguration          NOP_Process
//#define USBdev_SetConfiguration          NOP_Process
#define USBdev_GetInterface              NOP_Process
#define USBdev_SetInterface              NOP_Process
#define USBdev_GetStatus                 NOP_Process
#define USBdev_ClearFeature              NOP_Process
#define USBdev_SetEndPointFeature        NOP_Process
#define USBdev_SetDeviceFeature          NOP_Process
//#define USBdev_SetDeviceAddress          NOP_Process

// USB CDC class-specific request codes
#define SEND_ENCAPSULATED_COMMAND   0x00
#define GET_ENCAPSULATED_RESPONSE   0x01
#define SET_COMM_FEATURE            0x02
#define GET_COMM_FEATURE            0x03
#define CLEAR_COMM_FEATURE          0x04
#define SET_LINE_CODING             0x20
#define GET_LINE_CODING             0x21
#define SET_CONTROL_LINE_STATE      0x22
#define SEND_BREAK                  0x23


// Function prototypes
void USBdev_Init(void);
void USBdev_Reset(void);
void USBdev_SetConfiguration(void);
void USBdev_ClearFeature(void);
void USBdev_SetDeviceAddress(void);
void USBdev_Status_In(void);
void USBdev_Status_Out(void);
RESULT USBdev_Data_Setup(uint8_t);
RESULT USBdev_NoData_Setup(uint8_t);
RESULT USBdev_Get_Interface_Setting(uint8_t Interface, uint8_t AlternateSetting);
uint8_t *USBdev_GetDeviceDescriptor(uint16_t);
uint8_t *USBdev_GetConfigDescriptor(uint16_t);
uint8_t *USBdev_GetStringDescriptor(uint16_t);

uint8_t *VCOM_GetLineCoding(uint16_t Length);
uint8_t *VCOM_SetLineCoding(uint16_t Length);

#endif // __USB_PROP_H
