// Define to prevent recursive inclusion
#ifndef __USB_LIB_H
#define __USB_LIB_H


#include "hw_config.h"
#include "usb_type.h"
#include "usb_core.h"
#include "usb_regs.h"
#include "usb_pwr.h"


// Public defines
#define HIBYTE(word)        (word >> 8)
#define LOBYTE(word)        (word  & 0xFF)

// Definition of "USBbmRequestType"
#define REQUEST_TYPE      0x60  // Mask to get request type
#define STANDARD_REQUEST  0x00  // Standard request
#define CLASS_REQUEST     0x20  // Class request
#define VENDOR_REQUEST    0x40  // Vendor request
#define RECIPIENT         0x1F  // Mask to get recipient


// Public variables
//  Points to the DEVICE_INFO structure of current device
//  The purpose of this register is to speed up the execution
extern DEVICE_INFO *pInformation;
//  Points to the DEVICE_PROP structure of current device
//  The purpose of this register is to speed up the execution
extern DEVICE_PROP*	pProperty;
extern void(*pEpInt_IN[ 7])(void);   // Handles IN  interrupts
extern void(*pEpInt_OUT[7])(void);   // Handles OUT interrupts
extern uint8_t EPindex;              // The number of current endpoint, it will be used to specify an endpoint
//  Temporary save the state of Rx & Tx status.
//  Whenever the Rx or Tx state is changed, its value is saved
//  in this variable first and will be set to the EPRB or EPRA
//  at the end of interrupt process
extern uint16_t SaveState;
extern uint16_t wInterrupt_Mask; // Contains interrupt mask
extern USER_STANDARD_REQUESTS *pUser_Standard_Requests;
extern __IO uint16_t SaveRState; // cells saving status during interrupt servicing
extern __IO uint16_t SaveTState; // cells saving status during interrupt servicing
extern __IO DEVICE_STATE bDeviceState; // USB device status


// Function prototypes
void USB_Init(void);
void CTR_LP(void);
void CTR_HP(void);
void UserToPMABufferCopy(uint8_t *pbUsrBuf, uint16_t wPMABufAddr, uint16_t wNBytes);
void PMAToUserBufferCopy(uint8_t *pbUsrBuf, uint16_t wPMABufAddr, uint16_t wNBytes);
uint32_t USB_SIL_Init(void);
uint32_t USB_SIL_Write(uint8_t bEpAddr, uint8_t* pBufferPointer, uint32_t wBufferSize);
uint32_t USB_SIL_Read(uint8_t bEpAddr, uint8_t* pBufferPointer);

#endif // __USB_LIB_H
