#ifndef __USB_CDC_H
#define __USB_CDC_H


#include  "usbd_ioreq.h"


// CDC endpoints
#define CDC_IN_EP                                   0x81 // EP1 for data IN
#define CDC_OUT_EP                                  0x03 // EP1 for data OUT
#define CDC_CMD_EP                                  0x82 // EP2 for CDC commands

// CDC endpoints packet sizes
#define CDC_DATA_HS_MAX_PACKET_SIZE                 512  // USB HighSpeed endpoint IN & OUT packet size
#define CDC_DATA_FS_MAX_PACKET_SIZE                 64   // USB FullSpeed endpoint IN & OUT packet size
#define CDC_CMD_PACKET_SIZE                         8    // Control endpoint packet size

#define USB_CDC_CONFIG_DESC_SIZ                     67
#define CDC_DATA_HS_IN_PACKET_SIZE                  CDC_DATA_HS_MAX_PACKET_SIZE
#define CDC_DATA_HS_OUT_PACKET_SIZE                 CDC_DATA_HS_MAX_PACKET_SIZE

#define CDC_DATA_FS_IN_PACKET_SIZE                  CDC_DATA_FS_MAX_PACKET_SIZE
#define CDC_DATA_FS_OUT_PACKET_SIZE                 CDC_DATA_FS_MAX_PACKET_SIZE

// CDC definitions
#define CDC_SEND_ENCAPSULATED_COMMAND               0x00
#define CDC_GET_ENCAPSULATED_RESPONSE               0x01
#define CDC_SET_COMM_FEATURE                        0x02
#define CDC_GET_COMM_FEATURE                        0x03
#define CDC_CLEAR_COMM_FEATURE                      0x04
#define CDC_SET_LINE_CODING                         0x20
#define CDC_GET_LINE_CODING                         0x21
#define CDC_SET_CONTROL_LINE_STATE                  0x22
#define CDC_SEND_BREAK                              0x23


// Public type definitions

// CDC line coding structure
typedef struct {
	uint32_t bitrate;
	uint8_t  format;
	uint8_t  paritytype;
	uint8_t  datatype;
} USBD_CDC_LineCodingTypeDef;

// CDC interface structure
typedef struct _USBD_CDC_Itf {
	int8_t (* Init)    (void);
	int8_t (* DeInit)  (void);
	int8_t (* Control) (uint8_t, uint8_t *, uint16_t);
	int8_t (* Receive) (uint8_t *, uint32_t *);
} USBD_CDC_ItfTypeDef;

// CDC class handle
typedef struct {
	uint32_t data[CDC_DATA_HS_MAX_PACKET_SIZE / 4]; // Force 32-bit alignment
	uint8_t  CmdOpCode;
	uint8_t  CmdLength;
	uint8_t  *RxBuffer;
	uint8_t  *TxBuffer;
	uint32_t RxLength;
	uint32_t TxLength;
  	__IO uint32_t TxState;
	__IO uint32_t RxState;
} USBD_CDC_HandleTypeDef;


// Public variables
extern USBD_ClassTypeDef USBD_CDC;


// Function prototypes
uint8_t USBD_CDC_RegisterInterface(USBD_HandleTypeDef *pdev, USBD_CDC_ItfTypeDef *fops);
uint8_t USBD_CDC_SetTxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pBuf, uint16_t length);
uint8_t USBD_CDC_SetRxBuffer(USBD_HandleTypeDef *pdev, uint8_t *pBuf);
uint8_t USBD_CDC_ReceivePacket(USBD_HandleTypeDef *pdev);
uint8_t USBD_CDC_TransmitPacket(USBD_HandleTypeDef *pdev);

#endif // __USB_CDC_H
