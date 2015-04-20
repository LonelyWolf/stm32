#include "usbd_cdc_if.h"


// USB device handle
USBD_HandleTypeDef USBD_Device;

// The CDC line coding parameters
USBD_CDC_LineCodingTypeDef LineCoding = {
		115200, // baud rate
		0x00,   // stop bits: 1
		0x00,   // parity: none
		0x08    // number of bits: 8
};

// RX buffer size (must be greater or equal to CDC_DATA_FS_OUT_PACKET_SIZE)
#define APP_RX_DATA_SIZE  1024
// TX buffer size
#define APP_TX_DATA_SIZE  1024

// Received data from CDC will be stored in this buffer
uint8_t UserRXBuf[APP_RX_DATA_SIZE];
uint32_t UserRXBufCur = 0; // Points to next available character in RX buffer
uint32_t UserRXBufLen = 0; // Counts number of valid characters in RX buffer

// Buffer to store data to be transfered to CDC
uint8_t UserTXBuf[APP_TX_DATA_SIZE];
uint32_t UserTXBufCur = 0;


// Private function prototypes
static int8_t CDC_Itf_Init(void);
static int8_t CDC_Itf_DeInit(void);
static int8_t CDC_Itf_Control(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Itf_Receive(uint8_t* pbuf, uint32_t *Len);


// CDC interface control procedures
USBD_CDC_ItfTypeDef USBD_CDC_fops = {
		CDC_Itf_Init,
		CDC_Itf_DeInit,
		CDC_Itf_Control,
		CDC_Itf_Receive
};


// Initialize the CDC media low layer
static int8_t CDC_Itf_Init(void) {
	// Set Application Buffers
	USBD_CDC_SetTxBuffer(&USBD_Device,UserTXBuf,0);
	USBD_CDC_SetRxBuffer(&USBD_Device,UserRXBuf);

	// Initialize buffer pointers
	UserRXBufCur = 0;
	UserRXBufLen = 0;

	return (USBD_OK);
}

// Deinitialize the CDC media low layer
static int8_t CDC_Itf_DeInit(void) {
	return (USBD_OK);
}

// Manage the CDC class requests
// input:
//   cmd - command code
//   pBuf - pointer to the data buffer containing command data (request parameters)
//   length - size of buffer
static int8_t CDC_Itf_Control(uint8_t cmd, uint8_t* pbuf, uint16_t length) {
	switch (cmd) {
		case CDC_SEND_ENCAPSULATED_COMMAND:
			// Add your code here
			break;
		case CDC_GET_ENCAPSULATED_RESPONSE:
			// Add your code here
			break;
		case CDC_SET_COMM_FEATURE:
			// Add your code here
			break;
		case CDC_GET_COMM_FEATURE:
			// Add your code here
			break;
		case CDC_CLEAR_COMM_FEATURE:
			// Add your code here
			break;
		case CDC_SET_LINE_CODING:
			LineCoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) | (pbuf[2] << 16) | (pbuf[3] << 24));
			LineCoding.format     = pbuf[4];
			LineCoding.paritytype = pbuf[5];
			LineCoding.datatype   = pbuf[6];
			// Set the new configuration
			// ComPort_Config(); // Configure the UART peripheral according to new settings
			break;
		case CDC_GET_LINE_CODING:
			pbuf[0] = (uint8_t)(LineCoding.bitrate);
			pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
			pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
			pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
			pbuf[4] = LineCoding.format;
			pbuf[5] = LineCoding.paritytype;
			pbuf[6] = LineCoding.datatype;
			// Add your code here
			break;
		case CDC_SET_CONTROL_LINE_STATE:
			// Add your code here
			break;
		case CDC_SEND_BREAK:
			// Add your code here
			break;
		default:
			break;
	}

	return (USBD_OK);
}

// Process data received from CDC interface
// input:
//   pBuf - pointer to the data buffer
//   length - pointer to the variable with a size of buffer
static int8_t CDC_Itf_Receive(uint8_t* pBuf, uint32_t *length) {
	uint32_t i;

	// Copy received data from the RX buffer to the RX buffer
	for (i = 0; i < *length; i++) UserTXBuf[i] = UserRXBuf[i];

	// Transmit received data back (loop-back mode)
	USBD_CDC_SetTxBuffer(&USBD_Device,&UserTXBuf[0],*length);
	USBD_CDC_TransmitPacket(&USBD_Device);

	// Initiate next USB packet transfer
	USBD_CDC_SetRxBuffer(&USBD_Device,&UserRXBuf[0]);
	USBD_CDC_ReceivePacket(&USBD_Device);

	return (USBD_OK);
}

// Send data from specified buffer over CDC interface
// input:
//   pBuf - pointer to the data buffer
//   length - size of buffer
uint8_t CDC_Itf_Transmit(uint8_t* pBuf, uint16_t length) {
	uint8_t result = USBD_OK;

	USBD_CDC_SetTxBuffer(&USBD_Device,pBuf,length);
	result = USBD_CDC_TransmitPacket(&USBD_Device);

	return result;
}
