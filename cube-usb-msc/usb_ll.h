#ifndef __USB_LL_H
#define __USB_LL_H


#include "stm32l1xx.h"


// USB peripheral structure
typedef struct {
	__IO uint16_t EP0R;           // USB Endpoint 0 register,       Address offset: 0x00
	__IO uint16_t RESERVED0;      // Reserved
	__IO uint16_t EP1R;           // USB Endpoint 1 register,       Address offset: 0x04
	__IO uint16_t RESERVED1;      // Reserved
	__IO uint16_t EP2R;           // USB Endpoint 2 register,       Address offset: 0x08
	__IO uint16_t RESERVED2;      // Reserved
	__IO uint16_t EP3R;           // USB Endpoint 3 register,       Address offset: 0x0C
	__IO uint16_t RESERVED3;      // Reserved
	__IO uint16_t EP4R;           // USB Endpoint 4 register,       Address offset: 0x10
	__IO uint16_t RESERVED4;      // Reserved
	__IO uint16_t EP5R;           // USB Endpoint 5 register,       Address offset: 0x14
	__IO uint16_t RESERVED5;      // Reserved
	__IO uint16_t EP6R;           // USB Endpoint 6 register,       Address offset: 0x18
	__IO uint16_t RESERVED6;      // Reserved
	__IO uint16_t EP7R;           // USB Endpoint 7 register,       Address offset: 0x1C
	__IO uint16_t RESERVED7[17];  // Reserved
	__IO uint16_t CNTR;           // Control register,              Address offset: 0x40
	__IO uint16_t RESERVED8;      // Reserved
	__IO uint16_t ISTR;           // Interrupt status register,     Address offset: 0x44
	__IO uint16_t RESERVED9;      // Reserved
	__IO uint16_t FNR;            // Frame number register,         Address offset: 0x48
	__IO uint16_t RESERVEDA;      // Reserved
	__IO uint16_t DADDR;          // Device address register,       Address offset: 0x4C
	__IO uint16_t RESERVEDB;      // Reserved
	__IO uint16_t BTABLE;         // Buffer table address register, Address offset: 0x50
	__IO uint16_t RESERVEDC;      // Reserved
} USB_TypeDef;

// Endpoint bit positions
#define USB_EP_CTR_RX      (uint32_t)0x00008000 // Correct TRansfer RX
#define USB_EP_DTOG_RX     (uint32_t)0x00004000 // Data TOGGLE RX
#define USB_EPRX_STAT      (uint32_t)0x00003000 // RX STATus bit field
#define USB_EP_SETUP       (uint32_t)0x00000800 // SETUP
#define USB_EP_T_FIELD     (uint32_t)0x00000600 // TYPE
#define USB_EP_KIND        (uint32_t)0x00000100 // KIND
#define USB_EP_CTR_TX      (uint32_t)0x00000080 // EndPoint Correct TRansfer TX
#define USB_EP_DTOG_TX     (uint32_t)0x00000040 // Data TOGGLE TX
#define USB_EPTX_STAT      (uint32_t)0x00000030 // TX STATus bit field
#define USB_EPADDR_FIELD   (uint32_t)0x0000000F // ADDRess FIELD

// EndPoint register MASK (no toggle fields)
#define USB_EP_TYPE_MASK   (uint32_t)0x00000600 // TYPE Mask
#define USB_EP_BULK        (uint32_t)0x00000000 // BULK
#define USB_EP_CONTROL     (uint32_t)0x00000200 // CONTROL
#define USB_EP_ISOCHRONOUS (uint32_t)0x00000400 // ISOCHRONOUS
#define USB_EP_INTERRUPT   (uint32_t)0x00000600 // INTERRUPT
#define USB_EP_T_MASK      (~USB_EP_T_FIELD & USB_EPREG_MASK)

// EP_TYPE[1:0] endpoint type
#define USB_EPREG_MASK     (USB_EP_CTR_RX | USB_EP_SETUP | USB_EP_T_FIELD |USB_EP_KIND |USB_EP_CTR_TX | USB_EPADDR_FIELD)

// EP_KIND endpoint KIND
#define USB_EPKIND_MASK    (~USB_EP_KIND & USB_EPREG_MASK)

// STAT_TX[1:0] status for TX transfer
#define USB_EP_TX_DIS      (uint32_t)0x00000000 // TX disabled
#define USB_EP_TX_STALL    (uint32_t)0x00000010 // TX stalled
#define USB_EP_TX_NAK      (uint32_t)0x00000020 // TX NAKed
#define USB_EP_TX_VALID    (uint32_t)0x00000030 // TX valid
#define USB_EPTX_DTOG1     (uint32_t)0x00000010 // TX data toggle bit1
#define USB_EPTX_DTOG2     (uint32_t)0x00000020 // TX data toggle bit2
#define USB_EPTX_DTOGMASK  (USB_EPTX_STAT | USB_EPREG_MASK)

// STAT_RX[1:0] status for RX transfer
#define USB_EP_RX_DIS      (uint32_t)0x00000000 // RX disabled
#define USB_EP_RX_STALL    (uint32_t)0x00001000 // RX stalled
#define USB_EP_RX_NAK      (uint32_t)0x00002000 // RX NAKed
#define USB_EP_RX_VALID    (uint32_t)0x00003000 // RX valid
#define USB_EPRX_DTOG1     (uint32_t)0x00001000 // RX data toggle bit1
#define USB_EPRX_DTOG2     (uint32_t)0x00002000 // RX data toggle bit2
#define USB_EPRX_DTOGMASK  (USB_EPRX_STAT | USB_EPREG_MASK)


// USB initialization structure
typedef struct {
	uint32_t dev_endpoints;           // Device Endpoints number, depends on the used USB core
	                                  // This parameter must be a number between Min_Data = 1 and Max_Data = 15
	uint32_t speed;                   // USB Core speed, can be any value of USB_Core_Speed
	uint32_t EP0_MPS;                 // Set the Endpoint 0 Max Packet size, can be any value of USB_EP0_MPS
	uint32_t PHY_itface;              // Select the used PHY interface, can be any value of USB_Core_PHY
	uint32_t low_power_enable;        // Enable or disable Low Power mode
} USB_InitTypeDef;

// USB endpoint
typedef struct {
	uint8_t   num;          // Endpoint number, must be a number between Min_Data = 1 and Max_Data = 15
	uint8_t   is_in;        // Endpoint direction, must be a number between Min_Data = 0 and Max_Data = 1
	uint8_t   is_stall;     // Endpoint stall condition, must be a number between Min_Data = 0 and Max_Data = 1
	uint8_t   type;         // Endpoint type, can be any value of USB_EP_Type
	uint16_t  pmaadress;    // PMA Address, can be any value between Min_addr = 0 and Max_addr = 1K
	uint16_t  pmaaddr0;     // PMA Address0, can be any value between Min_addr = 0 and Max_addr = 1K
	uint16_t  pmaaddr1;     // PMA Address1, can be any value between Min_addr = 0 and Max_addr = 1K
	uint8_t   doublebuffer; // Double buffer enable (0 or 1)
	uint32_t  maxpacket;    // Endpoint Max packet size, must be a number between Min_Data = 0 and Max_Data = 64KB
	uint8_t   *xfer_buff;   // Pointer to transfer buffer
	uint32_t  xfer_len;     // Current transfer length
	uint32_t  xfer_count;   // Partial transfer length in case of multiple packet transfer
} USB_EPTypeDef;

// USB state
typedef enum {
	USB_RESET    = 0x00,
	USB_READY    = 0x01,
	USB_ERROR    = 0x02,
	USB_BUSY     = 0x03,
	USB_TIMEOUT  = 0x04
} USB_StateTypeDef;

// USB double buffered endpoint direction
typedef enum {
	USB_EP_DBUF_OUT,
	USB_EP_DBUF_IN,
	USB_EP_DBUF_ERR,
} USB_EP_DBUF_DIR;

// USB handle
typedef struct {
	USB_TypeDef           *Instance;    // Register base address
	USB_InitTypeDef        Init;        // USB required parameters
	__IO uint8_t           USB_Address; // USB Address
	USB_EPTypeDef          IN_ep[8];    // IN endpoint parameters
	USB_EPTypeDef          OUT_ep[8];   // OUT endpoint parameters
	__IO USB_StateTypeDef  State;       // USB communication state
	uint32_t               Setup[12];   // Setup packet buffer
	void                  *pData;       // Pointer to upper stack Handler
} USB_HandleTypeDef;

// USB device FS
#define USB_BASE          (APB1PERIPH_BASE + 0x00005C00) // USB_IP Peripheral Registers base address
#define USB_PMAADDR       (APB1PERIPH_BASE + 0x00006000) // USB_IP Packet Memory Area base address

// USB device FS
#define USB               ((USB_TypeDef *)USB_BASE)

// USB EP0 MPS
#define DEP0CTL_MPS_64    0
#define DEP0CTL_MPS_32    1
#define DEP0CTL_MPS_16    2
#define DEP0CTL_MPS_8     3

#define USB_EP0MPS_64     DEP0CTL_MPS_64
#define USB_EP0MPS_32     DEP0CTL_MPS_32
#define USB_EP0MPS_16     DEP0CTL_MPS_16
#define USB_EP0MPS_08     DEP0CTL_MPS_8

// USB endpoint type
#define USB_EP_TYPE_CTRL  0 // Control
#define USB_EP_TYPE_ISOC  1 // Isochronous
#define USB_EP_TYPE_BULK  2 // Bulk
#define USB_EP_TYPE_INTR  3 // Interrupt

// USB Core PHY
#define USB_PHY_EMBEDDED  2

// USB Core Speed
#define USB_SPEED_HIGH    0 // Not Supported
#define USB_SPEED_FULL    2

// Endpoint numbers
#define USB_ENDP0         (uint8_t)0
#define USB_ENDP1         (uint8_t)1
#define USB_ENDP2         (uint8_t)2
#define USB_ENDP3         (uint8_t)3
#define USB_ENDP4         (uint8_t)4
#define USB_ENDP5         (uint8_t)5
#define USB_ENDP6         (uint8_t)6
#define USB_ENDP7         (uint8_t)7

// Endpoint Kind
#define USB_SNG_BUF       0
#define USB_DBL_BUF       1

// USB buffer table beginning
#define BTABLE_ADDRESS                  (0x000)


// Magic macros

// Write to the specified USB endpoint register (EPxR)
#define USB_SET_ENDPOINT(USBx,bEpNum,wRegValue) (*(&(USBx)->EP0R + (bEpNum) * 2) = (uint16_t)(wRegValue))
// Read from the specified USB endpoint register (EPxR)
#define USB_GET_ENDPOINT(USBx,bEpNum)           (*(&(USBx)->EP0R + (bEpNum) * 2))

/**
  * @brief  sets the type in the endpoint register(bits EP_TYPE[1:0])
  * @param  USBx: USB peripheral instance register address.
  * @param  bEpNum: Endpoint Number.
  * @param  wType: Endpoint Type.
  * @retval None
  */
// Set the type in the endpoint register (bits EP_TYPE[1:0])
#define USB_SET_EPTYPE(USBx,bEpNum,wType) (USB_SET_ENDPOINT((USBx),(bEpNum),((USB_GET_ENDPOINT((USBx),(bEpNum)) & USB_EP_T_MASK) | (wType))))

// Read transmission byte count for the specified endpoint
#define USB_EP_TX_CNT(USBx,bEpNum) ((uint32_t *)(((((USBx)->BTABLE + (bEpNum) * 8) + 2) * 2) + ((uint32_t)(USBx) + 0x400)))
// Read reception byte count for the specified endpoint
#define USB_EP_RX_CNT(USBx,bEpNum) ((uint32_t *)(((((USBx)->BTABLE + (bEpNum) * 8) + 6) * 2) + ((uint32_t)(USBx) + 0x400)))

// Write counter for the TX buffer.
#define USB_SET_EP_TX_CNT(USBx,bEpNum,wCount) (*USB_EP_TX_CNT((USBx),(bEpNum)) = (wCount))

// Read counter of the TX buffer
#define USB_GET_EP_TX_CNT(USBx,bEpNum) ((uint16_t)(*USB_EP_TX_CNT((USBx),(bEpNum))) & 0x3ff)
// Read counter of the RX buffer
#define USB_GET_EP_RX_CNT(USBx,bEpNum) ((uint16_t)(*USB_EP_RX_CNT((USBx),(bEpNum))) & 0x3ff)

// Write buffer 0 address of a double buffer endpoind
#define USB_SET_EP_DBUF0_CNT(USBx,bEpNum,bDir,wCount) {       \
	if ((bDir) == USB_EP_DBUF_OUT) {                          \
		USB_SET_EP_RX_DBUF0_CNT((USBx),(bEpNum),(wCount));    \
	} else if((bDir) == USB_EP_DBUF_IN) {                     \
		*USB_EP_TX_CNT((USBx),(bEpNum)) = (uint32_t)(wCount); \
	}                                                         \
}

// Write buffer 1 address of a double buffer endpoind
#define USB_SET_EP_DBUF1_CNT(USBx,bEpNum,bDir,wCount) {       \
	if ((bDir) == USB_EP_DBUF_OUT) {                          \
		USB_SET_EP_RX_CNT((USBx),(bEpNum),(wCount));          \
	} else if ((bDir) == USB_EP_DBUF_IN) {                    \
		*USB_EP_RX_CNT((USBx),(bEpNum)) = (uint32_t)(wCount); \
	}                                                         \
}

// Write buffer 0 and 1 address of a double buffer endpoint
#define USB_SET_EP_DBUF_CNT(USBx,bEpNum,bDir,wCount) {     \
	USB_SET_EP_DBUF0_CNT((USBx),(bEpNum),(bDir),(wCount)); \
	USB_SET_EP_DBUF1_CNT((USBx),(bEpNum),(bDir),(wCount)); \
}

// Read buffer 0 RX/TX counter for double buffering
#define USB_GET_EP_DBUF0_CNT(USBx,bEpNum) (USB_GET_EP_TX_CNT((USBx),(bEpNum)))
// Read buffer 1 RX/TX counter for double buffering
#define USB_GET_EP_DBUF1_CNT(USBx,bEpNum) (USB_GET_EP_RX_CNT((USBx),(bEpNum)))

// Set EP_KIND bit in an endpoint register
#define USB_SET_EP_KIND(USBx,bEpNum) (USB_SET_ENDPOINT((USBx),(bEpNum), \
			(USB_EP_CTR_RX | USB_EP_CTR_TX | ((USB_GET_ENDPOINT((USBx),(bEpNum)) | USB_EP_KIND) & USB_EPREG_MASK))))
// Clear EP_KIND bit in an endpoint register
#define USB_CLEAR_EP_KIND(USBx,bEpNum) (USB_SET_ENDPOINT((USBx),(bEpNum), \
			(USB_EP_CTR_RX | USB_EP_CTR_TX | (USB_GET_ENDPOINT((USBx),(bEpNum)) & USB_EPKIND_MASK))))

// Set EP_KIND bit in an endpoint register for double buffer
#define USB_SET_EP_DBUF(USBx,bEpNum) USB_SET_EP_KIND((USBx),(bEpNum))
// Clear EP_KIND bit in an endpoint register for double buffer
#define USB_CLEAR_EP_DBUF(USBx,bEpNum) USB_CLEAR_EP_KIND((USBx),(bEpNum))

// Clear the CTR_RX bit in an endpoint register
#define USB_CLEAR_RX_EP_CTR(USBx,bEpNum) (USB_SET_ENDPOINT((USBx),(bEpNum),USB_GET_ENDPOINT((USBx),(bEpNum)) & 0x7FFF & USB_EPREG_MASK))
// Clear the CTR_TX bit in an endpoint register
#define USB_CLEAR_TX_EP_CTR(USBx,bEpNum) (USB_SET_ENDPOINT((USBx),(bEpNum),USB_GET_ENDPOINT((USBx),(bEpNum)) & 0xFF7F & USB_EPREG_MASK))

// Toggle DTOG_RX bit in an endpoint register
#define USB_RX_DTOG(USBx,bEpNum) (USB_SET_ENDPOINT((USBx),(bEpNum), \
			USB_EP_CTR_RX | USB_EP_CTR_TX | USB_EP_DTOG_RX | (USB_GET_ENDPOINT((USBx),(bEpNum)) & USB_EPREG_MASK)))
// Toggles DTOG_TX bit in an endpoint register
#define USB_TX_DTOG(USBx,bEpNum) (USB_SET_ENDPOINT((USBx),(bEpNum), \
			USB_EP_CTR_RX | USB_EP_CTR_TX | USB_EP_DTOG_TX | (USB_GET_ENDPOINT((USBx),(bEpNum)) & USB_EPREG_MASK)))

// Clear DTOG_RX bit in an endpoint register
#define USB_CLEAR_RX_DTOG(USBx,bEpNum) if (USB_GET_ENDPOINT((USBx),(bEpNum)) & USB_EP_DTOG_RX) USB_RX_DTOG((USBx),(bEpNum));
// Clear DTOG_TX bit in an endpoint register
#define USB_CLEAR_TX_DTOG(USBx,bEpNum) if (USB_GET_ENDPOINT((USBx),(bEpNum)) & USB_EP_DTOG_TX) USB_TX_DTOG((USBx),(bEpNum));

// Write an address to an endpoint register
#define USB_SET_EP_ADDRESS(USBx,bEpNum,bAddr) USB_SET_ENDPOINT((USBx),(bEpNum), \
			USB_EP_CTR_RX | USB_EP_CTR_TX | (USB_GET_ENDPOINT((USBx),(bEpNum)) & USB_EPREG_MASK) | (bAddr))
// Read an address from an endpoint register
#define USB_GET_EP_ADDRESS(USBx,bEpNum)((uint8_t)(USB_GET_ENDPOINT((USBx),(bEpNum)) & USB_EPADDR_FIELD))

#define USB_EP_TX_ADDRESS(USBx,bEpNum) ((uint32_t *)(( ((USBx)->BTABLE + (bEpNum) * 8) * 2)      + ((uint32_t)(USBx) + 0x400)))
#define USB_EP_RX_ADDRESS(USBx,bEpNum) ((uint32_t *)(((((USBx)->BTABLE + (bEpNum) * 8) + 4) * 2) + ((uint32_t)(USBx) + 0x400)))

// Write an address to an TX endpoint register
#define USB_SET_EP_TX_ADDRESS(USBx,bEpNum,wAddr) (*USB_EP_TX_ADDRESS((USBx),(bEpNum)) = (((wAddr) >> 1) << 1))
// Write an address to an RX endpoint register
#define USB_SET_EP_RX_ADDRESS(USBx,bEpNum,wAddr) (*USB_EP_RX_ADDRESS((USBx),(bEpNum)) = (((wAddr) >> 1) << 1))

// Set buffer 0 address in a double buffer endpoint
#define USB_SET_EP_DBUF0_ADDR(USBx,bEpNum,wBuf0Addr) { USB_SET_EP_TX_ADDRESS((USBx),(bEpNum),(wBuf0Addr)); }
// Set buffer 1 address in a double buffer endpoint
#define USB_SET_EP_DBUF1_ADDR(USBx,bEpNum,wBuf1Addr) { USB_SET_EP_RX_ADDRESS((USBx),(bEpNum),(wBuf1Addr)); }

// Set addresses in a double buffer endpoint
#define USB_SET_EP_DBUF_ADDR(USBx,bEpNum,wBuf0Addr,wBuf1Addr) { \
	USB_SET_EP_DBUF0_ADDR((USBx),(bEpNum),(wBuf0Addr));         \
    USB_SET_EP_DBUF1_ADDR((USBx),(bEpNum),(wBuf1Addr));         \
}

// Sets counter of the RX buffer with number of blocks
#define USB_CALC_BLK32(dwReg,wCount,wNBlocks) {                  \
    (wNBlocks) = (wCount) >> 5;                                  \
    if (!((wCount) & 0x1f)) (wNBlocks)--;                        \
    *pdwReg = (uint16_t)((uint16_t)((wNBlocks) << 10) | 0x8000); \
}

#define USB_CALC_BLK2(dwReg,wCount,wNBlocks) { \
    (wNBlocks) = (wCount) >> 1;                \
    if ((wCount) & 0x1) (wNBlocks)++;          \
	*pdwReg = (uint16_t)((wNBlocks) << 10);    \
}

#define USB_SET_EP_CNT_RX_REG(dwReg,wCount) {        \
	uint16_t wNBlocks;                               \
	if ((wCount) > 62) {                             \
		USB_CALC_BLK32((dwReg),(wCount),wNBlocks);   \
	} else {                                         \
		USB_CALC_BLK2( (dwReg),(wCount),wNBlocks);   \
	}                                                \
}

// Write the reception byte count for the specified endpoint
#define USB_SET_EP_RX_CNT(USBx,bEpNum,wCount) { uint32_t *pdwReg = USB_EP_RX_CNT((USBx),(bEpNum)); USB_SET_EP_CNT_RX_REG(pdwReg,(wCount)); }

// Set the status for TX transfer (bits STAT_TX[1:0]
#define USB_SET_EP_TX_STATUS(USBx,bEpNum,wState) {                                      \
	register uint16_t _wRegVal = USB_GET_ENDPOINT((USBx),(bEpNum)) & USB_EPTX_DTOGMASK; \
	if (USB_EPTX_DTOG1 & (wState)) _wRegVal ^= USB_EPTX_DTOG1;                          \
	if (USB_EPTX_DTOG2 & (wState)) _wRegVal ^= USB_EPTX_DTOG2;                          \
	USB_SET_ENDPOINT((USBx),(bEpNum),(_wRegVal | USB_EP_CTR_RX | USB_EP_CTR_TX));       \
}

// Set the status of RX transfer (bits STAT_TX[1:0])
#define USB_SET_EP_RX_STATUS(USBx,bEpNum,wState) {                                      \
	register uint16_t _wRegVal = USB_GET_ENDPOINT((USBx),(bEpNum)) & USB_EPRX_DTOGMASK; \
    if (USB_EPRX_DTOG1 & (wState)) _wRegVal ^= USB_EPRX_DTOG1;                          \
    if (USB_EPRX_DTOG2 & (wState)) _wRegVal ^= USB_EPRX_DTOG2;                          \
    USB_SET_ENDPOINT((USBx),(bEpNum),(_wRegVal | USB_EP_CTR_RX | USB_EP_CTR_TX));       \
}

// Set the status of both RX and TX transfers (bits STAT_TX[1:0] & STAT_RX[1:0])
#define USB_SET_EP_TXRX_STATUS(USBx,bEpNum,wStaterx,wStatetx) {                                           \
	register uint32_t _wRegVal = USB_GET_ENDPOINT((USBx),(bEpNum)) & (USB_EPRX_DTOGMASK | USB_EPTX_STAT); \
    if (USB_EPRX_DTOG1 & (wStaterx)) _wRegVal ^= USB_EPRX_DTOG1;                                          \
    if (USB_EPRX_DTOG2 & (wStaterx)) _wRegVal ^= USB_EPRX_DTOG2;                                          \
    if (USB_EPTX_DTOG1 & (wStatetx)) _wRegVal ^= USB_EPTX_DTOG1;                                          \
    if (USB_EPTX_DTOG2 & (wStatetx)) _wRegVal ^= USB_EPTX_DTOG2;                                          \
    USB_SET_ENDPOINT((USBx),(bEpNum),_wRegVal | USB_EP_CTR_RX | USB_EP_CTR_TX);                           \
}

// Free buffer used from the application realizing it to the line toggles bit SW_BUF in the double buffered endpoint register
#define USB_FreeUserBuffer(USBx,bEpNum,bDir) { \
	if ((bDir) == USB_EP_DBUF_OUT) {           \
    	USB_TX_DTOG((USBx),(bEpNum));          \
	} else if ((bDir) == USB_EP_DBUF_IN) {     \
		USB_RX_DTOG((USBx),(bEpNum));          \
	}                                          \
}

// Free buffer used from the application realizing it to the line toggles bit SW_BUF in the double buffered endpoint register
#define USB_FreeUserBuffer(USBx,bEpNum,bDir) { \
	if ((bDir) == USB_EP_DBUF_OUT) {           \
		USB_TX_DTOG((USBx),(bEpNum));          \
	} else if ((bDir) == USB_EP_DBUF_IN) {     \
		USB_RX_DTOG((USBx),(bEpNum));          \
	}                                          \
}


// Public variables
extern USB_HandleTypeDef husb; // USB device handle


// Function prototypes
void HAL_USB_DataOutStageCallback(USB_HandleTypeDef *husb, uint8_t epnum);
void HAL_USB_DataInStageCallback(USB_HandleTypeDef *husb, uint8_t epnum);
void HAL_USB_SetupStageCallback(USB_HandleTypeDef *husb);
void HAL_USB_SOFCallback(USB_HandleTypeDef *husb);
void HAL_USB_ResetCallback(USB_HandleTypeDef *husb);
void HAL_USB_SuspendCallback(USB_HandleTypeDef *husb);
void HAL_USB_ResumeCallback(USB_HandleTypeDef *husb);
void HAL_USB_ISOOUTIncompleteCallback(USB_HandleTypeDef *husb, uint8_t epnum);
void HAL_USB_ISOINIncompleteCallback(USB_HandleTypeDef *husb, uint8_t epnum);
void HAL_USB_ConnectCallback(USB_HandleTypeDef *husb);
void HAL_USB_DisconnectCallback(USB_HandleTypeDef *husb);

void HAL_USB_Init(USB_HandleTypeDef *husb);
void HAL_USB_DeInit(USB_HandleTypeDef *husb);
void HAL_USB_Stop(USB_HandleTypeDef *husb);
void HAL_USB_SetAddress(USB_HandleTypeDef *husb, uint8_t address);
void HAL_USB_EP_Receive(USB_HandleTypeDef *husb, uint8_t ep_addr, uint8_t *pBuf, uint32_t len);
void HAL_USB_EP_Transmit(USB_HandleTypeDef *husb, uint8_t ep_addr, uint8_t *pBuf, uint32_t len);
void HAL_USB_EP_SetStall(USB_HandleTypeDef *husb, uint8_t ep_addr);
void HAL_USB_EP_Open(USB_HandleTypeDef *husb, uint8_t ep_addr, uint16_t ep_mps, uint8_t ep_type);
void HAL_USB_EP_Close(USB_HandleTypeDef *husb, uint8_t ep_addr);
void HAL_USB_EP_ClrStall(USB_HandleTypeDef *husb, uint8_t ep_addr);
void HAL_USB_EP_Flush(USB_HandleTypeDef *husb, uint8_t ep_addr);
void HAL_USB_SetConnectionState(USB_HandleTypeDef *husb, uint8_t state);
void HAL_USB_PMAConfig(USB_HandleTypeDef *husb, uint16_t ep_addr, uint16_t ep_kind, uint32_t pmaadress);
uint16_t HAL_USB_EP_GetRxCount(USB_HandleTypeDef *husb, uint8_t ep_addr);

#endif // __USB_LL_H
