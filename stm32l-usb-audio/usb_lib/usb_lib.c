#include "usb_lib.h"


// Private variables
// cells saving status during interrupt servicing
__IO uint16_t SaveRState;
__IO uint16_t SaveTState;
DEVICE_INFO	Device_Info;  // Device information of current device


// Public variables
//  Points to the DEVICE_INFO structure of current device
//  The purpose of this register is to speed up the execution
DEVICE_INFO *pInformation;
/*  Points to the DEVICE_PROP structure of current device */
/*  The purpose of this register is to speed up the execution */
DEVICE_PROP*	pProperty;
void (*pEpInt_IN[ 7])(void);   // Handles IN  interrupts
void (*pEpInt_OUT[7])(void);   // Handles OUT interrupts
uint8_t	EPindex;               // The number of current endpoint, it will be used to specify an endpoint
//  Temporary save the state of Rx & Tx status.
//  Whenever the Rx or Tx state is changed, its value is saved
//  in this variable first and will be set to the EPRB or EPRA
//  at the end of interrupt process
uint16_t SaveState;
uint16_t wInterrupt_Mask; // Contains interrupt mask
USER_STANDARD_REQUESTS *pUser_Standard_Requests; //


// Functions

/*******************************************************************************
* Function Name  : USB_Init
* Description    : USB system initialization
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USB_Init(void) {
	pInformation = &Device_Info;
	pInformation->ControlState = 2;
	pProperty = &Device_Property;
	pUser_Standard_Requests = &User_Standard_Requests;
	// Initialize devices one by one
	pProperty->Init();
}

/*******************************************************************************
* Function Name  : CTR_LP.
* Description    : Low priority Endpoint Correct Transfer interrupt's service
*                  routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void CTR_LP(void) {
	__IO uint16_t wEPVal = 0;

	// stay in loop while pending interrupts
	while ((wIstr = *ISTR) & ISTR_CTR) {
		// extract highest priority endpoint number
		EPindex = (uint8_t)(wIstr & ISTR_EP_ID);
		if (EPindex == 0) {
			// Decode and service control endpoint interrupt
			// calling related service routine
			// (Setup0_Process, In0_Process, Out0_Process)

			// save RX & TX status and set both to NAK
			SaveRState  = _GetENDPOINT(ENDP0);
			SaveTState  =  SaveRState & EPTX_STAT;
			SaveRState &=  EPRX_STAT;
			SetEPRxTxStatus(ENDP0,EP_RX_NAK,EP_TX_NAK);

			// DIR bit = origin of the interrupt
			if ((wIstr & ISTR_DIR) == 0) {
				// DIR = 0      => IN  int
				// DIR = 0 implies that (EP_CTR_TX = 1) always
				ClearEP_CTR_TX(ENDP0);
				In0_Process();
				// before terminate set Tx & Rx status
				SetEPRxTxStatus(ENDP0,SaveRState,SaveTState);

				return;
			} else {
				// DIR = 1 & CTR_RX       => SETUP or OUT int
				// DIR = 1 & (CTR_TX | CTR_RX) => 2 int pending
				wEPVal = _GetENDPOINT(ENDP0);
				if ((wEPVal &EP_SETUP) != 0) {
					ClearEP_CTR_RX(ENDP0); // SETUP bit kept frozen while CTR_RX = 1
					Setup0_Process();
					// before terminate set Tx & Rx status
					SetEPRxTxStatus(ENDP0,SaveRState,SaveTState);

					return;
				} else if ((wEPVal & EP_CTR_RX) != 0) {
					ClearEP_CTR_RX(ENDP0);
					Out0_Process();
					// before terminate set Tx & Rx status
					SetEPRxTxStatus(ENDP0,SaveRState,SaveTState);

					return;
				}
			}
		} else {
			// Decode and service non control endpoints interrupt
			// process related endpoint register
			wEPVal = _GetENDPOINT(EPindex);
			if ((wEPVal & EP_CTR_RX) != 0) {
				// clear int flag
				ClearEP_CTR_RX(EPindex);
				// call OUT service function
				(*pEpInt_OUT[EPindex-1])();
			}
			if ((wEPVal & EP_CTR_TX) != 0) {
				// clear int flag
				ClearEP_CTR_TX(EPindex);
				// call IN service function
				(*pEpInt_IN[EPindex-1])();
			}
		}
	}
}

/*******************************************************************************
* Function Name  : CTR_HP.
* Description    : High Priority Endpoint Correct Transfer interrupt's service
*                  routine.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void CTR_HP(void) {
	uint32_t wEPVal = 0;

	while (((wIstr = *ISTR) & ISTR_CTR) != 0) {
		*ISTR = CLR_CTR; // clear CTR flag
		// extract highest priority endpoint number
		EPindex = (uint8_t)(wIstr & ISTR_EP_ID);
		// process related endpoint register
		wEPVal = _GetENDPOINT(EPindex);
		if ((wEPVal & EP_CTR_RX) != 0) {
			// clear int flag
			ClearEP_CTR_RX(EPindex);
			// call OUT service function
			(*pEpInt_OUT[EPindex-1])();
	    } else if ((wEPVal & EP_CTR_TX) != 0) {
	    	// clear int flag
	    	ClearEP_CTR_TX(EPindex);
	    	// call IN service function
	    	(*pEpInt_IN[EPindex-1])();
	    }
	}
}

/*******************************************************************************
* Function Name  : UserToPMABufferCopy
* Description    : Copy a buffer from user memory area to packet memory area (PMA)
* Input          : - pbUsrBuf: pointer to user memory area.
*                  - wPMABufAddr: address into PMA.
*                  - wNBytes: no. of bytes to be copied.
* Output         : None.
* Return         : None	.
*******************************************************************************/
void UserToPMABufferCopy(uint8_t *pbUsrBuf, uint16_t wPMABufAddr, uint16_t wNBytes) {
	uint32_t n = (wNBytes + 1) >> 1;   /* n = (wNBytes + 1) / 2 */
	uint32_t i,temp1,temp2;
	uint16_t *pdwVal;

	pdwVal = (uint16_t *)(wPMABufAddr * 2 + PMAAddr);
	for (i = n; i != 0; i--) {
		temp1 = (uint16_t)*pbUsrBuf;
		pbUsrBuf++;
		temp2 = temp1 | (uint16_t)*pbUsrBuf << 8;
		*pdwVal++ = temp2;
		pdwVal++;
		pbUsrBuf++;
	}
}

/*******************************************************************************
* Function Name  : PMAToUserBufferCopy
* Description    : Copy a buffer from user memory area to packet memory area (PMA)
* Input          : - pbUsrBuf    = pointer to user memory area.
*                  - wPMABufAddr = address into PMA.
*                  - wNBytes     = no. of bytes to be copied.
* Output         : None.
* Return         : None.
*******************************************************************************/
void PMAToUserBufferCopy(uint8_t *pbUsrBuf, uint16_t wPMABufAddr, uint16_t wNBytes) {
	uint32_t n = (wNBytes + 1) >> 1;
	uint32_t i;
	uint32_t *pdwVal;

	pdwVal = (uint32_t *)(wPMABufAddr * 2 + PMAAddr);
	for (i = n; i != 0; i--) {
		*(uint16_t*)pbUsrBuf++ = *pdwVal++;
		pbUsrBuf++;
	}
}

/*******************************************************************************
* Function Name  : USB_SIL_Init
* Description    : Initialize the USB Device IP and the Endpoint 0.
* Input          : None.
* Output         : None.
* Return         : Status.
*******************************************************************************/
uint32_t USB_SIL_Init(void) {
	// USB interrupts initialization
	// clear pending interrupts
	*ISTR = 0;
	wInterrupt_Mask = IMR_MSK;
	// set interrupts mask
	*CNTR = wInterrupt_Mask;

	return 0;
}

/*******************************************************************************
* Function Name  : USB_SIL_Write
* Description    : Write a buffer of data to a selected endpoint.
* Input          : - bEpAddr: The address of the non control endpoint.
*                  - pBufferPointer: The pointer to the buffer of data to be written
*                    to the endpoint.
*                  - wBufferSize: Number of data to be written (in bytes).
* Output         : None.
* Return         : Status.
*******************************************************************************/
uint32_t USB_SIL_Write(uint8_t bEpAddr, uint8_t* pBufferPointer, uint32_t wBufferSize) {
	// Use the memory interface function to write to the selected endpoint
	UserToPMABufferCopy(pBufferPointer,GetEPTxAddr(bEpAddr & 0x7F),wBufferSize);

	// Update the data length in the control register
	SetEPTxCount((bEpAddr & 0x7F),wBufferSize);

	return 0;
}

/*******************************************************************************
* Function Name  : USB_SIL_Read
* Description    : Write a buffer of data to a selected endpoint.
* Input          : - bEpAddr: The address of the non control endpoint.
*                  - pBufferPointer: The pointer to which will be saved the
*                     received data buffer.
* Output         : None.
* Return         : Number of received data (in Bytes).
*******************************************************************************/
uint32_t USB_SIL_Read(uint8_t bEpAddr, uint8_t* pBufferPointer) {
	uint32_t DataLength;

	// Get the number of received data on the selected Endpoint
	DataLength = GetEPRxCount(bEpAddr & 0x7F);

	// Use the memory interface function to write to the selected endpoint
	PMAToUserBufferCopy(pBufferPointer,GetEPRxAddr(bEpAddr & 0x7F),DataLength);

	// Return the number of received data
	return DataLength;
}
