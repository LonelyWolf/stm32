#include "usb_lib.h"


/*******************************************************************************
* Function Name  : _SetENDPOINT
* Description    : Set the Endpoint register value.
* Input          : bEpNum: Endpoint Number. 
*                  wRegValue.
* Output         : None.
* Return         : None.
*******************************************************************************/
void _SetENDPOINT(uint8_t bEpNum, uint16_t wRegValue) {
	*(&USB->EP0R + (bEpNum << 1)) = (uint16_t)(wRegValue);
}

/*******************************************************************************
* Function Name  : _GetENDPOINT
* Description    : Return the Endpoint register value.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Endpoint register value.
*******************************************************************************/
uint16_t _GetENDPOINT(uint8_t bEpNum) {
	return (uint16_t)(*(&USB->EP0R + (bEpNum << 1)));
}

/*******************************************************************************
* Function Name  : SetEPType
* Description    : sets the type in the endpoint register.
* Input          : bEpNum: Endpoint Number. 
*                  wType: type definition.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SetEPType(uint8_t bEpNum, uint16_t wType) {
	_SetENDPOINT(bEpNum,(_GetENDPOINT(bEpNum) & EP_T_MASK) | wType);
}

/*******************************************************************************
* Function Name  : GetEPType
* Description    : Returns the endpoint type.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Endpoint Type
*******************************************************************************/
uint16_t GetEPType(uint8_t bEpNum) {
	return ((uint16_t)(*(&USB->EP0R + (bEpNum << 1))) & EP_T_FIELD);
}

/*******************************************************************************
* Function Name  : SetEPTxStatus
* Description    : Set the status of Tx endpoint.
* Input          : bEpNum: Endpoint Number. 
*                  wState: new state.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SetEPTxStatus(uint8_t bEpNum, uint16_t wState) {
    register uint16_t _wRegVal;

    _wRegVal = _GetENDPOINT(bEpNum) & EPTX_DTOGMASK;
    // toggle first bit ?
    if ((EPTX_DTOG1 & wState) != 0) _wRegVal ^= EPTX_DTOG1;
    // toggle second bit ?
    if ((EPTX_DTOG2 & wState) != 0) _wRegVal ^= EPTX_DTOG2;
    _SetENDPOINT(bEpNum,_wRegVal | EP_CTR_RX | EP_CTR_TX);
}

/*******************************************************************************
* Function Name  : SetEPRxStatus
* Description    : Set the status of Rx endpoint.
* Input          : bEpNum: Endpoint Number. 
*                  wState: new state.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SetEPRxStatus(uint8_t bEpNum, uint16_t wState) {
    register uint16_t _wRegVal;

    _wRegVal = _GetENDPOINT(bEpNum) & EPRX_DTOGMASK;
    // toggle first bit ?
	if ((EPRX_DTOG1 & wState) != 0) _wRegVal ^= EPRX_DTOG1;
    // toggle second bit ?
    if ((EPRX_DTOG2 & wState) != 0) _wRegVal ^= EPRX_DTOG2;
    _SetENDPOINT(bEpNum,_wRegVal | EP_CTR_RX | EP_CTR_TX);
}

/*******************************************************************************
* Function Name  : SetEPRxTxStatus
* Description    : sets the status for RX & TX (bits STAT_TX[1:0] & STAT_RX[1:0])
* Input          : bEpNum: Endpoint Number.
*                  wStateRX: new state of RX.
*                  wStateTX: new state of TX.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SetEPRxTxStatus(uint8_t bEpNum, uint16_t wStateRX, uint16_t wStateTX) {
	register uint32_t _wRegVal;

	_wRegVal = _GetENDPOINT(bEpNum) & (EPRX_DTOGMASK | EPTX_STAT);
    // toggle first bit ?
    if ((EPRX_DTOG1 & wStateRX) != 0) _wRegVal ^= EPRX_DTOG1;
    // toggle second bit ?
    if ((EPRX_DTOG2 & wStateRX) != 0) _wRegVal ^= EPRX_DTOG2;
    // toggle first bit ?
    if ((EPTX_DTOG1 & wStateTX) != 0) _wRegVal ^= EPTX_DTOG1;
    // toggle second bit ?
    if ((EPTX_DTOG2 & wStateTX) != 0) _wRegVal ^= EPTX_DTOG2;
    _SetENDPOINT(bEpNum, _wRegVal | EP_CTR_RX | EP_CTR_TX);
}

/*******************************************************************************
* Function Name  : SetDouBleBuffEPStall
* Description    : sets the status for Double Buffer Endpoint to STALL
* Input          : bEpNum: Endpoint Number. 
*                  bDir: Endpoint direction.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SetDouBleBuffEPStall(uint8_t bEpNum, uint8_t bDir) {
	uint16_t Endpoint_DTOG_Status;

	Endpoint_DTOG_Status = _GetENDPOINT(bEpNum);
	if (bDir == EP_DBUF_OUT) {
		// OUT double buffered endpoint
		_SetENDPOINT(bEpNum,Endpoint_DTOG_Status & ~EPRX_DTOG1);
	} else if (bDir == EP_DBUF_IN) {
		// IN double buffered endpoint
		_SetENDPOINT(bEpNum,Endpoint_DTOG_Status & ~EPTX_DTOG1);
	}
}

/*******************************************************************************
* Function Name  : GetEPTxStatus
* Description    : Returns the endpoint Tx status.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Endpoint TX Status
*******************************************************************************/
uint16_t GetEPTxStatus(uint8_t bEpNum) {
	return (uint16_t)_GetENDPOINT(bEpNum) & EPTX_STAT;
}

/*******************************************************************************
* Function Name  : GetEPRxStatus
* Description    : Returns the endpoint Rx status.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Endpoint RX Status
*******************************************************************************/
uint16_t GetEPRxStatus(uint8_t bEpNum) {
	return (uint16_t)_GetENDPOINT(bEpNum) & EPRX_STAT;
}

/*******************************************************************************
* Function Name  : SetEPTxValid
* Description    : Valid the endpoint Tx Status.
* Input          : bEpNum: Endpoint Number.  
* Output         : None.
* Return         : None.
*******************************************************************************/
void SetEPTxValid(uint8_t bEpNum) {
	SetEPTxStatus(bEpNum,EP_TX_VALID);
}

/*******************************************************************************
* Function Name  : SetEPRxValid
* Description    : Valid the endpoint Rx Status.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
void SetEPRxValid(uint8_t bEpNum) {
	SetEPRxStatus(bEpNum,EP_RX_VALID);
}

/*******************************************************************************
* Function Name  : SetEP_KIND
* Description    : Clear the EP_KIND bit.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
void SetEP_KIND(uint8_t bEpNum) {
	_SetENDPOINT(bEpNum,EP_CTR_RX | EP_CTR_TX | ((_GetENDPOINT(bEpNum) | EP_KIND) & EPREG_MASK));
}

/*******************************************************************************
* Function Name  : ClearEP_KIND
* Description    : set the  EP_KIND bit.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
void ClearEP_KIND(uint8_t bEpNum) {
	_SetENDPOINT(bEpNum,EP_CTR_RX | EP_CTR_TX | (_GetENDPOINT(bEpNum) & EPKIND_MASK));
}

/*******************************************************************************
* Function Name  : Clear_Status_Out
* Description    : Clear the Status Out of the related Endpoint
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
void Clear_Status_Out(uint8_t bEpNum) {
//	ClearEP_KIND(bEpNum);
	_SetENDPOINT(bEpNum,EP_CTR_RX | EP_CTR_TX | (_GetENDPOINT(bEpNum) & EPKIND_MASK));
}

/*******************************************************************************
* Function Name  : Set_Status_Out
* Description    : Set the Status Out of the related Endpoint
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
void Set_Status_Out(uint8_t bEpNum) {
//	SetEP_KIND(bEpNum);
	_SetENDPOINT(bEpNum,EP_CTR_RX | EP_CTR_TX | ((_GetENDPOINT(bEpNum) | EP_KIND) & EPREG_MASK));
}

/*******************************************************************************
* Function Name  : SetEPDoubleBuff
* Description    : Enable the double buffer feature for the endpoint. 
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
void SetEPDoubleBuff(uint8_t bEpNum) {
//	SetEP_KIND(bEpNum);
	_SetENDPOINT(bEpNum,EP_CTR_RX | EP_CTR_TX | ((_GetENDPOINT(bEpNum) | EP_KIND) & EPREG_MASK));
}

/*******************************************************************************
* Function Name  : ClearEPDoubleBuff
* Description    : Disable the double buffer feature for the endpoint. 
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
void ClearEPDoubleBuff(uint8_t bEpNum) {
//	ClearEP_KIND(bEpNum);
	_SetENDPOINT(bEpNum,EP_CTR_RX | EP_CTR_TX | (_GetENDPOINT(bEpNum) & EPKIND_MASK));
}

/*******************************************************************************
* Function Name  : GetTxStallStatus
* Description    : Returns the Stall status of the Tx endpoint.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Tx Stall status.
*******************************************************************************/
uint16_t GetTxStallStatus(uint8_t bEpNum) {
//	return GetEPTxStatus(bEpNum) == EP_TX_STALL;
	return ((_GetENDPOINT(bEpNum) & EPTX_STAT) == EP_TX_STALL);
}

/*******************************************************************************
* Function Name  : GetRxStallStatus
* Description    : Returns the Stall status of the Rx endpoint. 
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Rx Stall status.
*******************************************************************************/
uint16_t GetRxStallStatus(uint8_t bEpNum) {
//	return GetEPRxStatus(bEpNum) == EP_RX_STALL;
	return ((_GetENDPOINT(bEpNum) & EPRX_STAT) == EP_RX_STALL);
}

/*******************************************************************************
* Function Name  : ClearEP_CTR_RX
* Description    : Clear the CTR_RX bit.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
void ClearEP_CTR_RX(uint8_t bEpNum) {
	_SetENDPOINT(bEpNum,_GetENDPOINT(bEpNum) & 0x7FFF & EPREG_MASK);
}

/*******************************************************************************
* Function Name  : ClearEP_CTR_TX
* Description    : Clear the CTR_TX bit.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
void ClearEP_CTR_TX(uint8_t bEpNum) {
	_SetENDPOINT(bEpNum,_GetENDPOINT(bEpNum) & 0xFF7F & EPREG_MASK);
}

/*******************************************************************************
* Function Name  : ToggleDTOG_RX
* Description    : Toggle the DTOG_RX bit.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
void ToggleDTOG_RX(uint8_t bEpNum) {
	_SetENDPOINT(bEpNum,EP_CTR_RX | EP_CTR_TX | EP_DTOG_RX | (_GetENDPOINT(bEpNum) & EPREG_MASK));
}

/*******************************************************************************
* Function Name  : ToggleDTOG_TX
* Description    : Toggle the DTOG_TX bit.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
void ToggleDTOG_TX(uint8_t bEpNum) {
	_SetENDPOINT(bEpNum,EP_CTR_RX | EP_CTR_TX | EP_DTOG_TX | (_GetENDPOINT(bEpNum) & EPREG_MASK));
}

/*******************************************************************************
* Function Name  : ClearDTOG_RX.
* Description    : Clear the DTOG_RX bit.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
void ClearDTOG_RX(uint8_t bEpNum) {
	uint16_t EP = (uint16_t)(*(&USB->EP0R + (bEpNum << 1)));

	if (EP & EP_DTOG_RX) *(&USB->EP0R + (bEpNum << 1)) = (uint16_t)(EP_CTR_RX | EP_CTR_TX | EP_DTOG_RX | (EP & EPREG_MASK));
}

/*******************************************************************************
* Function Name  : ClearDTOG_TX.
* Description    : Clear the DTOG_TX bit.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
void ClearDTOG_TX(uint8_t bEpNum) {
	uint16_t EP = (uint16_t)(*(&USB->EP0R + (bEpNum << 1)));

	if (EP & EP_DTOG_TX) *(&USB->EP0R + (bEpNum << 1)) = (uint16_t)(EP_CTR_RX | EP_CTR_TX | EP_DTOG_TX | (EP & EPREG_MASK));
}

/*******************************************************************************
* Function Name  : SetEPAddress
* Description    : Set the endpoint address.
* Input          : bEpNum: Endpoint Number.
*                  bAddr: New endpoint address.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SetEPAddress(uint8_t bEpNum, uint8_t bAddr) {
//	_SetENDPOINT(bEpNum,EP_CTR_RX | EP_CTR_TX | ((uint16_t)(*(&USB->EP0R + (bEpNum << 1))) & EPREG_MASK) | bAddr);
	*(&USB->EP0R + (bEpNum << 1)) = (uint16_t)(EP_CTR_RX | EP_CTR_TX | ((uint16_t)(*(&USB->EP0R + (bEpNum << 1))) & EPREG_MASK) | bAddr);
}

/*******************************************************************************
* Function Name  : GetEPAddress
* Description    : Get the endpoint address.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Endpoint address.
*******************************************************************************/
uint8_t GetEPAddress(uint8_t bEpNum) {
	return (uint8_t)((uint16_t)(*(&USB->EP0R + (bEpNum << 1))) & EPADDR_FIELD);
}

/*******************************************************************************
* Function Name  : SetEPTxAddr
* Description    : Set the endpoint Tx buffer address.
* Input          : bEpNum: Endpoint Number.
*                  wAddr: new address. 
* Output         : None.
* Return         : None.
*******************************************************************************/
void SetEPTxAddr(uint8_t bEpNum, uint16_t wAddr) {
	*_pEPTxAddr(bEpNum) = wAddr & 0xFFFE;
}

/*******************************************************************************
* Function Name  : SetEPRxAddr
* Description    : Set the endpoint Rx buffer address.
* Input          : bEpNum: Endpoint Number.
*                  wAddr: new address.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SetEPRxAddr(uint8_t bEpNum, uint16_t wAddr) {
	*_pEPRxAddr(bEpNum) = wAddr & 0xFFFE;
}

/*******************************************************************************
* Function Name  : GetEPTxAddr
* Description    : Returns the endpoint Tx buffer address.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Rx buffer address. 
*******************************************************************************/
uint16_t GetEPTxAddr(uint8_t bEpNum) {
	return (uint16_t)*_pEPTxAddr(bEpNum);
}

/*******************************************************************************
* Function Name  : GetEPRxAddr.
* Description    : Returns the endpoint Rx buffer address.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Rx buffer address.
*******************************************************************************/
uint16_t GetEPRxAddr(uint8_t bEpNum) {
	return (uint16_t)*_pEPRxAddr(bEpNum);
}

/*******************************************************************************
* Function Name  : SetEPTxCount.
* Description    : Set the Tx count.
* Input          : bEpNum: Endpoint Number.
*                  wCount: new count value.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SetEPTxCount(uint8_t bEpNum, uint16_t wCount) {
	*_pEPTxCount(bEpNum) = wCount;
}

/*******************************************************************************
* Function Name  : SetEPRxCount
* Description    : Set the Rx count.
* Input          : bEpNum: Endpoint Number.
*                  wCount: the new count value.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SetEPRxCount(uint8_t bEpNum, uint16_t wCount) {
	uint32_t *pdwReg = _pEPRxCount(bEpNum);

	SetEPCountRxReg(pdwReg,wCount);
}

/*******************************************************************************
* Function Name  : SetEPCountRxReg.
* Description    : Set the Count Rx Register value.
* Input          : *pdwReg: point to the register.
*                  wCount: the new register value.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SetEPCountRxReg(uint32_t *pdwReg, uint16_t wCount) {
    uint16_t wNBlocks;

    if (wCount > 62) {
    	wNBlocks = wCount >> 5;
        if ((wCount & 0x1f) == 0) wNBlocks--;
        *pdwReg = (uint32_t)((wNBlocks << 10) | 0x8000);
    } else {
        wNBlocks = wCount >> 1;
        if ((wCount & 0x01) != 0) wNBlocks++;
        *pdwReg = (uint32_t)(wNBlocks << 10);
    }
}

/*******************************************************************************
* Function Name  : GetEPTxCount
* Description    : Get the Tx count.
* Input          : bEpNum: Endpoint Number. 
* Output         : None
* Return         : Tx count value.
*******************************************************************************/
uint16_t GetEPTxCount(uint8_t bEpNum) {
	return (uint16_t)(*_pEPTxCount(bEpNum)) & 0x3ff;
}

/*******************************************************************************
* Function Name  : GetEPRxCount
* Description    : Get the Rx count.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Rx count value.
*******************************************************************************/
uint16_t GetEPRxCount(uint8_t bEpNum) {
	return (uint16_t)(*_pEPRxCount(bEpNum)) & 0x3ff;
}

/*******************************************************************************
* Function Name  : SetEPDblBuf0Addr
* Description    : Set the Buffer 1 address.
* Input          : bEpNum: Endpoint Number
*                  wBuf0Addr: new address.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SetEPDblBuf0Addr(uint8_t bEpNum, uint16_t wBuf0Addr) {
//	SetEPTxAddr(bEpNum,wBuf0Addr);
	*_pEPTxAddr(bEpNum) = wBuf0Addr & 0xFFFE;
}

/*******************************************************************************
* Function Name  : SetEPDblBuf1Addr
* Description    : Set the Buffer 1 address.
* Input          : bEpNum: Endpoint Number
*                  wBuf1Addr: new address.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SetEPDblBuf1Addr(uint8_t bEpNum, uint16_t wBuf1Addr) {
//	SetEPRxAddr(bEpNum,wBuf1Addr);
	*_pEPRxAddr(bEpNum) = wBuf1Addr & 0xFFFE;
}

/*******************************************************************************
* Function Name  : SetEPDblBuffAddr
* Description    : Set the addresses of the buffer 0 and 1.
* Input          : bEpNum: Endpoint Number.
*                  wBuf0Addr: new address of buffer 0.
*                  wBuf1Addr: new address of buffer 1.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SetEPDblBuffAddr(uint8_t bEpNum, uint16_t wBuf0Addr, uint16_t wBuf1Addr) {
//	SetEPDblBuf0Addr(bEpNum,wBuf0Addr);
//	SetEPDblBuf1Addr(bEpNum,wBuf1Addr);
	*_pEPTxAddr(bEpNum) = wBuf0Addr & 0xFFFE;
	*_pEPRxAddr(bEpNum) = wBuf1Addr & 0xFFFE;
}

/*******************************************************************************
* Function Name  : GetEPDblBuf0Addr
* Description    : Returns the address of the Buffer 0.
* Input          : bEpNum: Endpoint Number.
* Output         : None.
* Return         : None.
*******************************************************************************/
uint16_t GetEPDblBuf0Addr(uint8_t bEpNum) {
//	return GetEPTxAddr(bEpNum);
	return (uint16_t)*_pEPTxAddr(bEpNum);
}

/*******************************************************************************
* Function Name  : GetEPDblBuf1Addr
* Description    : Returns the address of the Buffer 1.
* Input          : bEpNum: Endpoint Number.
* Output         : None.
* Return         : Address of the Buffer 1.
*******************************************************************************/
uint16_t GetEPDblBuf1Addr(uint8_t bEpNum) {
//	return GetEPRxAddr(bEpNum);
	return (uint16_t)*_pEPRxAddr(bEpNum);
}

/*******************************************************************************
* Function Name  : SetEPDblBuf0Count
* Description    : Set the number of bytes in the buffer 0 of a double Buffer 
*                  endpoint.
* Input          : bEpNum, bDir,  wCount
* Output         : None.
* Return         : None.
*******************************************************************************/
void SetEPDblBuf0Count(uint8_t bEpNum, uint8_t bDir, uint16_t wCount) {
    uint32_t *pdwReg = _pEPTxCount(bEpNum);

    if (bDir == EP_DBUF_OUT) {
    	// OUT endpoint
        SetEPCountRxReg(pdwReg,wCount);
    } else if (bDir == EP_DBUF_IN) {
    	// IN endpoint
    	*_pEPTxCount(bEpNum) = (uint32_t)wCount;
    }
}

/*******************************************************************************
* Function Name  : SetEPDblBuf1Count
* Description    : Set the number of bytes in the buffer 0 of a double Buffer 
*                  endpoint.
* Input          : bEpNum,  bDir,  wCount
* Output         : None.
* Return         : None.
*******************************************************************************/
void SetEPDblBuf1Count(uint8_t bEpNum, uint8_t bDir, uint16_t wCount) {
    if (bDir == EP_DBUF_OUT) {
    	// OUT endpoint
    	SetEPRxCount(bEpNum,wCount);
    } else if (bDir == EP_DBUF_IN) {
    	// IN endpoint
    	*_pEPRxCount(bEpNum) = (uint32_t)wCount;
    }
}

/*******************************************************************************
* Function Name  : SetEPDblBuffCount
* Description    : Set the number of bytes for a double Buffer endpoint.
* Input          : bEpNum,bDir, wCount
* Output         : None.
* Return         : None.
*******************************************************************************/
void SetEPDblBuffCount(uint8_t bEpNum, uint8_t bDir, uint16_t wCount) {
	SetEPDblBuf0Count(bEpNum,bDir,wCount);
	SetEPDblBuf1Count(bEpNum,bDir,wCount);
}

/*******************************************************************************
* Function Name  : GetEPDblBuf0Count
* Description    : Returns the number of byte received in the buffer 0 of a double
*                  Buffer endpoint.
* Input          : bEpNum: Endpoint Number.
* Output         : None.
* Return         : Endpoint Buffer 0 count
*******************************************************************************/
uint16_t GetEPDblBuf0Count(uint8_t bEpNum) {
//	return GetEPTxCount(bEpNum);
	return (uint16_t)(*_pEPTxCount(bEpNum)) & 0x3ff;
}

/*******************************************************************************
* Function Name  : GetEPDblBuf1Count
* Description    : Returns the number of data received in the buffer 1 of a double
*                  Buffer endpoint.
* Input          : bEpNum: Endpoint Number.
* Output         : None.
* Return         : Endpoint Buffer 1 count.
*******************************************************************************/
uint16_t GetEPDblBuf1Count(uint8_t bEpNum) {
//	return GetEPRxCount(bEpNum);
	return (uint16_t)(*_pEPRxCount(bEpNum)) & 0x3ff;
}

/*******************************************************************************
* Function Name  : GetEPDblBufDir
* Description    : gets direction of the double buffered endpoint
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : EP_DBUF_OUT, EP_DBUF_IN,
*                  EP_DBUF_ERR if the endpoint counter not yet programmed.
*******************************************************************************/
EP_DBUF_DIR GetEPDblBufDir(uint8_t bEpNum) {
	if ((uint16_t)(*_pEPRxCount(bEpNum) & 0xFC00) != 0) {
		return(EP_DBUF_OUT);
	} else if (((uint16_t)(*_pEPTxCount(bEpNum)) & 0x03FF) != 0) {
		return(EP_DBUF_IN);
	} else {
		return(EP_DBUF_ERR);
	}
}

/*******************************************************************************
* Function Name  : FreeUserBuffer
* Description    : free buffer used from the application realizing it to the line
                   toggles bit SW_BUF in the double buffered endpoint register
* Input          : bEpNum, bDir
* Output         : None.
* Return         : None.
*******************************************************************************/
void FreeUserBuffer(uint8_t bEpNum, uint8_t bDir) {
	if (bDir == EP_DBUF_OUT) {
		// OUT double buffered endpoint
//		ToggleDTOG_TX(bEpNum);
//		_SetENDPOINT(bEpNum,EP_CTR_RX | EP_CTR_TX | EP_DTOG_TX | ((uint16_t)(*(&USB->EP0R + (bEpNum << 1))) & EPREG_MASK));
		*(&USB->EP0R + (bEpNum << 1)) = (uint16_t)(EP_CTR_RX | EP_CTR_TX | EP_DTOG_TX | ((uint16_t)(*(&USB->EP0R + (bEpNum << 1))) & EPREG_MASK));
	} else if (bDir == EP_DBUF_IN) {
		// IN double buffered endpoint
//		ToggleDTOG_RX(bEpNum);
//		_SetENDPOINT(bEpNum,EP_CTR_RX | EP_CTR_TX | EP_DTOG_RX | ((uint16_t)(*(&USB->EP0R + (bEpNum << 1))) & EPREG_MASK));
		*(&USB->EP0R + (bEpNum << 1)) = (uint16_t)(EP_CTR_RX | EP_CTR_TX | EP_DTOG_RX | ((uint16_t)(*(&USB->EP0R + (bEpNum << 1))) & EPREG_MASK));
	}
}

/*******************************************************************************
* Function Name  : ByteSwap
* Description    : Swap two byte in a word.
* Input          : wSwW: word to Swap.
* Output         : None.
* Return         : resulted word.
*******************************************************************************/
uint16_t ByteSwap(uint16_t wSwW) {
	return (wSwW >> 8) | ((uint16_t)wSwW << 8);
}
