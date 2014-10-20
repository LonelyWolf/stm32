/**
  ******************************************************************************
  * @file    usb_regs.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    28-August-2012
  * @brief   Interface functions to USB cell registers
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */


#include "usb_lib.h"


// Private functions

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
	return _GetENDPOINT(bEpNum) & EP_T_FIELD;
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
    if (EPTX_DTOG1 & wState) _wRegVal ^= EPTX_DTOG1;
    // toggle second bit ?
    if (EPTX_DTOG2 & wState) _wRegVal ^= EPTX_DTOG2;

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
    if (EPRX_DTOG1 & wState) _wRegVal ^= EPRX_DTOG1;
    // toggle second bit ?
    if (EPRX_DTOG2 & wState) _wRegVal ^= EPRX_DTOG2;

    _SetENDPOINT(bEpNum,_wRegVal | EP_CTR_RX | EP_CTR_TX);
}

/*******************************************************************************
* Function Name  : SetEPRxStatus
* Description    : Set the status of Rx and Tx endpoints.
* Input          : bEpNum: Endpoint Number.
*                  wStateRx: new state of RX.
*                  wStateTx: new state of TX.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SetEPRxTxStatus(uint8_t bEpNum, uint16_t wStateRx, uint16_t wStateTx) {
    register uint32_t _wRegVal;

    _wRegVal = _GetENDPOINT(bEpNum) & (EPRX_DTOGMASK | EPTX_STAT);
    // toggle first bit ?
    if((EPRX_DTOG1 & wStateRx)!= 0) _wRegVal ^= EPRX_DTOG1;
    // toggle second bit ?
    if((EPRX_DTOG2 & wStateRx)!= 0) _wRegVal ^= EPRX_DTOG2;
    // toggle first bit ?
    if((EPTX_DTOG1 & wStateTx)!= 0) _wRegVal ^= EPTX_DTOG1;
    // toggle second bit ?
    if((EPTX_DTOG2 & wStateTx)!= 0) _wRegVal ^= EPTX_DTOG2;

    _SetENDPOINT(bEpNum, _wRegVal | EP_CTR_RX|EP_CTR_TX);
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
	if (bDir == EP_DBUF_OUT) { /* OUT double buffered endpoint */
		_SetENDPOINT(bEpNum, Endpoint_DTOG_Status & ~EPRX_DTOG1);
	} else if (bDir == EP_DBUF_IN) { /* IN double buffered endpoint */
		_SetENDPOINT(bEpNum, Endpoint_DTOG_Status & ~EPTX_DTOG1);
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
	return _GetENDPOINT(bEpNum) & EPTX_STAT;
}

/*******************************************************************************
* Function Name  : GetEPRxStatus
* Description    : Returns the endpoint Rx status.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Endpoint RX Status
*******************************************************************************/
uint16_t GetEPRxStatus(uint8_t bEpNum) {
	return _GetENDPOINT(bEpNum) & EPRX_STAT;
}

/*******************************************************************************
* Function Name  : SetEPTxValid
* Description    : Valid the endpoint Tx Status.
* Input          : bEpNum: Endpoint Number.  
* Output         : None.
* Return         : None.
*******************************************************************************/
void SetEPTxValid(uint8_t bEpNum) {
	SetEPTxStatus(bEpNum, EP_TX_VALID);
}

/*******************************************************************************
* Function Name  : SetEPRxValid
* Description    : Valid the endpoint Rx Status.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
void SetEPRxValid(uint8_t bEpNum) {
	SetEPRxStatus(bEpNum, EP_RX_VALID);
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
	ClearEP_KIND(bEpNum);
}

/*******************************************************************************
* Function Name  : Set_Status_Out
* Description    : Set the Status Out of the related Endpoint
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
void Set_Status_Out(uint8_t bEpNum) {
	SetEP_KIND(bEpNum);
}

/*******************************************************************************
* Function Name  : SetEPDoubleBuff
* Description    : Enable the double buffer feature for the endpoint. 
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
void SetEPDoubleBuff(uint8_t bEpNum) {
	SetEP_KIND(bEpNum);
}

/*******************************************************************************
* Function Name  : ClearEPDoubleBuff
* Description    : Disable the double buffer feature for the endpoint. 
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
void ClearEPDoubleBuff(uint8_t bEpNum) {
	ClearEP_KIND(bEpNum);
}

/*******************************************************************************
* Function Name  : GetTxStallStatus
* Description    : Returns the Stall status of the Tx endpoint.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Tx Stall status.
*******************************************************************************/
uint16_t GetTxStallStatus(uint8_t bEpNum) {
	return (GetEPTxStatus(bEpNum) == EP_TX_STALL);
}

/*******************************************************************************
* Function Name  : GetRxStallStatus
* Description    : Returns the Stall status of the Rx endpoint. 
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Rx Stall status.
*******************************************************************************/
uint16_t GetRxStallStatus(uint8_t bEpNum) {
	return (GetEPRxStatus(bEpNum) == EP_RX_STALL);
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
	if (_GetENDPOINT(bEpNum) & EP_DTOG_RX) ToggleDTOG_RX(bEpNum);
}

/*******************************************************************************
* Function Name  : ClearDTOG_TX.
* Description    : Clear the DTOG_TX bit.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : None.
*******************************************************************************/
void ClearDTOG_TX(uint8_t bEpNum) {
	if (_GetENDPOINT(bEpNum) & EP_DTOG_TX) ToggleDTOG_TX(bEpNum);
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
	_SetENDPOINT(bEpNum,EP_CTR_RX | EP_CTR_TX | (_GetENDPOINT(bEpNum) & EPREG_MASK) | bAddr);
}

/*******************************************************************************
* Function Name  : GetEPAddress
* Description    : Get the endpoint address.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Endpoint address.
*******************************************************************************/
uint8_t GetEPAddress(uint8_t bEpNum) {
	return _GetENDPOINT(bEpNum) & EPADDR_FIELD;
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
	*_pEPTxAddr(bEpNum) = ((wAddr >> 1) << 1);
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
	*_pEPRxAddr(bEpNum) = ((wAddr >> 1) << 1);
}

/*******************************************************************************
* Function Name  : GetEPTxAddr
* Description    : Returns the endpoint Tx buffer address.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Rx buffer address. 
*******************************************************************************/
uint16_t GetEPTxAddr(uint8_t bEpNum) {
	return *_pEPTxAddr(bEpNum);
}

/*******************************************************************************
* Function Name  : GetEPRxAddr.
* Description    : Returns the endpoint Rx buffer address.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Rx buffer address.
*******************************************************************************/
uint16_t GetEPRxAddr(uint8_t bEpNum) {
	return *_pEPRxAddr(bEpNum);
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
	uint16_t wNBlocks;
	uint32_t *pdwReg = _pEPRxCount(bEpNum);

	if (wCount > 62) {
	    wNBlocks = wCount >> 5;
	    if (!(wCount & 0x1f)) wNBlocks--;
	    *pdwReg = (uint32_t)((wNBlocks << 10) | 0x8000);
	} else {
	    wNBlocks = wCount >> 1;
	    if (wCount & 0x1) wNBlocks++;
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
	return (*_pEPTxCount(bEpNum)) & 0x3ff;
}

/*******************************************************************************
* Function Name  : GetEPRxCount
* Description    : Get the Rx count.
* Input          : bEpNum: Endpoint Number. 
* Output         : None.
* Return         : Rx count value.
*******************************************************************************/
uint16_t GetEPRxCount(uint8_t bEpNum) {
	return (*_pEPRxCount(bEpNum)) & 0x3ff;
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
	*_pEPRxAddr(bEpNum) = ((wBuf0Addr >> 1) << 1);
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
	*_pEPRxAddr(bEpNum) = ((wBuf1Addr >> 1) << 1);
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
	SetEPDblBuf0Addr(bEpNum,wBuf0Addr);
	SetEPDblBuf1Addr(bEpNum,wBuf1Addr);
}

/*******************************************************************************
* Function Name  : GetEPDblBuf0Addr
* Description    : Returns the address of the Buffer 0.
* Input          : bEpNum: Endpoint Number.
* Output         : None.
* Return         : None.
*******************************************************************************/
uint16_t GetEPDblBuf0Addr(uint8_t bEpNum) {
	return *_pEPTxAddr(bEpNum);
}

/*******************************************************************************
* Function Name  : GetEPDblBuf1Addr
* Description    : Returns the address of the Buffer 1.
* Input          : bEpNum: Endpoint Number.
* Output         : None.
* Return         : Address of the Buffer 1.
*******************************************************************************/
uint16_t GetEPDblBuf1Addr(uint8_t bEpNum) {
	return *_pEPRxAddr(bEpNum);
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
    uint16_t wNBlocks;


    if (bDir == EP_DBUF_OUT) {
    	// OUT endpoint
    	if (wCount > 62) {
    	    wNBlocks = wCount >> 5;
    	    if (!(wCount & 0x1f)) wNBlocks--;
    	    *pdwReg = (uint32_t)((wNBlocks << 10) | 0x8000);
    	} else {
    	    wNBlocks = wCount >> 1;
    	    if (wCount & 0x1) wNBlocks++;
    	    *pdwReg = (uint32_t)(wNBlocks << 10);
    	}
    } else if (bDir == EP_DBUF_IN) *_pEPTxCount(bEpNum) = (uint32_t)wCount; // IN endpoint
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
		SetEPRxCount(bEpNum,wCount); // OUT endpoint
	} else if (bDir == EP_DBUF_IN) *_pEPRxCount(bEpNum) = (uint32_t)wCount; // IN endpoint
}

/*******************************************************************************
* Function Name  : SetEPDblBuffCount
* Description    : Set the number of bytes for a double Buffer
*                  endpoint.
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
	return (*_pEPTxCount(bEpNum)) & 0x3ff;
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
	return (*_pEPRxCount(bEpNum)) & 0x3ff;
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
	} else return(EP_DBUF_ERR);
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
		ToggleDTOG_TX(bEpNum);
	} else if (bDir == EP_DBUF_IN) {
		// IN double buffered endpoint
		ToggleDTOG_RX(bEpNum);
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
