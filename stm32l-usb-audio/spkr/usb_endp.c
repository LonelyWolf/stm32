/**
  ******************************************************************************
    @file    usb_endp.c
    @author  MCD Application Team
    @version V4.0.0
    @date    21-January-2013
    @brief   Endpoint routines
  ******************************************************************************
    @attention

    <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>

    Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
    You may not use this file except in compliance with the License.
    You may obtain a copy of the License at:

           http://www.st.com/software_license_agreement_liberty_v2

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.

  ******************************************************************************
*/


#include "usb_lib.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_istr.h"
#include "usb_pwr.h"


uint8_t  stream_buffer[100];
uint16_t In_Data_Offset;
uint16_t Out_Data_Offset;


/*******************************************************************************
    Function Name  : EP1_OUT_Callback (from host to device)
    Description    : Endpoint 1 out callback routine.
    Input          : None.
    Output         : None.
    Return         : None.
*******************************************************************************/
void EP1_OUT_Callback(void) {
	uint16_t dlen; // received data length

	if (_GetENDPOINT(ENDP1) & EP_DTOG_TX) {
		// read from ENDP1_BUF0Addr buffer
		dlen = GetEPDblBuf0Count(ENDP1);
		PMAToUserBufferCopy(stream_buffer,ENDP1_BUF0Addr,dlen);
	} else {
		// read from ENDP1_BUF1Addr buffer
		dlen = GetEPDblBuf1Count(ENDP1);
		PMAToUserBufferCopy(stream_buffer,ENDP1_BUF1Addr,dlen);
	}

	FreeUserBuffer(ENDP1,EP_DBUF_OUT);
	In_Data_Offset += dlen;
}

/*******************************************************************************
* Function Name  : USB_Istr
* Description    : Start of frame callback function.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SOF_Callback(void) {
	In_Data_Offset  = 0;
	Out_Data_Offset = 0;
}
