/**
  ******************************************************************************
  * @file    usb_endp.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Endpoint routines
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
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
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_istr.h"
#include "usb_pwr.h"

// Interval between sending IN packets in frame number (1 frame = 1ms)
#define VCOMPORT_IN_FRAME_INTERVAL             5


// Receive buffer for virtual COM port
uint8_t VCOM_RX_BUF[VIRTUAL_COM_PORT_DATA_SIZE];


/*******************************************************************************
* Function Name  : EP1_IN_Callback (OUT transfers: from host to device)
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP1_IN_Callback (void) {
	Handle_USBAsynchXfer();
}

/*******************************************************************************
* Function Name  : EP3_OUT_Callback (IN transfers: from device to host)
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void EP3_OUT_Callback(void) {
	uint16_t USB_Rx_Cntr;
	uint8_t *ptr = VCOM_RX_BUF;

	// Get the received data buffer and update the counter
	USB_Rx_Cntr = USB_SIL_Read(EP3_OUT,VCOM_RX_BUF);

	// USB data will be immediately processed, this allow next USB traffic
	// being NAKed till the end of the USART Xfer

	// Send received data back (just for fun)
	while (USB_Rx_Cntr--) VCP_SendChar(*ptr++);

	// Enable the receive of data on EP3
	SetEPRxValid(ENDP3);
}

/*******************************************************************************
* Function Name  : SOF_Callback / INTR_SOFINTR_Callback (USB Start Of Frame)
* Description    :
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void SOF_Callback(void) {
	static uint32_t FrameCount = 0;

	if(bDeviceState == CONFIGURED) {
		if (FrameCount++ == VCOMPORT_IN_FRAME_INTERVAL) {
			// Reset the frame counter
			FrameCount = 0;
			// Check the data to be sent through IN pipe
			Handle_USBAsynchXfer();
		}
	}
}
