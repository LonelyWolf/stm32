/**
  ******************************************************************************
  * @file    hw_config.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Hardware Configuration & Setup
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

#include "usb_istr.h"
#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_pwr.h"


uint8_t  USB_TX_BUF[USB_TX_BUF_SIZE]; // Data buffer for USB send
uint32_t USB_TX_ptr_in  = 0; // Head byte in USB buffer
uint32_t USB_TX_ptr_out = 0; // Tail byte in USB buffer
uint32_t USB_TX_len     = 0; // Amount of data in USB buffer


static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len);


/*******************************************************************************
* Function Name  : USB_IRQHandler
* Description    : This function handles USB Low Priority interrupts
*                  requests.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_LP_IRQHandler(void) {
	USB_Istr();
}

/*******************************************************************************
* Function Name  : USB_FS_WKUP_IRQHandler
* Description    : This function handles USB WakeUp interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
void USB_FS_WKUP_IRQHandler(void) {
	EXTI->PR = USB_EXTI_LINE; // Clear IT bit for EXTI_Line18
}

/*******************************************************************************
* Function Name  : USB_HWConfig
* Description    : Configures USB peripheral and interrupts
* Input          : None.
* Return         : None.
*******************************************************************************/
void USB_HWConfig(void) {
	NVIC_InitTypeDef NVICInit;

	// Enable the SYSCFG module clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);

	// Configure the EXTI line 18 connected internally to the USB IP
	EXTI->PR    =  USB_EXTI_LINE; // Clear IT pending bit for EXTI18
	EXTI->IMR  |=  USB_EXTI_LINE; // Enable interrupt request from EXTI18
	EXTI->EMR  &= ~USB_EXTI_LINE; // Disable event on EXTI18
	EXTI->RTSR |=  USB_EXTI_LINE; // Trigger rising edge enabled
	EXTI->FTSR &= ~USB_EXTI_LINE; // Trigger falling edge disabled

	// Enable the USB clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB,ENABLE);

	// Configure USB interrupts
	// 2 bit for pre-emption priority, 2 bits for subpriority
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	// Enable the USB interrupt
	NVICInit.NVIC_IRQChannel = USB_LP_IRQn;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 2;
	NVICInit.NVIC_IRQChannelSubPriority = 0;
	NVICInit.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVICInit);

	// Enable the USB Wake-up interrupt
	NVICInit.NVIC_IRQChannel = USB_FS_WKUP_IRQn;
	NVICInit.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_Init(&NVICInit);
}

/*******************************************************************************
* Function Name  : Enter_LowPowerMode
* Description    : Power-off system clocks and power while entering suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(void) {
	// Set the device state to suspend
	bDeviceState = SUSPENDED;
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode
* Description    : Restores system clocks and power while exiting suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void) {
	DEVICE_INFO *pInfo = &Device_Info;

	// Set the device state to the correct state
//	if (pInfo->Current_Configuration != 0) {
//		 Device configured
//		bDeviceState = CONFIGURED;
//	} else {
//		bDeviceState = ATTACHED;
//	}
	bDeviceState = pInfo->Current_Configuration ? CONFIGURED : ATTACHED;

	// Enable SystemCoreClock
	SystemInit();
}

// Create the sertial number string descriptor
// author: MCD Application Team
void Get_SerialNum(void) {
	uint32_t devSerial0, devSerial1, devSerial2;

	devSerial0  = *(volatile uint32_t*)(0x1FF80050);
	devSerial1  = *(volatile uint32_t*)(0x1FF80054);
	devSerial2  = *(volatile uint32_t*)(0x1FF80064);
	devSerial0 += devSerial2;

	if (devSerial0) {
		IntToUnicode(devSerial0,&Virtual_Com_Port_StringSerial[2] ,8);
		IntToUnicode(devSerial1,&Virtual_Com_Port_StringSerial[18],4);
	}
}

// Convert 32-bit integer to Unicode character
// input:
//   value - 32-bit integer to convert
//   pbuf - pointer to result buffer
//   len - length of result buffer
// author: MCD Application Team
static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len) {
	uint8_t idx;

	for (idx = 0; idx < len; idx++) {
		if( ((value >> 28)) < 0xA ) {
			pbuf[idx << 1] = (value >> 28) + '0';
		} else {
			pbuf[idx << 1] = (value >> 28) + 'A' - 10;
		}
		value <<= 4;
		pbuf[2 * idx + 1] = 0;
	}
}

// Send data from buffer to USB
// author: MCD Application Team
void Handle_USBAsynchXfer(void) {
	uint16_t tx_ptr; // Pointer to first byte in USB buffer
	uint16_t tx_len; // Amount of data to be transfered

	if (USB_TX_ptr_out == USB_TX_BUF_SIZE) USB_TX_ptr_out = 0;
	if (USB_TX_ptr_out == USB_TX_ptr_in) return; // Nothing to send

	if (USB_TX_ptr_out > USB_TX_ptr_in) {
		// rollback
		USB_TX_len = USB_TX_BUF_SIZE - USB_TX_ptr_out;
	} else {
		USB_TX_len = USB_TX_ptr_in - USB_TX_ptr_out;
	}

	if (USB_TX_len > VIRTUAL_COM_PORT_DATA_SIZE) {
		tx_ptr = USB_TX_ptr_out;
		tx_len = VIRTUAL_COM_PORT_DATA_SIZE;
		USB_TX_ptr_out += VIRTUAL_COM_PORT_DATA_SIZE;
		USB_TX_len -= VIRTUAL_COM_PORT_DATA_SIZE;
	} else {
		tx_ptr = USB_TX_ptr_out;
		tx_len = USB_TX_len;
		USB_TX_ptr_out += USB_TX_len;
		USB_TX_len = 0;
	}

	UserToPMABufferCopy(&USB_TX_BUF[tx_ptr],ENDP1_TXADDR,tx_len);
	SetEPTxCount(ENDP1,tx_len);
	SetEPTxValid(ENDP1);
}

// Send single byte to VCOM
// input:
//   data - byte to send
void VCP_SendChar(uint8_t data) {
	// Put byte into data buffer
	USB_TX_BUF[USB_TX_ptr_in++] = data;

	// To avoid buffer overflow
	if (USB_TX_ptr_in >= USB_TX_BUF_SIZE) USB_TX_ptr_in = 0;
}
