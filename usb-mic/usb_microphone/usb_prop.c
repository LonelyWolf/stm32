/**
  ******************************************************************************
    @file    usb_prop.c
    @author  MCD Application Team
    @version V4.0.0
    @date    21-January-2013
    @brief   All processing related to Virtual Com Port Demo
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


// Includes
#include "usb_lib.h"
#include "usb_conf.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "hw_config.h"


// Private variables
uint8_t MIC_MUTE   = 0;
int16_t MIC_VOLUME = 0;
int16_t MIC_VOLUME_MIN = 0x8001;
int16_t MIC_VOLUME_MAX = 0x7FFF;
int16_t MIC_VOLUME_RES = 0x0001;


// Structures initializations
DEVICE Device_Table = {
		EP_NUM,
		1
};

DEVICE_PROP Device_Property = {
		USBdev_Init,
		USBdev_Reset,
		USBdev_Status_In,
		USBdev_Status_Out,
		USBdev_Data_Setup,
		USBdev_NoData_Setup,
		USBdev_Get_Interface_Setting,
		USBdev_GetDeviceDescriptor,
		USBdev_GetConfigDescriptor,
		USBdev_GetStringDescriptor,
		0,
		0x40 // bMaxPacketSize from USB_DeviceDescriptor
};

USER_STANDARD_REQUESTS User_Standard_Requests = {
		USBdev_GetConfiguration,
		USBdev_SetConfiguration,
		USBdev_GetInterface,
		USBdev_SetInterface,
		USBdev_GetStatus,
		USBdev_ClearFeature,
		USBdev_SetEndPointFeature,
		USBdev_SetDeviceFeature,
		USBdev_SetDeviceAddress
};

ONE_DESCRIPTOR Device_Descriptor = {
		(uint8_t *)USB_DeviceDescriptor,
		USB_DESC_SIZE_DEVICE
};

ONE_DESCRIPTOR Config_Descriptor = {
		(uint8_t *)Mic_ConfigDescriptor,
		MIC_DESC_SIZE_CONFIG
};

ONE_DESCRIPTOR String_Descriptor[4] = {
		{(uint8_t *)USB_StringLangID,  USB_STRING_SIZE_LANGID},
		{(uint8_t *)USB_StringVendor,  USB_STRING_SIZE_VENDOR},
		{(uint8_t *)USB_StringProduct, USB_STRING_SIZE_PRODUCT},
		{(uint8_t *)USB_StringSerial,  USB_STRING_SIZE_SERIAL},
};


// Public variables
extern uint8_t  packet_send;


// Private functions

/*******************************************************************************
    Function Name  : USBdev_Init.
    Description    : Mic init routine.
    Input          : None.
    Output         : None.
    Return         : None.
*******************************************************************************/
void USBdev_Init() {
	// Update the serial number string descriptor with the data from the unique ID
	Get_SerialNum();
	// Initialize the current configuration
	pInformation->Current_Configuration = 0;
	// Connect the device
	PowerOn();
	// Perform basic device initialization operations
	USB_SIL_Init();
	bDeviceState = UNCONNECTED;
}

/*******************************************************************************
    Function Name  : USBdev_Reset.
    Description    : Mic reset routine.
    Input          : None.
    Output         : None.
    Return         : None.
*******************************************************************************/
void USBdev_Reset() {
	// Set Mic device as not configured state
	pInformation->Current_Configuration = 0;

	// Current Feature initialization
	pInformation->Current_Feature = Mic_ConfigDescriptor[7];

//	*BTABLE = BTABLE_ADDRESS & 0xFFF8;
	*BTABLE = BTABLE_ADDRESS; // BTABLE_ADDRESS = 0

	// Initialize Endpoint 0
	SetEPType(ENDP0,EP_CONTROL);
	SetEPTxStatus(ENDP0,EP_TX_NAK);
	SetEPRxAddr(ENDP0,ENDP0_RXADDR);
	SetEPRxCount(ENDP0,Device_Property.MaxPacketSize);
	SetEPTxAddr(ENDP0,ENDP0_TXADDR);
	Clear_Status_Out(ENDP0);
	SetEPRxValid(ENDP0);

	// Initialize Endpoint 1 (as double buffer)
	SetEPType(ENDP1,EP_ISOCHRONOUS);
	SetEPDoubleBuff(ENDP1);
	SetEPDblBuffAddr(ENDP1,ENDP1_BUF0Addr,ENDP1_BUF1Addr);
	SetEPDblBuffCount(ENDP1,EP_DBUF_IN,Device_Property.MaxPacketSize);
	ClearDTOG_TX(ENDP1);
	ClearDTOG_RX(ENDP1);
	ToggleDTOG_RX(ENDP1);
	SetEPRxStatus(ENDP1,EP_RX_DIS);
	SetEPTxStatus(ENDP1,EP_TX_VALID);
	SetEPRxValid(ENDP0);

	// Set this device to response on default address
	SetDeviceAddress(0);

	bDeviceState = ATTACHED;

	packet_send = 1;
}

/*******************************************************************************
    Function Name  : USBdev_SetConfiguration.
    Description    : Update the device state to configured.
    Input          : None.
    Output         : None.
    Return         : None.
*******************************************************************************/
void USBdev_SetConfiguration(void) {
	if (Device_Info.Current_Configuration) bDeviceState = CONFIGURED;
}

/*******************************************************************************
    Function Name  : USBdev_SetConfiguration.
    Description    : Update the device state to addressed.
    Input          : None.
    Output         : None.
    Return         : None.
*******************************************************************************/
void USBdev_SetDeviceAddress(void) {
	bDeviceState = ADDRESSED;
}

/*******************************************************************************
    Function Name  : USBdev_Status_In.
    Description    : Mic Status In routine.
    Input          : None.
    Output         : None.
    Return         : None.
*******************************************************************************/
void USBdev_Status_In(void) {
}

/*******************************************************************************
    Function Name  : USBdev_Status_Out.
    Description    : Mic Status Out routine.
    Input          : None.
    Output         : None.
    Return         : None.
*******************************************************************************/
void USBdev_Status_Out(void) {
}

/*******************************************************************************
    Function Name  : USBdev_Data_Setup
    Description    : Handle the data class specific requests.
    Input          : None.
    Output         : None.
    Return         : USB_UNSUPPORT or USB_SUCCESS.
*******************************************************************************/
RESULT USBdev_Data_Setup(uint8_t RequestNo) {
	uint8_t *(*CopyRoutine)(uint16_t) = NULL;

	switch (pInformation->USBbmRequestType) {
		case 0xA1:
			UART_SendStr(USART2," -> GET FU\n");
			CopyRoutine = Mic_FU_Get;
			break;
		case 0x21:
			UART_SendStr(USART2," -> SET FU\n");
			CopyRoutine = Mic_FU_Set;
			break;
		case 0xA2:
			UART_SendStr(USART2," -> GET EP\n");
			CopyRoutine = Mic_EP_Get;
			break;
		case 0x22:
			UART_SendStr(USART2," -> SET EP\n");
			CopyRoutine = Mic_EP_Set;
			break;
		default:
			return USB_UNSUPPORT;
			break;
	}

	pInformation->Ctrl_Info.CopyData = CopyRoutine;
	pInformation->Ctrl_Info.Usb_wOffset = 0;
	(*CopyRoutine)(0);

	return USB_SUCCESS;
}

/*******************************************************************************
    Function Name  : USBdev_NoData_Setup
    Description    : Handle the no data class specific requests.
    Input          : None.
    Output         : None.
    Return         : USB_UNSUPPORT or USB_SUCCESS.
*******************************************************************************/
RESULT USBdev_NoData_Setup(uint8_t RequestNo) {
	return USB_UNSUPPORT;
}

/*******************************************************************************
    Function Name  : USBdev_GetDeviceDescriptor.
    Description    : Get the device descriptor.
    Input          : Length : uint16_t.
    Output         : None.
    Return         : The address of the device descriptor.
*******************************************************************************/
uint8_t *USBdev_GetDeviceDescriptor(uint16_t Length) {
	return Standard_GetDescriptorData(Length,&Device_Descriptor);
}

/*******************************************************************************
    Function Name  : USBdev_GetConfigDescriptor.
    Description    : Get the configuration descriptor.
    Input          : Length : uint16_t.
    Output         : None.
    Return         : The address of the configuration descriptor.
*******************************************************************************/
uint8_t *USBdev_GetConfigDescriptor(uint16_t Length) {
	return Standard_GetDescriptorData(Length,&Config_Descriptor);
}

/*******************************************************************************
    Function Name  : USBdev_GetStringDescriptor.
    Description    : Get the string descriptors according to the needed index.
    Input          : Length : uint16_t.
    Output         : None.
    Return         : The address of the string descriptors.
*******************************************************************************/
uint8_t *USBdev_GetStringDescriptor(uint16_t Length) {
	uint8_t wValue0 = pInformation->USBwValue0;

	if (wValue0 > 4) {
		return NULL;
	} else {
		return Standard_GetDescriptorData(Length,&String_Descriptor[wValue0]);
	}
}

/*******************************************************************************
    Function Name  : USBdev_Get_Interface_Setting.
    Description    : test the interface and the alternate setting according to the
                   supported one.
    Input1         : uint8_t: Interface : interface number.
    Input2         : uint8_t: AlternateSetting : Alternate Setting number.
    Output         : None.
    Return         : The address of the string descriptors.
*******************************************************************************/
RESULT USBdev_Get_Interface_Setting(uint8_t Interface, uint8_t AlternateSetting) {
	if (AlternateSetting > 1) {
		return USB_UNSUPPORT;
	} else if (Interface > 1) {
		return USB_UNSUPPORT;
	}
	return USB_SUCCESS;
}

// Handle feature unit GET control request
uint8_t *Mic_FU_Get(uint16_t Length) {
	if (Length) {
		switch (Length) {
			case 0x0001:
				// Mute control supports only CUR
				UART_SendStr(USART2,"  -> MUTE CUR\n");
				return &MIC_MUTE;
				break;
			case 0x0002:
				// Volume control
				UART_SendStr(USART2,"  -> VOLUME ");
				switch (pInformation->USBbRequest) {
					case GET_CUR:
						UART_SendStr(USART2,"CUR\n");
						return (uint8_t *)&MIC_VOLUME;
						break;
					case GET_MAX:
						UART_SendStr(USART2,"MAX\n");
						return (uint8_t *)&MIC_VOLUME_MAX;
						break;
					case GET_MIN:
						UART_SendStr(USART2,"MIN\n");
						return (uint8_t *)&MIC_VOLUME_MIN;
						break;
					case GET_RES:
						UART_SendStr(USART2,"RES\n");
						return (uint8_t *)&MIC_VOLUME_RES;
						break;
					default:
						break;
				}
				break;
			default:
				// Don't know how to handle this
				break;
		}

		return NULL;
	} else {
		switch (Length) {
			case 0x0001:
				// Mute control
				pInformation->Ctrl_Info.Usb_wLength = 1;
				break;
			case 0x0002:
				// Volume control
				pInformation->Ctrl_Info.Usb_wLength = 2;
				break;
			default:
				pInformation->Ctrl_Info.Usb_wLength = pInformation->USBwLengths.w;
				break;
		}

		return NULL;
	}
}

// Handle feature unit SET control request
uint8_t *Mic_FU_Set(uint16_t Length) {
	if (Length) {
		return (uint8_t *)(&MIC_MUTE);
	} else {
		pInformation->Ctrl_Info.Usb_wLength = pInformation->USBwLengths.w;

		return NULL;
	}
}

// Handle endpoint GET control request
uint8_t *Mic_EP_Get(uint16_t Length) {
	if (Length) {
		return (uint8_t *)(&MIC_MUTE);
	} else {
		pInformation->Ctrl_Info.Usb_wLength = pInformation->USBwLengths.w;

		return NULL;
	}
}

// Handle endpoint SET control request
uint8_t *Mic_EP_Set(uint16_t Length) {
	if (Length) {
		UART_SendStr(USART2,"EP: ");
		UART_SendHex8(USART2,pInformation->USBwIndexs.bw.bb0);
		UART_SendChar(USART2,'\n');

		if (pInformation->USBwIndexs.bw.bb0 != 0x81) {
			// Don't know to handle this EP
			UART_SendStr(USART2,"DOH!\n");
			return NULL;
		}
		UART_SendStr(USART2,"  -> SAMPLE RATE\n");

		if (Length != 3) {
			// Don't know how to handle this
			return NULL;
		}

		// Set sampling rate request
		UART_SendStr(USART2,"bRequest: ");
		UART_SendHex16(USART2,pInformation->USBbRequest);
		UART_SendChar(USART2,'\n');

/*
		UART_SendStr(USART2,"wValue: ");
		UART_SendHex16(USART2,pInformation->USBwValues.w);
		UART_SendChar(USART2,'\n');
		UART_SendStr(USART2,"Length: ");
		UART_SendHex16(USART2,Length);
		UART_SendChar(USART2,'\n');
		UART_SendStr(USART2,"bRequest: ");
		UART_SendHex16(USART2,pInformation->USBbRequest);
		UART_SendChar(USART2,'\n');
*/

		return (uint8_t *)(&MIC_MUTE);
	} else {
		pInformation->Ctrl_Info.Usb_wLength = pInformation->USBwLengths.w;

		return NULL;
	}
}
