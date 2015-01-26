#include "usb_lib.h"
#include "usb_regs.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "usb_pwr.h"
#include "hw_config.h"


// Private variables
uint8_t Request = 0;

LINE_CODING linecoding = {
		115200, // baud rate
		0x00,   // stop bits - 1
		0x00,   // parity - none
		0x08    // no. of bits 8
};

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
		(uint8_t *)VCOM_ConfigDescriptor,
		VCOM_DESC_SIZE_CONFIG
};

ONE_DESCRIPTOR String_Descriptor[5] = {
		{(uint8_t *)USB_StringLangID,  USB_STRING_SIZE_LANGID},
		{(uint8_t *)USB_StringVendor,  USB_STRING_SIZE_VENDOR},
		{(uint8_t *)USB_StringProduct, USB_STRING_SIZE_PRODUCT},
		{(uint8_t *)USB_StringSerial,  USB_STRING_SIZE_SERIAL}
};


// Private functions

/*******************************************************************************
    Function Name  : USBdev_Init.
    Description    : USB device init routine.
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
    Description    : USB device reset routine.
    Input          : None.
    Output         : None.
    Return         : None.
*******************************************************************************/
void USBdev_Reset() {
	// Set DEVICE as not configured
	pInformation->Current_Configuration = 0;

	// Current Feature initialization
	pInformation->Current_Feature = VCOM_ConfigDescriptor[7];

	// Set DEVICE with the default Interface
	pInformation->Current_Interface = 0;

	// BTABLE_ADDRESS = 0
	USB->BTABLE = BTABLE_ADDRESS;

	// Initialize Endpoint 0
	SetEPType(ENDP0,EP_CONTROL);
	SetEPTxStatus(ENDP0,EP_TX_STALL);
	SetEPRxAddr(ENDP0,ENDP0_RXADDR);
	SetEPTxAddr(ENDP0,ENDP0_TXADDR);
	Clear_Status_Out(ENDP0);
	SetEPRxCount(ENDP0,Device_Property.MaxPacketSize);
	SetEPRxValid(ENDP0);

	// Initialize Endpoint 1
	SetEPType(ENDP1,EP_BULK);
	SetEPTxAddr(ENDP1,ENDP1_TXADDR);
	SetEPTxStatus(ENDP1,EP_TX_NAK);
	SetEPRxStatus(ENDP1,EP_RX_DIS);

	// Initialize Endpoint 2
	SetEPType(ENDP2,EP_INTERRUPT);
	SetEPTxAddr(ENDP2,ENDP2_TXADDR);
	SetEPRxStatus(ENDP2,EP_RX_DIS);
	SetEPTxStatus(ENDP2,EP_TX_NAK);

	// Initialize Endpoint 3
	SetEPType(ENDP3,EP_BULK);
	SetEPRxAddr(ENDP3,ENDP3_RXADDR);
	SetEPRxCount(ENDP3,VCOM_INT_SIZE);
	SetEPRxStatus(ENDP3,EP_RX_VALID);
	SetEPTxStatus(ENDP3,EP_TX_DIS);

	// Set this device to response on default address
	SetDeviceAddress(0);

	bDeviceState = ATTACHED;
}


/*******************************************************************************
* Function Name  : USBdev_SetConfiguration
* Description    : Handle the SetConfiguration request.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USBdev_SetConfiguration(void) {
	if (Device_Info.Current_Configuration) bDeviceState = CONFIGURED;
}

/*******************************************************************************
* Function Name  : USBdev_SetConfiguration.
* Description    : Update the device state to addressed.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USBdev_SetDeviceAddress(void) {
	bDeviceState = ADDRESSED;
}

/*******************************************************************************
    Function Name  : USBdev_Status_In.
    Description    : Status In routine.
    Input          : None.
    Output         : None.
    Return         : None.
*******************************************************************************/
void USBdev_Status_In(void) {
	if (Request == SET_LINE_CODING) Request = 0;
}

/*******************************************************************************
    Function Name  : USBdev_Status_Out.
    Description    : Status Out routine.
    Input          : None.
    Output         : None.
    Return         : None.
*******************************************************************************/
void USBdev_Status_Out(void) {
	// No actions here
}

/*******************************************************************************
* Function Name  : USBdev_Data_Setup.
* Description    : Handle the data class specific requests..
* Input          : RequestNo.
* Output         : None.
* Return         : RESULT.
*******************************************************************************/
RESULT USBdev_Data_Setup(uint8_t RequestNo) {
	uint8_t *(*CopyRoutine)(uint16_t);

	CopyRoutine = NULL;
	if (RequestNo == GET_LINE_CODING) {
		if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT)) {
			CopyRoutine = VCOM_GetLineCoding;
		}
	} else if (RequestNo == SET_LINE_CODING) {
		if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT)) {
			CopyRoutine = VCOM_SetLineCoding;
		}
		Request = SET_LINE_CODING;
	}

	if (CopyRoutine == NULL) return USB_UNSUPPORT;

	pInformation->Ctrl_Info.CopyData    = CopyRoutine;
	pInformation->Ctrl_Info.Usb_wOffset = 0;
	(*CopyRoutine)(0);

	return USB_SUCCESS;
}

/*******************************************************************************
* Function Name  : USBdev_NoData_Setup.
* Description    : Handle the no data class specific requests.
* Input          : RequestNo.
* Output         : None.
* Return         : RESULT.
*******************************************************************************/
RESULT USBdev_NoData_Setup(uint8_t RequestNo) {
	if (Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT)) {
		if (RequestNo == SET_COMM_FEATURE) {
			return USB_SUCCESS;
		} else if (RequestNo == SET_CONTROL_LINE_STATE) return USB_SUCCESS;
	}

	return USB_UNSUPPORT;
}

/*******************************************************************************
* Function Name  : USBdev_Get_Interface_Setting
* Description    : Test the interface and the alternate setting according to the
*                  supported one.
* Input          : uint8_t Interface, uint8_t AlternateSetting.
* Output         : None.
* Return         : RESULT.
*******************************************************************************/
RESULT USBdev_Get_Interface_Setting(uint8_t Interface, uint8_t AlternateSetting) {
	if (AlternateSetting > 0) {
		// in this application we don't have AlternateSetting
		return USB_UNSUPPORT;
	} else if (Interface > 0) {
		// in this application we have only 1 interface
		return USB_UNSUPPORT;
	}

	return USB_SUCCESS;
}

/*******************************************************************************
* Function Name  : USBdev_GetDeviceDescriptor
* Description    : Get the device descriptor.
* Input          : uint16_t Length.
* Output         : None.
* Return         : None.
*******************************************************************************/
uint8_t *USBdev_GetDeviceDescriptor(uint16_t Length) {
	return Standard_GetDescriptorData(Length,&Device_Descriptor);
}

/*******************************************************************************
* Function Name  : USBdev_GetConfigDescriptor
* Description    : Get the configuration descriptor.
* Input          : uint16_t Length.
* Output         : None.
* Return         : None.
*******************************************************************************/
uint8_t *USBdev_GetConfigDescriptor(uint16_t Length) {
	return Standard_GetDescriptorData(Length,&Config_Descriptor);
}

/*******************************************************************************
* Function Name  : USBdev_GetStringDescriptor
* Description    : Get the string descriptors according to the needed index.
* Input          : uint16_t Length.
* Output         : None.
* Return         : None.
*******************************************************************************/
uint8_t *USBdev_GetStringDescriptor(uint16_t Length) {
	uint8_t wValue0 = pInformation->USBwValue0;

	return (wValue0 > 4) ? NULL : Standard_GetDescriptorData(Length,&String_Descriptor[wValue0]);
}

/*******************************************************************************
* Function Name  : USBdev_Get_Interface_Setting.
* Description    : test the interface and the alternate setting according to the
*                  supported one.
* Input1         : uint8_t: Interface : interface number.
* Input2         : uint8_t: AlternateSetting : Alternate Setting number.
* Output         : None.
* Return         : The address of the string descriptors.
*******************************************************************************/
RESULT USBDev_Get_Interface_Setting(uint8_t Interface, uint8_t AlternateSetting) {
	if (AlternateSetting > 0) return USB_UNSUPPORT; else if (Interface > 1) return USB_UNSUPPORT;

	return USB_SUCCESS;
}

/*******************************************************************************
* Function Name  : VCOM_GetLineCoding.
* Description    : send the linecoding structure to the PC host.
* Input          : Length.
* Output         : None.
* Return         : Linecoding structure base address.
*******************************************************************************/
uint8_t *VCOM_GetLineCoding(uint16_t Length) {
	if (Length == 0) {
		pInformation->Ctrl_Info.Usb_wLength = sizeof(linecoding);

		return NULL;
	}

	return(uint8_t *)&linecoding;
}

/*******************************************************************************
* Function Name  : VCOM_SetLineCoding.
* Description    : Set the linecoding structure fields.
* Input          : Length.
* Output         : None.
* Return         : Linecoding structure base address.
*******************************************************************************/
uint8_t *VCOM_SetLineCoding(uint16_t Length) {
	if (Length == 0) {
		pInformation->Ctrl_Info.Usb_wLength = sizeof(linecoding);

		return NULL;
	}

	return(uint8_t *)&linecoding;
}
