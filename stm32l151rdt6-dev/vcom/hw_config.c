#include "hw_config.h"
#include "usb_lib.h"
#include "usb_istr.h"
#include "usb_pwr.h"
#include "usb_desc.h"
#include "misc.h"


// Private variables
uint8_t  USB_TX_BUF[USB_TX_BUF_SIZE]; // Data buffer for USB send
uint32_t USB_TX_ptr_in  = 0; // Head byte in USB buffer
uint32_t USB_TX_ptr_out = 0; // Tail byte in USB buffer
uint32_t USB_TX_len     = 0; // Amount of data in USB buffer


// Handle USB high priority interrupts
void USB_HP_IRQHandler(void) {
	CTR_HP();
}

// Handle USB low priority interrupts
void USB_LP_IRQHandler(void) {
	USB_Istr();
}

// Handle USB wake-up interrupt request
void USB_FS_WKUP_IRQHandler(void) {
	EXTI->PR = USB_EXTI_LINE; // Clear IT bit for EXTI_Line18
}

// Configure USB peripheral and interrupts
void USB_HWConfig(void) {
	NVIC_InitTypeDef NVICInit;

	// Enable the SYSCFG module clock
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// Configure the EXTI line 18 connected internally to the USB IP
	EXTI->PR    =  USB_EXTI_LINE; // Clear IT pending bit for EXTI18
	EXTI->IMR  |=  USB_EXTI_LINE; // Enable interrupt request from EXTI18
	EXTI->EMR  &= ~USB_EXTI_LINE; // Disable event on EXTI18
	EXTI->RTSR |=  USB_EXTI_LINE; // Trigger rising edge enabled
	EXTI->FTSR &= ~USB_EXTI_LINE; // Trigger falling edge disabled

	// Enable the USB clock
	RCC->APB1ENR |= RCC_APB1ENR_USBEN;

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

// Power-off system clocks and power while entering suspend mode
void Enter_LowPowerMode(void) {
	// Set the device state to suspend
	bDeviceState = SUSPENDED;
}

// Restores system clocks and power while exiting suspend mode
void Leave_LowPowerMode(void) {
	bDeviceState = Device_Info.Current_Configuration ? CONFIGURED : ATTACHED;

	// Enable SystemCoreClock
	SystemInit();
}

// Convert 32-bit integer to Unicode character
// input:
//   value - 32-bit integer to convert
//   pbuf - pointer to result buffer
//   len - length of result buffer
static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len) {
	uint8_t i;

	for (i = 0; i < len; i++) {
		if ((value >> 28) < 0xA) {
			pbuf[i << 1] = (value >> 28) + '0';
		} else {
			pbuf[i << 1] = (value >> 28) + 'A' - 10;
		}
		value <<= 4;
		pbuf[2 * i + 1] = 0;
	}
}

// Create the serial number string
void Get_SerialNum(void) {
	uint32_t devSerial0,devSerial1,devSerial2;

//	devSerial0  = *(__I uint32_t*)(0x1FF80050); // STM32 DevID (for STM32L Cat.1,2 devices)
//	devSerial1  = *(__I uint32_t*)(0x1FF80054);
//	devSerial2  = *(__I uint32_t*)(0x1FF80058);

	devSerial0  = *(__I uint32_t*)(0x1FF800D0); // STM32 DevID (for STM32L Cat.3,4,5 devices)
	devSerial1  = *(__I uint32_t*)(0x1FF800D4);
	devSerial2  = *(__I uint32_t*)(0x1FF800D8);

	devSerial0 += devSerial2;

	if (devSerial0) {
	    IntToUnicode(devSerial0,&USB_StringSerial[ 2],8);
	    IntToUnicode(devSerial1,&USB_StringSerial[18],4);
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

	if (USB_TX_len > BULK_MAX_PACKET_SIZE) {
		tx_ptr = USB_TX_ptr_out;
		tx_len = BULK_MAX_PACKET_SIZE;
		USB_TX_ptr_out += BULK_MAX_PACKET_SIZE;
		USB_TX_len -= BULK_MAX_PACKET_SIZE;
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
