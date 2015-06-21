// Define to prevent recursive inclusion -------------------------------------
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H


#include <stm32l1xx.h>


// Exported defines

#define USB_TX_BUF_SIZE       4096        // Size of USB data buffer (to send data to USB)

// Endpoints quantity used by the device
#define EP_NUM              (4)

// USB buffers description table
// buffer table base address
#define BTABLE_ADDRESS      (0x0000)

// EP0 - RX/TX buffer base address
#define ENDP0_RXADDR        (0x0040)
#define ENDP0_TXADDR        (0x0080)

// EP1 - TX buffer base address
#define ENDP1_TXADDR        (0x00C0)

// EP2 - TX buffer base address
#define ENDP2_TXADDR        (0x0100)

// EP3 - RX buffer base address
#define ENDP3_RXADDR        (0x0110)

// IMR_MSK - mask defining which events has to be handled by the device application software
#define IMR_MSK             (CNTR_CTRM | CNTR_WKUPM | CNTR_SUSPM | CNTR_ERRM | CNTR_SOFM | CNTR_ESOFM | CNTR_RESETM)

//#define CTR_CALLBACK
//#define DOVR_CALLBACK
//#define ERR_CALLBACK
//#define WKUP_CALLBACK
//#define SUSP_CALLBACK
//#define RESET_CALLBACK
#define SOF_CALLBACK
//#define ESOF_CALLBACK

// CTR service routines associated to defined endpoints
//#define  EP1_IN_Callback    NOP_Process
#define  EP2_IN_Callback    NOP_Process
#define  EP3_IN_Callback    NOP_Process
#define  EP4_IN_Callback    NOP_Process
#define  EP5_IN_Callback    NOP_Process
#define  EP6_IN_Callback    NOP_Process
#define  EP7_IN_Callback    NOP_Process

#define  EP1_OUT_Callback   NOP_Process
#define  EP2_OUT_Callback   NOP_Process
//#define  EP3_OUT_Callback   NOP_Process
#define  EP4_OUT_Callback   NOP_Process
#define  EP5_OUT_Callback   NOP_Process
#define  EP6_OUT_Callback   NOP_Process
#define  EP7_OUT_Callback   NOP_Process

#define BULK_MAX_PACKET_SIZE  0x0040      // Maximum packet size for bulk transfer

#define USB_EXTI_LINE         1 << 18     // USB EXTI line


// Exported variables
extern uint8_t  USB_TX_Buf[USB_TX_BUF_SIZE]; // Data buffer for USB send
extern uint32_t USB_TX_Len; // Count of bytes in USB buffer


// Functions prototypes
void USB_LP_IRQHandler(void);
void USB_FS_WKUP_IRQHandler(void);
void USB_HWConfig(void);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void Get_SerialNum(void);

void Handle_USBAsynchXfer(void);
void VCP_SendChar(uint8_t data);

#endif  // __HW_CONFIG_H
