///////////////////
// STM32L151RBT6 //
///////////////////


#include <stm32l1xx_rcc.h>
#include <stm32l1xx_gpio.h>
#include <stm32l1xx_syscfg.h>
#include <misc.h>

// USB related stuff
#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"

// My stuff
#include "delay.h"
#include "VCP.h"


uint32_t i = 0;


int main(void) {
	// Enable debugging when the MCU is in low power modes
	DBGMCU->CR |= DBGMCU_CR_DBG_SLEEP | DBGMCU_CR_DBG_STOP | DBGMCU_CR_DBG_STANDBY;

	// Initialize delay without callback
	Delay_Init(0);
	Delay_ms(1000);

	// Configure USB peripheral
	USB_HWConfig();

	// Initialize USB device
	USB_Init();

    while(1) {
    	Delay_ms(1000);
    	VCP_SendStr("VCP: ");
    	VCP_SendInt(i++);
    	VCP_SendChar('\n');
    }
}
