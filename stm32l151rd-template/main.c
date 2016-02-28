#include "main.h"


int main(void) {
	// Initialize the MCU clock system
	SystemInit();
	SetSysClock();
	SystemCoreClockUpdate();

    while(1);
}
