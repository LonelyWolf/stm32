#include "exti.h"


// Note: in order to be able to configure EXTI lines the SYSCFG peripheral must be enabled


// Initialize the EXTI lines(s) in range from 0 to 31 according to the specified parameters
// input:
//   EXTI_Line - specifies the EXTI line(s), any combination of EXTI_Line[0..31] values
//   EXTI_mode - mode of the EXTI line, one of EXTI_MODE_xx values
//   EXTI_trigger - trigger signal active edge, one of EXTI_TRG_xx values
void EXTI_cfg1(uint32_t EXTI_Line, uint32_t EXTI_mode, uint32_t EXTI_trigger) {
	// Clear IT pending bit for EXTI line
	EXTI_ClearFlag1(EXTI_Line);

	// Configure IRQ generation
	if (EXTI_mode & EXTI_MODE_IRQ) {
		EXTI->IMR1 |= EXTI_Line;
	} else {
		EXTI->IMR1 &= ~EXTI_Line;
	}

	// Configure event generation
	if (EXTI_mode & EXTI_MODE_EVT) {
		EXTI->EMR1 |= EXTI_Line;
	} else {
		EXTI->EMR1 &= ~EXTI_Line;
	}

	// Configure trigger on rising edge
	if (EXTI_trigger & EXTI_TRG_RISING) {
		EXTI->RTSR1 |= EXTI_Line;
	} else {
		EXTI->RTSR1 &= ~EXTI_Line;
	}

	// Configure trigger on falling edge
	if (EXTI_trigger & EXTI_TRG_FALLING) {
		EXTI->FTSR1 |= EXTI_Line;
	} else {
		EXTI->FTSR1 &= ~EXTI_Line;
	}
}

// Initialize the EXTI lines(s) in range from 32 to 39 according to the specified parameters
// input:
//   EXTI_Line - specifies the EXTI line(s), any combination of EXTI_Line[32..39] values
//   EXTI_mode - mode of the EXTI line, one of EXTI_MODE_xx values
//   EXTI_trigger - trigger signal active edge, one of EXTI_TRG_xx values
void EXTI_cfg2(uint32_t EXTI_Line, uint32_t EXTI_mode, uint32_t EXTI_trigger) {
	// Clear IT pending bit for EXTI line
	EXTI_ClearFlag2(EXTI_Line);

	// Configure IRQ generation
	if (EXTI_mode & EXTI_MODE_IRQ) {
		EXTI->IMR2 |= EXTI_Line;
	} else {
		EXTI->IMR2 &= ~EXTI_Line;
	}

	// Configure event generation
	if (EXTI_mode & EXTI_MODE_EVT) {
		EXTI->EMR2 |= EXTI_Line;
	} else {
		EXTI->EMR2 &= ~EXTI_Line;
	}

	// Configure trigger on rising edge
	if (EXTI_trigger & EXTI_TRG_RISING) {
		EXTI->RTSR2 |= EXTI_Line;
	} else {
		EXTI->RTSR2 &= ~EXTI_Line;
	}

	// Configure trigger on falling edge
	if (EXTI_trigger & EXTI_TRG_FALLING) {
		EXTI->FTSR2 |= EXTI_Line;
	} else {
		EXTI->FTSR2 &= ~EXTI_Line;
	}
}

// Select the GPIO pin used as EXTI line
// input:
//   port_src - GPIO port to be used as source for EXTI lines, one of EXTI_SRC_PORTx values
//   pin_src - EXTI line to be configured, one of EXTI_PIN_SRCx values
void EXTI_src(uint32_t port_src, uint32_t pin_src) {
	uint32_t shift = (pin_src & 0x03) << 2;

	SYSCFG->EXTICR[pin_src >> 2] &= ~(0x0F << shift);
	SYSCFG->EXTICR[pin_src >> 2] |= port_src << shift;
}
