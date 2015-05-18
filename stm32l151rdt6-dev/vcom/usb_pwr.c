#include "usb_lib.h"
#include "usb_pwr.h"


// Private variables
__IO DEVICE_STATE bDeviceState = UNCONNECTED; // USB device status
__IO bool fSuspendEnabled = FALSE; // true when suspend is possible
__IO uint32_t remotewakeupon = 0; // Remote wake-up state machine
__IO uint32_t EP[8]; // Array to store endpoint registers
__IO uint32_t rCounter = 0; // Resume_Later counter (trick to track cable disconnection)


// Private structures
struct {
	__IO RESUME_STATE eState;
	__IO uint8_t      bESOFcnt;
} ResumeS;


// Handle Switch-On conditions
// return: USB_SUCCESS
RESULT PowerOn(void) {
	// Connect internal pull-up on USB DP line
	SYSCFG->PMC |= (uint32_t)SYSCFG_PMC_USB_PU;
	// CNTR_PWDN = 0
	USB->CNTR = CNTR_FRES;
	// CNTR_FRES = 0
	USB->CNTR = 0;
	// Clear pending interrupts
	USB->ISTR = 0;
	// Set interrupt mask
	USB->CNTR = CNTR_RESETM | CNTR_SUSPM | CNTR_WKUPM;

	return USB_SUCCESS;
}

// Handle Switch-Off conditions
// return: USB_SUCCESS
RESULT PowerOff() {
	// disable all interrupts and force USB reset
	USB->CNTR = CNTR_FRES;
	// clear interrupt status register
	USB->ISTR = 0;
    // Disconnect internal pull-up on USB DP line
    SYSCFG->PMC &= (uint32_t)(~SYSCFG_PMC_USB_PU);
	// switch-off device
	USB->CNTR = CNTR_FRES | CNTR_PDWN;
	// sw variables reset
	// ...

	return USB_SUCCESS;
}

// Handle suspend conditions
// return USB_SUCCESS
void Suspend(void) {
	uint32_t i;
	__IO uint32_t savePWR_CR;

	// suspend preparation
	// ...
	
	// This a sequence to apply a force RESET to handle a robustness case
    
	// Store endpoints registers status
	for (i = 0; i < 8; i++) EP[i] = _GetENDPOINT(i);
	
	// unmask RESET flag and apply FRES
	USB->CNTR |= CNTR_RESETM | CNTR_FRES;
	// clear FRES
	USB->CNTR &= ~CNTR_FRES;
	// poll for RESET flag in ISTR
	while (!(USB->ISTR & ISTR_RESET));
	// clear RESET flag in ISTR
	USB->ISTR = CLR_RESET;
	// restore enpoints
	for (i=0; i < 8; i++) _SetENDPOINT(i,EP[i]);
	// Now it is safe to enter macrocell in suspend mode
	USB->CNTR |= CNTR_FSUSP;
	// force low-power mode in the macrocell
	USB->CNTR |= CNTR_LPMODE;
	
	// prepare entry in low power mode (STOP mode)
	// Select the regulator state in STOP mode
	savePWR_CR = PWR->CR;
	PWR->CR &= (uint32_t)0xFFFFFFFC; // Clear PDDS and LPDS bits
	PWR->CR |= PWR_CR_LPSDSR; // Set LPDS bit according to PWR_Regulator value

	// Set SLEEPDEEP bit of Cortex-M System Control Register
	SCB->SCR |= SCB_SCR_SLEEPDEEP;
	
	// enter system in STOP mode, only when wake-up flag in not set
	if (!(USB->ISTR & ISTR_WKUP)) {
		__WFI();
		// Reset SLEEPDEEP bit of Cortex-M System Control Register
		SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP);
	} else {
		// Clear Wakeup flag
		USB->ISTR = CLR_WKUP;
		// clear FSUSP to abort entry in suspend mode
		USB->CNTR &= ~CNTR_FSUSP;
		// restore sleep mode configuration
		// restore Power regulator config in sleep mode
		PWR->CR = savePWR_CR;
		// Reset SLEEPDEEP bit of Cortex-M System Control Register
		SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP);
	}
}

// Restore normal operations after wake-up
void Resume_Init(void) {
	// ------------------ ONLY WITH BUS-POWERED DEVICES ----------------------
	// restart the clocks
	// ...

	// CNTR_LPMODE = 0
	USB->CNTR &= ~CNTR_LPMODE;
	// restore full power ... on connected devices
	Leave_LowPowerMode();
	// reset FSUSP bit
	USB->CNTR = IMR_MSK;

  	// reverse suspend preparation
	// ...
}

// This is the state machine handling resume operations and
// timing sequence. The control is based on the Resume structure
// variables and on the ESOF interrupt calling this subroutine
// without changing machine state.
// input:
// eResumeSetVal: a state machine value (RESUME_STATE)
void Resume(RESUME_STATE eResumeSetVal) {
	if (eResumeSetVal != RESUME_ESOF) ResumeS.eState = eResumeSetVal;
	switch (ResumeS.eState) {
    case RESUME_EXTERNAL:
    	if (remotewakeupon == 0) {
    		Resume_Init();
    		ResumeS.eState = RESUME_OFF;
    	} else {
    	  	// RESUME detected during the RemoteWAkeup signalling => keep RemoteWakeup handling
    		ResumeS.eState = RESUME_ON;
    	}
    	break;
    case RESUME_INTERNAL:
    	Resume_Init();
    	ResumeS.eState = RESUME_START;
    	remotewakeupon = 1;
    	break;
    case RESUME_LATER:
    	ResumeS.bESOFcnt = 2;
    	ResumeS.eState = RESUME_WAIT;
    	break;
    case RESUME_WAIT:
    	ResumeS.bESOFcnt--;
//    	if (ResumeS.bESOFcnt == 0) ResumeS.eState = RESUME_START;
    	rCounter++;
    	if ((ResumeS.bESOFcnt == 0) && (rCounter > 500)) {
    		rCounter = 0;
    		ResumeS.eState = RESUME_START;
        	// USB cable disconnected:
    		PowerOff();
    		RCC->APB1ENR &= ~RCC_APB1ENR_USBEN;
    		NVIC_DisableIRQ(USB_LP_IRQn);
    		NVIC_DisableIRQ(USB_FS_WKUP_IRQn);
    		bDeviceState = SUSPENDED;
    	}
		break;
    case RESUME_START:
    	USB->CNTR |= CNTR_RESUME;
    	ResumeS.eState = RESUME_ON;
    	ResumeS.bESOFcnt = 10;
    	break;
    case RESUME_ON:    
    	ResumeS.bESOFcnt--;
    	if (ResumeS.bESOFcnt == 0) {
    		USB->CNTR &= ~CNTR_RESUME;
    		ResumeS.eState = RESUME_OFF;
    		remotewakeupon = 0;
    	}
    	break;
    case RESUME_OFF:
    case RESUME_ESOF:
    default:
    	ResumeS.eState = RESUME_OFF;
    	break;
	}
}
