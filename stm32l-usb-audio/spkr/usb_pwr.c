/**
  ******************************************************************************
  * @file    usb_pwr.c
  * @author  MCD Application Team
  * @version V4.0.0
  * @date    21-January-2013
  * @brief   Connection/disconnection & power management
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
#include "usb_conf.h"
#include "usb_pwr.h"
#include "hw_config.h"


// Private variables
__IO uint32_t bDeviceState     = UNCONNECTED; // USB device status
__IO bool fSuspendEnabled      = TRUE;        // true when suspend is possible
__IO uint32_t remotewakeupon   = 0;           // Remote wake-up state machine
__IO uint32_t EP[8];                          // Array to store endpoint registers


// Private structures
struct {
	__IO RESUME_STATE eState;
	__IO uint8_t      bESOFcnt;
} ResumeS;


/*******************************************************************************
* Function Name  : PowerOn
* Description    :
* Input          : None.
* Output         : None.
* Return         : USB_SUCCESS.
*******************************************************************************/
RESULT PowerOn(void) {
	// cable plugged-in ?
	SYSCFG_USBPuCmd(ENABLE);
	// CNTR_PWDN = 0
	*CNTR = CNTR_FRES;
	// CNTR_FRES = 0
	*CNTR = 0;
	// Clear pending interrupts
	*ISTR = 0;
	// Set interrupt mask
	*CNTR = CNTR_RESETM | CNTR_SUSPM | CNTR_WKUPM;

	return USB_SUCCESS;
}

/*******************************************************************************
* Function Name  : PowerOff
* Description    : handles switch-off conditions
* Input          : None.
* Output         : None.
* Return         : USB_SUCCESS.
*******************************************************************************/
RESULT PowerOff() {
	// disable all interrupts and force USB reset
	*CNTR = CNTR_FRES;
	// clear interrupt status register
	*ISTR = 0;
	// Disable the Pull-Up
	SYSCFG_USBPuCmd(DISABLE);
	// switch-off device
	*CNTR = CNTR_FRES | CNTR_PDWN;
	// sw variables reset
	// ...

	return USB_SUCCESS;
}

/*******************************************************************************
* Function Name  : Suspend
* Description    : sets suspend mode operating conditions
* Input          : None.
* Output         : None.
* Return         : USB_SUCCESS.
*******************************************************************************/
void Suspend(void) {
	uint32_t i;
	__IO uint32_t savePWR_CR;

	// suspend preparation
	// ...
	
	// This a sequence to apply a force RESET to handle a robustness case
    
	// Store endpoints registers status
	for (i = 0; i < 8; i++) EP[i] = _GetENDPOINT(i);
	
	// unmask RESET flag and apply FRES
	*CNTR |= CNTR_RESETM | CNTR_FRES;
	// clear FRES
	*CNTR &= ~CNTR_FRES;
	// poll for RESET flag in ISTR
	while (!(*ISTR & ISTR_RESET));
	// clear RESET flag in ISTR
	*ISTR = CLR_RESET;
	// restore enpoints
	for (i=0; i < 8; i++) _SetENDPOINT(i,EP[i]);
	// Now it is safe to enter macrocell in suspend mode
	*CNTR |= CNTR_FSUSP;
	// force low-power mode in the macrocell
	*CNTR |= CNTR_LPMODE;
	
	// prepare entry in low power mode (STOP mode)
	// Select the regulator state in STOP mode
	savePWR_CR = PWR->CR;
	PWR->CR &= (uint32_t)0xFFFFFFFC; // Clear PDDS and LPDS bits
	PWR->CR |= PWR_Regulator_LowPower; // Set LPDS bit according to PWR_Regulator value

	// Set SLEEPDEEP bit of Cortex-M System Control Register
	SCB->SCR |= SCB_SCR_SLEEPDEEP;
	
	// enter system in STOP mode, only when wake-up flag in not set
	if (!(*ISTR & ISTR_WKUP)) {
		__WFI();
		// Reset SLEEPDEEP bit of Cortex-M System Control Register
		SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP);
	} else {
		// Clear Wakeup flag
		*ISTR = CLR_WKUP;
		// clear FSUSP to abort entry in suspend mode
		*CNTR &= ~CNTR_FSUSP;
		// restore sleep mode configuration
		// restore Power regulator config in sleep mode
		PWR->CR = savePWR_CR;
		// Reset SLEEPDEEP bit of Cortex-M System Control Register
		SCB->SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP);
	}
}

/*******************************************************************************
* Function Name  : Resume_Init
* Description    : Handles wake-up restoring normal operations
* Input          : None.
* Output         : None.
* Return         : USB_SUCCESS.
*******************************************************************************/
void Resume_Init(void) {
	// ------------------ ONLY WITH BUS-POWERED DEVICES ----------------------
	// restart the clocks
	// ...

	// CNTR_LPMODE = 0
	*CNTR &= ~CNTR_LPMODE;
	// restore full power ... on connected devices
	Leave_LowPowerMode();
	// reset FSUSP bit
	*CNTR = IMR_MSK;

  	// reverse suspend preparation
	// ...
}

/*******************************************************************************
* Function Name  : Resume
* Description    : This is the state machine handling resume operations and
*                 timing sequence. The control is based on the Resume structure
*                 variables and on the ESOF interrupt calling this subroutine
*                 without changing machine state.
* Input          : a state machine value (RESUME_STATE)
*                  RESUME_ESOF doesn't change ResumeS.eState allowing
*                  decrementing of the ESOF counter in different states.
* Output         : None.
* Return         : None.
*******************************************************************************/
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
    	if (ResumeS.bESOFcnt == 0) ResumeS.eState = RESUME_START;
    	break;
    case RESUME_START:
    	*CNTR |= CNTR_RESUME;
    	ResumeS.eState = RESUME_ON;
    	ResumeS.bESOFcnt = 10;
    	break;
    case RESUME_ON:    
    	ResumeS.bESOFcnt--;
    	if (ResumeS.bESOFcnt == 0) {
    		*CNTR &= ~CNTR_RESUME;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
