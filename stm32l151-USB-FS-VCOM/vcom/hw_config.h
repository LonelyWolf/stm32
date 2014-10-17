/**
  ******************************************************************************
  * @file    hw_config.h
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


// Define to prevent recursive inclusion -------------------------------------
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H


#include "stm32l1xx.h"
#include <stm32l1xx_rcc.h>
#include <stm32l1xx_gpio.h>
#include <stm32l1xx_pwr.h>
#include <stm32l1xx_syscfg.h>
#include <misc.h>


// Exported defines
#define USB_TX_BUF_SIZE       2048        // Size of USB data buffer (to send data to USB)
#define USB_EXTI_LINE         1 << 18     // USB EXTI line


// External variables
extern uint8_t  USB_TX_Buf[USB_TX_BUF_SIZE]; // Data buffer for USB send
extern uint32_t USB_TX_Len; // Count of bytes in USB buffer


// Functions prototypes
void USB_LP_IRQHandler(void);
void USB_FS_WKUP_IRQHandler(void);
void USB_HWConfig(void);
void Enter_LowPowerMode(void);
void Leave_LowPowerMode(void);
void USB_Cable_Config(FunctionalState NewState);
void Handle_USBAsynchXfer (void);
void Get_SerialNum(void);
void VCP_SendChar(uint8_t data);

#endif  // __HW_CONFIG_H
