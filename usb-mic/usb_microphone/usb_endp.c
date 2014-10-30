/**
  ******************************************************************************
    @file    usb_endp.c
    @author  MCD Application Team
    @version V4.0.0
    @date    21-January-2013
    @brief   Endpoint routines
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


#include "usb_lib.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "usb_istr.h"
#include "usb_pwr.h"


uint8_t  packet_send;


/*******************************************************************************
    Function Name  : EP1_IN_Callback (from device to host)
    Description    : Endpoint 1 IN callback routine.
    Input          : None.
    Output         : None.
    Return         : None.
*******************************************************************************/
void EP1_IN_Callback(void) {
	packet_send = 1;
}
