#include "usb_ll.h"


USB_HandleTypeDef husb; // USB device handle


// Initialize the low level USB device
// input:
//   husb - pointer to the USB device handle
void HAL_USB_Init(USB_HandleTypeDef *husb) {
	// USB state
	husb->State = USB_BUSY;

	// ---- Initialize the low level hardware: GPIO, CLOCK, NVIC...
	// For the STM32L products there is no need to configure the PA12/PA11 (USB DM/DP) pins couple as Alternate Function

	// Enable the USB peripheral clock
	RCC->APB1ENR |= RCC_APB1ENR_USBEN;

	// Enable the SYSCFG peripheral clock
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

	// Set USB interrupt priority
	NVIC_SetPriority(USB_LP_IRQn,7);

	// Enable USB interrupt
	NVIC_EnableIRQ(USB_LP_IRQn);

	// ---- End of low level hardware initialization
}

// Stop the low level USB device
// input:
//   husb - pointer to the USB device handle
void HAL_USB_DeInit(USB_HandleTypeDef *husb) {
	// disable all interrupts and force USB reset
	husb->Instance->CNTR = USB_CNTR_FRES;

	// clear interrupt status register
	husb->Instance->ISTR = 0;

	// switch-off the USB device
	husb->Instance->CNTR = (USB_CNTR_FRES | USB_CNTR_PDWN);

	// Disable the USB peripheral clock
	RCC->APB1ENR &= RCC_APB1ENR_USBEN;

	// Disable the USB interrupt
	NVIC_DisableIRQ(USB_LP_IRQn);

	// Change state
	husb->State = USB_READY;
}

// Stop the USB device
// input:
//   husb - pointer to the USB device handle
void HAL_USB_Stop(USB_HandleTypeDef *husb) {
	// Disable all interrupts and force the USB reset
	husb->Instance->CNTR = USB_CNTR_FRES;

	// Clear the interrupt status register
	husb->Instance->ISTR = 0;

	// Switch-off the USB device
	husb->Instance->CNTR = USB_CNTR_FRES | USB_CNTR_PDWN;
}

// Set the USB device address
// input:
//   husb - pointer to the USB device handle
//   address - new device address
void HAL_USB_SetAddress(USB_HandleTypeDef *husb, uint8_t address) {
	if (address == 0) {
		// Set device address and enable function
		husb->Instance->DADDR = USB_DADDR_EF;
	} else {
		// USB Address will be applied later
		husb->USB_Address = address;
	}
}

// Copy a data from the user memory area to the USB packet memory area (PMA)
//   USBx - pointer to the USB peripheral handle
//   pbUsrBuf - pointer to the user buffer
//   wPMABufAddr - address in PMA
//   wNBytes - number of bytes to be copied
static void USB_WritePMA(USB_TypeDef *USBx, uint8_t *pbUsrBuf, uint16_t wPMABufAddr, uint16_t wNBytes) {
	uint16_t *pdwVal = (uint16_t *)((wPMABufAddr << 1) + (uint32_t)USBx + 0x400);
	uint32_t i,temp1,temp2;

	for (i = (wNBytes + 1) >> 1; i--; ) {
		temp1 = (uint16_t) * pbUsrBuf;
		pbUsrBuf++;
		temp2 = temp1 | (uint16_t) * pbUsrBuf << 8;
		*pdwVal++ = temp2;
		pdwVal++;
		pbUsrBuf++;
	}
}

// Copy a data from the USB packet memory area to the user memory area
// input:
//   USBx - pointer to the USB peripheral handle
//   pbUsrBuf - pointer to the user buffer
//   wPMABufAddr - address in PMA
//   wNBytes - number of bytes to be copied
static void USB_ReadPMA(USB_TypeDef *USBx, uint8_t *pbUsrBuf, uint16_t wPMABufAddr, uint16_t wNBytes) {
	uint32_t *pdwVal = (uint32_t *)((wPMABufAddr << 1) + (uint32_t)USBx + 0x400);
	uint32_t i;

	for (i = (wNBytes + 1) >> 1; i--; ) {
		*(uint16_t*)pbUsrBuf++ = *pdwVal++;
		pbUsrBuf++;
	}
}

// Receive an amount of data
// input:
//   husb - pointer to the USB device handle
//   ep_addr - endpoint address
//   pBuf - pointer to the reception buffer
//   len - amount of data to be received
void HAL_USB_EP_Receive(USB_HandleTypeDef *husb, uint8_t ep_addr, uint8_t *pBuf, uint32_t len) {
	USB_EPTypeDef *ep = &husb->OUT_ep[ep_addr & 0x7F];

	// Setup and start the reception
	ep->xfer_buff  = pBuf;
	ep->xfer_len   = len;
	ep->xfer_count = 0;
	ep->is_in      = 0;
	ep->num        = ep_addr & 0x7F;

	// Multiple packet transfer
	if (ep->xfer_len > ep->maxpacket) {
		len = ep->maxpacket;
		ep->xfer_len -= len;
	} else  {
		len = ep->xfer_len;
		ep->xfer_len = 0;
	}

	// Configure and validate the RX endpoint
	if (ep->doublebuffer) {
		// Set the Double buffer counter
		USB_SET_EP_DBUF1_CNT(husb->Instance,ep->num,ep->is_in,len);
	} else {
		// Set the RX buffer count
		USB_SET_EP_RX_CNT(husb->Instance,ep->num,len);
	}

	// Validate transfer
	USB_SET_EP_RX_STATUS(husb->Instance,ep->num,USB_EP_RX_VALID);
}

// Transmit an amount of data
// input:
//   husb - pointer to the USB device handle
//   ep_addr - endpoint address
//   pBuf - pointer to the transmission buffer
//   len - amount of data to be received
void HAL_USB_EP_Transmit(USB_HandleTypeDef *husb, uint8_t ep_addr, uint8_t *pBuf, uint32_t len) {
	USB_EPTypeDef *ep = &husb->IN_ep[ep_addr & 0x7F];
	uint16_t pmabuffer = 0;

	// Setup and start the transmission
	ep->xfer_buff  = pBuf;
	ep->xfer_len   = len;
	ep->xfer_count = 0;
	ep->is_in      = 1;
	ep->num        = ep_addr & 0x7F;

	// Multiple packet transfer
	if (ep->xfer_len > ep->maxpacket) {
		len = ep->maxpacket;
		ep->xfer_len -= len;
	} else {
		len = ep->xfer_len;
		ep->xfer_len = 0;
	}

	// Configure and validate the TX endpoint
	if (ep->doublebuffer) {
		// Set the Double buffer counter
		USB_SET_EP_DBUF1_CNT(husb->Instance,ep->num,ep->is_in,len);
		// Write the data to the USB endpoint
		pmabuffer = (USB_GET_ENDPOINT(husb->Instance,ep->num) & USB_EP_DTOG_TX) ? ep->pmaaddr1 : ep->pmaaddr0;
		USB_WritePMA(husb->Instance,ep->xfer_buff,pmabuffer,len);
		USB_FreeUserBuffer(husb->Instance,ep->num,ep->is_in);
	} else {
		// Single buffer
		USB_WritePMA(husb->Instance,ep->xfer_buff,ep->pmaadress,len);
		USB_SET_EP_TX_CNT(husb->Instance,ep->num, len);
	}

	// Validate transfer
	USB_SET_EP_TX_STATUS(husb->Instance,ep->num,USB_EP_TX_VALID);
}

// Set a STALL condition for an endpoint
// input:
//   husb - pointer to the USB device handle
//   ep_addr - endpoint address
void HAL_USB_EP_SetStall(USB_HandleTypeDef *husb, uint8_t ep_addr) {
	USB_EPTypeDef *ep;

	ep = ((0x80 & ep_addr) == 0x80) ? &husb->IN_ep[ep_addr & 0x7F] : &husb->OUT_ep[ep_addr];
	ep->is_stall = 1;
	ep->num      = ep_addr & 0x7F;
	ep->is_in    = ((ep_addr & 0x80) == 0x80);

	if (ep->num) {
		if (ep->is_in) {
			// Set STALL status for TX
			USB_SET_EP_TX_STATUS(husb->Instance,ep->num,USB_EP_TX_STALL);
		} else {
			// Set STALL status for RX
			USB_SET_EP_RX_STATUS(husb->Instance,ep->num,USB_EP_RX_STALL);
		}
	} else {
		// Set STALL status for both RX and TX
		USB_SET_EP_TXRX_STATUS(husb->Instance,ep->num,USB_EP_RX_STALL,USB_EP_TX_STALL);
	}
}

// Open and configure an endpoint
// input:
//   husb - pointer to the USB device handle
//   ep_addr - endpoint address
//   ep_mps - endpoint maximum packet size
//   ep_type - endpoint type
void HAL_USB_EP_Open(USB_HandleTypeDef *husb, uint8_t ep_addr, uint16_t ep_mps, uint8_t ep_type) {
	USB_EPTypeDef *ep;

	ep = ((0x80 & ep_addr) == 0x80) ? &husb->IN_ep[ep_addr & 0x7F] : &husb->OUT_ep[ep_addr & 0x7F];
	ep->num       = ep_addr & 0x7F;
	ep->is_in     = ((0x80 & ep_addr) != 0);
	ep->maxpacket = ep_mps;
	ep->type      = ep_type;

	// Set type of endpoint
	switch (ep->type) {
		case USB_EP_TYPE_CTRL:
			USB_SET_EPTYPE(husb->Instance,ep->num,USB_EP_CONTROL);
			break;
		case USB_EP_TYPE_BULK:
			USB_SET_EPTYPE(husb->Instance,ep->num,USB_EP_BULK);
			break;
		case USB_EP_TYPE_INTR:
			USB_SET_EPTYPE(husb->Instance, ep->num, USB_EP_INTERRUPT);
			break;
		case USB_EP_TYPE_ISOC:
			USB_SET_EPTYPE(husb->Instance, ep->num, USB_EP_ISOCHRONOUS);
			break;
		default:
			break;
	}

	// Set endpoint address
	USB_SET_EP_ADDRESS(husb->Instance,ep->num,ep->num);

	if (ep->doublebuffer) {
		// Double buffered
		USB_SET_EP_DBUF(husb->Instance,ep->num);
		// Set buffer address for double buffered mode
		USB_SET_EP_DBUF_ADDR(husb->Instance,ep->num,ep->pmaaddr0,ep->pmaaddr1);
		if (ep->is_in==0) {
			// Clear the data toggle bits for both IN and OUT endpoints
			USB_CLEAR_RX_DTOG(husb->Instance,ep->num);
			USB_CLEAR_TX_DTOG(husb->Instance,ep->num);
			// Reset value of the data toggle bits for the endpoint out
			USB_TX_DTOG(husb->Instance, ep->num);
			// Set VALID status for the RX endpoint
			USB_SET_EP_RX_STATUS(husb->Instance,ep->num,USB_EP_RX_VALID);
			// Set DISABLED status for the TX endpoint
			USB_SET_EP_TX_STATUS(husb->Instance,ep->num,USB_EP_TX_DIS);
		} else {
			// Clear the data toggle bits for the both IN and OUT endpoints
			USB_CLEAR_RX_DTOG(husb->Instance,ep->num);
			USB_CLEAR_TX_DTOG(husb->Instance,ep->num);
			USB_RX_DTOG(husb->Instance,ep->num);
			// Configure DISABLE status for both IN and OUT endpoints
			USB_SET_EP_TX_STATUS(husb->Instance,ep->num,USB_EP_TX_DIS);
			USB_SET_EP_RX_STATUS(husb->Instance,ep->num,USB_EP_RX_DIS);
		}
	} else {
		// Single buffered
		if (ep->is_in) {
			// Set the endpoint TX buffer address
			USB_SET_EP_TX_ADDRESS(husb->Instance, ep->num, ep->pmaadress);
			USB_CLEAR_TX_DTOG(husb->Instance, ep->num);
			// Configure NAK status for the endpoint
			USB_SET_EP_TX_STATUS(husb->Instance, ep->num, USB_EP_TX_NAK);
		} else {
			// Set the endpoint RX buffer address
			USB_SET_EP_RX_ADDRESS(husb->Instance,ep->num,ep->pmaadress);
			// Set the endpoint TX buffer counter
			USB_SET_EP_RX_CNT(husb->Instance,ep->num,ep->maxpacket);
			USB_CLEAR_RX_DTOG(husb->Instance,ep->num);
			// Configure VALID status for the endpoint
			USB_SET_EP_RX_STATUS(husb->Instance,ep->num,USB_EP_RX_VALID);
		}
	}
}

// Deactivate an endpoint
// input:
//   husb - pointer to the USB device handle
//   ep_addr - endpoint address
void HAL_USB_EP_Close(USB_HandleTypeDef *husb, uint8_t ep_addr) {
	USB_EPTypeDef *ep;

	ep = ((ep_addr & 0x80) == 0x80) ? &husb->IN_ep[ep_addr & 0x7F] : &husb->OUT_ep[ep_addr & 0x7F];
	ep->num   = ep_addr & 0x7F;
	ep->is_in = ((0x80 & ep_addr) != 0);

	if (ep->doublebuffer == 0) {
		// Double buffer
		if (ep->is_in==0) {
			// Clear the data toggle bits for the both IN and OUT endpoints
			USB_CLEAR_RX_DTOG(husb->Instance,ep->num);
			USB_CLEAR_TX_DTOG(husb->Instance,ep->num);
			// Reset value of the data toggle bits for the endpoint OUT
			USB_TX_DTOG(husb->Instance,ep->num);
			USB_SET_EP_RX_STATUS(husb->Instance,ep->num,USB_EP_RX_DIS);
			USB_SET_EP_TX_STATUS(husb->Instance,ep->num,USB_EP_TX_DIS);
		} else {
			// Clear the data toggle bits for the both IN and OUT endpoints
			USB_CLEAR_RX_DTOG(husb->Instance,ep->num);
			USB_CLEAR_TX_DTOG(husb->Instance,ep->num);
			USB_RX_DTOG(husb->Instance,ep->num);
			// Configure DISABLE status for the both IN and OUT endpoints
			USB_SET_EP_TX_STATUS(husb->Instance,ep->num,USB_EP_TX_DIS);
			USB_SET_EP_RX_STATUS(husb->Instance,ep->num,USB_EP_RX_DIS);
		}
	} else {
		// Single buffer
		if (ep->is_in) {
			// Clear TX DTOG bit and configure DISABLE status for the endpoint
			USB_CLEAR_TX_DTOG(husb->Instance,ep->num);
			USB_SET_EP_TX_STATUS(husb->Instance,ep->num,USB_EP_TX_DIS);
		} else {
			// Clear RX DTOG bit and configure DISABLE status for the endpoint
			USB_CLEAR_RX_DTOG(husb->Instance,ep->num);
			USB_SET_EP_RX_STATUS(husb->Instance,ep->num,USB_EP_RX_DIS);
		}
	}
}

// Clear a STALL condition for an endpoint
// input:
//   husb - pointer to the USB device handle
//   ep_addr - endpoint address
void HAL_USB_EP_ClrStall(USB_HandleTypeDef *husb, uint8_t ep_addr) {
	USB_EPTypeDef *ep;

	ep = ((ep_addr & 0x80) == 0x80) ? &husb->IN_ep[ep_addr & 0x7F] : &husb->OUT_ep[ep_addr];
	ep->is_stall = 0;
	ep->num      = ep_addr & 0x7F;
	ep->is_in    = ((ep_addr & 0x80) == 0x80);

	if (ep->is_in) {
		USB_CLEAR_TX_DTOG(husb->Instance,ep->num);
		USB_SET_EP_TX_STATUS(husb->Instance,ep->num,USB_EP_TX_VALID);
	} else {
		USB_CLEAR_RX_DTOG(husb->Instance,ep->num);
		USB_SET_EP_RX_STATUS(husb->Instance,ep->num,USB_EP_RX_VALID);
	}
}

// Flush an endpoint
// input:
//   husb - pointer to the USB device handle
//   ep_addr - endpoint address
void HAL_USB_EP_Flush(USB_HandleTypeDef *husb, uint8_t ep_addr) {
	// Does nothing?
}

// Connect/Disconnect the USB device
// input:
//   husb - pointer to the USB device handle
//   state - new state of the connection (0 = disconnected, any number = connected)
void HAL_USB_SetConnectionState(USB_HandleTypeDef *husb, uint8_t state) {
	if (state) {
		// Connect internal pull-up on USB DP line
		SYSCFG->PMC |= (uint32_t)SYSCFG_PMC_USB_PU;
	} else {
		// Disconnect internal pull-up on USB DP line
		SYSCFG->PMC &= (uint32_t)(~SYSCFG_PMC_USB_PU);
	}
}

// Configure PMA for endpoint
// input:
//   husb - pointer to the USB device handle
//   ep_addr - endpoint address
//   ep_kind - endpoint Kind
//               USB_SNG_BUF: Single Buffer used
//               USB_DBL_BUF: Double Buffer used
//   pmaadress - EP address in The PMA:
//        In case of single buffer endpoint this parameter is 16-bit value providing the address in PMA allocated to endpoint
//        In case of double buffer endpoint this parameter is a 32-bit value providing the endpoint buffer 0 address
//        in the LSB part of 32-bit value and endpoint buffer 1 address in the MSB part of 32-bit value
void HAL_USB_PMAConfig(USB_HandleTypeDef *husb, uint16_t ep_addr, uint16_t ep_kind, uint32_t pmaadress) {
	USB_EPTypeDef *ep;

	ep = ((ep_addr & 0x80) == 0x80) ? &husb->IN_ep[ep_addr & 0x7F] : &husb->OUT_ep[ep_addr];

	// Check if the endpoint is single or double Buffer
	if (ep_kind == USB_SNG_BUF) {
		// Single Buffer
		ep->doublebuffer = 0;
		// Configure the PMA
		ep->pmaadress = (uint16_t)pmaadress;
	} else {
		// Double Buffer Endpoint
		ep->doublebuffer = 1;
		// Configure the PMA
		ep->pmaaddr0 =  pmaadress & 0xFFFF;
		ep->pmaaddr1 = (pmaadress & 0xFFFF0000) >> 16;
	}
}

// Get received data size
// input:
//   husb - pointer to the USB device handle
//   ep_addr - endpoint address
// return: received data size
inline uint16_t HAL_USB_EP_GetRxCount(USB_HandleTypeDef *husb, uint8_t ep_addr) {
	return husb->OUT_ep[ep_addr & 0x7F].xfer_count;
}

// Handle USB Endpoint interrupt request
// input:
//   husb - pointer to the USB handle
static void USB_EP_ISR_Handler(USB_HandleTypeDef *husb) {
	USB_EPTypeDef *ep;
	uint16_t count = 0;
	uint8_t EPindex;
	__IO uint16_t wIstr;
	__IO uint16_t wEPVal = 0;

	// Stay in loop while pending interrupts present
	while ((wIstr = husb->Instance->ISTR) & USB_ISTR_CTR) {
		// Extract highest priority endpoint number
		EPindex = (uint8_t)(wIstr & USB_ISTR_EP_ID);

		if (EPindex == 0) {
			// Process the control endpoint

			// The DIR bit indicates direction of transaction
			if (wIstr & USB_ISTR_DIR) {
				// The DIR bit is set (OUT transaction: from host to the USB peripheral)
				// CTR_RX bit set indicates SETUP or OUT interrupt
				// CTR_TX or CTR_RX bits set indicates 2 interrupts pending
				ep = &husb->OUT_ep[0];
				wEPVal = USB_GET_ENDPOINT(husb->Instance,USB_ENDP0);
				if (wEPVal & USB_EP_SETUP) {
					// Get SETUP packet
					ep->xfer_count = USB_GET_EP_RX_CNT(husb->Instance,ep->num);
					USB_ReadPMA(husb->Instance,(uint8_t*)husb->Setup,ep->pmaadress,ep->xfer_count);
					// SETUP bit kept frozen while CTR_RX = 1
					USB_CLEAR_RX_EP_CTR(husb->Instance,USB_ENDP0);
					// Process SETUP packet
					HAL_USB_SetupStageCallback(husb);
				} else if (wEPVal & USB_EP_CTR_RX) {
					USB_CLEAR_RX_EP_CTR(husb->Instance,USB_ENDP0);
					// Get control data OUT packet
					ep->xfer_count = USB_GET_EP_RX_CNT(husb->Instance,ep->num);
					if (ep->xfer_count) {
						USB_ReadPMA(husb->Instance,ep->xfer_buff,ep->pmaadress,ep->xfer_count);
						ep->xfer_buff += ep->xfer_count;
					}
					// Process control data OUT packet
					HAL_USB_DataOutStageCallback(husb,0);
					USB_SET_EP_RX_CNT(husb->Instance,USB_ENDP0,ep->maxpacket);
					USB_SET_EP_RX_STATUS(husb->Instance,USB_ENDP0,USB_EP_RX_VALID);
				}
			} else {
				// The DIR bit is cleared (IN transaction: from USB peripheral to the host)
				// DIR = 0 implies that (EP_CTR_TX = 1) always
				USB_CLEAR_TX_EP_CTR(husb->Instance,USB_ENDP0);
				ep = &husb->IN_ep[0];
				ep->xfer_count = USB_GET_EP_TX_CNT(husb->Instance,ep->num);
				ep->xfer_buff += ep->xfer_count;
				// TX complete
				HAL_USB_DataInStageCallback(husb,0);
				if ((husb->USB_Address > 0) && (ep->xfer_len == 0)) {
					husb->Instance->DADDR = (husb->USB_Address | USB_DADDR_EF);
					husb->USB_Address = 0;
				}
			}
		} else {
			// A non control endpoint interrupt

			// Process related endpoint register
			wEPVal = USB_GET_ENDPOINT(husb->Instance,EPindex);
			if (wEPVal & USB_EP_CTR_RX) {
				// Clear the interrupt flag
				USB_CLEAR_RX_EP_CTR(husb->Instance,EPindex);
				ep = &husb->OUT_ep[EPindex];

				// OUT
				if (ep->doublebuffer) {
					// Double buffer
		        	if (USB_GET_ENDPOINT(husb->Instance,ep->num) & USB_EP_DTOG_RX) {
		        		// Read from endpoint BUF0Addr buffer
		        		count = USB_GET_EP_DBUF0_CNT(husb->Instance,ep->num);
		        		if (count) USB_ReadPMA(husb->Instance,ep->xfer_buff,ep->pmaaddr0,count);
		        	} else {
		        		// Read from endpoint BUF1Addr buffer
		        		count = USB_GET_EP_DBUF1_CNT(husb->Instance,ep->num);
		        		if (count) USB_ReadPMA(husb->Instance,ep->xfer_buff,ep->pmaaddr1,count);
		        	}
		        	USB_FreeUserBuffer(husb->Instance,ep->num,USB_EP_DBUF_OUT);
		        } else {
		        	// Single buffer
					count = USB_GET_EP_RX_CNT(husb->Instance,ep->num);
					if (count) USB_ReadPMA(husb->Instance,ep->xfer_buff,ep->pmaadress,count);
		        }

				// Multiple packet on the NON control OUT endpoint
				ep->xfer_count += count;
				ep->xfer_buff  += count;

				if ((ep->xfer_len == 0) || (count < ep->maxpacket)) {
					// RX complete
					HAL_USB_DataOutStageCallback(husb,ep->num);
				} else {
					HAL_USB_EP_Receive(husb, ep->num, ep->xfer_buff, ep->xfer_len);
				}
			} // if ((wEPVal & EP_CTR_RX)

			if (wEPVal & USB_EP_CTR_TX) {
				ep = &husb->IN_ep[EPindex];
				// Clear the intterrupt flag
				USB_CLEAR_TX_EP_CTR(husb->Instance,EPindex);

				// IN
				if (ep->doublebuffer) {
					// Double buffer
					if (USB_GET_ENDPOINT(husb->Instance,ep->num) & USB_EP_DTOG_TX) {
						// Read from endpoint BUF0Addr buffer
						ep->xfer_count = USB_GET_EP_DBUF0_CNT(husb->Instance,ep->num);
						if (ep->xfer_count) USB_WritePMA(husb->Instance,ep->xfer_buff,ep->pmaaddr0,ep->xfer_count);
					} else {
						// Read from endpoint BUF1Addr buffer
						ep->xfer_count = USB_GET_EP_DBUF1_CNT(husb->Instance,ep->num);
						if (ep->xfer_count) USB_WritePMA(husb->Instance,ep->xfer_buff,ep->pmaaddr1,ep->xfer_count);
					}
					USB_FreeUserBuffer(husb->Instance,ep->num,USB_EP_DBUF_IN);
				} else {
					// Single buffer
					ep->xfer_count = USB_GET_EP_TX_CNT(husb->Instance,ep->num);
					if (ep->xfer_count) USB_WritePMA(husb->Instance,ep->xfer_buff,ep->pmaadress,ep->xfer_count);
				}

				// Multiple packet on the NON control IN endpoint
				ep->xfer_count = USB_GET_EP_TX_CNT(husb->Instance,ep->num);
				ep->xfer_buff += ep->xfer_count;

				// Is this ZLP (zero length packet)?
				if (ep->xfer_len) {
					// Pending transfer
					HAL_USB_EP_Transmit(husb,ep->num,ep->xfer_buff,ep->xfer_len);
				} else {
					// ZLP --> TX complete
					HAL_USB_DataInStageCallback(husb,ep->num);
				}
			}
		}
	}
}

// Handle USB low priority interrupts
void USB_LP_IRQHandler(void) {
	uint32_t wInterrupt_Mask = 0;

	// USB correct transfer (endpoint has successfully completed a transaction)
	if (husb.Instance->ISTR & USB_ISTR_CTR) {
		// Servicing of the endpoint correct transfer interrupt clear of the CTR flag into the sub
	    USB_EP_ISR_Handler(&husb);
	}

	// USB peripheral has detected an active USB RESET signal
	if (husb.Instance->ISTR & USB_ISTR_RESET) {
		// Clear the RESET flag
		husb.Instance->ISTR &= ~USB_ISTR_RESET;
		// Call the USB reset procedure
	    HAL_USB_ResetCallback(&husb);
	    // Clear the USB address
	    HAL_USB_SetAddress(&husb,0);
	}

	// The MCU has not been able to respond in time to an USB memory request
	if (husb.Instance->ISTR & USB_ISTR_PMAOVR) {
		// Clear the overrun/underrun flag
		husb.Instance->ISTR &= ~USB_ISTR_PMAOVR;
	}

	// USB error
	//   NANS: no answer
	//   CRC: wrong CRC received
	//   BST: bit stuffing error detected in data
	//   FVIO: framing format violation (non-standard frame received)
	if (husb.Instance->ISTR & USB_ISTR_ERR) {
		// Usually it is safely to ignore this
		// Clear the ERR flag
		husb.Instance->ISTR &= ~USB_ISTR_ERR;
	}

	// In the suspend mode was detected an activity that wakes up the USB peripheral
	if (husb.Instance->ISTR & USB_ISTR_WKUP) {
		// Clear the LP_MODE flag (no low-power mode)
		husb.Instance->CNTR &= ~USB_CNTR_LP_MODE;
		// Set wInterrupt_Mask global variable
	    wInterrupt_Mask = USB_CNTR_CTRM | USB_CNTR_WKUPM | USB_CNTR_SUSPM | USB_CNTR_ERRM | USB_CNTR_ESOFM | USB_CNTR_RESETM;
	    // Set interrupt mask
	    husb.Instance->CNTR = wInterrupt_Mask;
	    // Call USB resume procedure
	    HAL_USB_ResumeCallback(&husb);
	    // Clear the WKUP flag
		husb.Instance->ISTR &= ~USB_ISTR_WKUP;
	}

	// No traffic has been received for 3ms, that indicates a suspend mode request from the USB bus
	if (husb.Instance->ISTR & USB_ISTR_SUSP) {
		// clear of the ISTR bit must be done after setting of CNTR_FSUSP
		husb.Instance->ISTR &= ~USB_ISTR_SUSP;

	    // Force low-power mode in the macrocell
	    husb.Instance->CNTR |= USB_CNTR_FSUSP;
	    husb.Instance->CNTR |= USB_CNTR_LP_MODE;

	    if (husb.Instance->ISTR & USB_ISTR_WKUP) {
		    // Call the suspend procedure
	    	HAL_USB_SuspendCallback(&husb);
	    }
	}

	// A SOF (start of frame) packet arrives through the USB bus (beginning a new USB frame)
	if (husb.Instance->ISTR & USB_ISTR_SOF) {
		// Clear the SOF flag
		husb.Instance->ISTR &= ~USB_ISTR_SOF;
		// Call SOF procedure
		HAL_USB_SOFCallback(&husb);
	}

	// An SOF packet is expected but not received
	if (husb.Instance->ISTR & USB_ISTR_ESOF) {
		// After a three consecutive ESOF interrupts without any traffic occurring in between,
		// a suspend interrupt will be generated. Therefore just clear flag.
		// Clear the ESOF flag
		husb.Instance->ISTR &= ~USB_ISTR_ESOF;
	}
}
