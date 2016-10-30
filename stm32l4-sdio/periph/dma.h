#ifndef __DMA_H
#define __DMA_H


#include <stm32l4xx.h>


// Definitions of DMA data transfer direction
#define DMA_DIR_P2M                ((uint32_t)0x00000000) // Peripheral to memory
#define DMA_DIR_M2P                DMA_CCR_DIR            // Memory to peripheral
#define DMA_DIR_M2M                DMA_CCR_MEM2MEM        // Memory to memory

// Definitions of DMA request
#define DMA_REQUEST_0              ((uint32_t)0x00000000U)
#define DMA_REQUEST_1              ((uint32_t)0x00000001U)
#define DMA_REQUEST_2              ((uint32_t)0x00000002U)
#define DMA_REQUEST_3              ((uint32_t)0x00000003U)
#define DMA_REQUEST_4              ((uint32_t)0x00000004U)
#define DMA_REQUEST_5              ((uint32_t)0x00000005U)
#define DMA_REQUEST_6              ((uint32_t)0x00000006U)
#define DMA_REQUEST_7              ((uint32_t)0x00000007U)

// Definitions of DMA channel index values
#define DMA_CHIDX_1                ((uint32_t)0x00000000U)
#define DMA_CHIDX_2                ((uint32_t)0x00000004U)
#define DMA_CHIDX_3                ((uint32_t)0x00000008U)
#define DMA_CHIDX_4                ((uint32_t)0x0000000CU)
#define DMA_CHIDX_5                ((uint32_t)0x00000010U)
#define DMA_CHIDX_6                ((uint32_t)0x00000014U)
#define DMA_CHIDX_7                ((uint32_t)0x00000018U)

// Definitions of DMA transfer priority levels
#define DMA_PRIORITY_LOW           ((uint32_t)0x00000000U) // Low
#define DMA_PRIORITY_MEDIUM        DMA_CCR_PL_0            // Medium
#define DMA_PRIORITY_HIGH          DMA_CCR_PL_1            // High
#define DMA_PRIORITY_VERYHIGH      DMA_CCR_PL              // Very high

// Definitions of DMA transfer mode
#define DMA_MODE_NORMAL            ((uint32_t)0x00000000U) // Normal mode
#define DMA_MODE_CIRCULAR          DMA_CCR_CIRC            // Circular mode

// Definitions of DMA channel flags
#define DMA_FLAG_GI                DMA_ISR_GIF1  // Global interrupt
#define DMA_FLAG_TC                DMA_ISR_TCIF1 // Transfer complete
#define DMA_FLAG_HT                DMA_ISR_HTIF1 // Half transfer
#define DMA_FLAG_TE                DMA_ISR_TEIF1 // Transfer error

// Definitions of clear DMA flag values
#define DMA_CF_GI                  DMA_IFCR_CGIF1  // Global interrupt
#define DMA_CF_TC                  DMA_IFCR_CTCIF1 // Transfer complete
#define DMA_CF_HT                  DMA_IFCR_CHTIF1 // Half transfer
#define DMA_CF_TE                  DMA_IFCR_CTEIF1 // Transfer error
#define DMA_CF_ALL                 (DMA_CF_GI | DMA_CF_TC | DMA_CF_HT | DMA_CF_TE)

// Definitions of DMA channel interrupts
#define DMA_IRQ_TC                 DMA_CCR_TCIE // Transfer complete
#define DMA_IRQ_HT                 DMA_CCR_HTIE // Halt transfer
#define DMA_IRQ_TE                 DMA_CCR_TEIE // Transfer error

// Definitions of DMA channel memory data alignment
#define DMA_MALIGN_8BIT            ((uint32_t)0x00000000U) // Byte
#define DMA_MALIGN_16BIT           DMA_CCR_MSIZE_0         // Half-word
#define DMA_MALIGN_32BIT           DMA_CCR_MSIZE_1         // Word

// Definitions of DMA channel peripheral data alignment
#define DMA_PALIGN_8BIT            ((uint32_t)0x00000000U) // Byte
#define DMA_PALIGN_16BIT           DMA_CCR_PSIZE_0         // Half-word
#define DMA_PALIGN_32BIT           DMA_CCR_PSIZE_1         // Word


// DMA states enumeration
typedef enum {
	DMA_STATE_RESET = 0x00, // DMA not initialized or disabled
	DMA_STATE_READY = 0x01, // DMA ready to use
	DMA_STATE_HT    = 0x02, // DMA half transfer flag
	DMA_STATE_TC    = 0x03, // DMA transfer complete flag
	DMA_STATE_BUSY  = 0x04, // DMA transaction is ongoing
	DMA_STATE_ERROR = 0x05  // DMA error
} DMA_State_TypeDef;

// DMA channel handle structure
typedef struct {
	DMA_TypeDef            *Instance; // DMA peripheral base address, one of DMAx values
	DMA_Channel_TypeDef    *Channel;  // Pointer to the DMA channel handler, one of DMAy_Channelx values
	uint32_t                ChIndex;  // DMA channel index, one of DMA_CHIDX_xx values
	uint32_t                Request;  // DMA request, one of DMA_REQUEST_xx values
	__IO DMA_State_TypeDef  State;    // State of the DMA channel
} DMA_HandleTypeDef;


// Public macros and functions

// Enable specified DMA channel
// input:
//   channel - pointer to the DMA channel handle
__STATIC_INLINE void DMA_EnableChannel(DMA_Channel_TypeDef *channel) {
	channel->CCR |= DMA_CCR_EN;
}

// Disable specified DMA channel
// input:
//   channel - pointer to the DMA channel handle
__STATIC_INLINE void DMA_DisableChannel(DMA_Channel_TypeDef *channel) {
	channel->CCR &= ~DMA_CCR_EN;
}

// Enable specified IRQ for DMA channel
// input:
//   channel - pointer to the DMA channel handle
//   irq - IRQ to enable, combination of DMA_IRQ_xx values
__STATIC_INLINE void DMA_EnableIRQ(DMA_Channel_TypeDef *channel, uint32_t irq) {
	channel->CCR |= (irq & (DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE));
}

// Disable specified IRQ for DMA channel
// input:
//   channel - pointer to the DMA channel handle
//   irq - IRQ to disable, combination of DMA_IRQ_xx values
__STATIC_INLINE void DMA_DisableIRQ(DMA_Channel_TypeDef *channel, uint32_t irq) {
	channel->CCR &= ~(irq & (DMA_CCR_TCIE | DMA_CCR_HTIE | DMA_CCR_TEIE));
}

// Get the specified flags for the specified DMA channel
// input:
//   DMAx - pointer to the DMA peripheral handler (DMA1, etc.)
//   channel_index - index of the channel to clear flags, one of DMA_CHIDX_xxx values
//   flags - flags to check, any combination of DMA_FLAG_xx values
// return: specified flags state, zero if all flags are reset
__STATIC_INLINE uint32_t DMA_GetFlags(DMA_TypeDef *DMAx, uint32_t channel_index, uint32_t flags) {
	return (DMAx->ISR & ((flags & (DMA_ISR_GIF1 | DMA_ISR_TCIF1 | DMA_ISR_HTIF1 | DMA_ISR_TEIF1)) << channel_index));
}

// Clear the specified flags for the specified DMA channel
// input:
//   DMAx - pointer to the DMA peripheral handler (DMA1, etc.)
//   channel_index - index of the channel to clear flags, one of DMA_CHIDX_xxx values
//   flags - flags to be cleared, any combination of DMA_CF_xx values
__STATIC_INLINE void DMA_ClearFlags(DMA_TypeDef *DMAx, uint32_t channel_index, uint32_t flags) {
	DMAx->IFCR |= (flags & DMA_CF_ALL) << channel_index;
}

// Set number of transactions for specified DMA channel
// input:
//   channel - pointer to the DMA channel handle
//   length - new number of DMA transactions
// note: channel must be disabled
__STATIC_INLINE void DMA_SetDataLength(DMA_Channel_TypeDef *channel, uint16_t length) {
	channel->CNDTR = length;
}

// Get number of remaining transactions for specified DMA channel
// input:
//   channel - pointer to the DMA channel handle
// return: number of remaining transactions
__STATIC_INLINE uint16_t DMA_GetDataLength(DMA_Channel_TypeDef *channel) {
	return (channel->CNDTR);
}


// Function prototypes
uint32_t DMA_GetChannelIndex(DMA_Channel_TypeDef *channel);
DMA_TypeDef *DMA_GetChannelPeripheral(DMA_Channel_TypeDef *channel);
void DMA_SetRequest(DMA_TypeDef *DMAx, uint32_t request, uint32_t index);

#endif // __DMA_H
