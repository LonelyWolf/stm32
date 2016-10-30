#include "dma.h"


// Calculate index for the specified DMA channel handle
// input:
//   channel - pointer to the DMA channel handle
// return: channel index, one of DMA_CHIDX_xx values
uint32_t DMA_GetChannelIndex(DMA_Channel_TypeDef *channel) {
	uint32_t idx;

	if ((uint32_t)(channel) < (uint32_t)(DMA2_Channel1)) {
		idx = (((uint32_t)channel - (uint32_t)DMA1_Channel1) / ((uint32_t)DMA1_Channel2 - (uint32_t)DMA1_Channel1)) << 2;
	} else {
		idx = (((uint32_t)channel - (uint32_t)DMA2_Channel1) / ((uint32_t)DMA2_Channel2 - (uint32_t)DMA2_Channel1)) << 2;
	}

	return idx;
}

// Determine the DMA peripheral for the specified DMA channel handle
// e.g. for the input value "DMA1_Channel4" it will return pointer to DMA1 handle
// input:
//   channel - pointer to the DMA channel handle
// return: DMA peripheral handle
DMA_TypeDef *DMA_GetChannelPeripheral(DMA_Channel_TypeDef *channel) {
	return ((uint32_t)(channel) < (uint32_t)(DMA2_Channel1)) ? DMA1 : DMA2;
}

// Configure DMA request for DMA channel
// note: no need to call this for MEMORY to MEMORY channels
void DMA_SetRequest(DMA_TypeDef *DMAx, uint32_t request, uint32_t index) {
	if (DMAx == DMA1) {
		// DMA1
		DMA1_CSELR->CSELR &= ~(DMA_CSELR_C1S << index);
		DMA1_CSELR->CSELR |= request << index;
	} else {
		// DMA2
		DMA2_CSELR->CSELR &= ~(DMA_CSELR_C1S << index);
		DMA2_CSELR->CSELR |= request << index;
	}
}
