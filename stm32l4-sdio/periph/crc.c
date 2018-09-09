#include "crc.h"


// Initialize the CRC peripheral to its initial state
void CRC_Init(void) {
	// Reset the CRC peripheral
	RCC->AHB1RSTR |=  RCC_AHB1RSTR_CRCRST;
	RCC->AHB1RSTR &= ~RCC_AHB1RSTR_CRCRST;
}

// Calculate the CRC value for data in the specified buffer
// input:
//   pBuf - pointer to the data buffer
//   length - size of the buffer (in bytes)
void CRC_CalcBuffer(register uint32_t *pBuf, register uint32_t length) {
	// Send a data to the CRC unit by 32-bit words to reduce number of memory transactions
	while (length > 3) {
#ifdef __GNUC__
		CRC_PutData32(__builtin_bswap32(*pBuf++));
#else
		CRC_PutData32((((*pBuf) >> 24) | (((*pBuf) & 0x00FF0000) >> 8) | (((*pBuf) & 0x0000FF00) << 8) | ((*pBuf) << 24)));
		pBuf++;
#endif // __GNUC__
		length -= 4;
	}

	// Send remnant of the data buffer if any
	if (length) {
		switch (length) {
			case 1:
				CRC_PutData8((uint8_t)(*pBuf));

				break;
			case 2:
#ifdef __GNUC__
				CRC_PutData16((uint16_t)__builtin_bswap16(*pBuf));
#else
				CRC_PutData16((uint16_t)((((*pBuf) >> 8) | (((*pBuf) & 0x00FF) << 8))));
#endif // __GNUC__

				break;
			case 3:
				CRC_PutData8((uint8_t)(*pBuf));
#ifdef __GNUC__
				CRC_PutData16((uint16_t)__builtin_bswap16(*pBuf >> 8));
#else
				CRC_PutData16((uint16_t)(((*pBuf >> 16) & 0x00FF) | (*pBuf & 0xFF00)));
#endif // __GNUC__

				break;
			default:
				// That's impossible!

				break;
		}
	}
}
