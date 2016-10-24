#ifndef __CRC_H
#define __CRC_H


#include <stm32l4xx.h>


// Definitions of polynomial length
#define CRC_PSIZE_32B              ((uint32_t)0x00000000U)                 // 32 bits
#define CRC_PSIZE_16B              CRC_CR_POLYSIZE_0                       // 16 bits
#define CRC_PSIZE_8B               CRC_CR_POLYSIZE_1                       // 8 bits
#define CRC_PSIZE_7B               (CRC_CR_POLYSIZE_1 | CRC_CR_POLYSIZE_0) // 7 bits

// Definitions of input data reverse
#define CRC_IN_NORMAL              ((uint32_t)0x00000000U)             // bit order not affected
#define CRC_IN_REV_BYTE            CRC_CR_REV_IN_0                     // by byte (8-bit)
#define CRC_IN_REV_HALFWORD        CRC_CR_REV_IN_1                     // by half-word (16-bit)
#define CRC_IN_REV_WORD            (CRC_CR_REV_IN_1 | CRC_CR_REV_IN_0) // by word (32-bit)

// Definitions of output data reverse
#define CRC_OUT_NORMAL             ((uint32_t)0x00000000U) // bit order not affected
#define CRC_OUT_REVERSED           CRC_CR_REV_OUT          // bit order reversed


// Public functions and macros

// Enable the CRC peripheral
__STATIC_INLINE void CRC_Enable(void) {
	RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;
}

// Disable the CRC peripheral
__STATIC_INLINE void CRC_Disable(void) {
	RCC->AHB1ENR &= ~RCC_AHB1ENR_CRCEN;
}

// Reset the CRC calculation unit
__STATIC_INLINE void CRC_Reset(void) {
	CRC->CR |= CRC_CR_RESET;
}

// Configure size of the polynomial
// input:
//   size - new polynomial size, one of CRC_PSIZE_xx values
__STATIC_INLINE void CRC_SetPolynomialSize(uint32_t size) {
	CRC->CR &= ~CRC_CR_POLYSIZE;
	CRC->CR |= size & CRC_CR_POLYSIZE;
}

// Configure initial CRC value
// input:
//   value - new initial CRC value
__STATIC_INLINE void CRC_SetInitValue(uint32_t value) {
	CRC->INIT = value;
}

// Configure polynomial value
// input:
//   poly - new value of the coefficients of the polynomial to be used in CRC calculation
__STATIC_INLINE void CRC_SetPolynomial(uint32_t poly) {
	CRC->POL = poly;
}

// Configure the reversal of the input data (bits order)
// input:
//   mode - new value of the bit reversal, one of CRC_IN_xx values
__STATIC_INLINE void CRC_SetInRevMode(uint32_t mode) {
	CRC->CR &= ~CRC_CR_REV_IN;
	CRC->CR |= mode & CRC_CR_REV_IN;
}

// Configure the reversal of the output data (bits order)
// input:
//   mode - new value of the bit reversal, one of CRC_OUT_xx values
__STATIC_INLINE void CRC_SetOutRevMode(uint32_t mode) {
	CRC->CR &= ~CRC_CR_REV_OUT;
	CRC->CR |= mode & CRC_CR_REV_OUT;
}

// Write the specified 8-bit value to the CRC calculation unit
// input:
//   data - 8-bit value
__STATIC_INLINE void CRC_PutData8(uint8_t data) {
	*(uint8_t __IO *)(&CRC->DR) = data;
}

// Write the specified 16-bit value to the CRC calculation unit
// input:
//   data - 16-bit value
__STATIC_INLINE void CRC_PutData16(uint16_t data) {
	*(uint16_t __IO *)(&CRC->DR) = data;
}

// Write the specified 8-bit value to the CRC calculation unit
// input:
//   data - 8-bit value
__STATIC_INLINE void CRC_PutData32(uint32_t data) {
	CRC->DR = data;
}

// Read a current CRC calculation result (7-bit)
// return: 7-bit value
__STATIC_INLINE uint8_t CRC_GetData7(void) {
	return (uint8_t)(CRC->DR & 0x0000007F);
}

// Read a current CRC calculation result (8-bit)
// return: 8-bit value
__STATIC_INLINE uint8_t CRC_GetData8(void) {
	return (uint8_t)(CRC->DR);
}

// Read a current CRC calculation result (16-bit)
// return: 16-bit value
__STATIC_INLINE uint16_t CRC_GetData16(void) {
	return (uint16_t)(CRC->DR);
}

// Read a current CRC calculation result (32-bit)
// return: 32-bit value
__STATIC_INLINE uint32_t CRC_GetData32(void) {
	return CRC->DR;
}


// Function prototypes
void CRC_Init(void);
//void CRC_CalcBuffer(uint8_t *pBuf, uint32_t length);
void CRC_CalcBuffer(register uint32_t *pBuf, register uint32_t length);

#endif // __CRC_H
