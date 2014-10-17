// Define to prevent recursive inclusion -------------------------------------
#ifndef __VCP_H
#define __VCP_H


// Function prototypes
void VCP_SendBuf(uint8_t *pBuf, uint32_t len);
void VCP_SendStr(char *str);
void VCP_SendInt(int32_t num);

#endif // __VCP_H
