#include "hw_config.h"
#include "VCP.h"


// Send buffer to VCP
// input:
//   pBuf - pointer to the buffer
//   len - length of the buffer
void VCP_SendBuf(uint8_t *pBuf, uint32_t len) {
	while (len--) VCP_SendChar(*pBuf++);
}

// Send buffer to VCP with substitute for unprintable characters
// input:
//   buf - pointer to the buffer
//   len - length of the buffer
//   subst - character for substitute
void VCP_SendBufPrintable(uint8_t *pBuf, uint32_t len, char subst) {
	char ch;

	while (len--) {
		ch = *pBuf++;
		VCP_SendChar(ch > 32 ? ch : subst);
	}
}

// Send string to VCP
// input:
//   str - pointer to the null-terminated string
void VCP_SendStr(char *str) {
	while (*str) VCP_SendChar(*str++);
}

// Send signed integer value as text to VCP
// input
//   num - integer value to send
void VCP_SendInt(int32_t num) {
	char str[10]; // 10 chars max for INT32_MAX
	int i = 0;

	if (num < 0) {
		VCP_SendChar('-');
		num *= -1;
	}
	do str[i++] = num % 10 + '0'; while ((num /= 10) > 0);
	while (i) VCP_SendChar(str[--i]);
}
