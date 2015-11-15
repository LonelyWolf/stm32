// Define to prevent recursive inclusion
#ifndef __USBD_IOREQ_H
#define __USBD_IOREQ_H


#include  "usbd_def.h"
#include  "usbd_core.h"


USBD_StatusTypeDef USBD_CtlSendData(USBD_HandleTypeDef *pdev, uint8_t *pBuf, uint16_t length);
USBD_StatusTypeDef USBD_CtlContinueSendData(USBD_HandleTypeDef *pdev, uint8_t *pBuf, uint16_t length);
USBD_StatusTypeDef USBD_CtlPrepareRx(USBD_HandleTypeDef *pdev, uint8_t *pBuf, uint16_t length);
USBD_StatusTypeDef USBD_CtlContinueRx(USBD_HandleTypeDef *pdev, uint8_t *pBuf, uint16_t length);
USBD_StatusTypeDef USBD_CtlSendStatus(USBD_HandleTypeDef *pdev);
USBD_StatusTypeDef USBD_CtlReceiveStatus(USBD_HandleTypeDef *pdev);
uint16_t USBD_GetRxCount(USBD_HandleTypeDef *pdev, uint8_t epnum);

#endif // __USBD_IOREQ_H
