#ifndef __SOON_QUEUE_H__
#define __SOON_QUEUE_H__

#ifndef _STDINT
	#include "stdint.h"
#endif

#include "0_GlobalValue.h"


uint8_t TxQueue_empty(TX_QUEUE_STRUCT *q);
void TxQueue_Init(TX_QUEUE_STRUCT *q);
void TxQueue_Send(TX_QUEUE_STRUCT *q, uint8_t data);
uint8_t TxQueue_Recive(TX_QUEUE_STRUCT *q);

uint8_t RxQueue_empty(RX_QUEUE_STRUCT *q);
void RxQueue_Init(RX_QUEUE_STRUCT *q);
void RxQueue_Send(RX_QUEUE_STRUCT *q, uint8_t data);
uint8_t RxQueue_Recive(RX_QUEUE_STRUCT *q);
void RxQueue_Clear(RX_QUEUE_STRUCT *q);
uint8_t RxQueue_Count(RX_QUEUE_STRUCT *q);

#endif

