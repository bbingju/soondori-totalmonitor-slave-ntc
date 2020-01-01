#ifndef __UTIL_H__
#define __UTIL_H__


#include <stdint.h>

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#include "0_GlobalValue.h"
#include "0_GlobalDefine.h"
#include "debug.h"

#define _LED_ON			GPIO_PIN_RESET
#define _LED_OFF		GPIO_PIN_SET
#define _RELAY_ON		GPIO_PIN_RESET
#define _RELAY_OFF	GPIO_PIN_SET

#define DATA_FULL_LENGTH 	6

/* typedef unsigned int	UINT; */
/* typedef unsigned char	BYTE; */


float ByteArrayToFloat(uint8_t *byteArray);
uint8_t CopyToArray(uint8_t* TagetArray, uint8_t* OriginalString, uint8_t CopyLength, uint8_t TotalLength);
void doMakeSendData(uint8_t* SendData, uint8_t Command, uint8_t* Data, uint8_t DataLength, uint8_t BufferLength);
void doMakeSendSlotData(uint8_t* SendData, uint8_t SlotNumber, uint8_t Command, uint8_t* Data, uint8_t  DataLength,	uint8_t BufferLength);
void doMakeSendTempData(    uint8_t* SendData,  uint8_t Command, uint8_t* Data, uint8_t  DataLength,  uint8_t BufferLength);
void doNOP(uint16_t count);
uint32_t midADC(uint32_t * inData);
void swap(uint32_t *a, uint32_t *b);
void doLedDisplay(uint8_t channel, uint8_t state);
void doRelayPlay(uint8_t state);
uint16_t CRC16_Make(uint8_t *byMsg, uint16_t len);
void doFindMyID(void);


#endif

