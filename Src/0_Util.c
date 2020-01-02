#include "0_Util.h"
#include <string.h>


GPIO_TypeDef *  LED[16] = {	LED_01_GPIO_Port,	LED_02_GPIO_Port,	LED_03_GPIO_Port,	LED_04_GPIO_Port,
                                LED_05_GPIO_Port,	LED_06_GPIO_Port,	LED_07_GPIO_Port,	LED_08_GPIO_Port,
                                LED_09_GPIO_Port,	LED_10_GPIO_Port,	LED_11_GPIO_Port,	LED_12_GPIO_Port,
                                LED_13_GPIO_Port,	LED_14_GPIO_Port,	LED_15_GPIO_Port,	LED_16_GPIO_Port };
uint16_t LED_PIN[16] = { LED_01_Pin,	LED_02_Pin,	LED_03_Pin,	LED_04_Pin,
                         LED_05_Pin,	LED_06_Pin,	LED_07_Pin,	LED_08_Pin,
                         LED_09_Pin,	LED_10_Pin,	LED_11_Pin,	LED_12_Pin,
                         LED_13_Pin,	LED_14_Pin,	LED_15_Pin,	LED_16_Pin };


/*********************************************************************
*	 StringCopyToArray
*  String 를 배열로 복사 한다. 복사한 뒤 남은 배열은 0으로 초기화 한다.
*  TagetArray			: 이곳으로 복사한다.
*  OriginalString : 이것을 복사한다.
*  CopyLength     : 복사할 길이
*	 TotalLength    : 타겟 배열의 총 길이
**********************************************************************/
uint8_t CopyToArray(uint8_t* TagetArray, uint8_t* OriginalString,
		uint8_t CopyLength, uint8_t TotalLength)
{
	for (int i = 0; i < TotalLength; i++) {
		if (i < CopyLength) {
			*TagetArray++ = *OriginalString++;
		}
		else {
			*TagetArray++ = 0;
		}
	}

	return TRUE;
}

void doMakeSendSlotData(uint8_t* SendData,	    uint8_t SlotNumber,
                        uint8_t Command,                uint8_t* Data,
                        uint8_t  DataLength,	uint8_t BufferLength)
{
	//uint8_t i;
	uniInt16 crc;

	*SendData++ = CMD_STX;
	*SendData++ = SlotNumber;
	*SendData++ = Command;
	CopyToArray(SendData, Data, DataLength, DATA_FULL_LENGTH);
	SendData -= 2;

	crc.UI16 =	CRC16_Make(&SendData[0], BufferLength - 4);
	SendData += (BufferLength - 4);

	*SendData++ = crc.UI8[0];
	*SendData++ = crc.UI8[1];

	*SendData = CMD_ETX;
}

void doMakeSendTempData(    uint8_t* SendData,  uint8_t Command,
			uint8_t* Data,
			uint8_t  DataLength,  uint8_t BufferLength)
{
	//uint8_t i;
	uniInt16 crc;

	*SendData++ = CMD_STX;
	*SendData++ = SysProperties.boardID;
	*SendData++ = Command;
	CopyToArray(SendData, Data, DataLength, BufferLength - 4);
	SendData -= 2;

	crc.UI16 =  CRC16_Make(&SendData[0], BufferLength - 4);
	SendData += (BufferLength - 4);

	*SendData++ = crc.UI8[0];
	*SendData++ = crc.UI8[1];

	*SendData = CMD_ETX;
}

/*********************************************************************
*	doNOP
*       NOP 명령 반복 루틴
*       count	 : 16비트 카운트
**********************************************************************/
void doNOP(uint16_t count)
{
        for (int i = 0; i < count; i++)
                asm("NOP");
}

uint32_t sortData[101];

__STATIC_INLINE void swap(uint32_t *a, uint32_t *b)
{
	register uint32_t tmp = *a;
	*a = *b;
	*b = tmp;
}

/**
 * using Bubble sort
 */
uint32_t midADC(uint32_t * inData, int nbr)
{
	memcpy(sortData, inData, nbr * sizeof(uint32_t));

	for (int i = 0; i < nbr; ++i) {
		for (int j = nbr; i < j; --j) {
			if (sortData[j - 1] > sortData[j])
				swap(&sortData[j - 1], &sortData[j]);
		}
	}

	/* for (int i = 0; i < nbr / 5; i++) { */
	/* 	DBG_LOG("%u %u %u %u %u\n", sortData[i * 5 + 0], */
	/* 		sortData[i * 5 + 1], sortData[i * 5 + 2], */
	/* 		sortData[i * 5 + 3], sortData[i * 5 + 4]); */
	/* } */

	return sortData[nbr / 2];
}

void doLedDisplay(uint8_t channel, uint8_t state)
{
        HAL_GPIO_WritePin(LED[channel], LED_PIN[channel], (GPIO_PinState)state);
}

void doRelayPlay(uint8_t state)
{
	if (state == _ON) {
		HAL_GPIO_WritePin(RELAY_SEL_GPIO_Port, RELAY_SEL_Pin, _RELAY_ON);
		HAL_GPIO_WritePin(LED_RELAY_GPIO_Port, LED_RELAY_Pin, _LED_ON);
		SysProperties.relayState = 1;
	}
	else {
		HAL_GPIO_WritePin(RELAY_SEL_GPIO_Port, RELAY_SEL_Pin, _RELAY_OFF);
		HAL_GPIO_WritePin(LED_RELAY_GPIO_Port, LED_RELAY_Pin, _LED_OFF);
		SysProperties.relayState = 0;
	}
}

// CRC16
uint16_t CRC16_Make(uint8_t *byMsg, uint16_t len)
{
	register uint16_t crc = 0xFFFF;
	register uint16_t i;
	register uint8_t  j;

	for (i = 0; i < len; i++) {
		crc ^= byMsg[i];
		for (j = 0; j < 8; j++) {
			uint16_t flag = (uint16_t)(crc & 0x0001);
			crc >>= 1;
			if (flag > 0) crc ^= 0xA001;
		}
	}
	return crc;
}

void doFindMyID(void)
{
    while (1) {
        if ((HAL_GPIO_ReadPin(SLOT_EN_00_GPIO_Port, SLOT_EN_00_Pin) == GPIO_PIN_RESET) ||
            (HAL_GPIO_ReadPin(SLOT_EN_01_GPIO_Port, SLOT_EN_01_Pin) == GPIO_PIN_RESET) ||
            (HAL_GPIO_ReadPin(SLOT_EN_02_GPIO_Port, SLOT_EN_02_Pin) == GPIO_PIN_RESET) ||
            (HAL_GPIO_ReadPin(SLOT_EN_03_GPIO_Port, SLOT_EN_03_Pin) == GPIO_PIN_RESET)) {
            break;
        }
    }

    if (HAL_GPIO_ReadPin(SLOT_EN_00_GPIO_Port, SLOT_EN_00_Pin) == GPIO_PIN_RESET) {
        SysProperties.boardID = 0; // 0: 사용 설정 안됨, 1 ~ 4 슬롯으로 설정 해야 함
    } else if (HAL_GPIO_ReadPin(SLOT_EN_01_GPIO_Port, SLOT_EN_01_Pin) == GPIO_PIN_RESET) {
        SysProperties.boardID = 1; // 0: 사용 설정 안됨, 1 ~ 4 슬롯으로 설정 해야 함
    } else if (HAL_GPIO_ReadPin(SLOT_EN_02_GPIO_Port, SLOT_EN_02_Pin) == GPIO_PIN_RESET) {
        SysProperties.boardID = 2; // 0: 사용 설정 안됨, 1 ~ 4 슬롯으로 설정 해야 함
    } else if (HAL_GPIO_ReadPin(SLOT_EN_03_GPIO_Port, SLOT_EN_03_Pin) == GPIO_PIN_RESET) {
        SysProperties.boardID = 3; // 0: 사용 설정 안됨, 1 ~ 4 슬롯으로 설정 해야 함
    }
}
