#include "0_Util.h"
#include <string.h>

uint8_t *bytes;
uint8_t BuzzerEnable;


GPIO_TypeDef *  LED[16] = {	LED_01_GPIO_Port,	LED_02_GPIO_Port,	LED_03_GPIO_Port,	LED_04_GPIO_Port,
                                LED_05_GPIO_Port,	LED_06_GPIO_Port,	LED_07_GPIO_Port,	LED_08_GPIO_Port,
                                LED_09_GPIO_Port,	LED_10_GPIO_Port,	LED_11_GPIO_Port,	LED_12_GPIO_Port,
                                LED_13_GPIO_Port,	LED_14_GPIO_Port,	LED_15_GPIO_Port,	LED_16_GPIO_Port };
uint16_t LED_PIN[16] = { LED_01_Pin,	LED_02_Pin,	LED_03_Pin,	LED_04_Pin,
                         LED_05_Pin,	LED_06_Pin,	LED_07_Pin,	LED_08_Pin,
                         LED_09_Pin,	LED_10_Pin,	LED_11_Pin,	LED_12_Pin,
                         LED_13_Pin,	LED_14_Pin,	LED_15_Pin,	LED_16_Pin };


/*********************************************************************
*	 ByteArrayToFloat
*  String 를 배열로 복사 한다. 복사한 뒤 남은 배열은 0으로 초기화 한다.
*  byteArray			: 원본 배열을 받는다. 4바이트
*	 return	                : float 로 변환하여 반환 한다.
**********************************************************************/
float ByteArrayToFloat(uint8_t *byteArray)
{
//	uint8_t data[4] = {0, 0, 0, 0};
	bytes = byteArray;
	uint32_t res = 0.0;

	res  =  ((uint32_t)*byteArray
		| ((uint32_t)*(byteArray + 1) << 8)
		| ((uint32_t)*(byteArray + 2) << 16)
		| ((uint32_t)*(byteArray + 3) << 24));

	return	*((float*)&res);
}

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
	for (int i = 0; i < TotalLength; i++)
	{
		if (i < CopyLength)
		{
			*TagetArray++ = *OriginalString++;
		}
		else
		{
			*TagetArray++ = 0;
		}
	}

	return TRUE;
}

/*********************************************************************
*	doMakeSendData
*	명령을 받앗ㅅ을때 리턴해주는 문자열 생성 함수
*	SendData		: 이곳으로 문자열을 생성한다.
*	Command                 : 1번 바이트의 command 의 바이트를 받는다. (0번 바이트부터 시작)
*	Data                    : 2번부터 기록될 문자열을 받는다.
*	DataLength		: Data 의 길이를 받는다.
*	BufferLength	: 전체 SendData의 길이를 받는다. etx의 위치를 확인
**********************************************************************/
#if 0
void doMakeSendData(  uint8_t* SendData,  uint8_t Command,
                            uint8_t* Data,
                            uint8_t  DataLength,  uint8_t BufferLength)
{
  //uint8_t i;
  uniInt16 crc;

  *SendData++ = CMD_STX;
  *SendData++ = Command;
  CopyToArray(SendData, Data, DataLength, DATA_FULL_LENGTH);
  SendData += 5;

  crc.UI16 =  CRC16_Make(&SendData[0], DATA_FULL_LENGTH + 1);

  *SendData++ = crc.UI8[0];
  *SendData++ = crc.UI8[1];

  *SendData = CMD_ETX;
}
#endif

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

uint32_t midADC(uint32_t * inData) /* using Bubble sort */
{
    uint8_t         i, j;
    uint32_t	sortData[11];

    sortData[0]     = *(inData++);
    sortData[1]     = *(inData++);
    sortData[2]     = *(inData++);
    sortData[3]     = *(inData++);
    sortData[4]     = *(inData++);
    sortData[5]     = *(inData++);
    sortData[6]     = *(inData++);
    sortData[7]     = *(inData++);
    sortData[8]     = *(inData++);
    sortData[9]     = *(inData++);
    sortData[10]	= *(inData);

    for(i = 0; i < 11 - 1; ++i)
    {
        for(j = 11 - 1; i < j; --j)
        {
            if(sortData[j - 1] > sortData[j])
                swap(&sortData[j - 1], &sortData[j]);
        }
    }
    return sortData[6];
}

void swap(uint32_t *a, uint32_t *b)
{
    uint32_t tmp = *a;
    *a = *b;
    *b = tmp;
}

void doLedDisplay(uint8_t channel, uint8_t state)
{
        HAL_GPIO_WritePin(LED[channel], LED_PIN[channel], (GPIO_PinState)state);
}

void doRelayPlay(uint8_t state)
{
    if(state == _ON)
    {
        HAL_GPIO_WritePin(RELAY_SEL_GPIO_Port, RELAY_SEL_Pin, _RELAY_ON);
        HAL_GPIO_WritePin(LED_RELAY_GPIO_Port, LED_RELAY_Pin, _LED_ON);
        SysProperties.relayState = 1;
    }
    else
    {
        HAL_GPIO_WritePin(RELAY_SEL_GPIO_Port, RELAY_SEL_Pin, _RELAY_OFF);
        HAL_GPIO_WritePin(LED_RELAY_GPIO_Port, LED_RELAY_Pin, _LED_OFF);
        SysProperties.relayState = 0;
    }
}

// CRC16
uint16_t CRC16_Make(uint8_t *byMsg, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint16_t i;
    uint8_t  j;

    for (i = 0; i < len; i++)
    {
        crc ^= byMsg[i];
        for (j = 0; j < 8; j++)
        {
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
