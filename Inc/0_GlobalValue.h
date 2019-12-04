#ifndef __GLOBAL_VALUE_H__
#define __GLOBAL_VALUE_H__


#ifndef _STDINT
	#include "stdint.h"
#endif

#include "main.h"
#include "0_GlobalDefine.h"
#include "cmsis_os.h"

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;;
extern UART_HandleTypeDef huart2;

extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

extern osSemaphoreId CountingSemUartRxHandle;
extern osSemaphoreId myAdcBinarySemHandle;
extern osSemaphoreId BinarySemUartTxHandle;

//extern uint8_t 	receiveData[10];    //MCU to NTC , Uart Buffer
//extern uint8_t 	transmtData[10];    //MCU to NTC , Uart Buffer

typedef union {
	float 		Float;
	uint32_t	UI32;
	uint16_t 	UI16[2];
	uint8_t 	UI8[4];	
	int16_t 	SI16[2];
	int8_t 		SIi8[4];	
}uni4Byte;

typedef union {
	uint16_t	u16_Value;
	uint8_t 	b_Value[2];
}uniShort;

typedef union {
	uint32_t	u32_Value;
	uint8_t 	b_Value[4];
}uniInt32;

typedef union {
	char			cChar;
	uint8_t 	b_Value;
}uniChar;

typedef union{
	uint16_t	UI16;
	uint8_t		UI8[2];
}uniInt16;

/*********************************************************************
*	Test Data
* Main board 와 Sensor board 에서 읽어낸 데이터 저장 공간 
**********************************************************************/
typedef enum {
	MBS_BATTERY = 0,
	MBS_RTD,
	MBS_TEMP,
	MBS_HUMI	
}MAINBOARDSENSOR;

typedef enum {
	DT_FLOAT = 0,
	DT_UINT16,
	DT_UINT8,
	DT_INT16,
	DT_INT8
}DATATYPE;

typedef enum {
	SBT_NTC = 1,
	SBT_RELAY,
	SBT_RTD,
	SBT_MULTI,
	SBT_BLUETOOTH
}SENSORBOARDTYPE;

typedef enum {
	TM_NORMAL_TEMP = 0,
	TM_OVER_TEMP
}TEMP_MODE;

typedef enum {
	LDM_NORMAL_TEMP = 0,	//정상
	LDM_OVER_TEMP,			//경고온도 초과
	LDM_SENSOR_CLOSE,		//센서 쇼트
	LDM_DONOT_CONNECT		//센서 없음 
}LED_DIPLAY_MODE;

typedef enum {
	RAF_APPLY = (uint8_t)0,
	RAF_DISAPPLY
}REVISION_APPLY_FLAG;

typedef struct 
{
	uni4Byte				Temperature[2][16];					// adc 완료후 온도값으로 환산된 값, 0 : 피대상물 온도, 1 : 환경온도 , 0~15채널 
	uni4Byte				AdcMidValue[2][16];					// 컨버팅 완료된 ADC 값중 중간 값,  0 : 피대상물 온도, 1 : 환경온도, 0~15채널 
	uni4Byte				Threshold[2][16];						// 경고 온도 저장 
	TEMP_MODE				overTempFlag[16];	
	LED_DIPLAY_MODE	        displayModeFlag[16];				// LED 표시 방법을 바꾸기 전에 상태가 바뀌는 것은 3번 체크해서 그 이상 같은 상태로 반복될때 변경 
	uni4Byte				ntcCalibrationTable[2][16];			// NTC 교정상수 , RTD - NTC
	uni4Byte				ntcCalibrationConstant;				// NTC + ntcCalibrationTable(교정상수) + 증감상수(ntcCalibrationConstant) 
	uint8_t					displayModeChangeCount[16];			// 위 플래그 사용을 위한 카운터 
	uint8_t					revisionApplyFlag;					// 보정 적용 플래그, 1: 표면온도모드(보정 적용) 0 : 측정온도 모드(보정 미적용)
	uni4Byte				revisionConstant;					// 보정용 접촉상수 
}TEST_DATA;
extern TEST_DATA TestData;

typedef struct 
{
	uint16_t	size;
	uint16_t 	head;
	uint16_t 	tail;
	uint16_t 	count;
	uint8_t	    ar[UART_RX_BUF_MAX];
}RX_QUEUE_STRUCT;
extern RX_QUEUE_STRUCT UartRxQueue;

typedef struct 
{
	uint16_t	size;
	uint16_t 	head;
	uint16_t 	tail;
	uint16_t 	count;
	uint8_t	    ar[UART_TX_BUF_MAX];
}TX_QUEUE_STRUCT;
extern TX_QUEUE_STRUCT UartTxQueue;

/*********************************************************************
*	System Properties
* 시스템에서 공용으로 사용 되는 변수 저장  
**********************************************************************/
typedef enum {
	DPM_NORMAL = 0,
	DPM_SETTING
}DISPLAYMODE;

typedef struct 
{								
    uint8_t             boardID;
    uint8_t             bootingWate[4];     //각 바이트에 Task별 부팅 완료를 표시 한다. 
                                      // 0 : DiaplayTask, 1 : SensorTask, 2 : UartTask, 3 : ParsingTask
	SENSORBOARDTYPE	    boardType[1];				// SENSORBOARDTYPE 사용 해서 선택, sensor board 종류 기록 
	uint8_t				boardEnable;
	uniChar				hwVersion[5];
	uniChar				fwVersion[5];
	uniInt32 			uuid[3];
	uint8_t				relayState;
}SYSTEM_STRUCT;
extern SYSTEM_STRUCT SysProperties;

extern uint8_t receiveData[20];
extern uint8_t transmtData[140];

#endif
