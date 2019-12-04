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
* Main board �� Sensor board ���� �о ������ ���� ���� 
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
	LDM_NORMAL_TEMP = 0,	//����
	LDM_OVER_TEMP,			//���µ� �ʰ�
	LDM_SENSOR_CLOSE,		//���� ��Ʈ
	LDM_DONOT_CONNECT		//���� ���� 
}LED_DIPLAY_MODE;

typedef enum {
	RAF_APPLY = (uint8_t)0,
	RAF_DISAPPLY
}REVISION_APPLY_FLAG;

typedef struct 
{
	uni4Byte				Temperature[2][16];					// adc �Ϸ��� �µ������� ȯ��� ��, 0 : �Ǵ�� �µ�, 1 : ȯ��µ� , 0~15ä�� 
	uni4Byte				AdcMidValue[2][16];					// ������ �Ϸ�� ADC ���� �߰� ��,  0 : �Ǵ�� �µ�, 1 : ȯ��µ�, 0~15ä�� 
	uni4Byte				Threshold[2][16];						// ��� �µ� ���� 
	TEMP_MODE				overTempFlag[16];	
	LED_DIPLAY_MODE	        displayModeFlag[16];				// LED ǥ�� ����� �ٲٱ� ���� ���°� �ٲ�� ���� 3�� üũ�ؼ� �� �̻� ���� ���·� �ݺ��ɶ� ���� 
	uni4Byte				ntcCalibrationTable[2][16];			// NTC ������� , RTD - NTC
	uni4Byte				ntcCalibrationConstant;				// NTC + ntcCalibrationTable(�������) + �������(ntcCalibrationConstant) 
	uint8_t					displayModeChangeCount[16];			// �� �÷��� ����� ���� ī���� 
	uint8_t					revisionApplyFlag;					// ���� ���� �÷���, 1: ǥ��µ����(���� ����) 0 : �����µ� ���(���� ������)
	uni4Byte				revisionConstant;					// ������ ���˻�� 
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
* �ý��ۿ��� �������� ��� �Ǵ� ���� ����  
**********************************************************************/
typedef enum {
	DPM_NORMAL = 0,
	DPM_SETTING
}DISPLAYMODE;

typedef struct 
{								
    uint8_t             boardID;
    uint8_t             bootingWate[4];     //�� ����Ʈ�� Task�� ���� �ϷḦ ǥ�� �Ѵ�. 
                                      // 0 : DiaplayTask, 1 : SensorTask, 2 : UartTask, 3 : ParsingTask
	SENSORBOARDTYPE	    boardType[1];				// SENSORBOARDTYPE ��� �ؼ� ����, sensor board ���� ��� 
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
