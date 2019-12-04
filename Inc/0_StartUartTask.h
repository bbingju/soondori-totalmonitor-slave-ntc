#ifndef __START_UART_TASK_H__
#define __START_UART_TASK_H__

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#include "0_GlobalDefine.h"
#include "0_GlobalValue.h"

#define RX_DATA_ID      RxData[1]
#define RX_DATA_CMD     RxData[2]
#define RX_DATA_BADY    RxData[3]

//////////////////////////////////////////////////////////////////
//	Private function prototypes
//////////////////////////////////////////////////////////////////
void StartUartTask(void const * argument);
void UartRxFunction(void);
void UnpackingRxQueue(void);
void JumpToFunction(void);
void doBoardTypeReq(void);
void doReqSlotID(void);
void doTempReq(void);
void doTempStateReq(void);
void doTresholdSet(void);
void doTresholdReq(void);
void doRevisionApplySet(void);
void doRevisionConstantSet(void);
void doRevisionApplyReq(void);
void doRevisionConstantReq(void);
void doCalibrationNTCTableCal(void);
void doCalibrationNTCConstantSet(void);
void doCalibrationNTCTableReq(void);
void doCalibrationNTCConstantReq(void);


void doSendUartToMCU(uint8_t * txData, uint8_t length);

#endif

