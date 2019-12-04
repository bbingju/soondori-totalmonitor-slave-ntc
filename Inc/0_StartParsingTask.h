#ifndef __START_PARSING_TASK_H__
#define __START_PARSING_TASK_H__

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#include "0_GlobalDefine.h"
#include "0_GlobalValue.h"


//////////////////////////////////////////////////////////////////
//	Private function prototypes
//////////////////////////////////////////////////////////////////
void StartParsingTask(void const * argument);
void JumpToFunction(void);
void doBoardTypeReq(void);
void doTempReq(uint8_t channel);
void doSendUartToMCU(uint8_t * txData);

/*
void doRevisionCmd(uint8_t cmd, uint8_t channel);
void doRelayCmd(uint8_t cmd, uint8_t data);
void doThresoldValue(uint8_t cmd, uint8_t channel);
void doAdcReq(uint8_t channel);
void doUUDIReq(uint8_t block);
void doFWVersion(void);
void doHWVersion(void);
void doSlotInfo(uint8_t cmd, uint8_t data);
void doBoardEnable(uint8_t cmd, uint8_t data);

void doSendUartToMCU(uint8_t * txData);
*/
#endif

