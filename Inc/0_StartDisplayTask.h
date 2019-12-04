#ifndef __START_DISPLAY_TASK_H__
#define __START_DISPLAY_TASK_H__

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#include "0_GlobalDefine.h"
#include "0_GlobalValue.h"





//void StartBluetooehTask(void const * argument);
//////////////////////////////////////////////////////////////////
//	Private function prototypes
//////////////////////////////////////////////////////////////////
void StartSensorTask(void const * argument);
void doCheckOverTemp(void);
void doLedFlickingDisplay(uint8_t channel, uint8_t overTempDisplayCt, uint8_t maxCount);


#endif

