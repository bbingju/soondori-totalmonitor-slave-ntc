#ifndef __START_SENSOR_TASK_H__
#define __START_SENSOR_TASK_H__

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#include "0_GlobalDefine.h"
#include "0_GlobalValue.h"

#define FLASE	0
#define TRUE	1




//void StartBluetooehTask(void const * argument);
//////////////////////////////////////////////////////////////////
//	Private function prototypes
//////////////////////////////////////////////////////////////////
void StartSensorTask(void const * argument);
void doReadADC(void);
float DoRevisionTemperature(float beforeRevision, float environmentTemp);
void doMuxAddressSet(uint8_t add);


#endif

