#ifndef __SENSOR_CAL_H__
#define __SENSOR_CAL_H__

#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

#ifndef _STDINT
	#include "stdint.h"
#endif


float Calc_BD_Temp(uint32_t val);
float Calc_BD_Humi(uint32_t val);
float Calc_Temp_NTC(uint32_t val);
float Calc_Temp_RTD(uint32_t val);


#endif
