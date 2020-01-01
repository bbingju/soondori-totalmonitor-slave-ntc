#include "routine_task.h"
#include "job_task.h"
#include "0_GlobalValue.h"
#include "0_SensorCal.h"
#include "0_Util.h"

static void sensor_init();
static void check_temperature_over(void);
static void flick_led(uint8_t channel, uint8_t overTempDisplayCt, uint8_t maxCount);

static float DoRevisionTemperature(float beforeRevision, float environmentTemp);
static void doMuxAddressSet(uint8_t add);

TEST_DATA TestData;

uint32_t	adc_value[11];
uint8_t         adc_count = 0;
uint8_t		adc_delay_break = FALSE;
uint8_t         adcNumber;

ADC_HandleTypeDef * adc[2] = {&hadc1, &hadc2};

GPIO_TypeDef *MUX_ADD[4] = { MUX_ADD0_GPIO_Port, MUX_ADD1_GPIO_Port,
                             MUX_ADD2_GPIO_Port, MUX_ADD3_GPIO_Port };
uint16_t      MUX_ADD_PIN[4] = { MUX_ADD0_Pin, MUX_ADD1_Pin, MUX_ADD2_Pin,
                                 MUX_ADD3_Pin };

GPIO_TypeDef *MUX_EN[2] = { MUX_EN0_GPIO_Port, MUX_EN1_GPIO_Port };
uint16_t MUX_EN_PIN[2] = { MUX_EN0_Pin, MUX_EN1_Pin };

uint8_t mux_enable;

void routine_task(void const *arg)
{
	static uint32_t tick_1, tick_2;

	sensor_init();

	tick_1 = tick_2 = osKernelSysTick();

	while (1) {

		if (osKernelSysTick() - tick_1 > 1) {
			post_job(JOB_TYPE_ROUTINE_MEASURE_TEMPERATURE, NULL, 0);
			tick_1 = osKernelSysTick();
		}

		if (osKernelSysTick() - tick_2 > 49) {
			post_job(JOB_TYPE_ROUTINE_CHECK_TEMP_OVER, NULL, 0);
			tick_2 = osKernelSysTick();
		}
	}
}

void check_temperature_over_n_flick_led(void)
{
	static uint8_t overTempDisplay = 0;
	check_temperature_over();

	for (int i = 0; i < 16; i++) {
		//센서 연결됨
		if (TestData.overTempFlag[i] == TM_NORMAL_TEMP) {
			if (TestData.AdcMidValue[0][i].UI32 > 3500) //센서 연결 안됨
			{
				if (TestData.displayModeFlag[i] == LDM_DONOT_CONNECT) {
					doLedDisplay(i, _LED_ON);
				} else {
					TestData.displayModeChangeCount[i]++;
					if (TestData.displayModeChangeCount[i] > 3) {
						TestData.displayModeFlag[i] = LDM_DONOT_CONNECT;
						TestData.displayModeChangeCount[i] = 0;
						doLedDisplay(i, _LED_ON);
					}
				}
			} else if (TestData.AdcMidValue[0][i].UI32 < 10) { //센서 쇼트됨
				if (TestData.displayModeFlag[i] == LDM_SENSOR_CLOSE) {
					flick_led(i, (overTempDisplay % 4), 2); // led 고속 점멸
				} else {
					TestData.displayModeChangeCount[i]++;
					if (TestData.displayModeChangeCount[i] > 3) {
						TestData.displayModeFlag[i] = LDM_SENSOR_CLOSE;
						TestData.displayModeChangeCount[i] = 0;
						flick_led(i, (overTempDisplay % 4), 2); // led 고속 점멸
					}
				}
			} else { //정상 온도
				if (TestData.displayModeFlag[i] == LDM_NORMAL_TEMP) {
					doLedDisplay(i, _LED_OFF);
				} else {
					TestData.displayModeChangeCount[i]++;
					if (TestData.displayModeChangeCount[i] > 3) {
						TestData.displayModeFlag[i] = LDM_NORMAL_TEMP;
						TestData.displayModeChangeCount[i] = 0;
						doLedDisplay(i, _LED_OFF);
					}
				}
			}
		} else {	//경고온도 초과
			if (TestData.displayModeFlag[i] == LDM_OVER_TEMP) {
				flick_led(i, overTempDisplay, 10);
			} else {
				TestData.displayModeChangeCount[i]++;
				if (TestData.displayModeChangeCount[i] > 3) {
					TestData.displayModeFlag[i] = LDM_OVER_TEMP;
					TestData.displayModeChangeCount[i] = 0;
					flick_led(i, overTempDisplay, 10);
				}
			}
		}
        }
}

static void flick_led(uint8_t channel, uint8_t overTempDisplayCt, uint8_t maxCount)
{
	if (overTempDisplayCt < maxCount) { //경고온도 초과
		doLedDisplay(channel, _LED_ON);
	} else if ((overTempDisplayCt >= maxCount) &&
		(overTempDisplayCt < (maxCount * 2))) {
		doLedDisplay(channel, _LED_OFF);
	}
}

static void check_temperature_over(void)
{
	uint8_t overTemp = 0;

	for (int i = 0; i < 16; i++) {
		if (TestData.Temperature[0][i].Float > TestData.Threshold[0][i].Float) {
			TestData.overTempFlag[i] = TM_OVER_TEMP;
			overTemp++;
		} else {
			TestData.overTempFlag[i] = TM_NORMAL_TEMP;
		}
	}

	if (overTemp == 0) {	//경고온도 초과된 센서가 0개일때
		doRelayPlay(_OFF);
	} else {
		doRelayPlay(_ON);
	}
}

static void sensor_init()
{
        HAL_ADC_Stop(&hadc1);
        HAL_ADC_Stop(&hadc2);
        HAL_ADCEx_Calibration_Start(&hadc1);
        HAL_ADCEx_Calibration_Start(&hadc2);

        HAL_GPIO_WritePin(MUX_EN[0], MUX_EN_PIN[0], GPIO_PIN_SET);	// disable
        HAL_GPIO_WritePin(MUX_EN[1], MUX_EN_PIN[1], GPIO_PIN_SET);	// disable

        adc_count = 0;

}

void read_sensors(void)
{
	uint8_t         channel;
	uint8_t         mux_add;
	uint8_t         inout;
	uint32_t        midAdc;
	float           calAdc;
	float		beforeRevision;

	mux_enable = adcNumber / 16;
	mux_add    = adcNumber % 16;
	inout      = adcNumber % 2;
	channel    = adcNumber / 2;

	HAL_GPIO_WritePin(MUX_EN[mux_enable], MUX_EN_PIN[mux_enable], GPIO_PIN_RESET);	// enable

	doMuxAddressSet(mux_add);
	//osDelay(1);
	//HAL_Delay(1);

	HAL_ADC_Start_IT(adc[mux_enable]);
	HAL_GPIO_WritePin(MUX_EN[0], MUX_EN_PIN[0], GPIO_PIN_SET);	// disable
	HAL_GPIO_WritePin(MUX_EN[1], MUX_EN_PIN[1], GPIO_PIN_SET);	// disable

	adc_value[adc_count] = HAL_ADC_GetValue(adc[mux_enable]);
	adc_count++;

	if(adc_count < 11)
		return;
	else
		adc_count = 0;

	midAdc = midADC(adc_value);	// ADC 중간값 저장
	calAdc = Calc_Temp_NTC(midAdc);	// ADC로 계산된 온도값 저장

	if((calAdc >= -10) && (calAdc <= 150))
	{
		TestData.AdcMidValue[inout][channel].UI32       = midAdc;

		//교정 과정 실행
		beforeRevision = calAdc + TestData.ntcCalibrationTable[inout][channel].Float    //RTD - NTC 로 계산된 보정 상수
			+ TestData.ntcCalibrationConstant.Float; //사용자가 임의로 추가한 증감 상수

		if (TestData.revisionApplyFlag == TRUE)	{ //보정 적용 상태
			if (inout == 0)	{ //접촉온도 위치에 보정온도를 삽입
				TestData.Temperature[inout][channel].Float = DoRevisionTemperature(TestData.Temperature[0][channel].Float,
												TestData.Temperature[1][channel].Float);
			}
			else	//환경온도 위치는 환경온도 고정
			{
				TestData.Temperature[inout][channel].Float = beforeRevision;
			}
		}
		else {		//보정 미적용 상태
			TestData.Temperature[inout][channel].Float = beforeRevision;
		}
	}
	adcNumber++;
	if (adcNumber > 31) {
		adcNumber = 0;
	}
}

static float DoRevisionTemperature(float beforeRevision, float environmentTemp)
{
        if (TestData.revisionConstant.Float == 0) {
                return (beforeRevision - TestData.revisionConstant.Float * environmentTemp)/(1 - TestData.revisionConstant.Float);
        }
        else {
                return beforeRevision;
        }
}

static void doMuxAddressSet(uint8_t add)
{
	for (int i = 0; i < 4; i++) {
		if((add & 0x01) == 001) {
			HAL_GPIO_WritePin(MUX_ADD[i], MUX_ADD_PIN[i], GPIO_PIN_SET);
		}
		else {
			HAL_GPIO_WritePin(MUX_ADD[i] ,MUX_ADD_PIN[i], GPIO_PIN_RESET);
		}

		add = add >> 1;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	/* //adc_delay_break = TRUE; */
	/* osSemaphoreRelease(myAdcBinarySemHandle); */
}
