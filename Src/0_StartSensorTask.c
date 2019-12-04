#include "0_StartSensorTask.h"
#include "0_GlobalValue.h"
#include "0_SensorCal.h"
#include "0_Util.h"


/*********************************************************************
*	Private variables
**********************************************************************/
TEST_DATA TestData;

uint32_t	adc_value[11];
uint8_t         adc_count = 0;
uint8_t		adc_delay_break = FALSE;
uint8_t         adcNumber;

ADC_HandleTypeDef * adc[2] = {&hadc1, &hadc2};

GPIO_TypeDef *MUX_ADD[4] = {MUX_ADD0_GPIO_Port, MUX_ADD1_GPIO_Port,
                            MUX_ADD2_GPIO_Port, MUX_ADD3_GPIO_Port};
uint16_t MUX_ADD_PIN[4] = {MUX_ADD0_Pin, MUX_ADD1_Pin, MUX_ADD2_Pin,
                           MUX_ADD3_Pin};

GPIO_TypeDef *MUX_EN[2] = {MUX_EN0_GPIO_Port, MUX_EN1_GPIO_Port};
uint16_t MUX_EN_PIN[2] = {MUX_EN0_Pin, MUX_EN1_Pin};

uint8_t                         mux_enable;


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    //adc_delay_break = TRUE;
    osSemaphoreRelease(myAdcBinarySemHandle);
}

/*********************************************************************
*	StartSensorTask
* 32개 센서 입력 task
**********************************************************************/
void StartSensorTask(void const * argument)
{
        /* init code for USB_DEVICE */
//	MX_USB_DEVICE_Init();

        /* USER CODE BEGIN 5 */
        HAL_ADC_Stop(&hadc1);
        HAL_ADC_Stop(&hadc2);
        HAL_ADCEx_Calibration_Start(&hadc1);
        HAL_ADCEx_Calibration_Start(&hadc2);

        HAL_GPIO_WritePin(MUX_EN[0], MUX_EN_PIN[0], GPIO_PIN_SET);	// disable
        HAL_GPIO_WritePin(MUX_EN[1], MUX_EN_PIN[1], GPIO_PIN_SET);	// disable

        adc_count = 0;

        doFindMyID();

        //Task 부팅 완료 플레그
        SysProperties.bootingWate[1] = TRUE;

    while(1)
    {
        if( (SysProperties.bootingWate[0] == TRUE) &&     // 0 : DiaplayTask,
            (SysProperties.bootingWate[1] == TRUE) &&     // 1 : SensorTask
            (SysProperties.bootingWate[2] == TRUE))       // 2 : UartTask
        {
          break;
        }
        osDelay(2);
    }


        /* Infinite loop */
        for(;;)
        {
                doReadADC();
                osDelay(1);
        }
        /* USER CODE END 5 */
}

void doReadADC(void)
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

    if(myAdcBinarySemHandle != NULL)
    {
        if(osSemaphoreWait(myAdcBinarySemHandle, 0) == osOK)
        {
            adc_value[adc_count] = HAL_ADC_GetValue(adc[mux_enable]);
            adc_count++;

            if(adc_count < 11)
                return;
            else
                adc_count = 0;

            midAdc = midADC(adc_value);					// ADC 중간값 저장
            calAdc = Calc_Temp_NTC(midAdc);				// ADC로 계산된 온도값 저장

            if((calAdc >= -10) && (calAdc <= 150))
            {
                TestData.AdcMidValue[inout][channel].UI32       = midAdc;

                //교정 과정 실행
                beforeRevision = calAdc + TestData.ntcCalibrationTable[inout][channel].Float    //RTD - NTC 로 계산된 보정 상수
                    + TestData.ntcCalibrationConstant.Float;				//사용자가 임의로 추가한 증감 상수

                if(TestData.revisionApplyFlag == TRUE)	//보정 적용 상태
                {
                    if(inout == 0)		//접촉온도 위치에 보정온도를 삽입
                    {
                        TestData.Temperature[inout][channel].Float = DoRevisionTemperature(TestData.Temperature[0][channel].Float,
                                                                                           TestData.Temperature[1][channel].Float);
                    }
                    else				//환경온도 위치는 환경온도 고정
                    {
                        TestData.Temperature[inout][channel].Float = beforeRevision;
                    }
                }
                else									//보정 미적용 상태
                {
                    TestData.Temperature[inout][channel].Float = beforeRevision;
                }
            }
            adcNumber++;
            if(adcNumber > 31)
            {
                adcNumber = 0;
            }
        }
    }
}

float DoRevisionTemperature(float beforeRevision, float environmentTemp)
{
        if(TestData.revisionConstant.Float == 0)
        {
                return (beforeRevision - TestData.revisionConstant.Float * environmentTemp)/(1 - TestData.revisionConstant.Float);
        }
        else
        {
                return beforeRevision;
        }
}

void doMuxAddressSet(uint8_t add)
{
    for(uint8_t i = 0; i < 4; i++)
    {
        if((add & 0x01) == 001)
        {
            HAL_GPIO_WritePin(MUX_ADD[i], MUX_ADD_PIN[i], GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(MUX_ADD[i] ,MUX_ADD_PIN[i], GPIO_PIN_RESET);
        }

        add = add >> 1;
    }
}
