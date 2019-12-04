#include "0_StartDisplayTask.h"
#include "0_GlobalValue.h"
#include "0_SensorCal.h"
#include "0_Util.h"

/*********************************************************************
*	Private variables
**********************************************************************/
uint8_t nvicResetFlag = 0;

GPIO_TypeDef * 		SLOT_EN[4] 	   = { SLOT_EN_00_GPIO_Port, SLOT_EN_01_GPIO_Port, SLOT_EN_02_GPIO_Port, SLOT_EN_03_GPIO_Port };
uint16_t	 		SLOT_EN_PIN[4] = { SLOT_EN_00_Pin, 		 SLOT_EN_01_Pin, 	   SLOT_EN_02_Pin, 		 SLOT_EN_03_Pin };

/*********************************************************************
*	StartDisplayTask
* LED �� RELAY ���� TASK
**********************************************************************/
void StartDisplayTask(void const * argument)
{
	/* USER CODE BEGIN 5 */
	portTickType	xLastWakeTime;	
	uint8_t 		i;//, j;
	uint8_t			overTempDisplay = 0;
	//GPIO_PinState A;//, B;

    //Task ���� �Ϸ� �÷��� 
    SysProperties.bootingWate[0] = TRUE;

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
	  xLastWakeTime = osKernelSysTick();

		doCheckOverTemp();
            
		for(i = 0; i < 16; i++)
		{
			//���� �����    
            if(TestData.overTempFlag[i] == TM_NORMAL_TEMP)
			{
				if(TestData.AdcMidValue[0][i].UI32 > 3500)				//���� ���� �ȵ� 
				{ 
					if(TestData.displayModeFlag[i] == LDM_DONOT_CONNECT)
					{
						doLedDisplay(i, _LED_ON);											
					}
					else
					{
						TestData.displayModeChangeCount[i]++;
						if(TestData.displayModeChangeCount[i] > 3)
						{
							TestData.displayModeFlag[i] = LDM_DONOT_CONNECT;
							TestData.displayModeChangeCount[i] = 0;
							doLedDisplay(i, _LED_ON);
						}
					}
				}
				else if(TestData.AdcMidValue[0][i].UI32 < 10)				//���� ��Ʈ��
				{
					if(TestData.displayModeFlag[i] == LDM_SENSOR_CLOSE)
					{
						doLedFlickingDisplay(i, (overTempDisplay % 4), 2); //led ��� ���� 
					}
					else
					{
						TestData.displayModeChangeCount[i]++;
						if(TestData.displayModeChangeCount[i] > 3)
						{
							TestData.displayModeFlag[i] = LDM_SENSOR_CLOSE;
							TestData.displayModeChangeCount[i] = 0;
							doLedFlickingDisplay(i, (overTempDisplay % 4), 2);	//led ��� ���� 
						}
					}
				}
				else	//���� �µ� 
				{
					if(TestData.displayModeFlag[i] == LDM_NORMAL_TEMP)
					{
						doLedDisplay(i, _LED_OFF);										
					}
					else
					{
						TestData.displayModeChangeCount[i]++;
						if(TestData.displayModeChangeCount[i] > 3)
						{
							TestData.displayModeFlag[i] = LDM_NORMAL_TEMP;
							TestData.displayModeChangeCount[i] = 0;
							doLedDisplay(i, _LED_OFF);								
						}
					}
				}
			}
			else	//���µ� �ʰ� 
			{
				if(TestData.displayModeFlag[i] == LDM_OVER_TEMP)
				{
					doLedFlickingDisplay(i, overTempDisplay, 10);	
				}
				else
				{
					TestData.displayModeChangeCount[i]++;
					if(TestData.displayModeChangeCount[i] > 3)
					{
						TestData.displayModeFlag[i] = LDM_OVER_TEMP;
						TestData.displayModeChangeCount[i] = 0;
						doLedFlickingDisplay(i, overTempDisplay, 10);
					}
				}
			}
		}
		
		overTempDisplay++;
		if(overTempDisplay > 20)
		{
			overTempDisplay = 0;
		}

        //SYSTEM RESET
/*        if(HAL_GPIO_ReadPin(SLOT_EN[SysProperties.boardID - '0'], SLOT_EN_PIN[SysProperties.boardID - '0']) == GPIO_PIN_RESET)
        {
            HAL_UART_Receive_DMA(&huart1, receiveData, UART_RX_DMA_SIZE);
        }*/
		
        osDelayUntil(&xLastWakeTime, 50);
	}
	/* USER CODE END 5 */ 
}

void doLedFlickingDisplay(uint8_t channel, uint8_t overTempDisplayCt, uint8_t maxCount)
{
	if(overTempDisplayCt < maxCount)												//���µ� �ʰ� 
	{
		doLedDisplay(channel, _LED_ON);
	}
	else if((overTempDisplayCt >= maxCount) && (overTempDisplayCt < (maxCount * 2)))
	{
		doLedDisplay(channel, _LED_OFF);
	}
}

void doCheckOverTemp(void)
{
	uint8_t 	overTemp = 0;
	uint8_t 	i = 0;

	for(i = 0; i < 16; i++)
	{
		if(TestData.Temperature[0][i].Float > TestData.Threshold[0][i].Float)
		{
			TestData.overTempFlag[i] = TM_OVER_TEMP;
			overTemp++;
		}
		else
		{
			TestData.overTempFlag[i] = TM_NORMAL_TEMP;
		}
	}
	
	if(overTemp == 0)	//���µ� �ʰ��� ������ 0���϶� 
	{
		doRelayPlay(_OFF);
	}
	else
	{
		doRelayPlay(_ON);
	}
}
