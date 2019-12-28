#include "0_StartUartTask.h"
#include "0_GlobalValue.h"
#include "0_Util.h"
#include "stm32f1xx_hal_usart.h"
#include "string.h"           //memset 용 include
#include "0_soonQueue.h"
#include "0_soonFlashMemory.h"

/*********************************************************************
*	Private variables
**********************************************************************/
uint8_t receiveData[20] = { 0 };
uint8_t transmtData[140] = { 0 };
uint8_t temp[140];
uint8_t RxData[20];
uint8_t count = 0;

uint8_t getSemaCount = 0;

uint8_t ereaserDataFalg = 0;

uniInt16 crcSave;

RX_QUEUE_STRUCT UartRxQueue;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* osSemaphoreRelease(CountingSemUartRxHandle); */
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_GPIO_WritePin(UART1_TX_EN_GPIO_Port, UART1_TX_EN_Pin, GPIO_PIN_RESET);  //TX 완료후 스위치IC OFF
    /* HAL_Delay(1); */
    /* osSemaphoreRelease(BinarySemUartTxHandle); */
}

/*********************************************************************
*	StartUartTask
*
**********************************************************************/
void StartUartTask(void const * argument)
{
    /* USER CODE BEGIN 5 */
    RxQueue_Clear(&UartRxQueue);
    HAL_UART_Receive_DMA(&huart1, receiveData, UART_RX_DMA_SIZE);

    ereaserDataFalg = FALSE;

    doFindMyID();

    //Task 부팅 완료 플레그
    SysProperties.bootingWate[2] = TRUE;

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
        HAL_UART_Receive_DMA(&huart1, receiveData, UART_RX_DMA_SIZE);

        do
        {
            UartRxFunction();
            UnpackingRxQueue();
            getSemaCount = osSemaphoreGetCount(CountingSemUartRxHandle);

            if(count == (UART_RX_DMA_SIZE - 1))
            {
                /*if(RxData[1] != SysProperties.boardID)
                  {
                  //util_mem_set((void*)&RxData[0], 0x00, sizeof(RxData));
                  }
                  else*/
                if(RxData[1] == SysProperties.boardID)
                {
                    JumpToFunction();
                }
                count = 0;
            }

            if((getSemaCount == 0) && (RxQueue_Count(&UartRxQueue) == 0))
            {
                break;
            }

            osDelay(2);
        }while(1);

        osDelay(10);
    }
    /* USER CODE END 5 */
}

void UartRxFunction(void)
{
    if(CountingSemUartRxHandle != NULL)
    {
        if(osSemaphoreWait(CountingSemUartRxHandle, 0) == osOK)
        {
            for(int i = 0; i < UART_RX_DMA_SIZE; i++)
            {
                //osDelay(1);
                RxQueue_Send(&UartRxQueue, receiveData[i]);
            }
        }
    }
    HAL_UART_Receive_DMA(&huart1, receiveData, UART_RX_DMA_SIZE);
    //osDelay(1);
}

void UnpackingRxQueue(void)
{
    uint16_t	etxLength  = 0;

    if(RxQueue_empty(&UartRxQueue) == FALSE)  //Rx 버퍼에 입력 데이터가 잇는지 확인
    {
        if((UartRxQueue.tail + huart1.RxXferSize - 1) >= UART_RX_BUF_MAX)
        {
            etxLength = UartRxQueue.tail + huart1.RxXferSize - 1 - UART_RX_BUF_MAX;
        }
        else
        {
            etxLength = UartRxQueue.tail + huart1.RxXferSize - 1;
        }

        if((RxQueue_Count(&UartRxQueue) >= UART_RX_DMA_SIZE) )//|| (RxQueue_Count(&UartRxQueue) >= 132))
        {
            //osDelay(1);
            if((UartRxQueue.ar[UartRxQueue.tail] == CMD_STX) && (UartRxQueue.ar[etxLength] == CMD_ETX) )
            {
                count = 0;
                RxData[0] = RxQueue_Recive(&UartRxQueue);
                //osDelay(1);
                if(RxData[0] == CMD_STX)
                {
                    do{
                        count++;
                        osDelay(1);
                        RxData[count] = RxQueue_Recive(&UartRxQueue);
                        if((RxQueue_empty(&UartRxQueue) == TRUE) || (count >= huart1.RxXferSize))
                        {
                            break;
                        }
                    }while(1);
                }
            }
            else
            {
                temp[0] = RxQueue_Recive(&UartRxQueue);
            }
        }
        else
        {
            RxQueue_Clear(&UartRxQueue);
        }
    }
    //osDelay(1);
}

static const char * cmdname(uint8_t cmd)
{
    switch (cmd) {
    case CMD_BOARD_TYPE:
        return "CMD_BOARD_TYPE";
    case CMD_BOARD_EN_REQ:
        return "CMD_BOARD_EN_REQ";
    case CMD_BOARD_EN_SET:
        return "CMD_BOARD_EN_SET";
    case CMD_BOARD_RESET:
        return "CMD_BOARD_RESET";
    case CMD_SLOT_ID_REQ:
        return "CMD_SLOT_ID_REQ";
    case CMD_HW_VER:
        return "CMD_HW_VER";
    case CMD_FW_VER:
        return "CMD_FW_VER";
    case CMD_UUID_REQ:
        return "CMD_UUID_REQ";
    case CMD_TEMP_REQ:
        return "CMD_TEMP_REQ";
    case CMD_TEMP_STATE_REQ:
        return "CMD_TEMP_STATE_REQ";
    case CMD_ADC_REQ:
        return "CMD_ADC_REQ";
    case CMD_THRESHOLD_REQ:
        return "CMD_THRESHOLD_REQ";
    case CMD_THRESHOLD_SET:
        return "CMD_THRESHOLD_SET";
    case CMD_RELAY_REQ:
        return "CMD_RELAY_REQ";
    case CMD_RELAY_SET:
        return "CMD_RELAY_SET";
    case CMD_REVISION_APPLY_SET:
        return "CMD_REVISION_APPLY_SET";
    case CMD_REVISION_CONSTANT_SET:
        return "CMD_REVISION_CONSTANT_SET";
    case CMD_REVISION_APPLY_REQ:
        return "CMD_REVISION_APPLY_REQ";
    case CMD_REVISION_CONSTANT_REQ:
        return "CMD_REVISION_CONSTANT_REQ";
    case CMD_CALIBRATION_NTC_CON_TABLE_CAL:
        return "CMD_CALIBRATION_NTC_CON_TABLE_CAL";
    case CMD_CALIBRATION_NTC_CONSTANT_SET:
        return "CMD_CALIBRATION_NTC_CONSTANT_SET";
    case CMD_CALIBRATION_NTC_CON_TABLE_REQ:
        return "CMD_CALIBRATION_NTC_CON_TABLE_REQ";
    case CMD_CALIBRATION_NTC_CONSTANT_REQ:
        return "CMD_CALIBRATION_NTC_CONSTANT_REQ";
    }

    return "";
}

void JumpToFunction(void)
{
    /* DBG_LOG("%s: ", __func__); */
    /* print_bytes(RxData, 20); */
    //if(SysProperties.boardID == RX_DATA_ID)
    if((RxData[0] == CMD_STX) && (RxData[13] == CMD_ETX))
    {
        DBG_LOG("[%s] %s: \n", __func__, cmdname(RX_DATA_CMD));
        switch(RX_DATA_CMD)
        {
        case CMD_BOARD_TYPE:
            doBoardTypeReq();
            break;
        case CMD_BOARD_EN_REQ:
            break;
        case CMD_BOARD_EN_SET:
            break;
        case CMD_BOARD_RESET:
            HAL_NVIC_SystemReset();
            break;
        case CMD_SLOT_ID_REQ:
            doReqSlotID();
            break;
        case CMD_HW_VER:
            break;
        case CMD_FW_VER:
            break;
        case CMD_UUID_REQ:
            break;
        case CMD_TEMP_REQ:
            doTempReq();
            break;
        case CMD_TEMP_STATE_REQ:
            doTempStateReq();
            break;
        case CMD_ADC_REQ:
            break;
        case CMD_THRESHOLD_REQ:
            doTresholdReq();
            break;
        case CMD_THRESHOLD_SET:
            doTresholdSet();
            break;
        case CMD_RELAY_REQ:
            break;
        case CMD_RELAY_SET:
            break;
        case CMD_REVISION_APPLY_SET:
            doRevisionApplySet();
            break;
        case CMD_REVISION_CONSTANT_SET:
            doRevisionConstantSet();
            break;
        case CMD_REVISION_APPLY_REQ:
            doRevisionApplyReq();
            break;
        case CMD_REVISION_CONSTANT_REQ:
            doRevisionConstantReq();
            break;
        case CMD_CALIBRATION_NTC_CON_TABLE_CAL:
            doCalibrationNTCTableCal();
            break;
        case CMD_CALIBRATION_NTC_CONSTANT_SET:
            doCalibrationNTCConstantSet();
            break;
        case CMD_CALIBRATION_NTC_CON_TABLE_REQ:
            doCalibrationNTCTableReq();
            break;
        case CMD_CALIBRATION_NTC_CONSTANT_REQ:
            doCalibrationNTCConstantReq();
            break;
        }
    }
}

void doReqSlotID(void)
{
    temp[0] = SysProperties.boardID;
    doMakeSendSlotData(transmtData, SysProperties.boardID, CMD_SLOT_ID_REQ, &temp[0], 1, SEND_DATA_LENGTH);
    doSendUartToMCU(transmtData, 12);
}

/*********************************************************************
*   doBoardTypeReq
*       보드의 타입을 리턴
**********************************************************************/
void doBoardTypeReq(void)
{
    temp[0] = SBT_NTC;   //보드 다입은 NTC 로 고정

    doMakeSendTempData(transmtData, CMD_BOARD_TYPE, &temp[0], 1, SEND_DATA_LENGTH);
    doSendUartToMCU(transmtData, 12);
}

void doTempReq()
{
    memcpy(temp, &TestData.Temperature[0][0].Float, 128);

    doMakeSendTempData(transmtData, CMD_TEMP_REQ, temp, 128, 134);
    crcSave.UI8[0] = transmtData[131];
    crcSave.UI8[1] = transmtData[132];

    doSendUartToMCU(transmtData, 134);
}

void doTempStateReq(void)
{
    memcpy(temp, TestData.displayModeFlag, 16);

    doMakeSendTempData(transmtData, CMD_TEMP_STATE_REQ, temp, 16, 38);
    doSendUartToMCU(transmtData, 38);
}

void doTresholdSet(void)
{
    uni4Byte        setTresholdTemp;
    uint8_t		i, j;

    setTresholdTemp.UI8[0] = RxData[4];
    setTresholdTemp.UI8[1] = RxData[5];
    setTresholdTemp.UI8[2] = RxData[6];
    setTresholdTemp.UI8[3] = RxData[7];

    if(RxData[3] == 0xFF)	//전체 변경
    {
        for(j = 0; j < 2; j++)
        {
            for(i = 0; i < 16; i++)
            {
                TestData.Threshold[j][i].Float = setTresholdTemp.Float;
            }
        }
    }
    else
    {
        TestData.Threshold[0][RxData[3]].Float = setTresholdTemp.Float;
    }

    doFlashWriteRevision();

    memcpy(temp, &TestData.Threshold[0][0].UI8[0], 128);
    doMakeSendTempData(transmtData, CMD_THRESHOLD_SET, &temp[0], 128, 134);
    doSendUartToMCU(transmtData, 134);
}

void doTresholdReq(void)
{
    memcpy(temp, &TestData.Threshold[0][0].UI8[0], 128);

    doMakeSendTempData(transmtData, CMD_THRESHOLD_REQ, &temp[0], 128, 134);
    doSendUartToMCU(transmtData, 134);
}

void doRevisionApplySet(void)
{
    TestData.revisionApplyFlag = RxData[3];
    doFlashWriteRevision();

    memset(transmtData, 0x00, sizeof(transmtData));
    doMakeSendTempData(transmtData, CMD_REVISION_APPLY_SET, &TestData.revisionApplyFlag, 1, 12);
    doSendUartToMCU(transmtData, 12);
}

void doRevisionConstantSet(void)
{
    TestData.revisionConstant.UI8[0] = RxData[3];
    TestData.revisionConstant.UI8[1] = RxData[4];
    TestData.revisionConstant.UI8[2] = RxData[5];
    TestData.revisionConstant.UI8[3] = RxData[6];
    doFlashWriteRevision();

    memset(transmtData, 0x00, sizeof(transmtData));
    doMakeSendTempData(transmtData, CMD_REVISION_CONSTANT_SET, &TestData.revisionConstant.UI8[0], 4, 12);
    doSendUartToMCU(transmtData, 12);
}

void doRevisionApplyReq(void)
{
    memset(transmtData, 0x00, sizeof(transmtData));
    doMakeSendTempData(transmtData, CMD_REVISION_APPLY_REQ, &TestData.revisionApplyFlag, 1, 12);
    doSendUartToMCU(transmtData, 12);
}

void doRevisionConstantReq(void)
{
    memset(transmtData, 0x00, sizeof(transmtData));
    doMakeSendTempData(transmtData, CMD_REVISION_CONSTANT_SET, &TestData.revisionConstant.UI8[0], 4, 12);
    doSendUartToMCU(transmtData, 12);
}

void doCalibrationNTCTableCal(void)
{
    uni4Byte	readRTDTemp;

    readRTDTemp.UI8[0] = RxData[3];
    readRTDTemp.UI8[1] = RxData[4];
    readRTDTemp.UI8[2] = RxData[5];
    readRTDTemp.UI8[3] = RxData[6];

    for(int j = 0; j < 2; j++)
    {
        for(int i = 0; i < 16; i++)
        {
            if(TestData.Temperature[j][i].Float != 0)
            {
                TestData.ntcCalibrationTable[j][i].Float = readRTDTemp.Float - TestData.Temperature[j][i].Float;
            }
        }
    }
    doFlashWriteRevision();

    doMakeSendTempData(transmtData, CMD_CALIBRATION_NTC_CON_TABLE_CAL, &TestData.ntcCalibrationTable[0][0].UI8[0], 64, 134);
    doSendUartToMCU(transmtData, 134);
}

void doCalibrationNTCConstantSet(void)
{
        TestData.ntcCalibrationConstant.UI8[0] = RxData[3];
        TestData.ntcCalibrationConstant.UI8[1] = RxData[4];
        TestData.ntcCalibrationConstant.UI8[2] = RxData[5];
        TestData.ntcCalibrationConstant.UI8[3] = RxData[6];

        doFlashWriteRevision();

        doMakeSendTempData(transmtData, CMD_CALIBRATION_NTC_CONSTANT_SET, &TestData.ntcCalibrationConstant.UI8[0], 4, 12);
        doSendUartToMCU(transmtData, 12);
}

void doCalibrationNTCTableReq(void)
{
        doMakeSendTempData(transmtData, CMD_CALIBRATION_NTC_CON_TABLE_CAL, &TestData.ntcCalibrationTable[0][0].UI8[0], 64, 134);
        doSendUartToMCU(transmtData, 134);
}

void doCalibrationNTCConstantReq(void)
{
        doMakeSendTempData(transmtData, CMD_CALIBRATION_NTC_CONSTANT_SET, &TestData.ntcCalibrationConstant.UI8[0], 4, 12);
        doSendUartToMCU(transmtData, 12);
}

void doSendUartToMCU(uint8_t * txData, uint8_t length)
{
    if(BinarySemUartTxHandle != NULL)
    {
        if(osSemaphoreWait(BinarySemUartTxHandle, 0) == osOK)
        {
            HAL_GPIO_WritePin(UART1_TX_EN_GPIO_Port, UART1_TX_EN_Pin, GPIO_PIN_SET);
            HAL_Delay(1);
            HAL_UART_Transmit_DMA(&huart1, txData, length);
        }
    }
}
