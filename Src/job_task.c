#include "job_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "frame.h"
#include "routine_task.h"
#include "0_soonFlashMemory.h"
#include "0_Util.h"
#include "debug.h"

#include <string.h>

struct job_s {
	JOB_TYPE_E type;
	struct internal_frame internal;
};

static osMailQDef(job_pool_q, 12, struct job_s);
static osMailQId(job_pool_q_id);

static void job_handler(struct job_s *job);
static void handle_to_internal(struct internal_frame *frm);
static void handle_from_internal(struct internal_frame *frm);

static void response_board_type_req(struct internal_frame *);
static void response_slot_id_req(struct internal_frame *);
static void response_temperature_req(struct internal_frame *);
static void response_temperature_state_req(struct internal_frame *);
static void response_threshold_req(struct internal_frame *);
static void doTresholdSet(struct internal_frame *);
static void doRevisionApplySet(struct internal_frame *);
static void doRevisionConstantSet(struct internal_frame *);
static void doRevisionApplyReq(struct internal_frame *);
static void doRevisionConstantReq(struct internal_frame *);
static void doCalibrationNTCTableCal(struct internal_frame *);
static void doCalibrationNTCConstantSet(struct internal_frame *);
static void doCalibrationNTCTableReq(struct internal_frame *);
static void doCalibrationNTCConstantReq(struct internal_frame *);

extern UART_HandleTypeDef huart1;

/**
 * @brief  Request a job to the task.
 * @param  argument: Not used
 * @retval None
 */
int post_job(JOB_TYPE_E type, void const *data, size_t datalen)
{
	struct job_s *job;
	job = (struct job_s *)osMailAlloc(job_pool_q_id, osWaitForever);
	if (!job) {
		DBG_LOG("%s: mail allocation failed\n", __func__);
		return -1;
	}

	job->type = type;
	memcpy(&job->internal, data, datalen);

	osMailPut(job_pool_q_id, job);
	return 0;
}

/**
 * @brief  Function implementing the JobTask.
 * @param  argument: Not used
 * @retval None
 */
void job_task(void const *argument)
{
	job_pool_q_id = osMailCreate(osMailQ(job_pool_q), NULL);

	doFindMyID();

	DBG_LOG("%s is started\n", __func__);

	/* uint32_t tick = HAL_GetTick(); */
	while (1) {
		osEvent event = osMailGet(job_pool_q_id, osWaitForever);
		struct job_s *job = (struct job_s *)event.value.p;
		/* tick = HAL_GetTick(); */
		job_handler(job);
		/* DBG_LOG("Elapsed tick: %u\n", HAL_GetTick () - tick); */
		osMailFree(job_pool_q_id, job);
	}
}

/**
 * job_handler
 */
static void job_handler(struct job_s *job)
{
	switch (job->type) {
	case JOB_TYPE_TO_INTERNAL:
		handle_to_internal(&job->internal);
		break;
	case JOB_TYPE_FROM_INTERNAL:
		handle_from_internal(&job->internal);
		break;
	case JOB_TYPE_ROUTINE_MEASURE_TEMPERATURE:
		read_sensors();
		break;
	case JOB_TYPE_ROUTINE_CHECK_TEMP_OVER:
		check_temperature_over_n_flick_led();
		break;
	default:
		return;
	}
}

static void handle_to_internal(struct internal_frame *frm)
{
	size_t frame_size = 0;
	static uint8_t buffer[255 + 7] = {0};

	frame_size = fill_internal_frame(buffer, frm->slot_id, frm->cmd,
					frm->datalen, frm->tx_data);
	DBG_LOG("JOB_TYPE_TO_INTERNAL:");
	DBG_DUMP(buffer, frame_size);

	/* if (frm->data) { */
	/*   free(frm->data); */
	/*   frm->data = NULL; */
	/* } */

	/* while (HAL_GPIO_ReadPin(UART1_TX_EN_GPIO_Port, UART1_TX_EN_Pin) == GPIO_PIN_SET) */
	/* 	__NOP(); */
	HAL_GPIO_WritePin(UART1_TX_EN_GPIO_Port, UART1_TX_EN_Pin, GPIO_PIN_SET);

	/* DBG_LOG("%s\n", __func__); */
	/* HAL_MultiProcessor_EnterMuteMode(&huart1); */

	/* while (!__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) ; */
	/* if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE)) { */
	/* 	DBG_LOG("IDLE\n"); */
	/* } else { */
	/* 	DBG_LOG("Not IDLE\n"); */
	/* } */

	/* while (READ_BIT(huart1.Instance->CR1, USART_CR1_RWU)) ; */
	/* 	__NOP(); */
	HAL_UART_Transmit_DMA(&huart1, buffer, frame_size);
}

static void handle_from_internal(struct internal_frame *frm)
{
	DBG_LOG("JOB_TYPE_FROM:");
	DBG_LOG("slot id: %d, cmd: 0x%02X, datalen: %d\n", frm->slot_id, frm->cmd,
		frm->datalen);

	switch (frm->cmd) {
	case INTERNAL_CMD_BOARD_TYPE_REQ:
		response_board_type_req(frm);
		break;
	case INTERNAL_CMD_RESET:
		HAL_NVIC_SystemReset();
		break;
	case INTERNAL_CMD_SLOT_ID_REQ:
		response_slot_id_req(frm);
		break;
	case INTERNAL_CMD_TEMPERATURE_REQ:
		response_temperature_req(frm);
		break;
	case INTERNAL_CMD_TEMPERATURE_STATE_REQ:
		response_temperature_state_req(frm);
		break;
	case INTERNAL_CMD_ADC_REQ:
		break;
	case INTERNAL_CMD_THRESHOLD_REQ:
		response_threshold_req(frm);
		break;
	case INTERNAL_CMD_THRESHOLD_SET:
		doTresholdSet(frm);
		break;
	case INTERNAL_CMD_RELAY_REQ:
		break;
	case INTERNAL_CMD_RELAY_SET:
		break;
	case INTERNAL_CMD_REVISION_CONSTANT_REQ:
		doRevisionConstantReq(frm);
		break;
	case INTERNAL_CMD_REVISION_CONSTANT_SET:
		doRevisionConstantSet(frm);
		break;
	case INTERNAL_CMD_REVISION_APPLY_REQ:
		doRevisionApplyReq(frm);
		break;
	case INTERNAL_CMD_REVISION_APPLY_SET:
		doRevisionApplySet(frm);
		break;
	case INTERNAL_CMD_CALIBRATION_NTC_CON_TABLE_CAL:
		doCalibrationNTCTableCal(frm);
		break;
	case INTERNAL_CMD_CALIBRATION_NTC_CON_TABLE_REQ:
		doCalibrationNTCTableReq(frm);
		break;
	case INTERNAL_CMD_CALIBRATION_NTC_CONSTANT_REQ:
		doCalibrationNTCConstantReq(frm);
		break;
	case INTERNAL_CMD_CALIBRATION_NTC_CONSTANT_SET:
		doCalibrationNTCConstantSet(frm);
		break;
	}
}

static void response_board_type_req(struct internal_frame *frm)
{
	static uint8_t board_type = 1; /* NTC */

	frm->datalen = 1;
	frm->tx_data = &board_type;

	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));
}

static void response_slot_id_req(struct internal_frame *frm)
{
	frm->datalen = 1;
	frm->tx_data = &SysProperties.boardID;

	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));
}

static void response_temperature_req(struct internal_frame *frm)
{
	frm->datalen = sizeof(float) * 32;
	frm->tx_data = &TestData.Temperature[0][0].Float;

	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));
}

static void response_temperature_state_req(struct internal_frame *frm)
{
	frm->datalen = 16;
	frm->tx_data = TestData.displayModeFlag;

	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));
}

static void response_threshold_req(struct internal_frame *frm)
{
	frm->datalen = sizeof(float) * 32;
	frm->tx_data = &TestData.Threshold[0][0].UI8[0];

	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));
}

static void doTresholdSet(struct internal_frame *frm)
{
	/* uint8_t channel = *((uint8_t *) frm->threshold_set.channel); */
	/* float to_set = *((float *) (((uint8_t *) frm->threshold_set.value) + 1)); */
	uint8_t channel = frm->threshold_set.channel;
	float to_set = frm->threshold_set.value;

	/* if (frm->data) { */
	/* 	free(frm->data); */
	/* 	frm->data = NULL; */
	/* } */

	if (channel == 0xFF) {
		for (int j = 0; j < 2; j++) {
			for (int i = 0; i < 16; i++) {
				TestData.Threshold[j][i].Float = to_set;
			}
		}
	} else {
		TestData.Threshold[0][channel].Float = to_set;
	}

	frm->datalen = sizeof(float) * 32;
	frm->tx_data = &TestData.Threshold[0][0].UI8[0];

	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));

	doFlashWriteRevision();
}

static void doRevisionApplySet(struct internal_frame *frm)
{
	TestData.revisionApplyFlag = *((uint8_t *) &frm->rx_data[0]);

	/* if (frm->data) { */
	/* 	free(frm->data); */
	/* 	frm->data = NULL; */
	/* } */

	frm->datalen = 1;
	frm->tx_data = &TestData.revisionApplyFlag;
	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));

	doFlashWriteRevision();
}

static void doRevisionConstantSet(struct internal_frame *frm)
{
	TestData.revisionConstant.Float = *((float *) frm->rx_data);

	/* if (frm->data) { */
	/* 	free(frm->data); */
	/* 	frm->data = NULL; */
	/* } */

	frm->datalen = sizeof(float);
	frm->tx_data = &TestData.revisionConstant.UI8[0];
	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));

	doFlashWriteRevision();
}

static void doRevisionApplyReq(struct internal_frame *frm)
{
	frm->datalen = 1;
	frm->tx_data = &TestData.revisionApplyFlag;
	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));
}

static void doRevisionConstantReq(struct internal_frame *frm)
{
	frm->datalen = sizeof(float);
	frm->tx_data = &TestData.revisionConstant.Float;
	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));
}

static void doCalibrationNTCTableCal(struct internal_frame *frm)
{
	float rtd_temp = *((float *) frm->rx_data);

	/* if (frm->data) { */
	/* 	free(frm->data); */
	/* 	frm->data = NULL; */
	/* } */

	for (int j = 0; j < 2; j++) {
		for (int i = 0; i < 16; i++) {
			if (TestData.Temperature[j][i].Float != 0) {
				TestData.ntcCalibrationTable[j][i].Float = rtd_temp - TestData.Temperature[j][i].Float;
			}
		}
	}

	frm->datalen = 64;
	frm->tx_data = &TestData.ntcCalibrationTable[0][0].UI8[0];
	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));

	doFlashWriteRevision();
}

static void doCalibrationNTCConstantSet(struct internal_frame *frm)
{
	TestData.ntcCalibrationConstant.Float = *((float *) frm->rx_data);

	/* if (frm->data) { */
	/* 	free(frm->data); */
	/* 	frm->data = NULL; */
	/* } */

	frm->datalen = sizeof(float);
	frm->tx_data = &TestData.ntcCalibrationConstant.UI8[0];
	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));

        doFlashWriteRevision();
}

static void doCalibrationNTCTableReq(struct internal_frame *frm)
{
	frm->datalen = sizeof(float) * 16;
	frm->tx_data = &TestData.ntcCalibrationTable[0][0].UI8[0];
	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));
}

static void doCalibrationNTCConstantReq(struct internal_frame *frm)
{
	frm->datalen = sizeof(float);
	frm->tx_data = &TestData.ntcCalibrationConstant.UI8[0];
	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    HAL_GPIO_WritePin(UART1_TX_EN_GPIO_Port, UART1_TX_EN_Pin, GPIO_PIN_RESET); //TX 완료후 스위치IC OFF
}
