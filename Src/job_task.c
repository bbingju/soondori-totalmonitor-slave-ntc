#include "job_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "internal_frame.h"
#include "routine_task.h"
#include "0_soonFlashMemory.h"
#include "0_Util.h"
#include "debug.h"

#include <string.h>

struct job_s {
	JOB_TYPE_E type;
	struct internal_frame internal;
};

static osMailQDef(job_pool_q, 8, struct job_s);
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

	HAL_GPIO_WritePin(UART1_TX_EN_GPIO_Port, UART1_TX_EN_Pin, GPIO_PIN_SET);

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
 * do job
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
		/* DBG_LOG("JOB_TYPE_ROUTINE_MEASURE_TEMPERATURE\n"); */
		read_sensors();
		break;
	case JOB_TYPE_ROUTINE_CHECK_TEMP_OVER:
		/* DBG_LOG("JOB_TYPE_ROUTINE_CHECK_TEMP_OVER\n"); */
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
					frm->datalen, frm->data);
	DBG_LOG("JOB_TYPE_TO_INTERNAL:");
	DBG_DUMP(buffer, frame_size);

	/* if (frm->data) { */
	/*   free(frm->data); */
	/*   frm->data = NULL; */
	/* } */

	HAL_GPIO_WritePin(UART1_TX_EN_GPIO_Port, UART1_TX_EN_Pin, GPIO_PIN_SET);
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
	frm->data = &board_type;

	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));
}

static void response_slot_id_req(struct internal_frame *frm)
{
	frm->datalen = 1;
	frm->data = &SysProperties.boardID;

	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));
}

static void response_temperature_req(struct internal_frame *frm)
{
	frm->datalen = sizeof(float) * 32;
	frm->data = &TestData.Temperature[0][0].Float;

	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));
}

static void response_temperature_state_req(struct internal_frame *frm)
{
	frm->datalen = 16;
	frm->data = TestData.displayModeFlag;

	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));
}

static void response_threshold_req(struct internal_frame *frm)
{
	frm->datalen = sizeof(float) * 32;
	frm->data = &TestData.Threshold[0][0].UI8[0];

	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));
}

static void doTresholdSet(struct internal_frame *frm)
{
	uint8_t channel = *((uint8_t *) frm->data);
	float to_set = *((float *) (((uint8_t *) frm->data) + 1));

	if (frm->data) {
		free(frm->data);
		frm->data = NULL;
	}

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
	frm->data = &TestData.Threshold[0][0].UI8[0];

	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));

	doFlashWriteRevision();
}

static void doRevisionApplySet(struct internal_frame *frm)
{
	TestData.revisionApplyFlag = *((uint8_t *) &frm->data[0]);

	if (frm->data) {
		free(frm->data);
		frm->data = NULL;
	}

	frm->datalen = 1;
	frm->data = &TestData.revisionApplyFlag;
	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));

	doFlashWriteRevision();
}

static void doRevisionConstantSet(struct internal_frame *frm)
{
	TestData.revisionConstant.Float = *((float *) frm->data);

	if (frm->data) {
		free(frm->data);
		frm->data = NULL;
	}

	frm->datalen = 4;
	frm->data = &TestData.revisionConstant.UI8[0];
	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));

	doFlashWriteRevision();
}

static void doRevisionApplyReq(struct internal_frame *frm)
{
	frm->datalen = 1;
	frm->data = &TestData.revisionApplyFlag;
	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));
}

static void doRevisionConstantReq(struct internal_frame *frm)
{
	frm->datalen = sizeof(float);
	frm->data = &TestData.revisionConstant.Float;
	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));
}

static void doCalibrationNTCTableCal(struct internal_frame *frm)
{
	float rtd_temp = *((float *) frm->data);

	if (frm->data) {
		free(frm->data);
		frm->data = NULL;
	}

	for (int j = 0; j < 2; j++) {
		for (int i = 0; i < 16; i++) {
			if (TestData.Temperature[j][i].Float != 0) {
				TestData.ntcCalibrationTable[j][i].Float = rtd_temp - TestData.Temperature[j][i].Float;
			}
		}
	}

	frm->datalen = 64;
	frm->data = &TestData.ntcCalibrationTable[0][0].UI8[0];
	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));

	doFlashWriteRevision();
}

static void doCalibrationNTCConstantSet(struct internal_frame *frm)
{
	TestData.ntcCalibrationConstant.Float = *((float *) frm->data);

	if (frm->data) {
		free(frm->data);
		frm->data = NULL;
	}

	frm->datalen = sizeof(float);
	frm->data = &TestData.ntcCalibrationConstant.UI8[0];
	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));

        doFlashWriteRevision();
}

static void doCalibrationNTCTableReq(struct internal_frame *frm)
{
	frm->datalen = 64;
	frm->data = &TestData.ntcCalibrationTable[0][0].UI8[0];
	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));
}

static void doCalibrationNTCConstantReq(struct internal_frame *frm)
{
	frm->datalen = 4;
	frm->data = &TestData.ntcCalibrationConstant.UI8[0];
	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(struct internal_frame));
}