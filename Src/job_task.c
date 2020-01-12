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

static osMailQDef(job_pool_q, 16, struct job_s);
static osMailQId(job_pool_q_id);

static int tx_completed = 1;

static void job_handler(struct job_s *job);
static void receive_from(struct internal_frame *frm);
static void response_to(struct internal_frame *frm);

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
static void doRevisionTRConstSet(struct internal_frame *);
static void doRevisionTRConstReq(struct internal_frame *);
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
	job = (struct job_s *)osMailCAlloc(job_pool_q_id, osWaitForever);
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
		response_to(&job->internal);
		break;
	case JOB_TYPE_FROM_INTERNAL:
		receive_from(&job->internal);
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

static void response_to(struct internal_frame *frm)
{
	size_t frame_size = 0;
	static uint8_t buffer[255 + 7] = {0};

	if (tx_completed) {
		frame_size = fill_internal_frame(buffer, frm->slot_id, frm->cmd,
						frm->datalen, &frm->data);
		DBG_LOG("JOB_TYPE_TO: cmd %d\r\n", frm->cmd);
		DBG_DUMP(buffer, frame_size);

		tx_completed = 0;
		HAL_GPIO_WritePin(UART1_TX_EN_GPIO_Port, UART1_TX_EN_Pin, GPIO_PIN_SET);
		HAL_UART_Transmit_DMA(&huart1, buffer, frame_size);

		while (tx_completed == 0)
			__NOP();
	}
}

static void receive_from(struct internal_frame *frm)
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
	case INTERNAL_CMD_REVISION_CONST_REQ:
		doRevisionConstantReq(frm);
		break;
	case INTERNAL_CMD_REVISION_CONST_SET:
		doRevisionConstantSet(frm);
		break;
	case INTERNAL_CMD_REVISION_APPLY_REQ:
		doRevisionApplyReq(frm);
		break;
	case INTERNAL_CMD_REVISION_APPLY_SET:
		doRevisionApplySet(frm);
		break;
	case INTERNAL_CMD_REVISION_TR_CONST_REQ:
		doRevisionTRConstReq(frm);
		break;
	case INTERNAL_CMD_REVISION_TR_CONST_SET:
		doRevisionTRConstSet(frm);
		break;
	case INTERNAL_CMD_CALIBRATION_NTC_TABLE_CAL:
		doCalibrationNTCTableCal(frm);
		break;
	case INTERNAL_CMD_CALIBRATION_NTC_TABLE_REQ:
		doCalibrationNTCTableReq(frm);
		break;
	case INTERNAL_CMD_CALIBRATION_NTC_CONST_REQ:
		doCalibrationNTCConstantReq(frm);
		break;
	case INTERNAL_CMD_CALIBRATION_NTC_CONST_SET:
		doCalibrationNTCConstantSet(frm);
		break;
	}
}

static void response_board_type_req(struct internal_frame *frm)
{
	static uint8_t board_type = 1; /* NTC */

	frm->board_type.v = board_type;
	frm->datalen = sizeof(frm->board_type);

	post_job(JOB_TYPE_TO_INTERNAL, frm, frm->datalen + 3);
}

static void response_slot_id_req(struct internal_frame *frm)
{
	frm->slot.id = SysProperties.boardID;
	frm->datalen = sizeof(frm->slot);

	post_job(JOB_TYPE_TO_INTERNAL, frm, frm->datalen + 3);
}

static void response_temperature_req(struct internal_frame *frm)
{
	frm->datalen = sizeof(frm->temperatures);
	memcpy(&frm->temperatures, TestData.temperatures, frm->datalen);

	post_job(JOB_TYPE_TO_INTERNAL, frm, frm->datalen + 3);
}

static void response_temperature_state_req(struct internal_frame *frm)
{
	frm->datalen = sizeof(frm->channel_status);
	memcpy(&frm->channel_status, TestData.displayModeFlag, frm->datalen);

	post_job(JOB_TYPE_TO_INTERNAL, frm, frm->datalen + 3);
}

static void response_threshold_req(struct internal_frame *frm)
{
	frm->datalen = sizeof(frm->thresholds);
	memcpy(&frm->thresholds, TestData.thresholds, frm->datalen);

	post_job(JOB_TYPE_TO_INTERNAL, frm, frm->datalen + 3);
}

static void doTresholdSet(struct internal_frame *frm)
{
	uint8_t channel = frm->threshold_set.channel;
	float to_set = frm->threshold_set.value;

	if (channel == 0xFF) {
		for (int j = 0; j < 2; j++) {
			for (int i = 0; i < 16; i++) {
				TestData.thresholds[j][i] = to_set;
			}
		}
	} else {
		TestData.thresholds[0][channel] = to_set;
	}

	frm->datalen  = sizeof(frm->thresholds);
	/* for (int j = 0; j < 2; j++) { */
	/* 	for (int i = 0; i < 16; i++) { */
	/* 		frm->thresholds.v[j][i] = TestData.thresholds[j][i]; */
	/* 	} */
	/* } */
	memcpy(&frm->thresholds, TestData.thresholds, frm->datalen);
	/* DBG_DUMP(frm, frm->datalen + 3); */
	post_job(JOB_TYPE_TO_INTERNAL, frm, frm->datalen + 3);

	doFlashWriteRevision();
}

static void doRevisionApplySet(struct internal_frame *frm)
{
	TestData.revision_applied = frm->revision_apply.enabled;

	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(frm->revision_apply) + 3);

	doFlashWriteRevision();
}

static void doRevisionConstantSet(struct internal_frame *frm)
{
	TestData.revision_const = frm->revision_const.v;

	/* frm->datalen = sizeof(float); */
	/* frm->revision_constant_set = TestData.revisionConstant; */
	post_job(JOB_TYPE_TO_INTERNAL, frm, sizeof(frm->revision_const) + 3);

	doFlashWriteRevision();
}

static void doRevisionApplyReq(struct internal_frame *frm)
{
	frm->datalen = sizeof(frm->revision_apply);
	frm->revision_apply.enabled = TestData.revision_applied;
	post_job(JOB_TYPE_TO_INTERNAL, frm, frm->datalen + 3);
}

static void doRevisionConstantReq(struct internal_frame *frm)
{
	frm->datalen = sizeof(frm->revision_const);
	frm->revision_const.v = TestData.revision_const;
	post_job(JOB_TYPE_TO_INTERNAL, frm, frm->datalen + 3);
}

static void doRevisionTRConstReq(struct internal_frame *frm)
{
	frm->revision_tr_const.r1 = TestData.revision_tr1;
	frm->revision_tr_const.r2 = TestData.revision_tr2;
	frm->datalen = 8;
	post_job(JOB_TYPE_TO_INTERNAL, frm, frm->datalen + 3);

	doFlashWriteRevision();
}

static void doRevisionTRConstSet(struct internal_frame *frm)
{
	frm->revision_tr_const.r1 = TestData.revision_tr1;
	frm->revision_tr_const.r2 = TestData.revision_tr2;
	frm->datalen = 8;
	post_job(JOB_TYPE_TO_INTERNAL, frm, frm->datalen + 3);
}

static void doCalibrationNTCTableCal(struct internal_frame *frm)
{
	float rtd_temp = *((float *) frm->rx_data);

	for (int j = 0; j < CHANNELS_TYPE_NBR; j++) {
		for (int i = 0; i < CHANNELS; i++) {
			if (TestData.temperatures[j][i] != 0.f) {
				TestData.ntc_correction_tbl[j][i] = rtd_temp - TestData.temperatures[j][i];
			}
		}
	}

	frm->datalen = sizeof(TestData.ntc_correction_tbl);
	memcpy(&frm->ntc_correction_tbl, TestData.ntc_correction_tbl, frm->datalen);
	post_job(JOB_TYPE_TO_INTERNAL, frm, frm->datalen + 3);

	doFlashWriteRevision();
}

static void doCalibrationNTCConstantSet(struct internal_frame *frm)
{
	TestData.ntcCalibrationConstant = *((float *) frm->rx_data);

	frm->datalen = sizeof(float);
	frm->ntc_correction_const.v = TestData.ntcCalibrationConstant;
	post_job(JOB_TYPE_TO_INTERNAL, frm, frm->datalen + 3);

        doFlashWriteRevision();
}

static void doCalibrationNTCTableReq(struct internal_frame *frm)
{
	frm->datalen = sizeof(TestData.ntc_correction_tbl);
	memcpy(&frm->ntc_correction_tbl, TestData.ntc_correction_tbl, frm->datalen);
	post_job(JOB_TYPE_TO_INTERNAL, frm, frm->datalen + 3);
}

static void doCalibrationNTCConstantReq(struct internal_frame *frm)
{
	frm->datalen = sizeof(float);
	frm->ntc_correction_const.v = TestData.ntcCalibrationConstant;
	post_job(JOB_TYPE_TO_INTERNAL, frm, frm->datalen + 3);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_GPIO_WritePin(UART1_TX_EN_GPIO_Port, UART1_TX_EN_Pin, GPIO_PIN_RESET); //TX 완료후 스위치IC OFF
	tx_completed = 1;
}
