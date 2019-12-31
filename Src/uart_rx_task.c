#include "uart_rx_task.h"
#include "0_GlobalValue.h"
#include "job_task.h"
#include "stm32f1xx_ll_dma.h"
#include "cmsis_os.h"
#include "main.h"
#include "frame.h"
#include "debug.h"

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))

static void uart_rx_check(void);
static void parse_rx(const void *data, size_t len);

static osMessageQDef(uart_rx_msg_q, 4, uint32_t);
static osMessageQId(uart_rx_msg_q_id);

static uint8_t uart_rx_buffer[32];

extern UART_HandleTypeDef huart1;

void uart_rx_notify() { osMessagePut(uart_rx_msg_q_id, 1, 0); }

/**
 * @brief  Function implementing the UartRxTask.
 * @param  argument: Not used
 * @retval None
 */
void uart_rx_task(void const *argument)
{
	uart_rx_msg_q_id = osMessageCreate(osMessageQ(uart_rx_msg_q), NULL);

	DBG_LOG("%s is started\n", __func__);

	HAL_UART_Receive_DMA(&huart1, uart_rx_buffer, ARRAY_LEN(uart_rx_buffer));

	while (1) {
		osEvent e = osMessageGet(uart_rx_msg_q_id, osWaitForever);
		if (e.status == osEventMessage)
			uart_rx_check();
	}
}

/**
 * @brief  Check for new data received with DMA
 */
static void uart_rx_check(void)
{
	static size_t old_pos;
	size_t pos;

	/* Calculate current position in buffer */
	pos = ARRAY_LEN(uart_rx_buffer) - LL_DMA_GetDataLength(DMA1, LL_DMA_CHANNEL_5);
	if (pos != old_pos) {    /* Check change in received data */
		if (pos > old_pos) { /* Current position is over previous one */
			/* We are in "linear" mode */
			/* Process data directly by subtracting "pointers" */
			parse_rx(&uart_rx_buffer[old_pos], pos - old_pos);
		} else {
			/* uart_send_string("received data2\r\n"); */
			/* We are in "overflow" mode */
			/* First process data to the end of buffer */
			parse_rx(&uart_rx_buffer[old_pos], ARRAY_LEN(uart_rx_buffer) - old_pos);
			/* Check and continue with beginning of buffer */
			if (pos > 0) {
				parse_rx(&uart_rx_buffer[0], pos);
			}
		}
	}
	old_pos = pos; /* Save current position as old */

	/* Check and manually update if we reached end of buffer */
	if (old_pos == ARRAY_LEN(uart_rx_buffer)) {
		old_pos = 0;
	}
}

/**
 * @brief           Process received data over UART
 * @note            Either process them directly or copy to other bigger buffer
 * @param[in]       data: Data to process
 * @param[in]       len: Length in units of bytes
 */
static void parse_rx(const void *data, size_t len)
{
	/* DBG_DUMP(data, len); */
	static struct internal_frame frm = {0};

	for (int i = 0; i < len; i++) {
		if (parse_internal_frame(&frm, data + i)) {
			if (SysProperties.boardID == frm.slot_id)
				post_job(JOB_TYPE_FROM_INTERNAL, &frm, sizeof(frm));
		}
	}

}
