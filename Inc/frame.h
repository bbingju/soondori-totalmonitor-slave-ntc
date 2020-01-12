#ifndef INTERNAL_FRAME_H
#define INTERNAL_FRAME_H

#include <stdint.h>
#include "cmsis_os.h"
#include "cmsis_compiler.h"

/**
 * Commands for Internal Communication
 */
#define INTERNAL_CMD_BOARD_TYPE_REQ 0x01
#define INTERNAL_CMD_RESET 0x02
#define INTERNAL_CMD_SLOT_ID_REQ 0x03
#define INTERNAL_CMD_TEMPERATURE_REQ 0x04
#define INTERNAL_CMD_TEMPERATURE_STATE_REQ 0x05
#define INTERNAL_CMD_ADC_REQ 0x06
#define INTERNAL_CMD_THRESHOLD_REQ 0x07
#define INTERNAL_CMD_THRESHOLD_SET 0x08
#define INTERNAL_CMD_RELAY_REQ 0x09
#define INTERNAL_CMD_RELAY_SET 0x0A
#define INTERNAL_CMD_REVISION_CONST_REQ 0x0B
#define INTERNAL_CMD_REVISION_CONST_SET 0x0C
#define INTERNAL_CMD_REVISION_APPLY_REQ 0x0D
#define INTERNAL_CMD_REVISION_APPLY_SET 0x0E
#define INTERNAL_CMD_CALIBRATION_NTC_TABLE_CAL 0x0F
#define INTERNAL_CMD_CALIBRATION_NTC_TABLE_REQ 0x10
#define INTERNAL_CMD_CALIBRATION_NTC_CONST_REQ 0x11
#define INTERNAL_CMD_CALIBRATION_NTC_CONST_SET 0x12
#define INTERNAL_CMD_REVISION_TR_CONST_REQ 0x13
#define INTERNAL_CMD_REVISION_TR_CONST_SET 0x14

#define INT_STX 0xFE
#define INT_ETX 0xFD
#define EXT_STX 0x7F
#define EXT_ETX 0x7E

struct internal_frame {
	uint8_t slot_id;
	uint8_t cmd;
	uint8_t datalen;
	__PACKED_UNION {
		uint8_t data[129];
		uint8_t rx_data[129];

		__PACKED_STRUCT {
			uint8_t v;
		} board_type;

		__PACKED_STRUCT {
			uint8_t id;
		} slot;

		__PACKED_STRUCT {
			float v[2][16];
		} temperatures;

		__PACKED_STRUCT {
			float v[2][16];
		} thresholds;

		__PACKED_STRUCT {
			uint8_t channel;
			float   value;
		} threshold_set;

		__PACKED_STRUCT {
			uint8_t enabled;
		} revision_apply;

		__PACKED_STRUCT {
			float v;
		} revision_const;

		__PACKED_STRUCT {
			float r1;
			float r2;
		} revision_tr_const;

		__PACKED_STRUCT {
			uint8_t flag[16];
		} channel_status;

		__PACKED_STRUCT {
			float m[2][16];
		} ntc_correction_tbl;

		__PACKED_STRUCT {
			float v;
		} ntc_correction_const;
	};
};

struct external_frame_rx {
	uint8_t cmd;
	uint8_t option;
	uint8_t ipaddr[4];
	uint8_t data[22];
};

struct external_frame_tx {
	uint8_t cmd;
	uint8_t option;
	/* STX, ETX까지 포함된 길이. 즉, data length + 20. */
	uint32_t datalen;
	uint8_t ipaddr[4];
	uint8_t datetime[6];
	uint8_t *data;
};

#ifdef __cplusplus
extern "C" {
#endif

int fill_internal_frame(uint8_t *buffer, uint8_t slot_id, uint8_t cmd, uint8_t datalen, void *data);
int parse_internal_frame(struct internal_frame *frm, uint8_t const *byte);

int fill_external_tx_frame(uint8_t *buffer, uint8_t cmd, uint8_t option,
			uint8_t *ipaddr, uint8_t *datetime, uint8_t* data, size_t datalen);
int parse_external_rx_frame(struct external_frame_rx *frm, uint8_t const *bytestream);

__STATIC_INLINE const char *int_cmd_str(uint8_t cmd)
{
	switch (cmd) {
	case INTERNAL_CMD_BOARD_TYPE_REQ:
		return "INTERNAL_CMD_BOARD_TYPE_REQ";
		/* case INTERNAL_CMD_BOARD_EN_REQ: */
		/*     return "INTERNAL_CMD_BOARD_EN_REQ"; */
		/* case INTERNAL_CMD_BOARD_EN_SET: */
		/*     return "INTERNAL_CMD_BOARD_EN_SET"; */
	case INTERNAL_CMD_SLOT_ID_REQ:
		return "INTERNAL_CMD_SLOT_ID_REQ";
		/* case INTERNAL_CMD_HW_VER: */
		/*     return "INTERNAL_CMD_HW_VER"; */
		/* case INTERNAL_CMD_FW_VER: */
		/*     return "INTERNAL_CMD_FW_VER"; */
		/* case INTERNAL_CMD_UUID_REQ: */
		/*     return "INTERNAL_CMD_UUID_REQ"; */
	case INTERNAL_CMD_ADC_REQ:
		return "INTERNAL_CMD_ADC_REQ";
	case INTERNAL_CMD_RELAY_REQ:
		return "INTERNAL_CMD_RELAY_REQ";
	case INTERNAL_CMD_RELAY_SET:
		return "INTERNAL_CMD_RELAY_SET";
	case INTERNAL_CMD_REVISION_APPLY_SET:
		return "INTERNAL_CMD_REVISION_APPLY_SET";
	case INTERNAL_CMD_REVISION_CONST_SET:
		return "INTERNAL_CMD_REVISION_CONST_SET";
	case INTERNAL_CMD_REVISION_APPLY_REQ:
		return "INTERNAL_CMD_REVISION_APPLY_REQ";
	case INTERNAL_CMD_REVISION_CONST_REQ:
		return "INTERNAL_CMD_REVISION_CONST_REQ";
	case INTERNAL_CMD_REVISION_TR_CONST_REQ:
		return "INTERNAL_CMD_REVISION_TR_CONST_REQ";
	case INTERNAL_CMD_REVISION_TR_CONST_SET:
		return "INTERNAL_CMD_REVISION_TR_CONST_SET";
	case INTERNAL_CMD_CALIBRATION_NTC_CONST_SET:
		return "INTERNAL_CMD_CALIBRATION_NTC_CONST_SET";
	case INTERNAL_CMD_CALIBRATION_NTC_CONST_REQ:
		return "INTERNAL_CMD_CALIBRATION_NTC_CONST_REQ";
	case INTERNAL_CMD_TEMPERATURE_STATE_REQ:
		return "INTERNAL_CMD_TEMPERATURE_STATE_REQ";
	case INTERNAL_CMD_TEMPERATURE_REQ:
		return "INTERNAL_CMD_TEMPERATURE_REQ";
	case INTERNAL_CMD_THRESHOLD_REQ:
		return "INTERNAL_CMD_THRESHOLD_REQ";
	case INTERNAL_CMD_THRESHOLD_SET:
		return "INTERNAL_CMD_THRESHOLD_SET";
	case INTERNAL_CMD_CALIBRATION_NTC_TABLE_CAL:
		return "INTERNAL_CMD_CALIBRATION_NTC_TABLE_CAL";
	case INTERNAL_CMD_CALIBRATION_NTC_TABLE_REQ:
		return "INTERNAL_CMD_CALIBRATION_NTC_TABLE_REQ";
	default:
		return "";
	}
}

#ifdef __cplusplus
}
#endif

#endif /* INTERNAL_FRAME_H */
