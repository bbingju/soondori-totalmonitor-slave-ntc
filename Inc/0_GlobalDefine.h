#ifndef __GLOBAL_DEFINE_H__
#define __GLOBAL_DEFINE_H__


/*********************************************************************
*		System Infomation
**********************************************************************/
#define _ON  	1
#define _OFF  0

/*********************************************************************
*		Bluetooth
**********************************************************************/
#define BLUETOOTH_HANDEL 					huart2

#define TRUE								1
#define FALSE								0

#define READY								0
#define BUSY								1

#define STANDBY								2
#define CONNECT								3

#define UART_TX_BUF_MAX 		    		120
#define UART_RX_BUF_MAX 		    		120
#define UART_RX_DMA_SIZE	        		14
#define BT_STX 								0x02
#define BT_ETX 								0x03

#define OPT_NULL							0x00

// to Slave UART
#define SEND_DATA_LENGTH					(uint8_t)12
#define CMD_BASE                			CMD_STX, (uint8_t)0x00, (uint8_t)0x00, (uint8_t)0x00, (uint8_t)0x00, (uint8_t)0x00, (uint8_t)0x00, (uint8_t)0x00, (uint8_t)0x00, CMD_ETX
#define CMD_STX     		    			(uint8_t)0x02
#define CMD_ETX     		    			(uint8_t)0x03
#define CMD_BOARD_TYPE		    			(uint8_t)'a'		//Board Type 요청
#define CMD_BOARD_EN_REQ					(uint8_t)'b'		//Board 사용 요청
#define CMD_BOARD_EN_SET					(uint8_t)'c'		//Board 사용 설정
#define CMD_BOARD_RESET		    			(uint8_t)'d'		//Board Reset
#define CMD_SLOT_ID_REQ						(uint8_t)'e'		//Slot 요청
#define CMD_HW_VER							(uint8_t)'f'		//HW Version 요청
#define CMD_FW_VER							(uint8_t)'g'		//FW Version 요청
#define CMD_UUID_REQ 						(uint8_t)'h'		//UUID 요청
#define CMD_TEMP_REQ 						(uint8_t)'i'		//Temp 요청
#define CMD_TEMP_STATE_REQ 					(uint8_t)'j'		//Temp 상태 요청
#define CMD_ADC_REQ							(uint8_t)'k'		//ADC 요청
#define CMD_THRESHOLD_REQ	    			(uint8_t)'l'		//경고온도 요청
#define CMD_THRESHOLD_SET	    			(uint8_t)'m'		//경고온도 설정
#define CMD_RELAY_REQ 		    			(uint8_t)'n'		//RELAY 상태 요청
#define CMD_RELAY_SET 		    			(uint8_t)'o'		//RELAY 사용 설정

#define CMD_REVISION_APPLY_SET     			(uint8_t)'p'		//보정 적용
#define CMD_REVISION_CONSTANT_SET			(uint8_t)'q'		//접촉상수 설정
#define CMD_REVISION_APPLY_REQ      		(uint8_t)'r'		//보정상태 요청
#define CMD_REVISION_CONSTANT_REQ			(uint8_t)'s'		//접촉상수 요청

#define CMD_CALIBRATION_NTC_CON_TABLE_CAL	(uint8_t)'t'		//NTC교정상수 계산해라
#define CMD_CALIBRATION_NTC_CONSTANT_SET	(uint8_t)'u'		//NTC교정상수 증감 설정
#define CMD_CALIBRATION_NTC_CON_TABLE_REQ	(uint8_t)'v'		//NTC교정상수 요청
#define CMD_CALIBRATION_NTC_CONSTANT_REQ	(uint8_t)'w'		//NTC교정증감 요청

#define CMD_REQ         	    			(uint8_t)0x01
#define CMD_SET         	    			(uint8_t)0x02



#define CMD_REQ         	    			(uint8_t)0x01
#define CMD_SET         	    			(uint8_t)0x02

#define OVER_TEMP							(1)
#define NORMAL_TEMP							(0)


extern void RESET_USART2_Init(void);


/*********************************************************************
*		0_StartGetSensorTask.c
**********************************************************************/

#define SENSOR_BUF_MAX			5380	//3축 + endcoder + switch = 8byte * 32개(16msec) * 21회 = 5376byte(336msec) 이며, 5376 + 4 로 버퍼 생성 
#define SENSING_STOP 			0
#define SENSING_START   		1
#define SENSING_SAVE			2
#define SENSING_START_SAVE		4

/*********************************************************************
*		0_Util.c
**********************************************************************/
//extern uint8_t StringCopyToArray(uint8_t* TagetArray, uint8_t* OriginalString, uint8_t CopyLength, uint8_t TotalLength);



#endif

