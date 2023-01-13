/*! ----------------------------------------------------------------------------
 * @file    frame.h
 * @brief
 *
 * @attention
 *
 * Copyright 2019 (c) NEC Labs America
 *
 * All rights reserved.
 *
 * @author Md Shaifur Rahman
 */

#ifndef _FRAME_
#define _FRAME_

#ifdef __cplusplus
extern "C" {
#endif
/* function prototype starts here  */
	#include "deca_headers.h"

#define USB_LOG	1 //logging enable
#define CIR_DATA_LEN 4065

/*-------Common definitions-----*/
	#define SPEED_OF_LIGHT      			(299702547.0)     // in m/s in air
	#define CLOCK_DRIFT_UUS					(2000)
	#define UINT32_MAX_HALF					(2147483647)
	//#define UINT64_MAX			  (0xFFFFFFFFFFFFFFFF)
	#define TIMESTAMP40_MAX_VAL				(0xFFFFFFFFFF)
/*--------------------------------------delay params------------------------------------------------------*/
	#define	UUS_TO_DTU 						(65536)
	#define ANTENNA_DELAY_UUS 				(16436)

	#define PRE_TIMEOUT_UUS 		(8)
	#define TX_DELAY_IN_SLOT_UUS	(500)
//	#define TX_TO_RX_UUS			(150)
//	#define RX_TO_TX_UUS			(4000)	//(3500)
//	#define RX_TIMEOUT_UUS 			(8000)	//(4500)

	#define TIMESTAMP_LEN			(5) //in bytes
	#define UNKNOWN_TS				0
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

	#define FRAME_CTRL_LEN		(2)
	#define ADDR_LEN 			(1)
	#define PAN_ID_LEN			(2)
	#define PAYLOAD_INDX		( FRAME_CTRL_LEN + 1 + PAN_ID_LEN + ADDR_LEN + ADDR_LEN + 1 )

	#define FRAME_CTRL_BYTE_0   0x41
	#define FRAME_CTRL_BYTE_1	0x88

	#define INIT_FCODE 			0x01
	#define POLL_FCODE			0x02
	#define RESP_FCODE			0x03
	#define	FINAL_FCODE			0x04
	#define SHOUT_FCODE			0x05

	#define BROADCAST_ADDR_BYTE 	0xFF

	#define MAX_NODES 				255  // maximum supported node, node id must be 0 through 255
	#define MAX_NODE_PAIRS		( (  (MAX_NODES)*( (MAX_NODES)-1)  )/2 )

uint8 ntm;
typedef struct{
	uint8 frame_ctrl[FRAME_CTRL_LEN];
	uint8 seq_num;
	uint8 pan_id[PAN_ID_LEN];
	//uint8 dest_addr[ADDR_LEN];
	uint8 dest_addr;   //Node number destination.
	//uint8 source_addr[ADDR_LEN];
	uint8 source_addr;    //Node number source.
	uint8 fcode; //Type of frame e.g., poll, resp, final.
} Common_Header;

	#define MAX_FRAME_LEN	(127)

typedef union{
	uint8 frame_byte[MAX_FRAME_LEN];
	Common_Header chdr;
}Frame_Buffer;

Frame_Buffer tx_buf;
Frame_Buffer rx_buf;
//uint8 last_recv_src_addr[ADDR_LEN];

uint8 pan_id[PAN_ID_LEN];
uint8 node_id;
//uint8 my_addr[ADDR_LEN];


void set_irq_handlers();

void generate_init_frame();
void generate_poll_frame(uint8 dest_addr);
void generate_resp_frame(uint8 dest_addr);
void generate_final_frame(uint8 dest_addr);
void generate_shout_frame();


extern uint16 error_frame_count;
/*------------callback-related------------------------------*/
typedef enum{
	RX_OK,
	RX_TOUT,
	RX_ERROR,
	UNKNOWN_EVENT
}Event;

extern volatile Event last_event;
extern volatile int tx_done;

extern uint16 last_recv_frame_len;

extern uint16 init_frame_count;
extern uint16 poll_frame_count;
extern uint16 resp_frame_count;
extern uint16 final_frame_count;
extern uint16 shout_frame_count;

//----------neighbor list-------//
uint16 neighbor_ts[MAX_NODES];
float link_quality[MAX_NODES];
extern volatile uint16 elapsed_time ;
//--------------------------------------//

typedef enum {SYS_TS, TX_TS, RX_TS}TS_TYPE;
uint64 read_timestamp(TS_TYPE ts_type);

#ifdef __cplusplus
}
#endif

#endif /* _FRAME_ */
