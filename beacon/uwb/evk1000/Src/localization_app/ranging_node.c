/*! ----------------------------------------------------------------------------
 * @file    ranging_node.h
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

#include <ctype.h>

#include "deca_headers.h"
#include  "frame.h"
#include "utility.h"
#include "usb_util.h"

#include "ranging_node.h"

#undef USB_LOG

#define NLIST_LIVENESS MAX_NODES


#define LOG_BUF_LINE_LEN 	200

#ifdef USB_LOG
	static  char log_buf_line[LOG_BUF_LINE_LEN];
#endif


#define usb_response_buf_LEN 300

#define CMD_RANGE 		1
#define CMD_SEND_ID		2
#define	CMD_SEND_NLIST  3
#define CMD_SHOUT		4
#define CMD_SEND_LOG    5

#define USB_RESPONSE_BUF_LEN 600


static char lcd_line1[16];
static char lcd_line2[16];

static uint8 usb_tx_seq_no = 0;

//static uint8 host_cmd[USB_RX_BUF_LEN];

typedef struct _Range_Val{
	uint8 node_id;
	uint16 range_cm;
	/*---------------------*/
	uint64 poll_tx_ts;
	uint64 poll_rx_ts;
	uint64 resp_rx_ts;
	uint64 final_tx_ts;
	uint64 final_rx_ts;
}Range_Val;
static Range_Val* range_vals;
static int ranging_node_count = 0;

static uint32 poll_finishing_ts = UNKNOWN_TS;
static uint32 final_finishing_ts = UNKNOWN_TS;
static uint64 resp_tx_ts = UNKNOWN_TS;
static uint16 slot_time_uus = 0;
static int cur_usb_cmd_seq_no = 1;

int debug_delayed_scheduled = FALSE;

static uint8 usb_response_buf[DW_USB_TX_BUF_LEN];


void display_id_on_lcd(char* line2_txt){
	sprintf(lcd_line1, "Node: %u", node_id);
	strcpy(lcd_line2, line2_txt);
	print_lcd2(lcd_line1, lcd_line2);
}

int send_init(){
	dwt_forcetrxoff();
	//prepare frame
	generate_init_frame();
	tx_buf.frame_byte[PAYLOAD_INDX] = (uint8) (slot_time_uus/1000);
	tx_buf.frame_byte[PAYLOAD_INDX+1] = (uint8) ranging_node_count;
	int i;
	for(i=0; i<ranging_node_count; ++i){
		tx_buf.frame_byte[PAYLOAD_INDX+1+1+i] =  (uint8) range_vals[i].node_id;
	}

	//now transmit
	tx_done = FALSE;
	int payload_size = PAYLOAD_INDX+2+(1+1+ranging_node_count);
    dwt_writetxdata(payload_size, tx_buf.frame_byte, 0);
	dwt_writetxfctrl(payload_size, 0, 1);
    if(dwt_starttx(DWT_START_TX_IMMEDIATE)==DWT_ERROR){
    	return 1;
    }
    while(tx_done == FALSE) continue; //block on tx-completion
    tx_done = FALSE;
/*---------------------------------------------------------------------------------*/
#ifdef USB_LOG
	sprintf(log_buf_line,"LOG:%lu INIT sent\n", dwt_readtxtimestamphi32());
	usb_save_log(log_buf_line, strlen(log_buf_line));
#endif
/*---------------------------------------------------------------------------------*/
    return 0;
}

int wait_for_poll(){
	int recv_poll_count = 0;

	uint64 init_tx_ts_64 = read_timestamp(TX_TS);
	poll_finishing_ts = (uint32) ( (init_tx_ts_64 +
			      ( (uint64) ranging_node_count * slot_time_uus+slot_time_uus )*UUS_TO_DTU )>>8);
	int i;
	while(TRUE){
		dwt_setrxtimeout( 0 );
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
		Event cur_event = wait_till_event( poll_finishing_ts );
		if(cur_event == RX_OK){
			if( rx_buf.chdr.fcode == POLL_FCODE){
				for(i=0;i<ranging_node_count;++i){
					if(range_vals[i].node_id == rx_buf.chdr.source_addr){
						range_vals[i].poll_rx_ts = read_timestamp(RX_TS);
						++recv_poll_count;
/*---------------------------------------------------------------------------------*/
#ifdef USB_LOG
						sprintf(log_buf_line,"LOG:%lu rx POLL frm %u\n", dwt_readrxtimestamphi32(), rx_buf.chdr.source_addr);
						usb_save_log(log_buf_line, strlen(log_buf_line));
#endif
/*---------------------------------------------------------------------------------*/
					}
				}
			}
		}
		else if(cur_event == UNKNOWN_EVENT)
			break; // poll-rx time exceeded
	}
	return recv_poll_count;
}

int send_resp(){
	dwt_forcetrxoff();
	//prepare frame
	generate_resp_frame(BROADCAST_ADDR_BYTE);

	//now transmit
	uint64  poll_finishing_ts_64 = (uint64) poll_finishing_ts;
	uint32 delayed_ts = (uint32)((( poll_finishing_ts_64 << 8) + ( (uint64) TX_DELAY_IN_SLOT_UUS )*UUS_TO_DTU) >> 8);
	int i;
	int poll_count = 0;
	// add expected final-src node count and list here
	for(i=0; i<ranging_node_count; ++i){
		if(range_vals[i].poll_rx_ts != UNKNOWN_TS){
			tx_buf.frame_byte[PAYLOAD_INDX+1+poll_count] =  (uint8) range_vals[i].node_id;
			++poll_count;
		}
	}
	tx_buf.frame_byte[PAYLOAD_INDX]= poll_count;

	tx_done = FALSE;
	dwt_setdelayedtrxtime(delayed_ts);

	int payload_size = PAYLOAD_INDX+2+1+poll_count;
    dwt_writetxdata(payload_size, tx_buf.frame_byte, 0);
	dwt_writetxfctrl(payload_size, 0, 1);
    if(dwt_starttx(DWT_START_TX_IMMEDIATE)==DWT_ERROR) return 1;

    while(tx_done == FALSE) continue; //block on tx-completion
    tx_done = FALSE;
    /*---------------------------------------------------------------------------------*/
    #ifdef USB_LOG
    	sprintf(log_buf_line,"LOG:%lu RESP Sent\n", dwt_readtxtimestamphi32());
    	usb_save_log(log_buf_line, strlen(log_buf_line));
    #endif
    /*---------------------------------------------------------------------------------*/
	return 0;
}

int wait_for_final(uint8 final_frame_count){
	int final_count = 0;
	resp_tx_ts =  read_timestamp(TX_TS);
	//uint32 resp_tx_ts_32 = dwt_readtxtimestamphi32();
	//and start rx to wait for k finals
	final_finishing_ts = (uint32) ( (resp_tx_ts +
			      ( (uint64) final_frame_count * slot_time_uus+slot_time_uus )*UUS_TO_DTU )>>8);
	int i;
	while(TRUE){
		dwt_setrxtimeout( 0 );
		dwt_rxenable(DWT_START_RX_IMMEDIATE);
		Event cur_event = wait_till_event( final_finishing_ts );
		if(cur_event == RX_OK){
			if( rx_buf.chdr.fcode == FINAL_FCODE){
				//register the poll_rx_time
				for(i=0;i<ranging_node_count;++i){
					if(range_vals[i].node_id == rx_buf.chdr.source_addr){
						// save poll_tx_ts, resp_rx_ts, final_tx_ts
						range_vals[i].poll_tx_ts = convert_timestamp(&rx_buf.frame_byte[PAYLOAD_INDX+0*TIMESTAMP_LEN], TIMESTAMP_LEN);
						range_vals[i].resp_rx_ts = convert_timestamp(&rx_buf.frame_byte[PAYLOAD_INDX+1*TIMESTAMP_LEN], TIMESTAMP_LEN);
						range_vals[i].final_tx_ts= convert_timestamp(&rx_buf.frame_byte[PAYLOAD_INDX+2*TIMESTAMP_LEN], TIMESTAMP_LEN);
						range_vals[i].final_rx_ts = read_timestamp(RX_TS);
						++final_count;
/*---------------------------------------------------------------------------------*/
#ifdef USB_LOG
						sprintf(log_buf_line,
				       "LOG:%lu rx FINAL frm %u: p:%lu,%u, r:%lu,%u, f:%lu,%u => p:%lu,%u, r:%lu,%u, f:%lu,%u \n",
								dwt_readrxtimestamphi32(),
								rx_buf.chdr.source_addr,
								(uint32)(range_vals[i].poll_tx_ts>>8), (uint8)(range_vals[i].poll_tx_ts),
								(uint32)(range_vals[i].resp_rx_ts>>8), (uint8)(range_vals[i].resp_rx_ts),
								(uint32)(range_vals[i].final_tx_ts>>8), (uint8)(range_vals[i].final_tx_ts),
								(uint32)(range_vals[i].poll_rx_ts>>8), (uint8)(range_vals[i].poll_rx_ts),
								(uint32)(resp_tx_ts>>8), (uint8) (resp_tx_ts),
								(uint32)(range_vals[i].final_rx_ts>>8), (uint8)(range_vals[i].final_rx_ts)
								);

						usb_save_log(log_buf_line, strlen(log_buf_line));

#endif
/*---------------------------------------------------------------------------------*/
					}
				}
			}
		}
		else if(cur_event == UNKNOWN_EVENT) break; // final-rx time exceeded
	}

	return final_count;
}

void calculate_all_ranges(){
	int i;
	for(i=0;i<ranging_node_count;++i){
		if(   range_vals[i].poll_tx_ts != UNKNOWN_TS &&  range_vals[i].poll_rx_ts!= UNKNOWN_TS &&
			  resp_tx_ts != UNKNOWN_TS &&  range_vals[i].resp_rx_ts!= UNKNOWN_TS &&
			  range_vals[i].final_tx_ts != UNKNOWN_TS && range_vals[i].final_rx_ts!= UNKNOWN_TS
		)
			range_vals[i].range_cm = (uint32) 100*calculate_range(
					range_vals[i].poll_tx_ts,   range_vals[i].poll_rx_ts,
					resp_tx_ts,   range_vals[i].resp_rx_ts,
					range_vals[i].final_tx_ts, range_vals[i].final_rx_ts
			);
	}

}

int run_twr(){
	//send init
	if( send_init() != 0) {
/*---------------------------------------------------------------------------------*/
#ifdef USB_LOG
		sprintf(log_buf_line,"LOG:%lu INIT tx failed!\n", dwt_readsystimestamphi32());
		usb_save_log(log_buf_line, strlen(log_buf_line));
#endif
/*---------------------------------------------------------------------------------*/
		return 1;
	}
	//collect poll frames
	uint8 poll_count = wait_for_poll();
	if( poll_count == 0){
/*---------------------------------------------------------------------------------*/
#ifdef USB_LOG
		sprintf(log_buf_line,"LOG:%lu 0 POLL rx!\n", dwt_readsystimestamphi32());
		usb_save_log(log_buf_line, strlen(log_buf_line));
#endif
/*---------------------------------------------------------------------------------*/
		return 1;
	}
	//send resp
	if( send_resp() != 0 ){
/*---------------------------------------------------------------------------------*/
#ifdef USB_LOG
		sprintf(log_buf_line,"LOG:%lu RESP tx failed!\n", dwt_readsystimestamphi32());
		usb_save_log(log_buf_line, strlen(log_buf_line));
#endif
/*---------------------------------------------------------------------------------*/
		return 1;
	}
	//collect final frames
	if( wait_for_final(poll_count) >0 ) 	//compute ranges, save it to range_vals and return
		calculate_all_ranges();
	else{
/*---------------------------------------------------------------------------------*/
#ifdef USB_LOG
		sprintf(log_buf_line,"LOG:%lu 0 FINAL rx!\n", dwt_readsystimestamphi32());
		usb_save_log(log_buf_line, strlen(log_buf_line));
#endif
/*---------------------------------------------------------------------------------*/
		return 1;
	}
	return 0;
}

int deca_shout(){
	dwt_forcetrxoff();
	//prepare frame
	generate_shout_frame();

	//now transmit
	tx_done = FALSE;
	uint32 wait_till_ts = (uint32)((read_timestamp(SYS_TS)+ slot_time_uus* UUS_TO_DTU)>>8);
	int payload_size = PAYLOAD_INDX+2;
    dwt_writetxdata(payload_size, tx_buf.frame_byte, 0);
	dwt_writetxfctrl(payload_size, 0, 1);
    if(dwt_starttx(DWT_START_TX_IMMEDIATE)== DWT_ERROR){
    	return 1;
    }
    wait_till(wait_till_ts);
    return 0;
}

void process_host_cmd(uint8* host_cmd, int host_cmd_len){
	is_usb_data_available = FALSE;
	int i = 0;
	int j;
	int cmd_no = atoi( ( char* )host_cmd );
	ranging_node_count = 0;
	if(cmd_no == CMD_SEND_LOG){
		while(i<host_cmd_len && isdigit(host_cmd[i])) ++i; //skip the current number
		while(i<host_cmd_len && !isdigit(host_cmd[i])) ++i; //skip the blanks
		cur_usb_cmd_seq_no = atoi( ( char* )(host_cmd+i));
/*---------------------------------------------------------------------------------*/
#ifdef USB_LOG
		sprintf(log_buf_line,"LOG: EC:%u, I:%u, P:%u, R:%u, F:%u, S:%u\nGOLGOLGOL\n",
															error_frame_count,
															init_frame_count,
															poll_frame_count,
															resp_frame_count,
															final_frame_count,
															shout_frame_count);
		usb_save_log(log_buf_line, strlen(log_buf_line));
		usb_flush_log();
#endif
/*---------------------------------------------------------------------------------*/
		sprintf((char*)usb_response_buf, "%u: %u %u =  END\n", usb_tx_seq_no++,(uint8)CMD_SEND_LOG,
				                                           (uint8) cur_usb_cmd_seq_no);
		usb_write((char*)usb_response_buf, strlen((char*)usb_response_buf), TRUE);
	}

	else if(cmd_no == CMD_RANGE){
		//----disable poll+final call back---
		//------------------------------------
		while(i<host_cmd_len && isdigit(host_cmd[i])) ++i; //skip the current number
		while(i<host_cmd_len && !isdigit(host_cmd[i])) ++i; //skip the blanks
		cur_usb_cmd_seq_no = atoi( ( char* )(host_cmd+i));

		while(i<host_cmd_len && isdigit(host_cmd[i])) ++i; //skip the current number
		while(i<host_cmd_len && !isdigit(host_cmd[i])) ++i; //skip the blanks
		slot_time_uus = 1000*(uint16)atoi( ( char* )(host_cmd+i));

		while(i<host_cmd_len && isdigit(host_cmd[i])) ++i; //skip the current number
		while(i<host_cmd_len && !isdigit(host_cmd[i])) ++i; //skip the blanks
		ranging_node_count = atoi( ( char* )(host_cmd+i));


		if(ranging_node_count <=0){
/*---------------------------------------------------------------------------------*/
#ifdef USB_LOG
			sprintf(log_buf_line,"LOG: EC:%u, I:%u, P:%u, R:%u, F:%u, S:%u\nGOLGOLGOL\n",
																error_frame_count,
																init_frame_count,
																poll_frame_count,
																resp_frame_count,
																final_frame_count,
																shout_frame_count);
			usb_save_log(log_buf_line, strlen(log_buf_line));
			usb_flush_log();
#endif
/*---------------------------------------------------------------------------------*/

			sprintf((char*) usb_response_buf, "%u: %u %u =  END\n", usb_tx_seq_no++, (uint8)CMD_RANGE, (uint8) cur_usb_cmd_seq_no);
			usb_write((char*)usb_response_buf, strlen((char*)usb_response_buf), TRUE);
			return;
		}
		range_vals = (Range_Val*)malloc(ranging_node_count * sizeof(Range_Val));
//-------------------------------------------------------------------------------------------------//
#ifdef USB_LOG
		sprintf(log_buf_line,"LOG:%lu n-list: ", dwt_readsystimestamphi32() );
		usb_save_log(log_buf_line, strlen(log_buf_line));
#endif
//-------------------------------------------------------------------------------------------------//
		for(j=0;j<ranging_node_count;++j){
			while(i<host_cmd_len && isdigit(host_cmd[i])) ++i; //skip the current number
			while(i<host_cmd_len && !isdigit(host_cmd[i])) ++i; //skip the blanks

			range_vals[j].node_id = atoi( ( char* ) (host_cmd+i));
//-------------------------------------------------------------------------------------------------//
#ifdef USB_LOG
			sprintf(log_buf_line," %d, ", range_vals[j].node_id);
			usb_save_log(log_buf_line, strlen(log_buf_line));
#endif
//---------------------------------------------------------------------------------------------------//
			range_vals[j].range_cm = 0;
			range_vals[j].poll_tx_ts = UNKNOWN_TS;
			range_vals[j].poll_rx_ts = UNKNOWN_TS;
			range_vals[j].resp_rx_ts = UNKNOWN_TS;
			range_vals[j].final_tx_ts = UNKNOWN_TS;
			range_vals[j].final_rx_ts = UNKNOWN_TS;
		}
//-------------------------------------------------------------------------------------------------//
#ifdef USB_LOG
		sprintf(log_buf_line,"\n");
		usb_save_log(log_buf_line, strlen(log_buf_line));
#endif
//---------------------------------------------------------------------------------------------------//
		run_twr();
/*---------------------------------------------------------------------------------*/
#ifdef USB_LOG
		sprintf(log_buf_line,"LOG: EC:%u, I:%u, P:%u, R:%u, F:%u, S:%u\nGOLGOLGOL\n",
															error_frame_count,
															init_frame_count,
															poll_frame_count,
															resp_frame_count,
															final_frame_count,
															shout_frame_count);
		usb_save_log(log_buf_line, strlen(log_buf_line));
		usb_flush_log();
#endif
/*---------------------------------------------------------------------------------*/
		sprintf((char*) usb_response_buf, "%u: %u %u = ",
				usb_tx_seq_no++, (uint8)CMD_RANGE, (uint8) cur_usb_cmd_seq_no);
		char temp_buf[20];
		for(i=0; i<ranging_node_count;++i){
			if(i==0)
				sprintf(temp_buf, " %u %d", range_vals[i].node_id, range_vals[i].range_cm);
			else
				sprintf(temp_buf, ", %u %d", range_vals[i].node_id, range_vals[i].range_cm);
			strcat((char*)usb_response_buf, temp_buf);
		}
		strcat((char*)usb_response_buf, " END\n");
		usb_write((char*)usb_response_buf, strlen((char*)usb_response_buf), TRUE);
		//free all allocated memory
		free(range_vals);
		//------------------------------------
	}
	else if(cmd_no == CMD_SEND_ID){
		while(i<host_cmd_len && isdigit(host_cmd[i])) ++i; //skip the current number
		while(i<host_cmd_len && !isdigit(host_cmd[i])) ++i; //skip the blanks
		cur_usb_cmd_seq_no = atoi( ( char* )(host_cmd+i));

		while(i<host_cmd_len && isdigit(host_cmd[i])) ++i; //skip the current number
		while(i<host_cmd_len && !isdigit(host_cmd[i])) ++i; //skip the blanks
		int ip3 = atoi( ( char* )(host_cmd+i));

		while(i<host_cmd_len && isdigit(host_cmd[i])) ++i; //skip the current number
		while(i<host_cmd_len && !isdigit(host_cmd[i])) ++i; //skip the blanks
		int ip2 = atoi( ( char* )(host_cmd+i));

		while(i<host_cmd_len && isdigit(host_cmd[i])) ++i; //skip the current number
		while(i<host_cmd_len && !isdigit(host_cmd[i])) ++i; //skip the blanks
		int ip1 = atoi( ( char* )(host_cmd+i));

		while(i<host_cmd_len && isdigit(host_cmd[i])) ++i; //skip the current number
		while(i<host_cmd_len && !isdigit(host_cmd[i])) ++i; //skip the blanks
		int ip0 = atoi( ( char* )(host_cmd+i));


		error_frame_count =  init_frame_count = poll_frame_count = resp_frame_count = final_frame_count = shout_frame_count = 0;
/*---------------------------------------------------------------------------------*/
#ifdef USB_LOG
		sprintf(log_buf_line,"LOG: EC:%u, I:%u, P:%u, R:%u, F:%u, S:%u\nGOLGOLGOL\n",
															error_frame_count,
															init_frame_count,
															poll_frame_count,
															resp_frame_count,
															final_frame_count,
															shout_frame_count);
		usb_save_log(log_buf_line, strlen(log_buf_line));
		usb_flush_log();
#endif
/*---------------------------------------------------------------------------------*/
		sprintf((char*)usb_response_buf, "%u: %u %u = %u END\n", usb_tx_seq_no++,(uint8)CMD_SEND_ID,
				                                           (uint8) cur_usb_cmd_seq_no, node_id);
		usb_write((char*)usb_response_buf, strlen((char*)usb_response_buf), TRUE);
		sprintf(lcd_line1, "Node ID: %u", node_id);
		sprintf(lcd_line2, "%u.%u.%u.%u", (uint8)ip3, (uint8)ip2, (uint8)ip1, (uint8)ip0 );
		//display_id_on_lcd("Node ID Sent");
		print_lcd2(lcd_line1, lcd_line2);
		Sleep(100);
	}
	else if(cmd_no == CMD_SEND_NLIST){
		while(i<host_cmd_len && isdigit(host_cmd[i])) ++i; //skip the current number
		while(i<host_cmd_len && !isdigit(host_cmd[i])) ++i; //skip the blanks
		cur_usb_cmd_seq_no = atoi( ( char* )(host_cmd+i));

/*---------------------------------------------------------------------------------*/
#ifdef USB_LOG
		sprintf(log_buf_line,"LOG: EC:%u, I:%u, P:%u, R:%u, F:%u, S:%u\nGOLGOLGOL\n",
															error_frame_count,
															init_frame_count,
															poll_frame_count,
															resp_frame_count,
															final_frame_count,
															shout_frame_count);
		usb_save_log(log_buf_line, strlen(log_buf_line));
		usb_flush_log();
#endif
/*---------------------------------------------------------------------------------*/

		sprintf((char*)usb_response_buf, "%u: %u %u = ", usb_tx_seq_no++,
				              (uint8)CMD_SEND_NLIST, (uint8) cur_usb_cmd_seq_no );
		char temp_buf[20];
		int total_neighbors = 0;
		for(i=0; i<MAX_NODES;++i){
			if( neighbor_ts[i]==0 ) continue;
			if(total_neighbors==0)
				sprintf(temp_buf, "%u %u %d", i,
						(elapsed_time - neighbor_ts[i]),
						(int) link_quality[i] );
			else
				sprintf(temp_buf, ", %u %u %d", i,
						(elapsed_time - neighbor_ts[i]),
						(int) link_quality[i] );
			strcat((char*)usb_response_buf, temp_buf);
			++total_neighbors;
		}
		strcat((char*)usb_response_buf, " END\n");
		usb_write((char*)usb_response_buf, strlen((char*)usb_response_buf), TRUE);
	}
	else if(cmd_no == CMD_SHOUT){
		int status = deca_shout();
/*---------------------------------------------------------------------------------*/
#ifdef USB_LOG
		sprintf(log_buf_line,"LOG: EC:%u, I:%u, P:%u, R:%u, F:%u, S:%u\nGOLGOLGOL\n",
															error_frame_count,
															init_frame_count,
															poll_frame_count,
															resp_frame_count,
															final_frame_count,
															shout_frame_count);
		usb_save_log(log_buf_line, strlen(log_buf_line));
		usb_flush_log();
#endif
/*---------------------------------------------------------------------------------*/
		sprintf((char*)usb_response_buf, "%u: %u %u = %u END\n", usb_tx_seq_no++,
															(uint8)CMD_SHOUT,
				                                            (uint8) cur_usb_cmd_seq_no,
															 status);
		usb_write((char*)usb_response_buf, strlen((char*)usb_response_buf), TRUE);
	}
}

void process_init_frame(){
	int i;
	slot_time_uus = (uint16)rx_buf.frame_byte[PAYLOAD_INDX]*1000;
	int cur_node_count =  (uint8) rx_buf.frame_byte[PAYLOAD_INDX+1];
	for(i=0; i<cur_node_count;++i ){
		if(node_id == rx_buf.frame_byte[PAYLOAD_INDX+1+1+i]){
/*---------------------------------------------------------------------------------*/
#ifdef USB_LOG
			sprintf(log_buf_line,"LOG:%lu INIT rx frm %u \n", dwt_readrxtimestamphi32(), rx_buf.chdr.source_addr );
			usb_save_log(log_buf_line, strlen(log_buf_line));
#endif
/*---------------------------------------------------------------------------------*/
			dwt_forcetrxoff();
			//schedule poll_frame
			generate_poll_frame( rx_buf.chdr.source_addr );
			uint64 init_rx_ts = read_timestamp(RX_TS);
			uint64 cur_delay =  ( (uint64)(i+1)  * slot_time_uus )* UUS_TO_DTU;
			uint32 delayed_ts = (uint32) (   ( init_rx_ts+ cur_delay ) >> 8 );
			tx_done  = FALSE;
			dwt_setdelayedtrxtime(delayed_ts);
			int payload_size = PAYLOAD_INDX+2;
		    dwt_writetxdata(payload_size, tx_buf.frame_byte, 0);
			dwt_writetxfctrl(payload_size, 0, 1);

			if(dwt_starttx( DWT_START_TX_DELAYED ) == DWT_ERROR){
/*---------------------------------------------------------------------------------*/
#ifdef USB_LOG
				uint32 debug_cur_ts = dwt_readsystimestamphi32();
				sprintf(log_buf_line,"LOG:%lu NO POLL tx (%lu)!!\n", debug_cur_ts, delayed_ts);
				usb_save_log(log_buf_line, strlen(log_buf_line));
#endif
/*---------------------------------------------------------------------------------*/
				Sleep(1);
				return;
			}

			while(tx_done == FALSE) continue; //block till poll tx is done
			Sleep(1);
/*---------------------------------------------------------------------------------*/
#ifdef USB_LOG
			sprintf(log_buf_line,"LOG:%lu POLL tx\n", dwt_readtxtimestamphi32());
		    usb_save_log(log_buf_line, strlen(log_buf_line));
#endif
/*---------------------------------------------------------------------------------*/
		}
	}
}

void process_resp_frame(){
	int i;
	int cur_node_count =  (uint8) rx_buf.frame_byte[PAYLOAD_INDX];
	for(i=0; i<cur_node_count;++i ){
		if(node_id == rx_buf.frame_byte[PAYLOAD_INDX+1+i]){
/*---------------------------------------------------------------------------------*/
#ifdef USB_LOG
			sprintf(log_buf_line,"LOG:%lu RESP rx frm %u\n", dwt_readrxtimestamphi32(), rx_buf.chdr.source_addr );
			usb_save_log(log_buf_line, strlen(log_buf_line));
#endif
/*---------------------------------------------------------------------------------*/
			dwt_forcetrxoff();
			//schedule final_frame
			generate_final_frame( rx_buf.chdr.source_addr );

			uint64 cur_delay =  ( (uint64)(i+1)  * slot_time_uus  )* UUS_TO_DTU;

			uint64 poll_tx_ts = read_timestamp(TX_TS);
			uint64 resp_rx_ts = read_timestamp(RX_TS);
			uint32 final_tx_ts_32 =  (uint32) ((  resp_rx_ts+cur_delay )>>8) ;
			uint64 final_tx_ts = (  (  (uint64)(final_tx_ts_32 & 0xFFFFFFFEUL) ) << 8  ) + ANTENNA_DELAY_UUS;

			//now put the ts on payload
			set_frame_timestamp(&tx_buf.frame_byte[PAYLOAD_INDX+0*TIMESTAMP_LEN], poll_tx_ts ); //poll_tx_ts
			set_frame_timestamp(&tx_buf.frame_byte[PAYLOAD_INDX+1*TIMESTAMP_LEN], resp_rx_ts ); //resp_rx_ts
			set_frame_timestamp(&tx_buf.frame_byte[PAYLOAD_INDX+2*TIMESTAMP_LEN], final_tx_ts ); //final_tx_ts

			tx_done = FALSE;
			dwt_setdelayedtrxtime(final_tx_ts_32);
			int payload_size = PAYLOAD_INDX+2+3*TIMESTAMP_LEN;
		    dwt_writetxdata(payload_size, tx_buf.frame_byte, 0);
			dwt_writetxfctrl(payload_size, 0, 1);
			if(dwt_starttx( DWT_START_TX_DELAYED)==DWT_ERROR){
/*---------------------------------------------------------------------------------*/
#ifdef USB_LOG
				sprintf(log_buf_line,"LOG:%lu NO FINAL tx!!\n", dwt_readsystimestamphi32());
				usb_save_log(log_buf_line, strlen(log_buf_line));
#endif
/*---------------------------------------------------------------------------------*/
				Sleep(1);
				return;
			}
			while(tx_done == FALSE) continue; //block till final tx is done
/*---------------------------------------------------------------------------------*/
			Sleep(1);
#ifdef USB_LOG
			sprintf(log_buf_line,"LOG:%lu FINAL tx\n", dwt_readtxtimestamphi32());
			usb_save_log(log_buf_line, strlen(log_buf_line));
#endif
/*---------------------------------------------------------------------------------*/
		}
	}
}



void handle_frame_event(){
	if(last_event == RX_OK){
	    if(rx_buf.chdr.fcode == INIT_FCODE) {
	    	process_init_frame();
	    }
	    else  if(rx_buf.chdr.fcode == RESP_FCODE) {
	    	process_resp_frame();
	    }
	}
	last_event = UNKNOWN_EVENT;
	dwt_setrxtimeout( 0 );
	dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void run_ranging_node(){
	init_decawave_device(TRUE);

	reset_usb_log();

	sprintf(lcd_line1, "Node: %u", node_id);
	sprintf(lcd_line2, "...............");
	print_lcd2(lcd_line1, lcd_line2);
	Sleep(500);

	dwt_forcetrxoff();
	dwt_setrxtimeout( 0 );
	dwt_rxenable(DWT_START_RX_IMMEDIATE);

	while(TRUE){
		if(last_event != UNKNOWN_EVENT){
			handle_frame_event();
		}
		else if(is_usb_data_available == TRUE){
			process_host_cmd(usb_rx_buf, usb_rx_buf_len);
		}
	}
}

