/*! ----------------------------------------------------------------------------
 * @file    frame.c
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

#include "deca_headers.h"
#include <string.h>

#include "frame.h"

volatile uint16 elapsed_time = 0;
volatile Event last_event = UNKNOWN_EVENT;
volatile int tx_done = FALSE;
uint16 error_frame_count = 0;

uint16 init_frame_count=0;
uint16 poll_frame_count=0;
uint16 resp_frame_count=0;
uint16 final_frame_count=0;
uint16 shout_frame_count=0;

uint16 last_recv_frame_len = 0;
static uint8 tx_seq_num=0;


float get_link_metric(){   //Function to determine if the node is in LoS or not.
	/*
	 * pw_diff = 10log10( ( f1^2+f1^2+f3^2 )/ c ) - 170log10 (2)
	 * noise_to_amp_ratio = std_noise/f2
	 * metric = 100/(pw_diff*noise_to_amp_ratio)
	 */
	dwt_rxdiag_t cur_rx_diag;
	uint32 cir_pwr_reg;
	uint16 cir_pwr;
	float pw_diff;
	float noise_to_fp_ratio;
	float metric;

	dwt_readdiagnostics(&cur_rx_diag);
	cir_pwr_reg = dwt_read32bitoffsetreg(0x12, 0x04);
	cir_pwr = (uint16) ( cir_pwr_reg >> 16);
	pw_diff = -10.*log10( ( pow(cur_rx_diag.firstPathAmp1, 2.)+\
			 pow(cur_rx_diag.firstPathAmp2, 2.) +\
			 pow(cur_rx_diag.firstPathAmp3, 2.)
		    )/ cir_pwr
		  ) + 51.1751;
	noise_to_fp_ratio = (float) cur_rx_diag.stdNoise/cur_rx_diag.firstPathAmp2;
	metric = 100./(noise_to_fp_ratio * pw_diff);

	return metric;
}


static void set_common_frame_field(){
	tx_buf.chdr.frame_ctrl[0] = FRAME_CTRL_BYTE_0;
	tx_buf.chdr.frame_ctrl[1] = FRAME_CTRL_BYTE_1;
	tx_buf.chdr.seq_num = tx_seq_num++;
	memcpy(tx_buf.chdr.pan_id, pan_id, PAN_ID_LEN);
	tx_buf.chdr.source_addr = node_id;
}

void generate_init_frame(){
	set_common_frame_field();
	tx_buf.chdr.dest_addr = BROADCAST_ADDR_BYTE ;
	tx_buf.chdr.fcode = INIT_FCODE;
}

void generate_poll_frame(uint8 dest_addr){
	set_common_frame_field();
	tx_buf.chdr.dest_addr = dest_addr;
	tx_buf.chdr.fcode = POLL_FCODE;
}

void generate_resp_frame(uint8 dest_addr){
	set_common_frame_field();
	tx_buf.chdr.dest_addr = dest_addr;
	tx_buf.chdr.fcode = RESP_FCODE;
}

void generate_final_frame(uint8 dest_addr){
	set_common_frame_field();
	tx_buf.chdr.dest_addr = dest_addr;
	tx_buf.chdr.fcode = FINAL_FCODE;
}

void generate_shout_frame(){
	set_common_frame_field();
	tx_buf.chdr.dest_addr = BROADCAST_ADDR_BYTE ;
	tx_buf.chdr.fcode = SHOUT_FCODE;
}

/*------------------------------IRQ handlers--------------------------------------------------*/
static void cb_tx_ok(const dwt_cb_data_t *dwt_cb_data){
	tx_done = TRUE;
}

static void cb_rx_ok(const dwt_cb_data_t *dwt_cb_data){
	last_event = RX_OK;
	last_recv_frame_len = dwt_cb_data->datalength;
    dwt_readrxdata(rx_buf.frame_byte, last_recv_frame_len, 0);
    /**-------save the neighbor list-------**/
    if(rx_buf.chdr.source_addr<MAX_NODES){
    	neighbor_ts[ rx_buf.chdr.source_addr  ] = elapsed_time;
    	link_quality[ rx_buf.chdr.source_addr ] = 0.75*get_link_metric()+\
    									          0.25*link_quality[ rx_buf.chdr.source_addr ];
    }
    /*------------------------------------*/
    if(rx_buf.chdr.fcode == INIT_FCODE) init_frame_count++;
    else if(rx_buf.chdr.fcode == POLL_FCODE) poll_frame_count++;
    else if(rx_buf.chdr.fcode == RESP_FCODE) resp_frame_count++;
    else if(rx_buf.chdr.fcode == FINAL_FCODE) final_frame_count++;
    else if(rx_buf.chdr.fcode == SHOUT_FCODE) shout_frame_count++;
    else error_frame_count++;
}

static void cb_rx_tout(const dwt_cb_data_t *dwt_cb_data){
	last_event = RX_TOUT;
}

static void cb_rx_error(const dwt_cb_data_t *dwt_cb_data){
	dwt_forcetrxoff();
	dwt_rxreset();
	last_event = RX_ERROR;
	error_frame_count++;
}

void set_irq_handlers(){
	dwt_setcallbacks( &cb_tx_ok, &cb_rx_ok, &cb_rx_tout, &cb_rx_error );
	dwt_setinterrupt(
			  DWT_INT_TFRS
			| DWT_INT_RFCG
			| DWT_INT_RFTO
			| DWT_INT_RXPTO
			| DWT_INT_RPHE
			| DWT_INT_SFDT
			| DWT_INT_RFSL
			| DWT_INT_RFCE
			, 1);
}

uint64 read_timestamp(TS_TYPE ts_type){
	uint8 ts[5];
	uint64 ts_64 = 0;
	//uint32  debug_ts_32 ;
	if(ts_type == SYS_TS)
		dwt_readsystime(ts);
	else if(ts_type == TX_TS){
		//debug_ts_32 = dwt_readtxtimestamphi32();
		dwt_readtxtimestamp(ts);
	}
	else if(ts_type == RX_TS)
		dwt_readrxtimestamp(ts);
	int i;
    for (i = 4; i >= 0; i--)
    {
        ts_64 <<= 8;
        ts_64 |= ts[i];
    }
	return ts_64;
}


