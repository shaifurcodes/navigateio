/*! ----------------------------------------------------------------------------
 * @file    utility.c
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
#include "frame.h"

#include "utility.h"

#define MAX_CHAR_PER_LCD_LINE 16
static int led_toggle = 0;

static dwt_config_t config = {
	2,               /* Channel number. */
	DWT_PRF_64M,     /* Pulse repetition frequency. */
	DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
	DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
	9,               /* TX preamble code. Used in TX only. */
	9,               /* RX preamble code. Used in RX only. */
	1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
	DWT_BR_110K,     /* Data rate. */
	DWT_PHRMODE_STD, /* PHY header mode. */
	(1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

static void init_lcd(void)
{
    uint8_t initseq[9] = { 0x39, 0x14, 0x55, 0x6D, 0x78, 0x38 /*0x3C*/, 0x0C, 0x01, 0x06 };
    uint8_t command = 0x0;
    int j = 100000;

    writetoLCD( 9, 0,  initseq); //init seq
    while(j--);

    command = 0x2 ;  //return cursor home
    writetoLCD( 1, 0,  &command);
    command = 0x1 ;  //clear screen
    writetoLCD( 1, 0,  &command);
}

static uint8 read_s1_switch(void){
	uint8 s1_switch_byte = 0x0
			|port_is_switch_on(TA_SW1_3) << 0
    		| port_is_switch_on(TA_SW1_4) << 1
    		| port_is_switch_on(TA_SW1_5) << 2
		    | port_is_switch_on(TA_SW1_6) << 3
    		| port_is_switch_on(TA_SW1_7) << 4
    		| port_is_switch_on(TA_SW1_8) << 5;
    return s1_switch_byte;
}

void tim2_handler_func(){
	if(led_toggle%2 ==0) led_on(LED_ALL);
	else led_off(LED_ALL);
	++led_toggle;
	/*----------------*/
	++elapsed_time;
}

static void init_frame_params(){
	node_id = read_s1_switch();
	int i=0;

//	for(i=0; i<ADDR_LEN;++i)
//		my_addr[i] = 0x00;
//	my_addr[0] = node_id;

	for(i=0; i<PAN_ID_LEN;++i)
		pan_id[i] = 0x00;
	pan_id[0] = 'N';
	pan_id[1] = 'E';
}

void increase_tx_power(){
	uint32 smtx_reg = dwt_read32bitreg(SYS_CFG_ID);
	uint32 tx_power_setting = dwt_read32bitreg (TX_POWER_ID);

	smtx_reg = smtx_reg | (SYS_CFG_DIS_STXP);
	tx_power_setting = (tx_power_setting & 0xFF0000FF) | (0x001F1F00);

	dwt_write32bitreg (TX_POWER_ID, tx_power_setting);
	dwt_write32bitreg(SYS_CFG_ID, smtx_reg);
}

void init_decawave_device(int enable_irq ){

    port_set_deca_isr(dwt_isr);
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    init_lcd();

    port_set_dw1000_slowrate();
    dwt_initialise(DWT_LOADUCODE); //<-----without error checking--->
    port_set_dw1000_fastrate();

    dwt_configure(&config);

    if(ANTENNA_DELAY_UUS>0){
    	 dwt_setrxantennadelay(ANTENNA_DELAY_UUS);
    	 dwt_settxantennadelay(ANTENNA_DELAY_UUS);
    }

    increase_tx_power();

    if(enable_irq == TRUE)
    	set_irq_handlers();

    init_frame_params();
    ntm =  (uint8) (dwt_read8bitoffsetreg(0x2E, 0x0806) & 0x0F);
    //enable_next_hops = TRUE;
    tim2_handler = &tim2_handler_func;
	print_lcd2("Decawave-DW1000", "configuring.....");
	Sleep(500);
}


static char* pad_blanks(const char* str ){
	char*line_str =  (char*) malloc(MAX_CHAR_PER_LCD_LINE);
	int line_str_len = strlen(str);

	if ( line_str_len <= MAX_CHAR_PER_LCD_LINE){
		strcpy(line_str, str);
		int i;
		for(i=line_str_len; i<MAX_CHAR_PER_LCD_LINE; ++i){
			line_str[i] = 32; /*ASCII 32 = space char*/
		}
	}
	else{
		strncpy(line_str, str, MAX_CHAR_PER_LCD_LINE);
	}
    return line_str;
}



void print_lcd(const char* str){
	char* lcd_line1_str = pad_blanks(str) ;
    lcd_display_str(lcd_line1_str);
    free(lcd_line1_str);
}


void print_lcd2(const char* str1, const char* str2){
	char* lcd_line1_str = pad_blanks(str1) ;
	char* lcd_line2_str = pad_blanks(str2) ;
    lcd_display_str2(lcd_line1_str, lcd_line2_str);
    free(lcd_line1_str);
    free(lcd_line2_str);
}




uint64 convert_timestamp(uint8 *ts_bytes, uint16 ts_len){
	uint64 ts_64 = 0;
	int i;
    for (i = 0; i < ts_len; i++)
    {
        ts_64 += ((uint64) ts_bytes[i]) << (i*8);
    }
	return ts_64;
}

double calculate_range(
		uint64 poll_tx_ts,
		uint64 poll_rx_ts,
		uint64 resp_tx_ts,
		uint64 resp_rx_ts,
		uint64 final_tx_ts,
		uint64 final_rx_ts){
//
//	uint32 poll_tx_ts_32 = (uint32)(poll_tx_ts>>8);
//	uint32 poll_rx_ts_32 = (uint32)(poll_rx_ts>>8);
//	uint32 resp_tx_ts_32 = (uint32)(resp_tx_ts>>8);
//	uint32 resp_rx_ts_32 = (uint32)(resp_rx_ts>>8);
//	uint32 final_tx_ts_32 = (uint32)(final_tx_ts>>8);
//	uint32 final_rx_ts_32 = (uint32)(final_rx_ts>>8);

	double Ra, Rb, Da, Db;

	if(resp_rx_ts >= poll_tx_ts)
		Ra = (double)(resp_rx_ts - poll_tx_ts);
	else
		Ra = (double)( ( (uint64)TIMESTAMP40_MAX_VAL - poll_tx_ts )+resp_rx_ts+1 );


	if(final_rx_ts >= resp_tx_ts)
		Rb = (double)(final_rx_ts - resp_tx_ts);
	else
		Rb = (double)( ( (uint64)TIMESTAMP40_MAX_VAL - resp_tx_ts )+ final_rx_ts+1 );

	if(final_tx_ts >= resp_rx_ts)
		Da = (double)(final_tx_ts - resp_rx_ts);
	else
		Da = (double)( ( (uint64)TIMESTAMP40_MAX_VAL - resp_rx_ts  )+final_tx_ts+1 );

	if(resp_tx_ts>= poll_rx_ts)
		Db = (double)(resp_tx_ts - poll_rx_ts);
	else
		Db = (double)( ( (uint64)TIMESTAMP40_MAX_VAL - poll_rx_ts )+resp_tx_ts+1 );

//    Rb = (double)(final_rx_ts - resp_tx_ts);
//    Da = (double)(final_tx_ts - resp_rx_ts);
//    Db = (double)(resp_tx_ts - poll_rx_ts);

    uint64 tof_dtu;
    uint64 Ra_Rb;
    uint64 Da_Db;
    uint64 diff_Ra_Rb_Da_Db;
    double tof;
    double distance_a, distance_b, distance;
    //tof_dtu= (int64)( (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db) );
	Ra_Rb = Ra * Rb;
	Da_Db = Da * Db;
	if(Ra_Rb > Da_Db) diff_Ra_Rb_Da_Db = Ra_Rb - Da_Db;
	else
		return 0;

    if((Ra+Da)==0)
    	return 0;
    else{
		tof_dtu = (uint64)( diff_Ra_Rb_Da_Db / (2*(Ra + Da )) );
		tof = tof_dtu * DWT_TIME_UNITS;
		distance_a = tof * SPEED_OF_LIGHT;
    }
    if((Rb+Db)==0)
    	return 0;
    else{
		tof_dtu = (uint64)( diff_Ra_Rb_Da_Db / (2*(Rb + Db )) );
		tof = tof_dtu * DWT_TIME_UNITS;
		distance_b = tof * SPEED_OF_LIGHT;
    }

    distance = 0.5*(distance_a+distance_b);
    return distance;
}

void set_frame_timestamp(uint8 *frame_ts, uint64 input_ts){
    int i;
    for (i = 0; i < TIMESTAMP_LEN; i++)
    {
        frame_ts[i] = (uint8) input_ts;
        input_ts >>= 8;
    }
}

void wait_till(uint32 ts){
	uint32 cur_time = dwt_readsystimestamphi32();
	/****check for delayed counter****/
	if(cur_time>ts){
		if( (cur_time - ts) < UINT32_MAX_HALF) return;
	}
	else{
		if( (ts-cur_time) >= UINT32_MAX_HALF) return;
	}
	/**************************************/
	if(dwt_readsystimestamphi32()>ts){
		while(dwt_readsystimestamphi32()>ts){} //overflow handling
	}
	while(dwt_readsystimestamphi32()<=ts){}
	return;
}


#define UNKNOWN_TS 0
Event wait_till_event(uint32 ts){
	if(ts == UNKNOWN_TS)
		return UNKNOWN_EVENT;

	last_event = UNKNOWN_EVENT;
	Event cur_event = UNKNOWN_EVENT;
	uint32 cur_time = dwt_readsystimestamphi32();
	/****check for delayed counter****/
	if(cur_time>ts){
		if( (cur_time - ts) < UINT32_MAX_HALF)
			return cur_event;
	}
	else{
		if( (ts-cur_time) >= UINT32_MAX_HALF)
			return cur_event;
	}
	/**************************************/
	if(dwt_readsystimestamphi32()>ts){
		while(dwt_readsystimestamphi32()>ts){
			if(last_event != UNKNOWN_EVENT){
				cur_event = last_event;
				last_event = UNKNOWN_EVENT;
				return cur_event;
			}
		} //overflow handling
	}
	while(dwt_readsystimestamphi32()<=ts){
		if(last_event != UNKNOWN_EVENT){
			cur_event = last_event;
			last_event = UNKNOWN_EVENT;
			return cur_event;
		}
	}
	return cur_event;
}


//void wait_and_overhear(uint32 delayed_ts){
//	/*-----------------over hear all the neighbors--------*/
//	while(TRUE){
//		dwt_forcetrxoff();
//		dwt_rxenable(DWT_START_RX_IMMEDIATE);
//		Event cur_event = wait_till_event(delayed_ts);
//		if(cur_event == RX_OK){
//			// save the overheard neighbor
//			is_neighbor[ rx_buf.chdr.source_addr ]= TRUE;
//		}
//		else if(cur_event == UNKNOWN_EVENT) //TIME UP
//		{
//			dwt_forcetrxoff();
//			return;
//		}
//	}
//	/*------------------------------------------------------*/
//}



