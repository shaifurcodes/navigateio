
#include "cir_util.h"

#include "deca_headers.h"
#include "frame.h"
#include "utility.h"
#include "usb_util.h"

#include "math.h"




void run_link_quality_test(){
#undef USB_LOG

	static char lcd_line1[16];
	static char lcd_line2[16];

	init_decawave_device(TRUE);

	sprintf(lcd_line1, "Node: %u", node_id);
	sprintf(lcd_line2, "...............");
	print_lcd2(lcd_line1, lcd_line2);
	Sleep(500);

	if(node_id%2==1){
		//float cur_metric = 0.;
		while(TRUE){
			last_event = UNKNOWN_EVENT;
			dwt_forcetrxoff();
			dwt_setrxtimeout( 0 );
			dwt_rxenable(DWT_START_RX_IMMEDIATE);

			while(last_event == UNKNOWN_EVENT){}
			if(last_event == RX_OK){
				last_event = UNKNOWN_EVENT;

				sprintf(lcd_line2, "%u: %u",rx_buf.chdr.source_addr, (uint16) link_quality[rx_buf.chdr.source_addr] );
				print_lcd2(lcd_line1, lcd_line2);
				//Sleep(10);
			}
		}
	}
	else{
		uint32  wait_till_ts;
		uint16 i;
		while(TRUE){
			dwt_forcetrxoff();
			generate_init_frame();
			//transmit the frame
			tx_done = FALSE;
		    dwt_writetxdata(PAYLOAD_INDX+2, tx_buf.frame_byte, 0);
			dwt_writetxfctrl(PAYLOAD_INDX+2, 0, 1);
		    dwt_starttx(DWT_START_TX_IMMEDIATE);
		    while(tx_done == FALSE) {}

		    wait_till_ts = dwt_readtxtimestamphi32();

		    for(i=0; i<1000; ++i){
		    	wait_till_ts += 250000;
		    }
		    wait_till(wait_till_ts);
		    sprintf(lcd_line2, "SEQ# %u",tx_buf.chdr.seq_num);
		    print_lcd2(lcd_line1, lcd_line2);
		}
	}

}
