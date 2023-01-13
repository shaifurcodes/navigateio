/*! ----------------------------------------------------------------------------
 * @file    dw_tracking_application.c
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
#include "ranging_node.h"
#include "utility.h"
#include "frame.h"
#include "tracking_application.h"
#include "usb_util.h"
#include "cir_util.h"

extern void (*tim2_handler)(void);
void run_tracking_app(){
//	enable_next_hops = TRUE;
//	ignore_init_frame = FALSE;
//
//	master_inter_ranging_delay_ms = 10;
//	master_hop2_ranging_factor = 3;

	/******************************/
	run_ranging_node();
	//run_usb_com_test();
	//run_link_quality_test();
}
