/*! ----------------------------------------------------------------------------
 * @file    utility.h
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

#ifndef _UTILITY_
#define _UTILITY_

#ifdef __cplusplus
extern "C" {
#endif
/* function prototype starts here  */
	#include "deca_headers.h"
	#include "frame.h"

	void init_decawave_device(int enable_irq);

	uint64 convert_timestamp(uint8 *ts_bytes, uint16 ts_len);
	double calculate_range(
			uint64 poll_tx_ts,
			uint64 poll_rx_ts,
			uint64 resp_tx_ts,
			uint64 resp_rx_ts,
			uint64 final_tx_ts,
			uint64 final_rx_ts);

	void print_lcd(const char* str);
	void print_lcd2(const char* str1, const char* str2);
	void set_frame_timestamp(uint8 *frame_ts, uint64 input_ts);
	void wait_till(uint32 ts);
	Event wait_till_event(uint32 ts);

	void (*tim2_handler)(void);
#ifdef __cplusplus
}
#endif

#endif /* _UTILITY_ */
