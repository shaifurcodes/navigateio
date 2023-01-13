/*! ----------------------------------------------------------------------------
 * @file    dw_usb_com.h
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

#ifndef _DW_USB_COM_
#define _DW_USB_COM_

#include "frame.h"

#ifdef __cplusplus
extern "C" {
#endif
/* function prototype starts here  */

	#include "usbd_cdc_if.h"
	#include "deca_headers.h"
	#define DW_USB_MAX_POLLING_DURATION_MS (80)
	#define DW_USB_TX_BUF_LEN 4065	  //(64*(CDC_DATA_FS_MAX_PACKET_SIZE))
	#define USB_RX_BUF_LEN	300

	uint8 usb_tx_buf[DW_USB_TX_BUF_LEN] ;
	//uint8 usb_rx_buf[USB_RX_BUF_LEN];
	extern volatile int is_usb_data_available;
	extern uint8 usb_rx_buf[MAX_USB_RX_BUF];
	extern uint16 usb_rx_buf_len;

	void clear_usb_buffer();
	int flush_usb_rx_buf();
	int usb_write(char *buf, uint16 buf_len, int flush_immediate);

	void run_usb_com_test();

#ifdef USB_LOG
	void usb_save_log(char *buf, uint16 buf_len);
	void usb_flush_log();
	void reset_usb_log();
#endif

#ifdef __cplusplus
}
#endif

#endif /* _DW_USB_COM_ */
