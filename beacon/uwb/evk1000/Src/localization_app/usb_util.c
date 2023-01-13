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

#include "usb_util.h"

#include "deca_headers.h"
#include "frame.h"
#include "utility.h"


#define MAX_LOG_BUF_LEN 8000

#ifdef USB_LOG
	static char log_buf[MAX_LOG_BUF_LEN];
#endif

uint16 log_buf_len = 0;

extern USBD_HandleTypeDef hUsbDeviceFS;
static uint16  usb_tx_buf_byte_count = 0;
//static int prev_rx_seq_no = 0;

#pragma GCC optimize ("O0")
int flush_usb_tx_buf(){
	if(hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) return 1;

	USBD_CDC_HandleTypeDef  *usb_handle = (USBD_CDC_HandleTypeDef*)(hUsbDeviceFS.pClassData);
	uint16 cur_buf_offset = 0;
	uint16_t transmit_len = 0;
	while(cur_buf_offset < usb_tx_buf_byte_count){
		transmit_len = MIN(CDC_DATA_FS_MAX_PACKET_SIZE, (usb_tx_buf_byte_count - cur_buf_offset));
		if(transmit_len > 0){
			uint16_t cur_polling_duration = 0;
			while(TRUE){
				if(usb_handle->TxState == 0) break;
				if(cur_polling_duration > DW_USB_MAX_POLLING_DURATION_MS) break;
				Sleep(1);
				cur_polling_duration += 1;
			}
			if(usb_handle->TxState == 0) {
				if(CDC_Transmit_FS( (usb_tx_buf+cur_buf_offset), transmit_len) == USBD_OK ){
					cur_buf_offset += transmit_len;
				}
			}
			else
				return 1;
		}
	}
	usb_tx_buf_byte_count = 0;
	return 0;
}

int usb_write(char *buf, uint16 buf_len, int flush_immediate){
	if(DW_USB_TX_BUF_LEN  - usb_tx_buf_byte_count < buf_len ){
		clear_usb_buffer();
		if(flush_usb_tx_buf() == 1){
			return 1;
		}
	}
	if(buf_len > DW_USB_TX_BUF_LEN) return 1;
	if(buf_len<=0) return 0;

	memcpy( (usb_tx_buf+usb_tx_buf_byte_count), buf, buf_len);
	usb_tx_buf_byte_count += buf_len;
	if( flush_immediate == TRUE){
		if(flush_usb_tx_buf() == 1)
			return 1;
	}
	return 0;
}

void clear_usb_buffer(){
	usb_tx_buf_byte_count = 0;
}

void run_usb_com_test(){
	char lcd_line1[16];
	char lcd_line2[16];
	int cur_usb_data_len = 0;
	uint8 usb_tx_msg_counter = 0;
	char usb_tx_msg[21];

	init_decawave_device(FALSE);

	sprintf(lcd_line1, "Node: %u", node_id);
	sprintf(lcd_line2, "...............");
	print_lcd2(lcd_line1, lcd_line2);
	Sleep(500);

	dwt_forcetrxoff();
	while(TRUE){
		if(is_usb_data_available == TRUE){
			is_usb_data_available = FALSE;
			cur_usb_data_len = usb_rx_buf_len;
			if(cur_usb_data_len > 16)
				cur_usb_data_len = 16;
			strncpy(lcd_line2, (char*)usb_rx_buf, cur_usb_data_len);
			print_lcd2("usb RX data", lcd_line2);
			//usb_rx_buf, usb_rx_buf_len;
		}else{
			usb_tx_msg_counter++;
			sprintf(usb_tx_msg, "%u Hello World\n", usb_tx_msg_counter);
			if(usb_write(usb_tx_msg, strlen(usb_tx_msg), TRUE) == 1){
			//---------------
				strcpy(lcd_line2, usb_tx_msg);
				print_lcd2("NO USB!!", lcd_line2);
			}
			else{
				//---------------
				strcpy(lcd_line2, usb_tx_msg);
				print_lcd2("USB TX DONE", lcd_line2);
			}
		}
		Sleep(500);
	}
}
//int usb_read(uint8 *buf){
//	/*
//	 * block till new data in the rx buffer,
//	 * retrieve the payload size,
//	 * copy the payload data to buf and return
//	 *
//	 * */
//	if(hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)  return 0;
//
//	volatile int rx_seq_no = prev_rx_seq_no;
//	int i,j;
//
//	while(TRUE){
//		USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &usb_rx_buf[0]);
//		USBD_CDC_ReceivePacket(&hUsbDeviceFS);
//		rx_seq_no = atoi((char*)usb_rx_buf);
//		if(rx_seq_no==prev_rx_seq_no) continue;
//		i= 0 ;
//		while('0'<=usb_rx_buf[i]  && usb_rx_buf[i]<='9' && i < USB_RX_BUF_LEN) ++i;
//
//		while( !('0'<=usb_rx_buf[i]  && usb_rx_buf[i]<='9') && i < USB_RX_BUF_LEN) ++i;
//
//		j=0;
//		while(usb_rx_buf[i]!='\n' && i < USB_RX_BUF_LEN){
//			buf[j++] = usb_rx_buf[i++];
//		}
//		buf[j] = '\0';
//		prev_rx_seq_no = rx_seq_no;
//		return j;
//	}
//	return 0;
//}

//void usb_read_test(){
//	init_decawave_device(TRUE);
//	print_lcd("USB READ TEST 2");
//	uint8 usb_rx_buf[USB_RX_BUF_LEN];
//	USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
//	if(hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED){
//		print_lcd("unconfig usb!");
//		return;
//	}
//	int i;
//	char lcd_buf[16];
//	char seq_no_buf[4];
//	int usb_frame_seq_no = 0;
//	int old_usb_frame_seq_no =  usb_frame_seq_no;
//	while(TRUE){
////		if (hcdc->RxState != 0){
//		    old_usb_frame_seq_no =  usb_frame_seq_no;
//			USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &usb_rx_buf[0]);
//			USBD_CDC_ReceivePacket(&hUsbDeviceFS);
//			for(i=0; i<3 ; ++i){
//				if(usb_rx_buf[i]=='\n')
//					break;
//			}
//			strncpy(seq_no_buf, (char* )usb_rx_buf, i);
//			usb_frame_seq_no = atoi(seq_no_buf);
//			if(usb_frame_seq_no == old_usb_frame_seq_no)
//				continue;
//			strncpy(lcd_buf, (char*)usb_rx_buf, i );
//			print_lcd2("USB DATA", lcd_buf);
//		}
////	}
//}
#ifdef USB_LOG
	void usb_save_log(char *buf, uint16 buf_len){
		if( MAX_LOG_BUF_LEN  -1 <= log_buf_len+buf_len)
			return;
		strcpy(log_buf+log_buf_len, buf);
		log_buf_len += buf_len;
	}

	void usb_flush_log(){
		uint16 i = 0;
		uint16 tx_size = 0;
		while(i < log_buf_len){
			tx_size = log_buf_len - i;

			if(tx_size > CDC_DATA_FS_MAX_PACKET_SIZE)
				tx_size = CDC_DATA_FS_MAX_PACKET_SIZE;

			usb_write(log_buf+i, tx_size, TRUE) ;
			i += tx_size;
		}
		log_buf_len = 0;
	}
	void reset_usb_log(){
		log_buf_len = 0;
	}
#endif





