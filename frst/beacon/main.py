#!/usr/bin/python3.8

import  pyudev
import serial
import socket
import time

import sys
import signal
from sx126x import Lora

def sigint_handler(signal, frame):
    print('Exiting program.....')
    sys.exit(0)

class FRSTRPI(object):
    def __init__(self):
        self.uwb_node_id = -1
        self.uwb_usb_port_id = ''

        self.CMD_RANGE = 1
        self.CMD_SEND_ID = 2
        self.CMD_SEND_NLIST = 3
        self.CMD_SHOUT = 4
        self.CMD_SEND_LOG = 5

        self.USB_TERMINATION_STR = 'END'
        self.USB_OP_LATENCY_MSEC = 10.0 #TODO: try decreasing latency time
        self.USB_BAUDRATE = 115200
        self.usb_cmd_counter = 0

        self.slot_time_msec = 8

        self.DW1000_VENDOR_ID = '0483'
        self.DW1000_PRODUCT_ID = '5740'

        self.LORA_PORT_RPI = '/dev/ttyAMA0'

        self.lora_node = Lora(serial_port=self.LORA_PORT_RPI)
        if not self.find_dw1000_via_usb():
            print("Did not find DW1000 via USB!!")
            exit(1)
        self.LORA_PERMITTED_CHARS_SET = [' ', ',', '=']
        self.set_uwb_node_id()
        return

    #---------------------------utilitiy methods-------------------------------------#
    def find_usb_port(self, vendor_id, product_id):
        context = pyudev.Context()
        for device in context.list_devices(subsystem='tty', ID_BUS='usb'):
            cur_device_attrs = dict(device.properties)
            if 'ID_VENDOR_ID' in cur_device_attrs.keys() and 'ID_MODEL_ID' in cur_device_attrs.keys():
                if cur_device_attrs['ID_VENDOR_ID'] == vendor_id and \
                        cur_device_attrs['ID_MODEL_ID'] == product_id:
                    if 'DEVNAME' in cur_device_attrs.keys():
                        return str(cur_device_attrs['DEVNAME'])
        return ''

    #----------------------------UWB USB API methods ------------------------------------#
    def find_dw1000_via_usb(self):
        max_retries = 5
        attempt = 1
        while attempt <= max_retries:
            attempt += 1
            if not self.uwb_usb_port_id:
                self.uwb_usb_port_id = self.find_usb_port( self.DW1000_VENDOR_ID, self.DW1000_PRODUCT_ID )
            if self.uwb_usb_port_id:
                print('Found DW1000 via USB')
                return True
            time.sleep(1)
        return False

    def get_uwb_usb_data(self, usb_msg, usb_read_timeout):
        with serial.Serial(self.uwb_usb_port_id,
                           baudrate=self.USB_BAUDRATE,
                           timeout=self.USB_OP_LATENCY_MSEC/1000) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            usb_str = str(usb_msg)+'\n'
            ser.write(usb_str.encode())

            finishing_ts = time.time()+usb_read_timeout
            while(time.time()<finishing_ts):
                cur_usb_rx_msg = ser.readline()
                cur_usb_rx_msg = cur_usb_rx_msg.decode('ascii').strip()
                if len(cur_usb_rx_msg) == 0: #did not receive anything, uwb perhaps got disconnected
                    time.sleep(usb_read_timeout/1000.)
                    continue

                if ':' in cur_usb_rx_msg:
                    _ , cur_usb_rx_msg = cur_usb_rx_msg.split(':') #separate the usb-rx counter

                if self.USB_TERMINATION_STR in cur_usb_rx_msg:
                    cur_usb_rx_msg = cur_usb_rx_msg.replace(self.USB_TERMINATION_STR, '')
                    return cur_usb_rx_msg
        return ''

    def set_uwb_node_id(self):
        '''
        :return:
        '''
        my_hostname = socket.gethostname()
        rpi_no = ''.join([char for char in my_hostname[::-1] if char.isdigit()])[::-1]

        self.usb_cmd_counter = (self.usb_cmd_counter + 1) % 256
        usb_msg = str(self.CMD_SEND_ID)+' '+str(self.usb_cmd_counter)+' '+str("RPI")+str(rpi_no)

        usb_read_timeout = 10*self.USB_OP_LATENCY_MSEC/1000.0
        uwb_response = self.get_uwb_usb_data(usb_msg, usb_read_timeout=usb_read_timeout)

        if not '=' in uwb_response:
            print("Unable to read node-ID from UWB via USB!!")
            exit(1)
        usb_cmd_plus_seq, src_node_id = uwb_response.split('=')
        usb_cmd, usb_cmd_seq_no = usb_cmd_plus_seq.split()
        usb_cmd, usb_cmd_seq_no = int(usb_cmd), int(usb_cmd_seq_no)
        if(usb_cmd == self.CMD_SEND_ID  and usb_cmd_seq_no == self.usb_cmd_counter):
            self.uwb_node_id = int(src_node_id)
            with open("uwb_node_id.txt","w") as f:
                f.write(str(self.uwb_node_id))
        return

    def get_uwb_ranges(self, nlist, slot_time_msec):
        nlist_count = len(nlist)
        if nlist_count >0:
            self.usb_cmd_counter = (self.usb_cmd_counter + 1) % 256
            usb_msg = str(self.CMD_RANGE)+' '+str(self.usb_cmd_counter)+\
                      ' '+str( slot_time_msec )+\
                      ' '+str(nlist_count)
            for i in nlist:
                usb_msg += ' '+str(i)

            usb_read_timeout = 2 * self.USB_OP_LATENCY_MSEC/1000. \
                               + (2*nlist_count+2)*self.slot_time_msec/1000.

            uwb_response = self.get_uwb_usb_data(usb_msg, usb_read_timeout=usb_read_timeout)
            if not '=' in uwb_response:
                return  ''
            usb_cmd_plus_seq, nr_list_str = uwb_response.split('=')
            usb_cmd, usb_cmd_seq_no = usb_cmd_plus_seq.split()
            usb_cmd, usb_cmd_seq_no = int(usb_cmd), int(usb_cmd_seq_no)

            if( usb_cmd == self.CMD_RANGE and usb_cmd_seq_no == self.usb_cmd_counter):
                return nr_list_str
        return ''

    #-------------------------generic Methods-------------------------------------------#
    def process_lora_msgs(self, lora_msgs):
        '''
        tasks:
            1. list all lora messages
            2. process lora messages directed to me one by one
        :param lora_msgs:
        :return:
        '''
        if not 'E' in lora_msgs:
            return
        lora_msg_set = lora_msgs.split('E')
        for msg in lora_msg_set:
            if not msg:
                continue
            if not '=' in msg:
                continue
            msg_prefix, msg_suffix = msg.split('=')
            if not msg_prefix:
                continue
            msg_prefix = msg_prefix.split()
            if len(msg_prefix) != 5:
                continue
            if not 'cb' in msg_prefix[0] and not 'bb' in msg_prefix[0]:
                continue
            lora_seq_no = int(msg_prefix[1])
            if int(msg_prefix[2]) != self.uwb_node_id:
                continue
            lora_src_node = msg_prefix[3]
            lora_cmd_type = msg_prefix[4]

            if lora_cmd_type=='r':
                if msg_suffix:
                    rlist = []
                    for n1 in msg_suffix:
                        rlist.append(int(n1))
                    range_response = self.get_uwb_ranges(nlist=rlist, slot_time_msec=self.slot_time_msec)
                    lora_range_reponse = 'bc '+str(lora_seq_no)+' '+str(self.uwb_node_id)+' '+str(lora_src_node)+' '+str(lora_cmd_type)+"="+range_response+" E"
                    self.lora_node.lora_send(lora_range_reponse)
            elif lora_cmd_type=='s':
                #TODO
                continue
            else:
                continue

    def run_beacon(self):
        while True:
            try:
                time.sleep(0.1)
                lora_recv_msgs = self.lora_node.lora_receive()
                lora_recv_msgs = ''.join(letter for letter in lora_recv_msgs if letter.isalnum() or letter in self.LORA_PERMITTED_CHARS_SET)
                print("debug: lora-recv-msg: "+str(lora_recv_msgs))
                self.process_lora_msgs(lora_recv_msgs)
            except Exception as ex:
                print(ex)
                exit(1)
        return


if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigint_handler)
    frstrpi = FRSTRPI()
    frstrpi.run_beacon()



