#!/usr/bin/python3

import logging
import  pyudev
import serial
import socket
import time
import threading
import sys
import signal
from queue import Queue
from math import log



from lora_sx126x import Lora
from pressure_sensor_lps22hb import LPS22HB

def sigint_handler(signal, frame):
    sys.exit(0)

class FRSTRPI(object):
    def __init__(self):
        self.init_logging()

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
            logging.error("Did not find DW1000 via USB, quitting program !!")
            exit(1)

        self.CONTROLLER_ID = 0
        self.LORA_MSG_START_MARK = 'S'
        self.LORA_MSG_END_MARK = 'E'
        self.LORA_RANGING_CMD = 'r'
        self.last_msg_marker = 'E'
        self.lora_incoming_msg_q = Queue()
        self.cur_lora_msg = ''
        self.thread_list = {}

        self.UNSET_SENSOR_VALUE = -1.0
        self.pressure_reading_interval_sec = 1.0
        self.pressure_hpa = self.UNSET_SENSOR_VALUE
        self.is_sensor_available = True
        try:
            self.pressure_sensor = LPS22HB()
        except Exception as ex:
            logging.exception(ex)
            self.is_sensor_available = False
        if self.is_sensor_available:
            logging.info("Pressure-sensor LPS22HB detected via I2C(1) bus")
        self.temperature_celsius = self.UNSET_SENSOR_VALUE

        self.altitude_meter = 0.

        self.set_uwb_node_id()
        return

    #---------------------------utilitiy methods-------------------------------------#
    def init_logging(self):
        #log_file_name = '/home/pi/software/navigateio/frst/beacon/beacon.log'
        log_file_name = '/home/pi/navigateio-service.log'
        logging.basicConfig(
            filename= log_file_name,
            filemode='w',
            level=logging.DEBUG,
            format='%(asctime)s.%(msecs)03d, %(threadName)-10s: %(message)s',
            datefmt='%H:%M:%S')  # '%Y-%m-%d %H:%M:%S'
        return

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

    def pressure_to_z(self, pressure_hpa, temperature_celsius):
        #formula_src: https://sciencing.com/calculate-air-volume-5146908.html
        temp_F = temperature_celsius*1.8 + 32.
        try:
            return round(( ( log( pressure_hpa*100./101325. )*287.053 ) * \
                     (temp_F + 459.67)*5./9. )/(-9.8),3)
        except Exception as ex:
            logging.exception(ex)
        return 0.0

    #----------------------------UWB USB API methods ------------------------------------#
    def find_dw1000_via_usb(self):
        max_retries = 5
        attempt = 1
        while attempt <= max_retries:
            attempt += 1
            if not self.uwb_usb_port_id:
                self.uwb_usb_port_id = self.find_usb_port( self.DW1000_VENDOR_ID, self.DW1000_PRODUCT_ID )
            if self.uwb_usb_port_id:
                logging.info('Found DW1000 via USB')
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
        rpi_no = ''
        try:
            my_hostname = socket.gethostname()
            rpi_no = ''.join([char for char in my_hostname[::-1] if char.isdigit()])[::-1]
            logging.info("My RPI number: "+str(rpi_no))
        except Exception as ex:
            logging.exception(ex)

        self.usb_cmd_counter = (self.usb_cmd_counter + 1) % 256
        usb_msg = str(self.CMD_SEND_ID)+' '+str(self.usb_cmd_counter)+' '+str("RPI")+str(rpi_no)

        usb_read_timeout = 10*self.USB_OP_LATENCY_MSEC/1000.0
        uwb_response = self.get_uwb_usb_data(usb_msg, usb_read_timeout=usb_read_timeout)

        if not '=' in uwb_response:
            logging.error("Unable to read node-ID from UWB via USB, quiting program!!")
            exit(1)
        usb_cmd_plus_seq, src_node_id = uwb_response.split('=')
        usb_cmd, usb_cmd_seq_no = usb_cmd_plus_seq.split()
        usb_cmd, usb_cmd_seq_no = int(usb_cmd), int(usb_cmd_seq_no)
        if(usb_cmd == self.CMD_SEND_ID  and usb_cmd_seq_no == self.usb_cmd_counter):
            self.uwb_node_id = int(src_node_id)
            logging.info("node id set as "+str(self.uwb_node_id))
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

    #----------------------thread-methods-----------------------------------------------#
    def thread_lora_frame_receiver(self):
        while True:
            time.sleep(0) #thread yield
            try:
                lora_recv_msgs = self.lora_node.lora_receive()
                if not lora_recv_msgs:
                    continue
                for cindx, c in enumerate(lora_recv_msgs):
                    if c==self.last_msg_marker or c==self.LORA_MSG_START_MARK:
                        self.cur_lora_msg = ''
                        self.last_msg_marker = c
                    elif c==self.LORA_MSG_END_MARK:
                        if self.cur_lora_msg:
                            self.lora_incoming_msg_q.put(str(self.cur_lora_msg))
                        self.cur_lora_msg = ''
                        self.last_msg_marker = c
                    elif self.last_msg_marker==self.LORA_MSG_START_MARK:
                        self.cur_lora_msg = self.cur_lora_msg + c
            except Exception as ex:
                logging.exception(ex)
        return

    def thread_handle_controller_commands(self):
        while True:
            time.sleep(0)  # thread yield
            try:
                if self.lora_incoming_msg_q.empty():
                    continue
                cur_msg = self.lora_incoming_msg_q.get()
                if not '=' in cur_msg:
                    continue
                msg_prefix, msg_suffix = cur_msg.split('=')
                msg_prefix = msg_prefix.split()
                if len(msg_prefix) != 4:
                    continue
                if int(msg_prefix[0]) != self.uwb_node_id:
                    continue

                cmd_src, cmd_seq, cmd_type = int(msg_prefix[1]), int(msg_prefix[2]), msg_prefix[3]

                if cmd_src==self.CONTROLLER_ID and cmd_type==self.LORA_RANGING_CMD:
                    if msg_suffix:
                        msg_suffix = msg_suffix.split()
                        rlist = []
                        for n1 in msg_suffix:
                            rlist.append(int(n1))
                        logging.debug("starting ranging " + str(self.uwb_node_id) + "-->" + str(rlist))
                        range_response = self.get_uwb_ranges(nlist=rlist, slot_time_msec=self.slot_time_msec)
                        logging.debug("debug: range_response: " + str(range_response))
                        lora_range_reponse = self.LORA_MSG_START_MARK + ' ' + \
                                             str(cmd_src) + ' ' + \
                                             str(self.uwb_node_id) + ' ' + \
                                             str(cmd_seq) + ' ' + \
                                             str(cmd_type) + ' = ' + \
                                             range_response + ' ' + \
                                             '; ' + \
                                             'z '+str(self.altitude_meter) + \
                                             self.LORA_MSG_END_MARK
                        logging.debug("debug: lora_range_response: " + str(lora_range_reponse))
                        self.lora_node.lora_send(lora_range_reponse)
                        logging.debug("finished sending data over lora for: " + str(self.uwb_node_id) + "-->" + str(rlist))
            except Exception as ex:
                logging.exception(ex)
                continue
        return

    def thread_read_sensor_data(self):
        while True:
            time.sleep(self.pressure_reading_interval_sec)
            try:
                self.pressure_sensor.set_oneshot_reading_mode()
                while not self.pressure_sensor.is_new_pressure_val_available() and \
                      not self.pressure_sensor.is_new_temperature_val_available():
                    time.sleep(0.1)
                self.pressure_hpa = round(self.pressure_sensor.get_pressure_val(), 3)
                self.temperature_celsius = round(self.pressure_sensor.get_temperature_val(), 3 )
                alt_val = self.pressure_to_z(self.pressure_hpa, self.temperature_celsius)
                if alt_val != self.UNSET_SENSOR_VALUE:
                    self.altitude_meter = alt_val
            except Exception as ex:
                logging.exception(ex)
                continue
        return

    #-------------------------generic Methods-------------------------------------------#
    def run_beacon(self):
        try:
            self.thread_list['lora'] = threading.Thread(name='lora',
                                                               target=self.thread_lora_frame_receiver,
                                                               daemon=True)
            self.thread_list['cmdhndlr'] = threading.Thread(name='cmd-handler',
                                                               target=self.thread_handle_controller_commands,
                                                               daemon=True
                                                               )
            self.thread_list['sensor'] =  threading.Thread(name='sensor',
                                                           target=self.thread_read_sensor_data,
                                                           daemon=True
                                                            )
            self.thread_list['cmdhndlr'].start()
            self.thread_list['lora'].start()
            if self.is_sensor_available:
                self.thread_list['sensor'].start()
        except Exception as ex:
            logging.exception(ex)
            exit(1)

        while True:
            time.sleep(1)
        return

if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigint_handler)
    frstrpi = FRSTRPI()
    frstrpi.run_beacon()



