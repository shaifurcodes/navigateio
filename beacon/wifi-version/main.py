#!/usr/bin/python3.7

import threading
import logging
#from subprocess import  check_output
import socket
import time
from collections import  deque
#from math import  sqrt
#from scipy.stats import  linregress
import numpy as np
#import signal
#import  select
from itertools import islice
import netifaces as ni
import configparser
import math
import sys

#------------sensor related---------------#
import  pyudev
import serial
import busio
import board
import adafruit_bno055
import adafruit_bmp280
from kalman import myKalman

import json
from scipy.signal import butter, filtfilt
from copy import deepcopy
from pyquaternion import Quaternion

class RPIHandler(object):
    def __init__(self,
                 udp_port = 12345,
                 uwb_slot_time_msec = 8,
                 sensor_reading_interval_sec = 1,
                 sea_level_pressure_in_hg = 30.07
                 ):
        self.init_ts = time.time()
        self.CONFIG_FILE_NAME = './main.ini' #TODO: give full path
        #------------arguements------------------------#
        self.my_udp_port = udp_port
        self.slot_time_msec = uwb_slot_time_msec
        self.sensor_reading_interval_sec = sensor_reading_interval_sec
        self.sea_level_pressure_in_hg = sea_level_pressure_in_hg
        #------------constants-----------------#
        self.USB_BAUDRATE = 115200

        self.USB_OP_LATENCY_MSEC = 1.0
        self.USB_TERMINATION_STR = 'END'
        self.USB_LOG_STR = 'LOG'

        self.CMD_RANGE = 1
        self.CMD_SEND_ID = 2
        self.CMD_SEND_NLIST = 3
        self.CMD_SHOUT = 4
        self.CMD_SEND_LOG = 5

        self.DW1000_VENDOR_ID = '0483'
        self.DW1000_PRODUCT_ID = '5740'

        #self.BME280_I2C_ADDRESS = 0x76
        self.UDP_MSG_LIVENESS_MSEC = 100

        self.INCH_HG_TO_HPA = 33.86
        #-----------thread related---------------------------#
        self.my_threads = {}

        #--------anchor-related------------#
        self.anchors = {}

        #---------imu-relatede-------------------------#
        self.imu_device = None
        self.cur_lin_acceleration = []
        self.cur_quaternion = []
        self.imu_reset_pos_time = 2.0 
        self.controller_pos_set = False
        self.imu_location = [0.0,0.0]         #Gives X and Y location.
        self.controller_location = [0.0,0.0]  #This is the location informed by the controller
        self.uwb_angle = 0.0
        self.angle_reset_time = 2
        self.zupt_threshold = 0.15
        self.prev_angle_reset_time = 0

        #-----------altitude-related--------------------#
        self.MAX_ALTITUDE_HISTORY_LEN =120
        self.pressure_sensor_device = None
        self.altitude_history = deque([], maxlen=self.MAX_ALTITUDE_HISTORY_LEN)
        self.current_floor = 1 #unknown_floor
        self.floor_change_threshold_meter =  2.5
        self.max_floor  = 1
        self.lock_altitude_history = threading.Lock()
        #-----------misc class varibles------------------------------#
        self.enable_uwb_log_printing = True #<---change to false

        self.my_ip = ''

        self.uwb_node_id = -1
        self.uwb_usb_port_id = ''
        self.imu_usb_port_id = ''

        self.udp_rx_sock =  None
        self.udp_tx_sock = None
        self.udp_msg_cmd = None
        self.udp_msg_arg = None
        self.udp_msg_rx_ts = None
        self.udp_cmd_seq_no = None
        self.udp_msg_ip = None
        self.udp_msg_port = None

        self.lock_udp_msg = threading.Lock()
        self.event_udp_msg_available = threading.Event()
        self.event_udp_msg_available.clear()

        self.usb_rx_counter = 0
        self.usb_cmd_counter = 0

        self.config = configparser.ConfigParser()
        self.config.read(self.CONFIG_FILE_NAME)
        self._parse_config()
        #---------------gracefull program exit handling----------------------------#
        # self.stop_signal_received = threading.Event()
        # self.stop_signal_received.clear()
        # self.setup_signal_handlers()
        #--------------------------------------------------------------------------#
        return

    def _parse_config(self):
        self.floor_change_threshold_meter = float(self.config['DEFAULT']['floor_change_threshold_meter'])
        self.sea_level_pressure_in_hg = float(self.config['DEFAULT']['sea_level_pressure_inch_hg'])
        self.current_floor = int(self.config['DEFAULT']['current_floor'])
        self.max_floor = int(self.config['DEFAULT']['max_floor'])
        return

    def  _save_config(self):
        with open(self.CONFIG_FILE_NAME, 'w') as f:
            self.config.write(f)
        #print("saved config to files")
        return

    def init_logging(self):
        #log_file_name = '/usr/local/etc/trackio-beacon-app/main.log' #TODO: uncomment before deployment
        log_file_name = './main.log' #TODO: comment before deployment
        logging.basicConfig(
            filename= log_file_name,
            filemode='w',
            level=logging.ERROR, 
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

    def _set_sea_level_pressure(self):
        if self.pressure_sensor_device is None:
            print('No pressure sensor device detected')
            return
        try:
            self.pressure_sensor_device.sea_level_pressure = self.sea_level_pressure_in_hg * self.INCH_HG_TO_HPA
            print('Was able to set sea level pressure')
        except Exception as e:
            logging.error("\tError in setting sea-level-pressure: "+str(e))
            print("Error in setting sea-level-pressure: "+str(e))
        return

    def find_pressure_sensor_via_i2c(self):
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.pressure_sensor_device = adafruit_bme280.Adafruit_BME280_I2C(i2c, address=self.BME280_I2C_ADDRESS)
            self.pressure_sensor_device.sea_level_pressure = self.sea_level_pressure_in_hg * self.INCH_HG_TO_HPA
            self.pressure_sensor_device.mode = adafruit_bme280.MODE_NORMAL
            self.pressure_sensor_device.standby_period = adafruit_bme280.STANDBY_TC_500
            self.pressure_sensor_device.iir_filter = adafruit_bme280.IIR_FILTER_X16
            self.pressure_sensor_device.overscan_pressure = adafruit_bme280.OVERSCAN_X16
            self.pressure_sensor_device.overscan_humidity = adafruit_bme280.OVERSCAN_X1
            self.pressure_sensor_device.overscan_temperature = adafruit_bme280.OVERSCAN_X2
            _= self.pressure_sensor_device.altitude
        except Exception as e:
            logging.error("\tfind_pressure_sensor_via_i2c(..):"+str(e))
            self.pressure_sensor_device = None
            print('Could not find pressure sensor via I2C')
            return False
        print('Found pressure sensor via I2C')
        return True

    def find_imu_via_i2c(self):
        try:
            i2c = busio.I2C(board.SCL, board.SDA)
            self.imu_device = adafruit_bno055.BNO055_I2C(i2c)
            #_ = self.imu_device.linear_acceleration
        except Exception as e:
            logging.error("\tfind_imu_via_i2c(..):"+str(e))
            print('Could not find IMU via I2C')
            return False
        print('Found IMU via I2C')
        return True

    def is_my_ip_set(self, interface_name='w'):
        print(ni.interfaces())
        print(ni)
        print(ni.AF_INET)
        print(ni.ifaddresses('wlan0'))
        print(ni.ifaddresses('lo'))
        print(ni.ifaddresses('eth0'))
        for iface_name in ni.interfaces():
            if interface_name in iface_name and ni.AF_INET in ni.ifaddresses(iface_name):
                iface_ip = ni.ifaddresses(iface_name)[ni.AF_INET][0]['addr']
                print('interface name' + interface_name)
                print('interface ip' + iface_ip)
                if iface_ip:
                    self.my_ip = str(iface_ip)
                    return True
        return False

    #----------------floor-assignment-related------------------------------------#
    def _check_floor_change(self):
        '''
        check total change in altitude > threshold, change the floor
        :return:
        '''
        if len(self.altitude_history)< 3:
            return
        cur_altitude = np.median( [i for i in islice(self.altitude_history,  len(self.altitude_history)-3, len(self.altitude_history) )] )
        #logging.debug("\tcur_altitude: "+str(self.altitude_history))
        #logging.debug("\tcur_altitude: "+str(cur_altitude)+" cur_floor"+str(self.current_floor))
        for indx in range(2, len(self.altitude_history)):
            prev_altitude = np.median( [ i for i in islice( self.altitude_history, indx-2 , indx+1)] )
            if np.abs( cur_altitude - prev_altitude ) > self.floor_change_threshold_meter:
                if cur_altitude - prev_altitude < 0.:
                    self.current_floor = max(0, self.current_floor-1)
                else:
                    self.current_floor = min(self.max_floor, self.current_floor+1)
                self.altitude_history = deque([], maxlen=self.MAX_ALTITUDE_HISTORY_LEN)
                self.altitude_history.append(cur_altitude)
                self.config['DEFAULT']['current_floor'] = str(self.current_floor)
                self._save_config()
                #logging.debug("\t\talt-change:"+str(self.current_floor))
                break
        return

    #---------program-entry: triggers all threads-------------------------#
    def init_handler(self):
        self.init_logging()
        print("RPI program started")
        print(self.my_ip)
        # block until ip is assigned
        while True:
            if self.is_my_ip_set() == True:
                break
            time.sleep(1)

        print("Found IP: " + str(self.my_ip) + " port: " + str(self.my_udp_port))
        #print("Found IP: " + str(self.my_ip) + " port: " + str(self.my_udp_port))

        retries = 10
        while True:
            retries -= 1
            if self.find_dw1000_via_usb():
                self.set_uwb_node_id()
                print("UWB  device found on USB port: " + self.uwb_usb_port_id)
                break
            elif retries > 0:
                time.sleep(1)
            else:
                logging.error("UWB Device not connected to USB, quitting service!!")
                exit(1)

        self.imu_device = None
        retries = 5
        while True:
            retries -= 1
            if self.find_imu_via_i2c():
                print("IMU sensor found")
                break
            elif retries > 0:
                time.sleep(1)
            else:
                logging.error("IMU(BNO055) not connected to I2C")
                break #still allows to continue without pressure sensor

        self.pressure_sensor_device = None
        retries = 5
        while True:
            retries -= 1
            if self.find_pressure_sensor_via_i2c():
                print("Altitude sensor found")
                break
            elif retries > 0:
                time.sleep(1)
            else:
                logging.error("Altitude Sensor(BME280) not connected to I2C")
                break #still allows to continue without pressure sensor
                #exit(1)

        # now bind the ip and port
        self.udp_rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.udp_rx_sock.bind((str(self.my_ip), int(self.my_udp_port)))
        self.udp_tx_sock = self.udp_rx_sock

        return

    #-----------------------thread: controller_command_handler-----------------------------#
    def controller_command_handler_thread(self):
        '''

        :return:
        '''
        print("controller command handler thread started")
        #print("controller command handler thread started")
        #while not self.stop_signal_received.is_set():

        temp_uwb_angle = []


        while True:
            #time.sleep(0) # for thread yielding
            cur_udp_cmd = ''
            cur_udp_msg_arg = ''
            cur_udp_cmd_seq_no = ''
            cur_udp_msg_ip = ''
            cur_udp_msg_port = ''
            resp_udp_msg = ''
            self.event_udp_msg_available.wait(timeout=1)
            if not self.event_udp_msg_available.is_set():
                time.sleep(0)  # force thread yield
                continue
            else:
                print(self.event_udp_msg_available.is_set())
            #-copy messages to local data-structures
            self.event_udp_msg_available.clear()
            #logging.debug('BEGIN UDP msg processing')
            with self.lock_udp_msg:
                cur_udp_msg_arg = str(self.udp_msg_arg)
                cur_udp_cmd = str(self.udp_msg_cmd)
                cur_udp_cmd_seq_no = int(self.udp_cmd_seq_no)
                cur_udp_msg_ip = str(self.udp_msg_ip)
                cur_udp_msg_port = int(self.udp_msg_port)
                etime = round(time.time() - self.udp_msg_rx_ts, 3)

            #logging.debug('\tcmd :'+str(cur_udp_cmd))
            #logging.debug('\tmsg   :' + str(cur_udp_msg_arg))
            #logging.debug('\tseq no:' + str(cur_udp_cmd_seq_no))
            #logging.debug('\tetime :' + str(etime)+' sec')
            if etime > self.UDP_MSG_LIVENESS_MSEC/1000. :
                #logging.debug('\t\t etime over threashold ('+str(self.UDP_MSG_LIVENESS_MSEC / 1000.)
                #                                         +') skipping processing!!')
                continue
            resp_udp_msg = '' #format: seq# cmd_str my_node_id response_str
            if 'range' in cur_udp_cmd:
                uwb_range_str = ' '
                rlist_str = cur_udp_msg_arg
                rlist = rlist_str.split()
                if rlist:
                    rlist = [int(i) for i in rlist]
                    uwb_range_str = self.get_uwb_ranges(nlist=rlist,
                                                        slot_time_msec=self.slot_time_msec)

                neighbor_stat = self.get_uwb_neighbor_stats()
                #cur_heading, cur_lin_a, cur_altitude = -1., 0., self.current_floor
                #cur_altitude = self.current_floor
                resp_udp_msg = str(cur_udp_cmd_seq_no)+" : "+cur_udp_cmd+" "\
                               +str(self.uwb_node_id)+" = "+uwb_range_str+" ; "+\
                               neighbor_stat+" ; "+str(self.imu_location[0])+" , "+str(self.imu_location[1])+\
                               "; "+str(self.current_floor)

                print('UDP Message: ', resp_udp_msg)

                #resp_udp_msg = str(cur_udp_cmd_seq_no)+" : "+cur_udp_cmd+" "\
                #               +str(self.uwb_node_id)+" = "+uwb_range_str+" ; "+\
                #               neighbor_stat+" ; "+str( round(cur_heading, 2) )+" , "+str( round(cur_lin_a, 2) )+\
                #               "; "+str(self.current_floor)

                #logging.debug("\t\trecv range info for seq# "+str(cur_udp_cmd_seq_no)+" as "+resp_udp_msg)
                self.udp_tx_sock.sendto(str.encode(resp_udp_msg), (cur_udp_msg_ip, cur_udp_msg_port))
                #logging.debug("\tsent udp response for seq# "+str(cur_udp_cmd_seq_no))

            elif 'start' in cur_udp_cmd:
                self.set_uwb_node_id() #shows IP address on LCD screen

            elif 'send-node-id' in cur_udp_cmd:
                #logging.debug("\t\tsetting uwb node id")
                self.set_uwb_node_id()
                #logging.debug("\t\tfound uwb node id:"+str(self.uwb_node_id))
                resp_udp_msg = str(cur_udp_cmd_seq_no)+" : "+cur_udp_cmd+" "\
                               +str(self.uwb_node_id)+" = "+str(self.uwb_node_id)
                #logging.debug("\t\tpreparing to send resp_msg: " + str(resp_udp_msg))
                self.udp_tx_sock.sendto(str.encode(resp_udp_msg), (cur_udp_msg_ip, cur_udp_msg_port))
                #logging.debug("\tsent node-id as resp-msg: "+str(resp_udp_msg))

            elif 'param' in cur_udp_cmd:
                got_new_param = False
                param_name_val_pairs = cur_udp_msg_arg.split(',')
                for pnvp in param_name_val_pairs:
                    param_vals = pnvp.split()
                    if 'sea-level-pressure-inch-hg' in param_vals[0]:
                        self.sea_level_pressure_in_hg = float(param_vals[1])
                        self._set_sea_level_pressure()
                        self.config['DEFAULT']['sea_level_pressure_inch_hg'] = str(self.sea_level_pressure_in_hg)
                        got_new_param = True
                        #logging.debug("new sea level pressure inch hg " + param_vals[1])
                    elif 'current-floor' in param_vals[0]:
                        with self.lock_altitude_history:
                            self.current_floor = int(param_vals[1])
                            #logging.debug("debug: rx current_floor: "+str(self.current_floor) )
                            self.altitude_history = deque([], self.MAX_ALTITUDE_HISTORY_LEN)
                        self.config['DEFAULT']['current_floor'] = str(self.current_floor)
                        got_new_param = True
                        #logging.debug("new current_floor " + param_vals[1])
                    elif 'anchor' in param_vals[0]:
                        self.anchors[int(param_vals[1])] = [int(param_vals[2]), int(param_vals[3])]
                        self.max_floor = max(self.anchors.keys())
                        self.config['DEFAULT']['max_floor'] = str(self.max_floor)
                        got_new_param = True
                        #logging.debug("new anchors " + param_vals[1])
                    elif 'floor-change-threshold-meter' in param_vals[0]:
                        self.floor_change_threshold_meter = float(param_vals[1])
                        self.config['DEFAULT']['floor_change_threshold_meter'] = str(self.floor_change_threshold_meter)
                        got_new_param = True
                        #logging.debug("new floor-change-threshold-meter "+param_vals[1])

             #-----add-more-param-retrievals here--------------#
                if got_new_param:
                    #logging.debug("saving params inside command handler thread..")
                    self._save_config()
            #logging.debug('END udp msg processing')
            #print('pos' in cur_udp_cmd)
            elif 'pos' in cur_udp_cmd: #RKS added this
                cont_location = cur_udp_msg_arg.split(',')
                print("Received position from Controller: "+str(cont_location))
                #Update UWB angle
                try:
                    if((time.time() - self.prev_angle_reset_time) <= self.angle_reset_time):
                        y = float(self.controller_location[1] - float(cont_location[1]))
                        x = float(self.controller_location[0] - float(cont_location[0]))
                        print('X: ', x)
                        print('Y: ', y)
                        print('Controller Location: ', controller_location) 
                        if(y == 0.0 and x == 0.0):
                             cont_angle = 0
                        else:
                             #cont_angle = math.atan2(float(cont_location[1]),float(cont_location[0]))
                             cont_angle = math.atan2(y,x)

                        temp_uwb_angle.append(cont_angle)
                        if(len(temp_uwb_angle) > 15):
                            temp_uwb_angle.pop(0)

                        self.uwb_angle = np.mean(temp_uwb_angle)
                        #self.uwb_angle = cont_angle

                except Exception as e:
                    print("\tException in Line:437 -->"+str(e))
                    sys.exit()

                self.controller_location = [float(cont_location[0]),float(cont_location[1])]
                self.controller_pos_set = True
                self.prev_angle_reset_time = time.time()

            elif 'force_loc' in cur_udp_cmd: #RKS added this
                cont_location = cur_udp_msg_arg.split(',')
                self.controller_location = [float(cont_location[0]),float(cont_location[1])]
                self.controller_pos_set = True
                self.imu_location = deepcopy(self.controller_location)
                self.prev_angle_reset_time = time.time()

        return

    def QuartRotate(self, acc_lst,quart_lst):
        rot_acc = []
        for acc, quart in zip(acc_lst, quart_lst):
            qart = Quaternion(w=quart[0],x=quart[1],y=quart[2],z=quart[3])
            conj_qart = qart.conjugate
            temp_rot_acc = conj_qart.rotate(acc)
            rot_acc.append([temp_rot_acc[0],temp_rot_acc[1]])
        return(rot_acc)
        #rotated_acc.append(conj_qart.rotate(acc_lst))

    def LP_Filter(self,acc):
        SamplingFreq = 65 #Hz
        signal_freq = 30 #5Hz
        b,a = butter(1, (2*signal_freq)/(SamplingFreq), 'low',analog=False,output='ba')
        acc = filtfilt(b, a, acc)
        return(acc)

    def HP_Filter(self,acc):
        SamplingFreq = 65 #Hz
        signal_freq = 20 #5Hz
        b,a = butter(1, (2*signal_freq)/(SamplingFreq), 'high',analog=False,output='ba')
        acc = filtfilt(b, a, acc)
        return(acc)

    def get_vel(self,cur_vel, acc_lst,delta_time):
        new_vel = [cur_vel]
        for acc, dt in zip(acc_lst, delta_time):
            if(math.sqrt(acc[0]**2+acc[1]**2) < self.zupt_threshold):
                new_vel.append([0.0,0.0])
            else:
                new_vel.append([new_vel[-1][0] + acc[0] * dt, new_vel[-1][1] + acc[1] * dt])

        new_vel.pop(0)
        return new_vel

    def get_pos(self,cur_pos, cur_vel, acc_lst,delta_time):
        new_vel = self.get_vel(cur_vel, acc_lst,delta_time)
        new_pos = [cur_pos]
        #Remove integral drift

        for vel,dt in zip(new_vel, delta_time):
            new_pos.append([new_pos[-1][0] + (float(vel[0]) * dt), new_pos[-1][1] + (float(vel[1]) * dt)])

        new_pos.pop(0)
        print('New pos: ', new_post)
        return (new_pos, new_vel)

    def rotate_IMU_trajectory(self,pos_lst):

        posX = pos_lst[-1][0] - pos_lst[-2][0]
        posY = pos_lst[-1][1] - pos_lst[-2][1]


        if(posX ==0 and posY == 0):
            return (pos_lst[-1][0], pos_lst[-1][1])

        else:
            imu_angle = math.atan2(posY, posX)

        diff_angle = self.uwb_angle - imu_angle

        if(diff_angle < 0):
            diff_angle += 2*math.pi

        posX = (posX * math.cos(diff_angle) - posY * math.sin(diff_angle))
        posY = (posX * math.sin(diff_angle) + posY * math.cos(diff_angle))

        #print("Line:529: Diff_angle, PosX, PosY = "+ str(diff_angle) + ","+str(posX) +"," + str(posY))

        return([posX, posY])


    #-----------------------thread: IMU+Alt_sensor_handler---------------------------------#
    def imu_alt_sensor_handler_thread(self):
        logging.debug("IMU+Alt sensor thread started")
        prev_pressure_reading_ts = time.time()
        #---------------------------------------imu refresh rate related------------------------------------#
        #debug_prev_imu_stat_ts = time.time()
        #debug_prev_time = time.time()
        #debug_max_gap_time = -float('inf')
        #debug_avg_gap_time = 0.
        #debug_imu_none_val_count = 0
        #----------------------------------------------------------------------------------------------------#
        temp_pos = [[0.0,0.0]]
        final_pos = [0.0,0.0]
        temp_vel = [[0.0,0.0]]
        delta_time = 0
        rot_acc = []        
        temp_acc = []
        deltatime_lst = []
        temp_quart = []
        prev_time = 0
        prev_reset_time = 0

        per_sec_disp = [] 


        rf = myKalman(pos = temp_pos,
              vel = temp_vel,
              acc = temp_acc,
              deltatime = delta_time)

        while True:
            time.sleep(0.0)  # thread yield
        # --------------------imu reading related------------------------------------------#
            try:
                self.cur_lin_acceleration = np.array(self.imu_device.linear_acceleration)
                self.cur_quaternion = np.array(self.imu_device.quaternion)

            except Exception as e:
                logging.error("\t\t IMU error: "+str(e))
                pass

            #print(time.time(), self.cur_lin_acceleration,self.cur_quaternion)

            try:
                if(len(temp_acc) > 0 and len(temp_quart) > 0):
                    if(np.linalg.norm(self.cur_lin_acceleration - temp_acc[-1]) == 0.0\
                         and np.linalg.norm(self.cur_quaternion - temp_quart[-1]) == 0.0):
                         continue
            except Exception as e:
                print('attempted to divide by zero, cur_lin_acceleration and cur_quaternion likely missing');
                print('Cur_lin_acceleration: ', cur_lin_acceleration);
                print('Cur_quaternion: ', cur_quaternion);
                continue

            if(len(deltatime_lst) < 1):
                deltatime_lst.append(0)
            else:
                delta_time = time.time() - prev_time
                deltatime_lst.append(delta_time)

            temp_time = time.time()
            #print(str(time.time()) + str(self.cur_lin_acceleration))

            if((self.cur_lin_acceleration[0]**2+self.cur_lin_acceleration[1]**2) > 4):
                continue

            temp_acc.append(self.cur_lin_acceleration)
            temp_quart.append(self.cur_quaternion)

            if(len(temp_acc) > 10):
                temp_acc.pop(0)
                temp_quart.pop(0)

            #if(len(temp_acc) > 50):
            #    temp_acc.pop(0)
            #    temp_quart.pop(0)
            
            try:
                rot_acc = self.QuartRotate(temp_acc,temp_quart)
            except:
                print('Could not perform QuartRotate');
                continue

            if((time.time() - prev_reset_time) >= self.imu_reset_pos_time and self.controller_pos_set == True):
                prev_reset_time = time.time()
                self.imu_location = deepcopy(self.controller_location)
                final_pos = deepcopy(self.controller_location)
                temp_pos = [deepcopy(self.controller_location)]
                temp_vel = [[0.0, 0.0]]
                temp_acc = [[0.0, 0.0, 0.0]]
                rot_acc = [[0.0,0.0]]
                temp_quart = [[0.0,0.0,0.0,0.0]]
                deltatime_lst = [deltatime_lst[-1]]
                rf.update_loc(self.imu_location)
                self.controller_pos_set = False
                per_sec_disp = []
                continue
            else:
                temp_pos, temp_vel = self.get_pos(temp_pos[-1],temp_vel[-1], rot_acc, deltatime_lst)

                if(len(temp_pos) > 2):
                    disp = np.linalg.norm(np.array(temp_pos[-1]) - np.array(temp_pos[-2]))
                    if(len(per_sec_disp) > 0):
                        if(disp > (np.mean(per_sec_disp) + 2*np.std(per_sec_disp))):
                            continue

                    per_sec_disp.append(disp)
                    if(len(per_sec_disp) > 50):
                        per_sec_disp.pop(0)

                prev_time = temp_time

                if(temp_vel[-1][0] == temp_vel[-1][1] == 0):
                    continue

                pos = np.array(self.rotate_IMU_trajectory(temp_pos))
                final_pos = np.array(final_pos) + pos
                self.imu_location = rf.run_kf(final_pos, temp_vel, rot_acc, delta_time)
                final_pos = deepcopy(self.imu_location)
                print("Loc, acc = " + str(self.imu_location) + ","+ str(rot_acc[-1]))


                #Program about statistics.


            #---------------------------------debug-imu-reading-stat----------------------------------------------------------------#
          #  for v in self.cur_lin_acceleration:
          #      if v is None:
          #          debug_imu_none_val_count += 1
          #  for v in self.cur_quaternion:
          #      if v is None:
          #          debug_imu_none_val_count += 1

          #  debug_gap_time = time.time() - debug_prev_time
          #  debug_prev_time = time.time()
          #  debug_avg_gap_time = round((debug_avg_gap_time+debug_gap_time)/2., 3)

          #  if debug_gap_time > debug_max_gap_time:
          #      debug_max_gap_time = debug_gap_time
          #  if time.time() - debug_prev_imu_stat_ts>10:
          #      debug_prev_imu_stat_ts = time.time()
          #      logging.debug("============>imu stat: avg. gap time: "+str(debug_avg_gap_time)+" max gap time: "+str(round(debug_max_gap_time, 3))+\
          #                    " NONE val count: "+str(debug_imu_none_val_count))
          #      debug_imu_none_val_count = 0
          #  logging.debug( "gap: "+str(round(debug_gap_time, 3))+" cur lin. acceleration: "+str(self.cur_lin_acceleration)+"  cur quaternion: "+str(self.cur_quaternion)  )
            #----------------------------------------------------------------------------------------------------------------------#
        #--------------------pressure reading related---------------------------------------#
            if time.time() - prev_pressure_reading_ts > 0.5 :
                try:
                    with self.lock_altitude_history:
                        cur_altitude = self.pressure_sensor_device.altitude
                        prev_pressure_reading_ts = time.time()
                        #logging.debug("\t\tcur altitude: " + str(round(cur_altitude,3))+"===============================================")
                        self.altitude_history.append(cur_altitude)
                        self._check_floor_change()
                except Exception as e:
                    logging.error("\t\t Pressure sensor error: "+str(e))
                    pass
        return

    #-------------------------thread: UDP_packet_reciever-----------------------------------#
    def udp_pkt_receiver_thread(self):
        print('udp pkt receiver thread started')
        logging.debug("udp pkt receiver thread started")
        while True:
            time.sleep(0)  # force thread yield
            data, src_address = self.udp_rx_sock.recvfrom(1024)
            data = data.decode('ascii').strip()
            #logging.debug("Received udp pkt, data="+data)
            if len(data) == 0:
                continue
            src_ip, src_port = src_address
            src_ip, src_port = str(src_ip), int(src_port)
            seq_no, data = data.split(':')
            hdr, payload = data.split('=')
            hdr = hdr.split()
            if len(hdr)<2:
                continue
            target_node = int(hdr[1])
            if target_node == self.uwb_node_id or target_node  == -1:
                with self.lock_udp_msg:
                    self.udp_msg_cmd = str(hdr[0].strip())
                    self.udp_msg_arg = str(payload.strip())
                    self.udp_cmd_seq_no = int(seq_no)
                    self.udp_msg_rx_ts = time.time()
                    self.udp_msg_ip, self.udp_msg_port = src_ip, src_port
                self.event_udp_msg_available.set()
        return

    #----------------------------UWB USB API methods ------------------------------------#
    def get_uwb_usb_data(self, usb_msg, usb_read_timeout):
        usb_rx_msg = ''
        is_reading_log = False
        is_last_usb_line = False
        print('Getting UWB USB data')
        with serial.Serial(self.uwb_usb_port_id,
                           baudrate=self.USB_BAUDRATE,
                           timeout=self.USB_OP_LATENCY_MSEC/1000) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()

            usb_str = str(usb_msg)+'\n'
            ser.write(usb_str.encode())
            #print("\t\t\t\tUSB-Sent: " +usb_msg+" for tout:"+str(usb_read_timeout))  # print the log

            finishing_ts = time.time()+usb_read_timeout
            while(time.time()<finishing_ts):
                """print("readline: " ser.readline())"""
                cur_usb_rx_msg = ser.readline()

                cur_usb_rx_msg = cur_usb_rx_msg.decode('ascii').strip()
                print("Cur USB Message: ", cur_usb_rx_msg)
                if len(cur_usb_rx_msg) == 0: #did not receive anything, uwb perhaps got disconnected
                    time.sleep(usb_read_timeout/1000.)
                    continue

                if self.USB_LOG_STR in cur_usb_rx_msg:
                    is_reading_log = True

                if "GOL" in cur_usb_rx_msg:
                    is_reading_log = False
                    continue

                if is_reading_log:
                    # if self.enable_uwb_log_printing:
                    #     print("\t\t\t\t"+cur_usb_rx_msg) #print the log
                    continue

                if not is_reading_log:
                    #print("\t\t\t\tnon-log-msg: " + cur_usb_rx_msg)
                    if ':' in cur_usb_rx_msg:
                        self.usb_rx_counter, cur_usb_rx_msg = cur_usb_rx_msg.split(':')
                        self.usb_rx_counter = int(self.usb_rx_counter)

                    if self.USB_TERMINATION_STR in cur_usb_rx_msg:
                        cur_usb_rx_msg = cur_usb_rx_msg.replace(self.USB_TERMINATION_STR, '')
                        is_last_usb_line = True
                    print('Current USB Message: ', cur_usb_rx_msg)   
                    usb_rx_msg += cur_usb_rx_msg
                    if is_last_usb_line:
                        break
        print('USB Message: ', usb_rx_msg)
        return usb_rx_msg

    def set_uwb_node_id(self, ignore_disconnection_flag =  False):
        '''
        :return:
        '''
        self.usb_cmd_counter = (self.usb_cmd_counter + 1) % 256
        usb_msg = str(self.CMD_SEND_ID)+' '+str(self.usb_cmd_counter)+' '+str(self.my_ip)

        usb_read_timeout = 10*self.USB_OP_LATENCY_MSEC/1000.0
        uwb_response = self.get_uwb_usb_data(usb_msg, usb_read_timeout=usb_read_timeout)

        if not '=' in uwb_response:
            print("Unable to read node-ID from UWB via USB!!")
            logging.error("Unable to read node-ID from UWB via USB!!")
            self.uwb_node_id =  -1
            return
        usb_cmd_plus_seq, src_node_id = uwb_response.split('=')
        usb_cmd, usb_cmd_seq_no = usb_cmd_plus_seq.split()
        usb_cmd, usb_cmd_seq_no = int(usb_cmd), int(usb_cmd_seq_no)
        if(usb_cmd == self.CMD_SEND_ID  and usb_cmd_seq_no == self.usb_cmd_counter):
            self.uwb_node_id = int(src_node_id)
        return

    def get_uwb_neighbor_stats(self):
        self.usb_cmd_counter = (self.usb_cmd_counter + 1) % 256
        usb_msg = str(self.CMD_SEND_NLIST)+' '+str(self.usb_cmd_counter)

        usb_read_timeout = 4 * self.USB_OP_LATENCY_MSEC/1000.
        uwb_response = self.get_uwb_usb_data(usb_msg, usb_read_timeout=usb_read_timeout)
        print('UWB Response Stats: ', uwb_response)
        if not '=' in uwb_response:
            return  ''
        usb_cmd_plus_seq, nresponse_str = uwb_response.split('=')
        usb_cmd, usb_cmd_seq_no = usb_cmd_plus_seq.split()
        usb_cmd, usb_cmd_seq_no = int(usb_cmd), int(usb_cmd_seq_no)

        if( usb_cmd == self.CMD_SEND_NLIST and usb_cmd_seq_no == self.usb_cmd_counter):
            return nresponse_str
        return ''

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

            if self.enable_uwb_log_printing:
                usb_read_timeout += (2*nlist_count+2)*40*self.USB_OP_LATENCY_MSEC/1000.
            uwb_response = self.get_uwb_usb_data(usb_msg, usb_read_timeout=usb_read_timeout)
            print('UWB Response: ', uwb_response)
            if not '=' in uwb_response:
                return  ''
            usb_cmd_plus_seq, nr_list_str = uwb_response.split('=')
            usb_cmd, usb_cmd_seq_no = usb_cmd_plus_seq.split()
            usb_cmd, usb_cmd_seq_no = int(usb_cmd), int(usb_cmd_seq_no)

            if( usb_cmd == self.CMD_RANGE and usb_cmd_seq_no == self.usb_cmd_counter):
                print('NR List str: ', nr_list_str)
                return nr_list_str
        return ''

    #---------------------------------------------------------------------------------------------------#
    def run_handler(self):
        self.init_handler()
        #theads spawned
        self.my_threads['imu_alt_handler'] = threading.Thread(name='IMU-ALT-HNDLR',
                                                              target=self.imu_alt_sensor_handler_thread, daemon=True)
        if not self.pressure_sensor_device is None and not self.imu_device is None:
            self.my_threads['imu_alt_handler'].start()

        self.my_threads['controller_cmd_handler'] = threading.Thread(name="MSG-HNDLR",
                                                                     target=self.controller_command_handler_thread, daemon=True)
        self.my_threads['controller_cmd_handler'].start()

        self.my_threads['udp_pkt_recv_handler'] = threading.Thread(name='UDP-PKT-RECVR',
                                                                   target=self.udp_pkt_receiver_thread, daemon=True)
        self.my_threads['udp_pkt_recv_handler'].start()

        while True:
            time.sleep(1)

        #print('End of program')
        return

def start_handler():
    #------------------------------#
    udp_network_port = 12345
    uwb_slot_time_initial_msec = 8
    sensor_reading_interval_sec = 0.5
    sea_level_pressure_in_hg = 30.070

    #-------------------------------#
    rpih = RPIHandler(
                      udp_port=udp_network_port,
                      uwb_slot_time_msec=uwb_slot_time_initial_msec,
                      sensor_reading_interval_sec=sensor_reading_interval_sec,
                      sea_level_pressure_in_hg = sea_level_pressure_in_hg
                    )

    rpih.run_handler()
    return

if __name__ == '__main__':
    start_handler()
