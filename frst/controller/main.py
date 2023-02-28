#!/usr/bin/python3.8

import logging
import time
import sys
import signal
import numpy as np
from queue import  Queue
from lora_sx126x import Lora
import localization as lx

def sigint_handler(signal, frame):
    sys.exit(0)

class FRSTController(object):
    def __init__(self):
        self.init_logging()

        self.LORA_PORT = '/dev/ttyUSB0'
        try:
            self.lora_node = Lora(serial_port=self.LORA_PORT)
        except Exception as ex:
            logging.exception(ex)
            exit(1)

        self.node_list = [2, 3, 4, 6]
        self.mobile_node = 6
        self.static_nodes = [n for n in self.node_list if n!=self.mobile_node]
        self.last_ranging_static_node_indx = 0
        self.x, self.y, self.z = 0.,0.,0.

        self.LORA_RESP_WAIT_TIME_SEC = 2.5
        self.lora_msg_seq_no = 0
        self.controller_id = 0

        n = len(self.node_list)
        self.edm = np.zeros((n, n), dtype=np.float)
        self.MAX_RANG_FRESHNESS_SEC = 30
        self.ts_edm = np.full((n, n), fill_value=self.MAX_RANG_FRESHNESS_SEC, dtype=np.float)

        self.LORA_MSG_START_MARK = 'S'
        self.LORA_MSG_END_MARK = 'E'
        self.lora_incoming_msg_q = Queue()
        self.init_ts = time.time()
        return

    def init_logging(self):
        log_file_name = './controller.log'
        logging.basicConfig(
            filename= log_file_name,
            filemode='w',
            level=logging.DEBUG,
            format='%(asctime)s.%(msecs)03d, %(threadName)-10s: %(message)s',
            datefmt='%H:%M:%S')  # '%Y-%m-%d %H:%M:%S'
        return

    def localize(self):

        return

    def process_lora_msg(self, msg):
        if not '=' in msg:
            return
        msg_prefix, msg_suffix = msg.split('=')
        if len(msg_prefix) != 4:
            return
        msg_prefix = msg_prefix.split()
        if msg_prefix[0] != self.controller_id:
            return
        src, seq_no, cmd_type = int(msg_prefix[1]), int(msg_prefix[2]), msg_prefix[3]
        if self.lora_msg_seq_no > seq_no+2:
            return
        if not ';' in msg_suffix:
            return
        msg_suffix = msg_suffix.split(';')
        if cmd_type == 'r':
            src_indx = self.node_list.index(src)
            n1_r1_pairs = msg_suffix.split(',')
            for n1_r1 in n1_r1_pairs:
                n1, r1 = n1_r1.split()
                n1, r1 = int(n1), float(r1)
                if r1 == 0: continue
                n1_indx = self.node_list.index(n1)
                self.edm[src_indx, n1_indx] = self.edm[n1_indx, src_indx] = r1/100.
                self.ts_edm[src_indx, n1_indx] = self.ts_edm[n1_indx, src_indx] = round(time.time() - self.init_ts, 3)
        return

    def send_range_cmd(self, src, nlist):
        nlist_str = ''
        for n in nlist:
            nlist_str = nlist_str + str(n)
        lora_msg = 'S '+str(src)+' '+str(self.controller_id)+' '+str(self.lora_msg_seq_no)+' r = '+\
                                                                                        nlist_str+' E'
        self.lora_msg_seq_no += 1
        try:
            self.lora_node.lora_send(lora_msg)
            logging.debug(">>>>lora_msg:" + lora_msg)
        except Exception as ex:
            logging.exception(ex)
        return

    def enq_lora_msgs(self):
        try:
            finishing_ts = time.time() + self.LORA_RESP_WAIT_TIME_SEC
            while True:
                if time.time() > finishing_ts: break
                if self.lora_node.is_lora_msg_available(): break
                time.sleep(0.01)
                continue
            if not self.lora_node.is_lora_msg_available():
                return
            lora_recv_msgs = self.lora_node.lora_receive()
            if not lora_recv_msgs:
                return
            for cindx, c in enumerate(lora_recv_msgs):
                if c==self.last_msg_marker or c==self.LORA_MSG_START_MARK:
                    self.cur_lora_msg = ''
                    self.last_msg_marker = c
                elif c==self.LORA_MSG_END_MARK:
                    if self.cur_lora_msg:
                        logging.debug("cur_lora_msg: "+self.cur_lora_msg)
                        self.lora_incoming_msg_q.put(str(self.cur_lora_msg))
                    self.cur_lora_msg = ''
                    self.last_msg_marker = c
                elif self.last_msg_marker==self.LORA_MSG_START_MARK:
                    self.cur_lora_msg = self.cur_lora_msg + c
        except Exception as ex:
            logging.exception(ex)
        return

    def select_nodes_to_range(self):
        if self.lora_msg_seq_no%5==0:
            self.last_ranging_static_node_indx = \
                (self.last_ranging_static_node_indx + 1) % len(self.static_nodes)
            src =  self.static_nodes[self.last_ranging_static_node_indx]
            nlist = [n for n in self.node_list if n!=src]
            return src, nlist
        return self.mobile_node, self.static_nodes

    def run_controller(self):
        while True:
            try:
                src, nlist = self.select_nodes_to_range()
                self.send_range_cmd(src=src, nlist=nlist)
                self.enq_lora_msgs()
                while not self.lora_incoming_msg_q.empty():
                    self.process_lora_msg(msg = self.lora_incoming_msg_q.get())
                self.localize()
            except Exception as ex:
                logging.exception(ex)
                continue
        return

if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigint_handler)
    frst_controller = FRSTController()
    frst_controller.run_controller()