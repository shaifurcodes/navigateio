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
    def __init__(self, n1, n2, n3, y_dir):
        self.init_logging()

        self.LORA_PORT = '/dev/ttyUSB0'
        try:
            self.lora_node = Lora(serial_port=self.LORA_PORT)
        except Exception as ex:
            logging.exception(ex)
            exit(1)
        self.y_dir = y_dir
        self.node_list = [n1, n2, n3]
        self.mobile_node = n3
        self.static_nodes = [n for n in self.node_list if n!=self.mobile_node]
        self.last_ranging_static_node_indx = 0
        self.x = [-1. for _ in self.node_list]
        self.y = [-1. for _ in self.node_list]
        self.z = [-1. for _ in self.node_list]

        self.LORA_RESP_WAIT_TIME_SEC = 2.5
        self.lora_msg_seq_no = 0
        self.controller_id = 0
        self.last_msg_marker = 'E'

        n = len(self.node_list)
        self.edm = np.zeros((n, n), dtype=float)
        self.MAX_RANG_FRESHNESS_SEC = 30
        self.ts_edm = np.full((n, n), fill_value=self.MAX_RANG_FRESHNESS_SEC, dtype=float)

        self.LORA_MSG_START_MARK = 'S'
        self.LORA_MSG_END_MARK = 'E'
        self.lora_incoming_msg_q = Queue()
        self.lora_expected_resp_src = -1
        self.lora_recv_msg_srcs = []
        self.loc_data_file = './location_data.txt'
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

    def save_location_data(self):
        with open(self.loc_data_file, 'a') as f:
            cur_ts = round(time.time() - self.init_ts, 3)
            for n1_indx, n1 in enumerate(self.node_list):
                f_text = str(cur_ts)+', '+str(n1)+', '+str(self.x[n1_indx])+', '+str(self.y[n1_indx])+', '+str(self.z[n1_indx])+'\n'
                f.write(f_text)
        logging.debug("saved data to file....")
        return

    def localize_node(self, n1, n2_list):
        try:
            if len(n2_list) <= 1:
                return
            n1_indx = self.node_list.index(n1)

            mulitlat_solver = lx.Project(mode='2D', solver='LSE_GC')
            n1_loc_sovler, _ = mulitlat_solver.add_target()

            for n2 in n2_list:
                n2_indx = self.node_list.index(n2)
                n2_x, n2_y = self.x[n2_indx], self.y[n2_indx]
                mulitlat_solver.add_anchor(str(n2), (n2_x, n2_y))

            for n2 in n2_list:
                n2_indx = self.node_list.index(n2)
                d = self.edm[n1_indx, n2_indx]
                if d<=0.:
                    return
                n1_loc_sovler.add_measure(str(n2), d)

            mulitlat_solver.solve()
            if n1_loc_sovler.loc is None:
                return
            n1_x, n1_y = n1_loc_sovler.loc.x, n1_loc_sovler.loc.y
            if self.y_dir <0:
                n1_y = -abs(n1_y)
            else:
                n1_y = abs(n1_y)
            self.x[n1_indx], self.y[n1_indx] = n1_x, n1_y
            logging.debug("solved for loc: "+str(n1_x)+", "+str(n1_y))
        except Exception as e:
            logging.exception("loc-solver error: "+str(e))
        return

    def localize(self):
        logging.debug("attempting localization...")
        self.x[0], self.y[0] = 0.,0.

        self.y[1] = 0.0
        if self.edm[0, 1] > 0.:
            self.x[1] = float(self.edm[0, 1])
        else:
            logging.debug("!!!self.edm[0,1]: "+str(self.edm[0, 1]))
            return
        if self.edm[2, 0]>0. and self.edm[2, 1]>0.:
            self.localize_node(n1=self.mobile_node, n2_list=self.static_nodes)
            self.save_location_data()
        else:
            logging.debug("!!!self.edm[2,0]: " + str(self.edm[2, 0]))
            logging.debug("!!!self.edm[2,1]: " + str(self.edm[2, 1]))
        return

    def process_lora_msg(self, msg):
        logging.debug("processing lora msg: "+str(msg))
        if not '=' in msg:
            logging.debug("!!! not '=' in msg ")
            return
        msg_prefix, msg_suffix = msg.split('=')
        msg_prefix = msg_prefix.split()
        if len(msg_prefix) != 4:
            logging.debug("!!! len(msg_prefix) != 4")
            return
        if int(msg_prefix[0]) != self.controller_id:
            logging.debug("!!! msg_prefix[0] != self.controller_id")
            return
        src, seq_no, cmd_type = int(msg_prefix[1]), int(msg_prefix[2]), msg_prefix[3]
        logging.debug("src, seq_no, cmd_type: "+str(src)+" "+str(seq_no)+" "+str(cmd_type))
        self.lora_recv_msg_srcs.append(src)
        src_indx = self.node_list.index(src)

        if self.lora_msg_seq_no > seq_no+2:
            logging.debug("!!! self.lora_msg_seq_no > seq_no+2:")
            return

        if not ';' in msg_suffix:
            logging.debug("!!! not ';' in msg_suffix")
            return
        msg_suffix = msg_suffix.split(';')

        if cmd_type == 'r':
            logging.debug("processing ranges..")
            n1_r1_pairs = msg_suffix.split[0](',')
            for n1_r1 in n1_r1_pairs:
                n1, r1 = n1_r1.split()
                n1, r1 = int(n1), float(r1)
                if r1 == 0: continue
                n1_indx = self.node_list.index(n1)
                self.edm[n1_indx, src_indx] = r1 / 100.
                self.edm[src_indx, n1_indx] = r1 / 100.
                logging.debug("self.edm[src_indx, n1_indx] = self.edm[n1_indx, src_indx] = "+str(self.edm[n1_indx, src_indx]))
                self.ts_edm[src_indx, n1_indx] = self.ts_edm[n1_indx, src_indx] = round(time.time() - self.init_ts, 3)
        if len(msg_suffix) < 2:
            return
        if msg_suffix[1]:
            z_data = msg_suffix[1].split()
            if len(z_data)==2 and 'z' in z_data[0]:
                self.z[src_indx] = float(z_data[1])
        return

    def send_range_cmd(self, src, nlist):
        nlist_str = ''
        for n in nlist:
            nlist_str = nlist_str+' '+ str(n)
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
            if self.lora_node.is_lora_msg_available():
                lora_recv_msgs = self.lora_node.lora_receive()
                if not lora_recv_msgs:
                    return
                #logging.debug("lora-msg: " + lora_recv_msgs)
                for cindx, c in enumerate(lora_recv_msgs):
                    if c == self.last_msg_marker or c == self.LORA_MSG_START_MARK:
                        self.cur_lora_msg = ''
                        self.last_msg_marker = c
                    elif c == self.LORA_MSG_END_MARK:
                        if self.cur_lora_msg:
                            logging.debug("cur_lora_msg: " + self.cur_lora_msg)
                            self.lora_incoming_msg_q.put(str(self.cur_lora_msg))
                        self.cur_lora_msg = ''
                        self.last_msg_marker = c
                    elif self.last_msg_marker == self.LORA_MSG_START_MARK:
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
                self.lora_expected_resp_src = src
                finishing_ts = time.time() + self.LORA_RESP_WAIT_TIME_SEC
                self.lora_recv_msg_srcs = []
                while time.time() <= finishing_ts:
                    self.enq_lora_msgs()
                    while not self.lora_incoming_msg_q.empty():
                        self.process_lora_msg(msg = self.lora_incoming_msg_q.get())
                    if self.lora_expected_resp_src in self.lora_recv_msg_srcs:
                        break
                self.localize()
            except Exception as ex:
                logging.exception(ex)
                continue
        return

if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigint_handler)
    if len(sys.argv) < 5:
        print("Program requires 4 arguements: origin x-axis mobile y-direction for example 'sudo python3 main 2 4 6 +1' ")
        exit(0)
    try:
        n1 =  int(sys.argv[1])
        n2 = int(sys.argv[2])
        n3 = int(sys.argv[3])
        y_dir = 1
        if  int(sys.argv[4]) <0:
            y_dir = -1
        frst_controller = FRSTController(n1, n2, n3, y_dir)
        frst_controller.run_controller()
    except Exception as ex:
        print(ex)
        exit(1)
