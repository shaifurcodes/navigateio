from sx126x import Lora
import time
import sys
import signal

def sigint_handler(signal, frame):
    print('Exiting program.....')
    sys.exit(0)

class FRSTController(object):
    def __init__(self):
        self.LORA_PORT = '/dev/ttyUSB0'
        self.lora_node = Lora(serial_port=self.LORA_PORT)
        self.node_list = [2, 4, 6]
        self.LORA_RESP_WAIT_TIME_SEC = 2.0
        self.lora_msg_seq_no = 0
        self.controller_id = 0

        self.LORA_CMD_RANGE = 'r'
        self.LORA_PERMITTED_CHARS_SET = [' ', ',','=']

        self.init_ts = time.time()
        return

    def process_lora_range_response(self, src_node, range_response):
        '''
        :param src_node:
        :param range_response: "2 2323, 3 4343" etc.
        :return:
        '''
        node_range_pair = []
        if ',' in range_response:
            node_range_pair = range_response.split(',')
        else:
            node_range_pair = range_response
        for n1_r1 in node_range_pair:
            n1_r1 = n1_r1.split()
            if len(n1_r1)==2:
                cur_ts = round(time.time() - self.init_ts, 3)
                print(str(cur_ts)+" "+str(src_node)+" "+str(n1_r1[0])+" "+str(  round(float(n1_r1[1])/100.0, 3) )+" meter")
                #TODO: also write in file as csv format
        return

    def process_lora_msg(self, lora_msgs):
        '''
        task:
            1. separate messages by end marker
            2. for each msg:
                a) if target-reply, set the flag
                b) if range, print or log range
        :return:
        '''
        is_lora_response_latest =  False
        if not 'E' in lora_msgs:
            return  is_lora_response_latest
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
            if not 'bc' in msg_prefix[0]:
                continue
            if int(msg_prefix[1]) == self.lora_msg_seq_no:
                is_lora_response_latest = True
            lora_src_node = int(msg_prefix[2])
            if int(msg_prefix[3]) != self.controller_id:
                continue
            lora_cmd_type = str(msg_prefix[4])

            if lora_cmd_type=='r':
                self.process_lora_range_response(lora_src_node, msg_suffix)
            elif lora_cmd_type=='s':
                continue
            else:
                continue
        return is_lora_response_latest


    def run_controller(self):
        while True:
            try:
                time.sleep(0.1)
                for n1 in [4]: #self.node_list: #TODO: restore
                    self.lora_msg_seq_no += 1
                    lora_send_msg='cb '+str(self.lora_msg_seq_no)+' '+str(self.controller_id)+' '+str(n1)+' '+str(self.LORA_CMD_RANGE)+' ='
                    for n2 in [2, 6] # self.node_list: #TODO: restore
                        if n1!=n2:
                            lora_send_msg =lora_send_msg+' '+str(n2)
                    lora_send_msg =  lora_send_msg + ' E'
                    print("debug: lora-msg:"+lora_send_msg)
                    self.lora_node.lora_send(lora_send_msg)
                    finishing_ts = time.time()+self.LORA_RESP_WAIT_TIME_SEC
                    while time.time() <= finishing_ts:
                        time.sleep(0.1)
                        lora_recv_msgs = self.lora_node.lora_receive()
                        if not lora_recv_msgs:
                            continue
                        lora_recv_msgs = ''.join(letter for letter in lora_recv_msgs if letter.isalnum() or letter in self.LORA_PERMITTED_CHARS_SET)
                        print("debug:recv-lora-msg:"+str(lora_recv_msgs))
                        if self.process_lora_msg(lora_recv_msgs):
                            break
            except Exception as ex:
                print(ex)
                self.lora_node.close_serial_port()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigint_handler)
    frst_controller = FRSTController()
    frst_controller.run_controller()