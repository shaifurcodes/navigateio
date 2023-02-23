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
        self.node_list = [2, 3, 4]
        self.LORA_RESP_WAIT_TIME_SEC = 1.0
        return

    # def run_simple_raning(self):
    #     msg_counter = 0
    #     while True:
    #         try:
    #             self.lora_node.lora_send("cb 2 4") #TODO: ENSURE FORMAT cb
    #             time.sleep(5) #TODO: DECREASE SLEEEP TIME
    #             lora_msg = self.lora_node.lora_receive()
    #             msg_counter += 1
    #             print(str(msg_counter)+" lora_msg:"+str(lora_msg))
    #         except Exception as ex:
    #             print(ex)
    #             self.lora_node.close_serial_port()

    def run_ranging(self):
        while True:
            try:
                time.sleep(1)
                for n1 in self.node_list:
                    lora_msg='cb '+str(n1)
                    for n2 in self.node_list:
                        if n1!=n2:
                            lora_msg =lora_msg+' '+str(n2)
                    self.lora_node.lora_send(lora_msg)
                    finishing_ts = time.time()+self.LORA_RESP_WAIT_TIME_SEC
                    while time.time() < finishing_ts:
                        lora_msg = self.lora_node.lora_receive()
                        if not lora_msg:
                            continue
                        lora_msg = ''.join(letter for letter in lora_msg if letter.isalnum() or letter in [',',' ']) #TODO: recheck if characters are dropped
                        if 'bc' in lora_msg:
                            range_vals = lora_msg.split('bc')[-1].strip()
                            print("ranging node:"+str(n1)+"==============")
                            print("\trange values: (node-id range-in-cm):"+range_vals)
                            break
            except Exception as ex:
                print(ex)
                self.lora_node.close_serial_port()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigint_handler)
    frst_controller = FRSTController()
    frst_controller.run_ranging()