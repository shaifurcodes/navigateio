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
        self.node_list = [2, 4]
        self.LORA_RESP_WAIT_TIME_SEC = 0.4
        return

    def run_simple_raning(self):
        while True:
            try:
                self.lora_node.lora_send("2 4")
                time.sleep(1)
                lora_msg = self.lora_node.lora_receive()
                print(lora_msg)
            except Exception as ex:
                print(ex)
                self.lora_node.close_serial_port()

    def run_ranging(self):
        while True:
            try:
                for n1 in self.node_list:
                    lora_msg=str(n1)
                    for n2 in self.node_list:
                        if n1!=n2:
                            lora_msg =lora_msg+' '+str(n2)
                    self.lora_node.lora_send(lora_msg)
                    finishing_ts = time.time()+self.LORA_RESP_WAIT_TIME_SEC
                    while time.time() < finishing_ts:
                        lora_msg = self.lora_node.lora_receive()
                        lora_msg = ''.join(letter for letter in lora_msg if letter.isalnum() or letter in [',', ' ']) #TODO: recheck if characters are dropped
                        if lora_msg:
                            print("src-node: "+str(n1)+"=========")
                            print(lora_msg)
                            break
            except Exception as ex:
                print(ex)
                self.lora_node.close_serial_port()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigint_handler)
    frst_controller = FRSTController()
    frst_controller.run_simple_raning()