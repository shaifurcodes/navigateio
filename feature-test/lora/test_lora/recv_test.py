import sys
import signal
import time
from sx126x import Lora


def sigint_handler(signal, frame):
    print('Exiting program.....')
    sys.exit(0)

def recv_test():
    if len(sys.argv) < 2:
        print("Exiting....serial port number not found!!")
        exit()

    lora_node = Lora(serial_port=str(sys.argv[1]))

    print("=========recv-test====================(press CTRL+C to terminate program)")
    while True:
        try:
            time.sleep(0.1)
            lora_msg = lora_node.lora_receive()
            msgs = []
            filtered_msg = ''.join(letter for letter in lora_msg if letter.isalnum() or letter in [',', ' ','=']) #TODO: recheck if characters are dropped
            if 'E' in filtered_msg:
                msgs = filtered_msg.split('E')
            else:
                msgs = [filtered_msg]
            for msg in msgs:
                if msg:
                    print(msg)
        except Exception as ex:
            print(ex)
            break
    return


if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigint_handler)
    recv_test()
