from sx126x import Lora
import time
import sys
import signal

def sigint_handler(signal, frame):
    print('Exiting program.....')
    sys.exit(0)

def send_test():
    if len(sys.argv) < 2:
        print("Exiting....serial port number not found!!")
        exit()

    lora_node = Lora(serial_port=str(sys.argv[1]))
    my_count = 0
    my_string = " Hello"
    print("*********** send-test ********************** (press CTRL+C to terminate program)")
    while True:
        try:
            time.sleep(2)
            lora_msg = str(my_count)+my_string
            print(lora_msg)
            lora_node.lora_send(lora_msg)
            my_count += 1
        except Exception as ex:
            print(ex)
            lora_node.close_serial_port()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, sigint_handler)
    send_test()
