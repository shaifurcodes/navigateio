import logging
import serial

try:
    import RPi.GPIO as GPIO
except:
    pass

class Lora:
    def __init__(self, serial_port):
        serial_read_timeout_sec = 0.1
        self.is_rpi = False
        if 'ttyAMA0' in serial_port: #TODO: if tty in portname, assume rpi, revisit to make code better
            self.is_rpi = True
        if self.is_rpi:
            serial_read_timeout_sec = 0.2
        try:
            self.ser = serial.Serial(port=serial_port,
                                         baudrate= 9600,
                                         bytesize=serial.EIGHTBITS,
                                         parity=serial.PARITY_NONE,
                                         stopbits=serial.STOPBITS_ONE,
                                         timeout=serial_read_timeout_sec,
                                         xonxoff=True,
                                         rtscts=False,
                                         write_timeout=None,
                                         dsrdtr=False,
                                         inter_byte_timeout=None,
                                         exclusive=None)
        except Exception as ex:
            logging.exception(ex)
            exit(1)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

        self.M0 = 22
        self.M1 = 27
        if self.is_rpi:
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            GPIO.setup(self.M0, GPIO.OUT)
            GPIO.setup(self.M1, GPIO.OUT)

            GPIO.output(self.M1, GPIO.LOW)
            GPIO.output(self.M0, GPIO.LOW)

    def lora_send(self, data):
        lora_data = (str(data)).encode(encoding='utf-8')
        self.ser.reset_output_buffer()
        self.ser.write(lora_data)
        return

    def lora_receive(self):
        recv_msg = self.ser.read(size=self.ser.in_waiting)
        #self.ser.reset_input_buffer()
        recv_msg = recv_msg.decode(encoding='utf-8', errors='ignore')
        return recv_msg

    def is_lora_msg_available(self):
        if self.ser.in_waiting >0:
            return  True
        return False

    def close_serial_port(self):
        self.ser.close()
        return

