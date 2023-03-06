from math import log
try:
    import smbus
except Exception:
    pass

class LPS22HB(object):
    def __init__(self):
        # i2c address
        self.LPS22HB_I2C_ADDRESS = 0x5C
        #
        self.LPS_ID = 0xB1
        # Register
        self.LPS_INT_CFG = 0x0B  # Interrupt register
        self.LPS_THS_P_L = 0x0C  # Pressure threshold registers
        self.LPS_THS_P_H = 0x0D
        self.LPS_WHO_AM_I = 0x0F  # Who am I
        self.LPS_CTRL_REG1 = 0x10  # Control registers
        self.LPS_CTRL_REG2 = 0x11
        self.LPS_CTRL_REG3 = 0x12
        self.LPS_FIFO_CTRL = 0x14  # FIFO configuration register
        self.LPS_REF_P_XL = 0x15  # Reference pressure registers
        self.LPS_REF_P_L = 0x16
        self.LPS_REF_P_H = 0x17
        self.LPS_RPDS_L = 0x18  # Pressure offset registers
        self.LPS_RPDS_H = 0x19
        self.LPS_RES_CONF = 0x1A  # Resolution register
        self.LPS_INT_SOURCE = 0x25  # Interrupt register
        self.LPS_FIFO_STATUS = 0x26  # FIFO status register
        self.LPS_STATUS = 0x27  # Status register
        self.LPS_PRESS_OUT_XL = 0x28  # Pressure output registers
        self.LPS_PRESS_OUT_L = 0x29
        self.LPS_PRESS_OUT_H = 0x2A
        self.LPS_TEMP_OUT_L = 0x2B  # Temperature output registers
        self.LPS_TEMP_OUT_H = 0x2C
        self.LPS_RES = 0x33  # Filter reset register

        self._address = self.LPS22HB_I2C_ADDRESS
        self._bus = smbus.SMBus(1)
        self.reset()                         #Wait for reset to complete
        self._write_byte(self.LPS_CTRL_REG1 ,0x02)        #Low-pass filter disabled , output registers not updated until MSB and LSB have been read , Enable Block Data Update , Set Output Data Rate to 0

    def reset(self):
        Buf=self._read_u16(self.LPS_CTRL_REG2)
        Buf|=0x04
        self._write_byte(self.LPS_CTRL_REG2,Buf)               #SWRESET Set 1
        while Buf:
            Buf=self._read_u16(self.LPS_CTRL_REG2)
            Buf&=0x04

    def set_oneshot_reading_mode(self):
        Buf=self._read_u16(self.LPS_CTRL_REG2)
        Buf|=0x01                                         #ONE_SHOT Set 1
        self._write_byte(self.LPS_CTRL_REG2,Buf)

    def _read_byte(self,cmd):
        return self._bus.read_byte_data(self._address,cmd)

    def _read_u16(self,cmd):
        LSB = self._bus.read_byte_data(self._address,cmd)
        MSB = self._bus.read_byte_data(self._address,cmd+1)
        return (MSB	<< 8) + LSB

    def _write_byte(self,cmd,val):
        self._bus.write_byte_data(self._address,cmd,val)

    def is_new_pressure_val_available(self):
        if (self._read_byte(self.LPS_STATUS)&0x01)==0x01:
            return True
        return False

    def get_pressure_val(self):
        u8Buf = [0, 0, 0]
        u8Buf[0] = self._read_byte(self.LPS_PRESS_OUT_XL)
        u8Buf[1] = self._read_byte(self.LPS_PRESS_OUT_L)
        u8Buf[2] = self._read_byte(self.LPS_PRESS_OUT_H)
        pressure_val = ((u8Buf[2] << 16) + (u8Buf[1] << 8) + u8Buf[0]) / 4096.0
        return pressure_val

    def is_new_temperature_val_available(self):
        if (self._read_byte(self.LPS_STATUS) & 0x02) == 0x02:
            return True
        return False

    def get_temperature_val(self):
        u8Buf = [0, 0]
        u8Buf[0] = self._read_byte(self.LPS_TEMP_OUT_L)
        u8Buf[1] = self._read_byte(self.LPS_TEMP_OUT_H)
        temperature_val=((u8Buf[1]<<8)+u8Buf[0])/100.0
        return temperature_val


def pressure_to_z(pressure_hpa, temperature_celsius):
    #formula_src: https://sciencing.com/calculate-air-volume-5146908.html
    temp_F = temperature_celsius*1.8 + 32.
    try:
        return round(( ( log( pressure_hpa*100./101325. )*287.053 ) * \
                 (temp_F + 459.67)*5./9. )/(-9.8),3)
    except Exception as ex:
        pass
    return 0.0

if __name__ == '__main__':
    import time
    pressure_sensor = LPS22HB()
    init_ts = time.time()
    with open('./pressure_test_data.txt', "w") as f:
        while True:
            try:
                pressure_sensor.set_oneshot_reading_mode()
                while not pressure_sensor.is_new_pressure_val_available() and \
                        not pressure_sensor.is_new_temperature_val_available():
                    time.sleep(0.1)
                pressure_hpa = round(pressure_sensor.get_pressure_val(), 3)
                temperature_celsius = round(pressure_sensor.get_temperature_val(), 3)
                alt_val = pressure_to_z(pressure_hpa, temperature_celsius)
                etime = round( time.time() - init_ts, 3)
                ftext = str(etime)+", "+str(pressure_hpa)+", "+str(alt_val)+", "+str(temperature_celsius)
                f.write(ftext+"\n")
                print(ftext)
            except Exception as ex:
                print(ex)
                continue
