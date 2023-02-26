#!/usr/bin/python
# -*- coding:utf-8 -*-
import time
import sys
from pressure_sensor_lps22hb import LPS22HB


if __name__ == '__main__':
    print("\nPressure Sensor Test Program ...\n")
    if len(sys.argv) <3:
        print("program requires 2 arguements: duration and interval in seconds")
        exit(0)
    duration = float(sys.argv[1])
    interval = float(sys.argv[2])
    print("debug: duration ", duration)
    print("debug: interval ", interval)

    init_ts = time.time()
    finishing_ts = time.time()+duration

    pressure_sensor=LPS22HB()


    with open('./pressure_values.log','a') as f:
        while time.time() <= finishing_ts:
            pressure_sensor.set_oneshot_reading_mode()
            while not pressure_sensor.is_new_pressure_val_available():
                time.sleep(0.1)
            elapsed_ts = round(time.time() -  init_ts, 3)
            pressure_val = round( pressure_sensor.get_pressure_val(), 3 )
            f_string = str(elapsed_ts)+", "+str(pressure_val)
            f.write( f_string+"\n")
            print(f_string)
            time.sleep(interval)




