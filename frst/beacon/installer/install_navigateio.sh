#!/bin/bash
rm -rf /usr/local/etc/navigateio && \
mkdir -p /usr/local/etc/navigateio && \
cp /home/pi/software/navigateio/frst/beacon/main.py /usr/local/etc/navigateio/main.py && \
cp /home/pi/software/navigateio/frst/beacon/lora_sx126x.py /usr/local/etc/navigateio/lora_sx126x.py && \
cp /home/pi/software/navigateio/frst/beacon/pressure_sensor_lps22hb.py /usr/local/etc/navigateio/pressure_sensor_lps22hb.py && \
chmod +x /usr/local/etc/navigateio/main.py && \
cp /home/pi/software/navigateio/frst/beacon/installer/navigateio.service /etc/systemd/system/navigateio.service && \
systemctl enable navigateio.service && \
cp /home/pi/software/navigateio/frst/beacon/installer/100-navigateio.rules /etc/udev/100-rules.d && \
udevadm control --reload && \
systemctl stop navigateio.service && \
systemctl start navigateio.service 

