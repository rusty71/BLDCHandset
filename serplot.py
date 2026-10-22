import serial
import time

ser = serial.Serial('/dev/serial/by-id/usb-Arduino_RaspberryPi_Pico_875862E6308F71D3-if00', 115200, timeout=0, xonxoff=False, rtscts=False, dsrdtr=False)
#ser.flushInput()
#ser.flushOutput()
fp = open('test.csv', 'w')
while True:
    data_raw = ser.read()
    if len(data_raw) > 0:
        fp.write(data_raw.decode('utf-8'))
