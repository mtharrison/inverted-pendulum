#!/usr/bin/env python
import serial
import time

ser = serial.Serial('/dev/ttys016', 9600, rtscts=True,dsrdtr=True)
while True:
    ser.write(b'hello\n')
    print(ser.readline())
    time.sleep(1)
