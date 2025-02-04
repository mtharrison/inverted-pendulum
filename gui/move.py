import serial
import json

with serial.Serial('/dev/cu.usbmodem2101', 115200, timeout=0.1) as ser:
    ser.write(json.dumps([0, 1, 1]).encode())
    print(ser.read_until(']'))