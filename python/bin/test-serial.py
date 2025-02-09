import json
import serial

ser = serial.Serial('/dev/cu.usbmodem2101', 9600, timeout=1)

while True:
    try:
        ser.write(json.dumps({'id': 1, 'command': 'sense'}).encode())
        ser.write(b'\n')
        print(ser.readline())
    except KeyboardInterrupt:
        ser.close()
        break