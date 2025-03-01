import serial
import time

ser = serial.Serial(
    port="/dev/cu.usbmodem2201",
    baudrate=115200,
    timeout=1,
)

before = time.perf_counter()
ser.write(b'{"id": 0, "command": "sense"}\n')
response = ser.readline()
print(f'Elapsed time: {(time.perf_counter() - before) * 1000}ms')
print(response)