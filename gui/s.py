
import serial
import time
i = 0
with serial.Serial('/dev/cu.usbmodem2101', 115200, timeout=10) as ser:
    while True:
        i += 1
        before = time.perf_counter()
        ser.write(f'hello {i}\n'.encode())
        line = ser.readline()
        after = time.perf_counter()

        print(f'{i}: {line.decode().strip()} ({(after - before) * 1000}s)')
        time.sleep(1)
