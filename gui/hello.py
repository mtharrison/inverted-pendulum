
import serial
import msgpack
import time
import json
i = 0
with serial.Serial('/dev/cu.usbmodem2101', 115200, timeout=10) as ser:
    while True:
        # i += 1
        
        # ser.write(msgpack.packb([18, f'hello {i}\n'.encode()]))
        # out = b''
        # while ser.inWaiting() > 0:
        #     out += ser.read(100)

        before = time.perf_counter()
        ser.write(json.dumps({'command': 'respond', 'key': i}).encode())
        i+=1

        out = ser.read_until(b'}')
        if (len(out)):
            after = time.perf_counter()
            print("got", out, after - before)
        # unpacker = msgpack.Unpacker(ser)
        # for unpacked in unpacker:
        #     # print(unpacked)
