from serial_client import SerialCommunicator
from time import sleep

serial = SerialCommunicator("/dev/cu.usbmodem2101")
# print(serial.reset())
# while True:
print(serial.move(-2000))
# print(serial.sense())
    # sleep(1)

