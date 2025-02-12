from serial_client import SerialCommunicator

client = SerialCommunicator("/dev/cu.usbmodem2101")
print(client.sense())