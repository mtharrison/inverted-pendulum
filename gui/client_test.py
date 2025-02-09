from client import SerialCommunicator
import time

# Create a communicator object
comm = SerialCommunicator(port="/dev/cu.usbmodem2101")

print('sending step 1')
comm.step(100, lambda x: print(x))
print('sending step 2')
comm.step(200, lambda x: print(x))

time.sleep(30)