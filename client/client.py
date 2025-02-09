import json
import serial
import threading
import time
import os
from colorama import Fore, Back, Style


DEBUG = os.environ.get("DEBUG", False)
print("DEBUG:", DEBUG)

class SerialCommunicator:
    def __init__(self, port, baudrate=9600, read_timeout=1):
        """
        Initialize the communicator.

        :param port: Serial port (e.g., 'COM3' or '/dev/ttyUSB0')
        :param baudrate: Baudrate for the serial port.
        :param read_timeout: Timeout (in seconds) for reading from the port.
        """
        # Open the serial port.
        self.ser = serial.Serial(port, baudrate, timeout=read_timeout)

        # Dictionary to hold pending requests: {request_id: (threading.Event, response_container)}
        self.pending_requests = {}
        self.pending_lock = threading.Lock()

        # Counter for generating integer message ids.
        self.next_request_id = 1

        # Event to signal the reader thread to stop.
        self.stop_event = threading.Event()

        # Start a background thread that continuously reads from the serial port.
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()

    def _reader_loop(self):
        """Continuously read lines from the serial port and dispatch responses."""
        while not self.stop_event.is_set():
            try:
                line = self.ser.readline()
                if not line:
                    continue  # Timeout, no data received.
                # Decode the line and parse the JSON.
                try:
                    message = json.loads(line.decode('utf-8').strip())
                except json.JSONDecodeError:
                    if DEBUG:
                        print(Fore.WHITE + Style.DIM + "[SERIAL]:", line.decode('utf-8').strip() + Style.RESET_ALL)
                    continue
                request_id = message.get('id')
                if request_id is not None:
                    # If the message contains an id, look for a matching pending request.
                    with self.pending_lock:
                        pending = self.pending_requests.pop(request_id, None)
                    if pending:
                        event, response_container = pending
                        response_container['response'] = message
                        event.set()  # Unblock the waiting thread.
                    else:
                        # No pending request found; this might be an unsolicited response.
                        self.handle_async_message(message)
                else:
                    # Handle messages without an id (asynchronous messages).
                    self.handle_async_message(message)
            except Exception as e:
                # In production code you may want to log this error.
                print(f"Error reading from serial port: {e}")

    def handle_async_message(self, message):
        print("Received message:", message)

    def send_request(self, command, timeout=5, **params):
        # Generate a unique integer id for this request.
        with self.pending_lock:
            request_id = self.next_request_id
            self.next_request_id += 1

        message = {'id': request_id, 'command': command}
        if params:
            message['params'] = params
        message_str = json.dumps(message)

        # Write the message (terminated by a newline) to the serial port.
        self.ser.write((message_str + "\n").encode('utf-8'))

        # Create an event and container for the response.
        event = threading.Event()
        response_container = {}

        # Store them in the pending_requests dictionary.
        with self.pending_lock:
            self.pending_requests[request_id] = (event, response_container)

        # Wait for the event to be set by the reader thread.
        # You can adjust the timeout as needed.
        if event.wait(timeout):
            return response_container['response']
        else:
            # If no response arrives in time, remove the pending request and raise an error.
            with self.pending_lock:
                self.pending_requests.pop(request_id, None)
            raise TimeoutError(f"Timed out waiting for response to command '{command}'")

    def observe(self):
        return self.send_request("observe")
    
    def step(self, action):
        return self.send_request("step", action=action)
    
    def reset(self):
        return self.send_request("reset", timeout=30)

    def close(self):
        self.stop_event.set()
        self.reader_thread.join(timeout=2)
        self.ser.close()


# Example usage:
if __name__ == "__main__":
    # Replace 'COM3' or '/dev/ttyUSB0' with your actual serial port.
    communicator = SerialCommunicator(port='/dev/cu.usbmodem2101', baudrate=9600)

    try:
        # The get_observation() method hides the fact that a message is sent and a response is waited on.
        response = communicator.observe()
        print("Observation response:", response)

        # The step() method sends a message and waits for a response.
        response = communicator.step(1)
        print("Step response:", response)

        # The reset() method waits for a response, but with a longer timeout.
        response = communicator.reset()
        print("Reset response:", response)

    except TimeoutError as e:
        print(e)

    finally:
        communicator.close()

