import json
import serial
import threading
import time
import os
from colorama import Fore, Back, Style
from typing import Optional, Callable, Dict, Any

class SerialCommunicator:
    def __init__(self, port, baudrate=9600, read_timeout=1, dummy=False):
        """
        Initialize the communicator.

        :param port: Serial port (e.g., 'COM3' or '/dev/ttyUSB0')
        :param baudrate: Baudrate for the serial port.
        :param read_timeout: Timeout (in seconds) for reading from the port.
        """
        if dummy:
            self.ser = serial.serial_for_url("loop://", timeout=read_timeout)
        else:
            self.ser = serial.Serial(port, baudrate, timeout=read_timeout)

        # Dictionary to hold callbacks for pending requests: {request_id: (callback_fn, timeout_time)}
        self.pending_requests: Dict[int, tuple[Callable[[dict], None], float]] = {}
        self.pending_lock = threading.Lock()

        # Counter for generating integer message ids
        self.next_request_id = 1

        # Event to signal the reader thread to stop
        self.stop_event = threading.Event()

        # Start background threads
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.timeout_thread = threading.Thread(target=self._timeout_checker, daemon=True)
        self.reader_thread.start()
        self.timeout_thread.start()

    def _timeout_checker(self):
        """Check for timed out requests and call their callbacks with None."""
        while not self.stop_event.is_set():
            current_time = time.time()
            to_remove = []
            callbacks_to_call = []

            with self.pending_lock:
                for request_id, (callback, timeout_time) in self.pending_requests.items():
                    if current_time > timeout_time:
                        to_remove.append(request_id)
                        callbacks_to_call.append(callback)

                for request_id in to_remove:
                    self.pending_requests.pop(request_id)

            # Call callbacks outside the lock
            for callback in callbacks_to_call:
                try:
                    callback(None)  # None indicates timeout
                except Exception as e:
                    print(f"Error in timeout callback: {e}")

            time.sleep(0.1)  # Sleep to prevent busy-waiting

    def _reader_loop(self):
        """Continuously read lines from the serial port and dispatch responses."""
        while not self.stop_event.is_set():
            try:
                line = self.ser.readline()
                if not line:
                    continue  # Timeout, no data received

                try:
                    message = json.loads(line.decode("utf-8").strip())
                except json.JSONDecodeError:
                    print(
                        Fore.WHITE + Style.DIM + "[SERIAL]:",
                        line.decode("utf-8").strip() + Style.RESET_ALL,
                    )
                    continue

                request_id = message.get("id")
                if request_id is not None:
                    # If the message contains an id, look for a matching callback
                    callback = None
                    with self.pending_lock:
                        if request_id in self.pending_requests:
                            callback, _ = self.pending_requests.pop(request_id)

                    if callback:
                        try:
                            callback(message)
                        except Exception as e:
                            print(f"Error in response callback: {e}")
                    else:
                        # No callback found; this might be an unsolicited response
                        self.handle_async_message(message)
                else:
                    # Handle messages without an id (asynchronous messages)
                    self.handle_async_message(message)

            except Exception as e:
                print(f"Error reading from serial port: {e}")

    def handle_async_message(self, message):
        """Handle unsolicited messages."""
        print("Received async message:", message)

    def send_request(self, command: str, callback: Callable[[dict], None], timeout: float = 5, **params):
        """
        Send a request and register a callback for the response.
        
        :param command: The command to send
        :param callback: Function to call with the response
        :param timeout: Time in seconds to wait for response before calling callback with None
        :param params: Additional parameters for the command
        :return: The request ID
        """
        with self.pending_lock:
            request_id = self.next_request_id
            self.next_request_id += 1
            timeout_time = time.time() + timeout
            self.pending_requests[request_id] = (callback, timeout_time)

        message = {"id": request_id, "command": command}
        if params:
            message["params"] = params
        message_str = json.dumps(message)

        self.ser.write((message_str + "\n").encode("utf-8"))
        return request_id

    def observe(self, callback: Callable[[dict], None]):
        """Non-blocking observe command."""
        return self.send_request("observe", callback)

    def step(self, action, callback: Callable[[dict], None]):
        """Non-blocking step command."""
        return self.send_request("step", callback, action=action)

    def reset(self, callback: Callable[[dict], None]):
        """Non-blocking reset command."""
        return self.send_request("reset", callback, timeout=30)

    def close(self):
        """Close the serial connection and stop background threads."""
        self.stop_event.set()
        self.reader_thread.join(timeout=2)
        self.timeout_thread.join(timeout=2)
        self.ser.close()