import serial
import json
import threading
import queue
from typing import Dict, Any, Optional
import time
import sys
import os

DEBUG = os.getenv("DEBUG", "false").lower() == "true"

class SerialCommunicator:
    def __init__(self, port: str ='/dev/cu.usbmodem2101', baudrate: int = 115200):
        """
        Initialize serial communication with JSON message handling.
        
        Args:
            port: Serial port name (e.g., 'COM1' or '/dev/ttyUSB0')
            baudrate: Baud rate for serial communication
        """
        self.serial = serial.Serial(port, baudrate)
        self.response_queues: Dict[int, queue.Queue] = {}
        self.current_id = 0
        self.lock = threading.Lock()
        
        # Start reading thread
        self.running = True
        self.reader_thread = threading.Thread(target=self._read_responses, daemon=True)
        self.reader_thread.start()
    
    def _get_next_id(self) -> int:
        """Get next message ID with thread safety."""
        with self.lock:
            msg_id = self.current_id
            self.current_id += 1
            return msg_id
    
    def _read_responses(self):
        """Background thread to continuously read from serial port."""
        buffer = ""
        while self.running:
            if self.serial.in_waiting:
                char = self.serial.read().decode()
                buffer += char
                
                if char == '\n':  # Assuming JSON messages end with newline
                    try:
                        response = json.loads(buffer)
                        if 'id' in response:
                            msg_id = response['id']
                            if msg_id in self.response_queues:
                                self.response_queues[msg_id].put(response)
                    except json.JSONDecodeError:
                        if DEBUG:
                            print(f"SERIAL: {buffer}")
                    buffer = ""
            else:
                time.sleep(0.01)  # Prevent busy waiting
    
    def _send_command(self, command: str, params: Optional[Dict] = None, timeout: float = 5.0) -> Dict:
        """
        Send a command and wait for its response.
        
        Args:
            command: Command name
            params: Optional parameters for the command
            timeout: Maximum time to wait for response in seconds
        
        Returns:
            Response data as dictionary
        
        Raises:
            TimeoutError: If response isn't received within timeout
            RuntimeError: If serial port is closed
        """
        if not self.serial.is_open:
            raise RuntimeError("Serial port is closed")
            
        msg_id = self._get_next_id()
        message = {
            "id": msg_id,
            "command": command
        }
        if params:
            message["params"] = params
            
        # Create queue for this request
        self.response_queues[msg_id] = queue.Queue()
        
        # Send request
        self.serial.write((json.dumps(message) + '\n').encode())
        
        try:
            # Wait for response
            response = self.response_queues[msg_id].get(timeout=timeout)
            return response
        except queue.Empty:
            raise TimeoutError(f"No response received for command '{command}' within {timeout} seconds")
        finally:
            # Clean up
            del self.response_queues[msg_id]
    
    def observe(self) -> Dict[str, Any]:
        """Get current observation from the device."""
        return self._send_command("observe")
    
    def step(self, action: int) -> Dict[str, Any]:
        """Set a parameter on the device."""
        return self._send_command("step", {"action": action})
    
    def reset(self) -> Dict[str, Any]:
        """Set a parameter on the device."""
        return self._send_command("reset", timeout=30.0)
    
    def close(self):
        """Close the serial connection and stop the reading thread."""
        self.running = False
        if self.reader_thread.is_alive():
            self.reader_thread.join()
        if self.serial.is_open:
            self.serial.close()


if __name__ == "__main__":
    comm = SerialCommunicator()
    # for i in range(5):
    #     print(comm.step(i))
    #     time.sleep(1)
    print(comm.reset())
    comm.close()