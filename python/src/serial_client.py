import json
import serial
import time
from colorama import Fore, Style
from typing import Optional, Dict, Any


class SerialCommunicator:
    def __init__(self, port, baudrate=9600, dummy=False):
        """
        Initialize the communicator.

        :param port: Serial port (e.g., 'COM3' or '/dev/ttyUSB0')
        :param baudrate: Baudrate for the serial port.
        :param dummy: Use a dummy serial port for testing.
        """
        if dummy:
            self.ser = serial.serial_for_url("loop://", timeout=0)
        else:
            self.ser = serial.Serial(
                port, baudrate, timeout=0, rtscts=True, dsrdtr=True
            )
        self.next_request_id = 1

    def handle_async_message(self, message: dict) -> None:
        """Handle unsolicited messages."""
        print("Received async message:", message)

    def _send_command(
        self, command: str, params: Optional[Dict[str, Any]] = None, timeout: float = 5
    ) -> Dict[str, Any]:
        request_id = self.next_request_id
        self.next_request_id += 1

        message = {"id": request_id, "command": command}
        
        if params is not None:
            message["params"] = params
        print(message)
        message_str = json.dumps(message) + "\n"
        self.ser.write(message_str.encode("utf-8"))

        buffer = b""
        start_time = time.time()
        while True:
            if time.time() - start_time > timeout:
                raise TimeoutError(
                    f"Command {command} timed out after {timeout} seconds for {request_id}"
                )

            # Read all available data
            data = self.ser.read(self.ser.in_waiting or 1)
            buffer += data

            # Process each complete line
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                line = line.strip()
                if not line:
                    continue

                try:
                    response = json.loads(line.decode("utf-8"))
                except json.JSONDecodeError:
                    print(
                        Fore.WHITE + Style.DIM + "[SERIAL]:",
                        line.decode("utf-8", errors="replace") + Style.RESET_ALL,
                    )
                    continue
                
                print('resp', response)

                if response.get("id") == request_id:
                    return response
                else:
                    self.handle_async_message(response)

            # Sleep to prevent busy waiting
            time.sleep(0.01)

    def sense(self, timeout: float = 5) -> Dict[str, Any]:
        """Send a sense command and wait for the response."""
        return self._send_command("sense", timeout=timeout)

    def move(self, distance: int, timeout: float = 5) -> Dict[str, Any]:
        """Send a move command with the specified distance and wait for the response."""
        return self._send_command("move", {"distance": distance}, timeout=timeout)

    def reset(self, timeout: float = 30) -> Dict[str, Any]:
        """Send a reset command and wait for the response."""
        return self._send_command("reset", timeout=timeout)

    def close(self) -> None:
        """Close the serial connection."""
        self.ser.close()
