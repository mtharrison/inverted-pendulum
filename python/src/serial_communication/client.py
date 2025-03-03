import json
import serial
import time
from colorama import Fore, Style
from typing import Optional, Dict, Any


class SerialCommunicator:
    def __init__(self, port, baudrate=115200, dummy=False):
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

    def parse_packet(self, packet: str) -> Dict[str, Any]:
        """Parse a packet received from the serial port as "id=%d|current_position=%d|velocity=%f|theta=%f|angular_velocity=%f|limitL=%d|limitR=%d|speed=%f|enabled=%d|resetting=%d|extent=%d\n"""
        response = {}
        for part in packet.split("|"):
            key, value = part.split("=")
            if (
                key == "resetting"
                or key == "enabled"
                or key == "limitL"
                or key == "limitR"
            ):
                value = bool(int(value))
            elif key == "current_position" or key == "id" or key == "extent":
                value = int(value)
            else:
                try:
                    value = float(value)
                except ValueError:
                    pass
            response[key] = value
        return response

    def handle_async_message(self, message: dict) -> None:
        """Handle unsolicited messages."""
        print("Received async message:", message)

    def _send_command(
        self, command: str, params: Optional[Dict[str, Any]] = None, timeout: float = 5
    ) -> Dict[str, Any]:
        request_id = self.next_request_id
        self.next_request_id += 1

        id = request_id

        if command == "move":
            packet = f"{id}|{command}|{params['speed']}\n".encode("utf-8")
            # print(f'Sending packet: {packet}')
            self.ser.write(packet)
        else:
            packet = f"{id}|{command}\n".encode("utf-8")
            # print(f'Sending packet: {packet}')
            self.ser.write(packet)

        buffer = b""
        start_time = time.time()
        while True:
            if time.time() - start_time > timeout:
                print(
                    f"Command {command} timed out after {timeout} seconds for {request_id}"
                )
                return

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
                    response = self.parse_packet(line.decode("utf-8"))
                except Exception as e:
                    print(
                        Fore.WHITE + Style.DIM + "[SERIAL]:",
                        line.decode("utf-8", errors="replace") + Style.RESET_ALL,
                    )
                    continue

                if response.get("id") == request_id:
                    return response
                else:
                    self.handle_async_message(response)

            # Sleep to prevent busy waiting
            time.sleep(0.000001)

    def sense(self, timeout: float = 1) -> Dict[str, Any]:
        """Send a sense command and wait for the response."""
        return self._send_command("sense", timeout=timeout)

    def move(self, speed: int, timeout: float = 1) -> Dict[str, Any]:
        """Send a move command with the specified distance and wait for the response."""
        return self._send_command("move", {"speed": speed}, timeout=timeout)

    def reset(self, timeout: float = 1) -> Dict[str, Any]:
        """Send a reset command and wait for the response."""
        return self._send_command("reset", timeout=timeout)

    def close(self) -> None:
        """Close the serial connection."""
        self.ser.close()
