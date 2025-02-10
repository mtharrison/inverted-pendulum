import os
import json
import serial
import threading
import pytest
import sys

def mock_server(port: str):
    """Mock server that reads commands and sends predefined responses."""
    with serial.Serial(port, baudrate=9600, timeout=1) as ser:
        while True:
            line = ser.readline()
            if line:
                try:
                    data = json.loads(line.decode().strip())
                    response = {"id": data["id"], "status": "ok"}
                    if data["command"] == "sense":
                        response["data"] = {"value": 100}
                    elif data["command"] == "move":
                        response["moved"] = data["params"]["distance"]
                    elif data["command"] == "reset":
                        response["reset"] = True
                    ser.write(json.dumps(response).encode() + b"\n")
                except Exception as e:
                    print(f"Mock server error: {e}")

@pytest.fixture
def virtual_serial_pair():
    """Fixture to create a virtual serial pair and start the mock server."""
    if sys.platform not in ('linux', 'darwin'):
        pytest.skip("Test requires a Unix-like system with PTY support")

    # Create a PTY (pseudo-terminal) pair
    master_fd, slave_fd = os.openpty()
    print(f"Master FD: {master_fd}, Slave FD: {slave_fd}")
    master_name = os.ttyname(master_fd)
    print(f"Master PTY: {master_name}")
    slave_name = os.ttyname(slave_fd)

    # Start the mock server in a background thread
    server_thread = threading.Thread(
        target=mock_server, args=(master_name,), daemon=True
    )
    server_thread.start()

    yield slave_name  # Provide the slave port to the test

    # Cleanup PTY file descriptors
    os.close(master_fd)
    os.close(slave_fd)

@pytest.mark.skipif(sys.platform not in ('linux', 'darwin'), reason="Requires Unix-like system")
def test_serial_commands(virtual_serial_pair):
    """Test blocking commands with a virtual serial connection."""
    communicator = SerialCommunicator(port=virtual_serial_pair, baudrate=9600)
    
    try:
        # Test sense command
        sense_response = communicator.sense(timeout=2)
        assert sense_response.get("data") == {"value": 100}, "Sense response mismatch"

        # Test move command
        move_response = communicator.move(5, timeout=2)
        assert move_response.get("moved") == 5, "Move response mismatch"

        # Test reset command
        reset_response = communicator.reset(timeout=2)
        assert reset_response.get("reset") is True, "Reset response mismatch"
    
    finally:
        communicator.close()