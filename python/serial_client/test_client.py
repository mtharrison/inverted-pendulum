import re
import subprocess
import threading
import json
import os
from client import SerialCommunicator


class VirtualSerialPair:
    def __init__(self):
        self.proc = subprocess.Popen(
            ["socat", "-d", "-d", "pty,raw,echo=0", "pty,raw,echo=0"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )

        ports = []
        pattern = re.compile(r"PTY is (\S+)")

        while True:
            line = self.proc.stderr.readline()
            if not line:
                break

            match = pattern.search(line)
            if match:
                port_path = match.group(1)
                ports.append(port_path)

            if len(ports) == 2:
                break

        self.port1 = ports[0]
        self.port2 = ports[1]

    def __del__(self):
        self.proc.kill()


class MockSerialEndpoint:
    def __init__(self, port):
        self.stop = False
        self.port = port
        self.thread = threading.Thread(target=self.read_from_port)
        self.thread.start()
        self.requests = []
        self.state = {
            "position": 0,
            "velocity": 0,
            "theta": 0,
            "angular_velocity": 0,
            "limitL": False,
            "limitR": False,
            "target": 0,
            "enabled": False,
            "resetting": False,
        }

    def read_from_port(self):
        self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY)
        while not self.stop:
            data = os.read(self.fd, 1024)
            if not data:
                break

            request = json.loads(data.decode("utf-8"))
            self.requests.append(request)
            id = request["id"]
            command = request["command"]

            if command == "sense":
                response = {"status": "ok", **self.state, "id": id}
            elif command == "move":
                self.state["target"] += request["params"]["distance"]
                response = {"status": "ok", "id": id}
            elif command == "reset":
                self.state["resetting"] = True
                response = {"status": "ok", "id": id}
            else:
                response = {"id": id, "error": "Unknown command"}

            response_str = json.dumps(response) + "\n"
            os.write(self.fd, response_str.encode("utf-8"))

        os.close(self.fd)

    def __del__(self):
        self.stop = True


def test_sense():
    virtual = VirtualSerialPair()
    mock = MockSerialEndpoint(port=virtual.port1)

    client = SerialCommunicator(port=virtual.port2)

    payload = client.sense()

    assert payload["status"] == "ok"
    assert payload["position"] == 0
    assert payload["velocity"] == 0
    assert payload["theta"] == 0
    assert payload["angular_velocity"] == 0
    assert payload["limitL"] == False
    assert payload["limitR"] == False
    assert payload["target"] == 0
    assert payload["enabled"] == False
    assert payload["resetting"] == False

    assert mock.requests[0]["command"] == "sense"
    assert mock.requests[0]["id"] == 1


def test_unknown_command():
    virtual = VirtualSerialPair()
    mock = MockSerialEndpoint(port=virtual.port1)

    client = SerialCommunicator(port=virtual.port2)

    payload = client._send_command("unknown_command")

    assert payload["error"] == "Unknown command"

    assert mock.requests[0]["command"] == "unknown_command"
    assert mock.requests[0]["id"] == 1


def test_move():
    virtual = VirtualSerialPair()
    mock = MockSerialEndpoint(port=virtual.port1)

    client = SerialCommunicator(port=virtual.port2)

    payload = client.move(12)

    assert payload["status"] == "ok"

    assert mock.requests[0]["command"] == "move"
    assert mock.requests[0]["params"]["distance"] == 12
    assert mock.requests[0]["id"] == 1

    payload = client.sense()
    assert payload["target"] == 12


def test_reset():
    virtual = VirtualSerialPair()
    mock = MockSerialEndpoint(port=virtual.port1)

    client = SerialCommunicator(port=virtual.port2)

    payload = client.reset()

    assert payload["status"] == "ok"

    assert mock.requests[0]["command"] == "reset"
    assert mock.requests[0]["id"] == 1

    payload = client.sense()
    assert payload["resetting"] == True
