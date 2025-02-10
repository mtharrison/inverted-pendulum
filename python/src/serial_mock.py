import os
import json
import threading


class MockSerialEndpoint:
    def __init__(self, port, on_sense=None, on_move=None, on_reset=None):
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
        self.on_sense = on_sense
        self.on_move = on_move
        self.on_reset = on_reset

    def read_from_port(self):
        self.fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY)
        buffer = b""
        while not self.stop:
            try:
                data = os.read(self.fd, 1)
                buffer += data
                if not data:
                    break

                if b"\n" not in buffer:
                    continue

                request = json.loads(buffer)
                buffer = b""
                self.requests.append(request)
                self.requests = self.requests[-10:]
                id = request["id"]
                command = request["command"]

                if command == "sense":
                    response = self.on_sense(self.state, request)
                elif command == "move":
                    response = self.on_move(self.state, request)
                elif command == "reset":
                    response = self.on_reset(self.state, request)
                else:
                    response = {"id": id, "error": "Unknown command"}

                response_str = json.dumps(response) + "\n"
                os.write(self.fd, response_str.encode("utf-8"))
            except Exception as e:
                print(f"Error: {e}")

        os.close(self.fd)

    def __del__(self):
        self.stop = True
