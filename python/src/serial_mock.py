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
