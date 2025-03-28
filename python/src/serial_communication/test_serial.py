from client import SerialCommunicator
from virtual import VirtualSerialPair
from mock import MockSerialEndpoint


def create_mock(port):
    def on_sense(state, request):
        return {"status": "ok", **state, "id": request["id"]}

    def on_move(state, request):
        state["target"] += request["params"]["distance"]
        return {"status": "ok", "id": request["id"]}

    def on_reset(state, request):
        state["resetting"] = True
        return {"status": "ok", "id": request["id"]}

    return MockSerialEndpoint(
        port=port, on_sense=on_sense, on_move=on_move, on_reset=on_reset
    )


def test_sense():
    with VirtualSerialPair() as (port1, port2):
        mock = create_mock(port1)
        client = SerialCommunicator(port=port2)

        payload = client.sense()

        assert payload["status"] == "ok"
        assert payload["position"] == 0
        assert payload["velocity"] == 0
        assert payload["theta"] == 0
        assert payload["angular_velocity"] == 0
        assert not payload["limitL"]
        assert not payload["limitR"]
        assert payload["target"] == 0
        assert not payload["enabled"]
        assert not payload["resetting"]

        assert mock.requests[0]["command"] == "sense"
        assert mock.requests[0]["id"] == 1


def test_unknown_command():
    with VirtualSerialPair() as (port1, port2):
        mock = create_mock(port1)

        client = SerialCommunicator(port=port2)

        payload = client._send_command("unknown_command")

        assert payload["error"] == "Unknown command"

        assert mock.requests[0]["command"] == "unknown_command"
        assert mock.requests[0]["id"] == 1


def test_move():
    with VirtualSerialPair() as (port1, port2):
        mock = create_mock(port1)

        client = SerialCommunicator(port=port2)

        payload = client.move(12)

        assert payload["status"] == "ok"

        assert mock.requests[0]["command"] == "move"
        assert mock.requests[0]["params"]["distance"] == 12
        assert mock.requests[0]["id"] == 1

        payload = client.sense()
        assert payload["target"] == 12


def test_reset():
    with VirtualSerialPair() as (port1, port2):
        mock = create_mock(port1)

        client = SerialCommunicator(port=port2)

        payload = client.reset()

        assert payload["status"] == "ok"

        assert mock.requests[0]["command"] == "reset"
        assert mock.requests[0]["id"] == 1

        payload = client.sense()
        assert payload["resetting"]
