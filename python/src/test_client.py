from serial_client import SerialCommunicator
from serial_virtual import VirtualSerialPair
from serial_mock import MockSerialEndpoint


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
    assert not payload["limitL"]
    assert not payload["limitR"]
    assert payload["target"] == 0
    assert not payload["enabled"]
    assert not payload["resetting"]

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
    assert payload["resetting"]
