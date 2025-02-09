import serialprotocol_pb2

message = serialprotocol_pb2.RequestState()
message.ParseFromString(b"\x08\x01")
