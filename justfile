proto:
  protoc --proto_path=. --python_out=./gui serialprotocol.proto
  protoc --proto_path=. --nanopb_out=firmware/src serialprotocol.proto
