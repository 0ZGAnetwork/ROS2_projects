Two applications communicate with each other using ZeroMQ library. The first application send complex data structure to the second application using serialization. The second application deserialize received data and print the received structure

run :
# protoc -I=. --python_out=. ./person.proto