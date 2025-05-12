import smbus2
import zmq
import json
from time import sleep

bus = smbus2.SMBus(18)
context = zmq.Context()
serverSocket = context.socket(zmq.PUB)
port = 9872
serverSocket.bind("tcp://*:"+str(port))
time = 0.0
while True:
    sleep(0.5)
    time += 0.05
    sin_data = bus.read_i2c_block_data(0x3a, 0x01, 2)
    x = int.from_bytes(sin_data, "little")
    print(sin_data)
    data = {
        "timestamp": time,
        "sin_data": x

    }

    serverSocket.send_string(json.dumps(data))