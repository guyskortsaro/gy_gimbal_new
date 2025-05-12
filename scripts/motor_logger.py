import smbus2

bus = smbus2.SMBus(18)
# data = bus.read_word_data(0x3a, 0x01)
data = bus.read_i2c_block_data(0x3a, 0x01, 2)
# data =bus.read_byte_data(0x3a, 0x01)
# print(data)
# convert bytes to int
print(int.from_bytes(data, "little"))