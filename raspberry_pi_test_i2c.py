import smbus
import time
bus = smbus.SMBus(1)
address = 0x18


while True:
    for x in range(10):
        res=bus.write_byte_data(address, x, x+5)
    for x in range(10):
        res=bus.read_byte_data(address, x)
        print(res)
    time.sleep(1)
