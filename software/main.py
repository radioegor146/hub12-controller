import usb.core
import usb.util
import time
import random

dev = usb.core.find(idVendor=0xe146, idProduct=0x1337)

if dev is None:
    raise ValueError('Device not found')

dev.set_configuration()

cfg = dev.get_active_configuration()
print(cfg)
intf = cfg[(0,0)]

data_out_ep = usb.util.find_descriptor(intf, custom_match = lambda e: e.bEndpointAddress == 0x01)
reset_out_ep = usb.util.find_descriptor(intf, custom_match = lambda e: e.bEndpointAddress == 0x02)

reset_out_ep.write([0] * 64)

start_time = time.time()
total_bytes = 0
for i in range(100000):
    data = [0xFF] * 5376
    data_out_ep.write(data)
    total_bytes += 5376
    speed = total_bytes / (time.time() - start_time)
    if i % 100 == 0:
        print(speed * 8, speed / 5376)
    