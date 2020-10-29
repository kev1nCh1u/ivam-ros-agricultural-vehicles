#############################################################################
# python3
#############################################################################

import numpy as np
import serial
import time
import sys 


g_ser_rfid = serial.Serial('/dev/ttyUSB0', 38400)


def recv():
    while True:
        data = g_ser_rfid.read_all().hex()
        if data == b'':
            continue
        else:
            break
    return data

while 1:

    g_ser_rfid.write(bytes([170, 221, 0, 3, 1, 12, 13]))
                       # aa dd 00 03 01 0c 0d

    time.sleep(0.1)

    data = recv()
    print(data)
    time.sleep(0.1)
    recv()
