# -*- coding: utf-8 -*-

#############################################################################
# python magnetic_rail
# by Kevin Chiu 2020
#############################################################################

import numpy as np
import serial
import time
import sys

g_ser_magnet = serial.Serial('/dev/ttyUSB0', 115200, bytesize=8,
                             parity=serial.PARITY_EVEN, stopbits=1, timeout=0.07)


def magnetFuc():
    num = None
    offset = None
    width = None

    data_raw = g_ser_magnet.read()

    # int_raw = int.from_bytes(data_raw, byteorder='big')
    int_raw = int(data_raw.encode('hex'), 16)

    num = int_raw//64
    id = int_raw % 64

    if(id == 57 and num == 1):
        data_raw1 = g_ser_magnet.read()
        data_raw2 = g_ser_magnet.read()
        # int_raw1 = int.from_bytes(data_raw1, byteorder='big')
        # int_raw2 = int.from_bytes(data_raw2, byteorder='big')
        int_raw1 = int(data_raw1.encode('hex'), 16)
        int_raw2 = int(data_raw2.encode('hex'), 16)
        P_N = (int_raw1 & 64) >> 6
        if(P_N == 1):
            offset = int_raw1 & 63
        else:
            offset = -(int_raw1 & 63)
        width = (((int_raw2 & 192) >> 2)+((int_raw2 & 63)-41))*2

    elif(id == 57 and num != 0):
        data_raw1 = g_ser_magnet.read()
        # int_raw1 = int.from_bytes(data_raw1, byteorder='big')
        int_raw1 = int(data_raw1.encode('hex'), 16)
        P_N = (int_raw1 & 64) >> 6
        if(P_N == 1):
            offset = int_raw1 & 63
        else:
            offset = -(int_raw1 & 63)

    # print("num:{} offset:{} width:{}".format(num, offset, width))
    return num, offset, width


while 1:
    num, offset, width = magnetFuc()
    print("num:{} offset:{} width:{}".format(num, offset, width))
