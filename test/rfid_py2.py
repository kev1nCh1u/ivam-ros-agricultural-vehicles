# -*- coding: utf-8 -*-

#############################################################################
# python2 rfid
# by Kevin Chiu 2020
#############################################################################

import numpy as np
import serial
import time
import sys


g_ser_rfid = serial.Serial('/dev/ttyUSB0', 38400, bytesize=8,
                           parity=serial.PARITY_EVEN, stopbits=1, timeout=0.07)


while 1:

    g_ser_rfid.write([0xaa, 0xdd, 0x00, 0x03, 0x01, 0x0c, 0x0d])

    read = g_ser_rfid.read(18)
    read_hex = read.encode('hex')
    if(len(read_hex) == 18):
        print(read_hex)


class RfidFuc(object):
    """
    docstring
    """
    pass
