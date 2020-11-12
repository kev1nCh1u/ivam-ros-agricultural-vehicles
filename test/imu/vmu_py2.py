# -*- coding: utf-8 -*-

#############################################################################
# python vmu931
# by Kevin Chiu 2020
#############################################################################

import numpy as np
import serial
import time
import sys
import struct

g_ser_vmu = serial.Serial('/dev/ttyACM0', 115200, bytesize=8,
                             parity=serial.PARITY_EVEN, stopbits=1, timeout=0.07)

if(g_ser_vmu.read().encode('hex') == ""):
    print("serial error!!!")
    exit()


def vmuFuc() :
    while 1:
        vmu_ser = np.zeros((30), dtype=np.int)
        loop = 0
        while 1:
            try:
                input_data = int(g_ser_vmu.read().encode('hex'), 16)
            # en_data = int(input_data.encode('hex'))
            except:
                input_data = 0
            if(input_data == 0x01):
                vmu_ser[loop] = input_data
                loop += 1
                break
        while 1:
            input_data = int(g_ser_vmu.read().encode('hex'), 16)
            # en_data = int(input_data.encode('hex'))
            vmu_ser[loop] = input_data
            loop += 1
            if(input_data == 0x04):
                break

        if(vmu_ser[2] == ord('e')): # 7
            vmu_x = struct.unpack('>f', bytearray(list(vmu_ser[7:11])))
            vmu_y = struct.unpack('>f', bytearray(list(vmu_ser[11:15])))
            vmu_z = struct.unpack('>f', bytearray(list(vmu_ser[15:19])))
            break
    vmu_data = [vmu_x[0], vmu_y[0], vmu_z[0]]
    return vmu_data



print("start......")
while 1:
    vmu_data = vmuFuc()

    print('%.4f %.4f %.4f'%(vmu_data[0],vmu_data[1],vmu_data[2]))
