#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

import numpy as np
import serial
import time
import sys

from std_msgs.msg import String
from magnetic_rail.msg import MrMsg


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



def rfid_talker():
    pub = rospy.Publisher('rfid_msg', MrMsg, queue_size=10)
    rospy.init_node('rfid_talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():

        num, offset, width = magnetFuc()
        rospy.loginfo(num, offset, width)
        pub.publish(rfid_id)
        rate.sleep()

if __name__ == '__main__':
    try:
        rfid_talker()
    except rospy.ROSInterruptException:
        pass
