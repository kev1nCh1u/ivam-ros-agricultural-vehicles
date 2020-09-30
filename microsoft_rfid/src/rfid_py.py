#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

import numpy as np
import serial
import time
import sys


g_ser_rfid = serial.Serial('/dev/ttyUSB0', 38400, bytesize=8,
                           parity=serial.PARITY_EVEN, stopbits=1, timeout=0.07)

def rfidFuc():
    g_ser_rfid.write([0xaa, 0xdd, 0x00, 0x03, 0x01, 0x0c, 0x0d])
    read = g_ser_rfid.read(18)
    read_hex = read.encode('hex')
    if(len(read_hex) == 18):
        # print(read_hex)
        return read_hex

def rfid_talker():
    pub = rospy.Publisher('rfid_id', String, queue_size=10)
    rospy.init_node('rfid_talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():

        rfid_id = rfidFuc()
        rospy.loginfo(rfid_id)
        pub.publish(rfid_id)
        rate.sleep()


if __name__ == '__main__':
    try:
        rfid_talker()
    except rospy.ROSInterruptException:
        pass
