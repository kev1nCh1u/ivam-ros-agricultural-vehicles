#!/usr/bin/env python
# license removed for brevity

#############################################################################
# python ros rfid
# by Kevin Chiu 2020
#############################################################################
import rospy
from std_msgs.msg import String

import numpy as np
import serial
import time
import sys


def rfidFuc():
    g_ser_rfid.write([0xaa, 0xdd, 0x00, 0x03, 0x01, 0x0c, 0x0d])
    read = g_ser_rfid.read(18)
    read_hex = read.encode('hex')
    if(len(read_hex) == 18):
        # print(read_hex)
        return read_hex


def rfid_talker():
    pub = rospy.Publisher('rfid_msg', String, queue_size=10)
    rospy.init_node('rfid_talker', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():

        rfid_id = rfidFuc()
        rospy.loginfo(rfid_id)
        pub.publish(rfid_id)
        rate.sleep()


if __name__ == '__main__':
    try:
        input_argv = sys.argv
        input_port = input_argv[1]
        input_baudrate = input_argv[2]
        print('====== input setting ======')
    except:
        input_port = "/dev/ttyUSB0"
        input_baudrate = "38400"
        print('====== defalt setting ======')
    print("port: " + input_port)
    print("baudrate: " + input_baudrate)
    print('=========================')
    g_ser_rfid = serial.Serial(input_port, input_baudrate, bytesize=8,
                               parity=serial.PARITY_EVEN, stopbits=1, timeout=0.07)

    time.sleep(1)

    try:
        rfid_talker()
    except rospy.ROSInterruptException:
        pass
