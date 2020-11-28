#!/usr/bin/env python
# license removed for brevity
# -*- coding: utf-8 -*-

#############################################################################
# python ros vmu931
# by Kevin Chiu 2020
#############################################################################
import rospy
from std_msgs.msg import String

import numpy as np
import serial
import time
import sys
import struct

from std_msgs.msg import Float64

g_vmu_msg = Float64()

############################################################################
# vmu reaad function
############################################################################
def vmuFuc() :
    while 1:
        vmu_ser = np.zeros((40), dtype=np.int)
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
            vmu_x = struct.unpack('>f', bytearray(list(vmu_ser[7:11])))[0]
            vmu_y = struct.unpack('>f', bytearray(list(vmu_ser[11:15])))[0]
            vmu_z = struct.unpack('>f', bytearray(list(vmu_ser[15:19])))[0]
            if(vmu_x != 0.0 and vmu_y != 0.0 and vmu_z != 0.0):
                break
    return vmu_x, vmu_y, vmu_z


##############################################################################
# vmu talker node
###########################################################################
def vmu_talker():
    global g_vmu_msg

    # serial_work = threading.Thread(target=publisher_thread)
    # serial_work.start()

    pub = rospy.Publisher('Send_IMU', Float64, queue_size=1000)
    rospy.init_node('vmu_talker', anonymous=True)
    # rate = rospy.Rate(100)  # 10hz
    while not rospy.is_shutdown():
        
        vmu_x, vmu_y, vmu_z = vmuFuc()
        # print('vmu_x:%.4f vmu_y:%.4f vmu_z:%.4f'%(vmu_x, vmu_y, vmu_z))
        
        g_vmu_msg = vmu_z

        rospy.loginfo(g_vmu_msg)
        pub.publish(g_vmu_msg)

        # rate.sleep()

############################################################################
# main
#############################################################################
if __name__ == '__main__':
    print("vmu start......")
    ################input ############################
    try:
        input_argv = sys.argv
        input_port = input_argv[1]
        input_baudrate = input_argv[2]
        print('====== input setting ======')
    ################# defalt #######################
    except:
        input_port = "/dev/ttyACM0"
        input_baudrate = "115200"
        print('====== defalt setting ======')
    ################# serial connect ################
    print("port: " + input_port)
    print("baudrate: " + input_baudrate)
    print('=========================')
    try:
        g_ser_vmu = serial.Serial(input_port, input_baudrate, bytesize=8,
                                 parity=serial.PARITY_EVEN, stopbits=1, timeout=0.07)
    except:
        print('\033[91m' + "serial error!!!")
        while 1:
            continue
    time.sleep(1)

    ################### test serial #################
    if(g_ser_vmu.read().encode('hex') == ""):
        print('\033[91m' + "serial error!!!")
        while 1:
            continue

    try:
        vmu_talker()
    except rospy.ROSInterruptException:
        pass