#!/usr/bin/env python
# license removed for brevity
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String

import socket
HOST = '127.0.0.1'
# HOST = '10.1.1.2'
PORT = 8000

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
server.bind((HOST, PORT))
server.listen(10)

def tcpFuc():
    conn, addr = server.accept()

    # clientMessage = str(conn.recv(1024), encoding='utf-8') # python3
    clientMessage = str(conn.recv(1024).encode('utf-8')) # python2
    print('Client message:', clientMessage)

    serverMessage = 'Roger !'
    conn.sendall(serverMessage.encode())
    conn.close()

    return clientMessage

def tcp_server():
    pub = rospy.Publisher('tcp_msg', String, queue_size=10)
    rospy.init_node('tcp_server', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        tcp_str = tcpFuc()
        rospy.loginfo(tcp_str)
        pub.publish(tcp_str)
        # rate.sleep()

if __name__ == '__main__':
    try:
        tcp_server()
    except rospy.ROSInterruptException:
        pass