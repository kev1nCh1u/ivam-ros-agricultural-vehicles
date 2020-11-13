#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from listen_joystick.msg import joystick

def callback(data):
    rospy.loginfo(data)
    
def listener():

      rospy.init_node('listen_joystick_py', anonymous=True)
  
      rospy.Subscriber("joystick", joystick, callback)

      rospy.spin()
  
if __name__ == '__main__':
    listener()