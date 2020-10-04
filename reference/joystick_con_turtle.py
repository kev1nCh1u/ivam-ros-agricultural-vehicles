#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from listen_joystick.msg import joystick
from geometry_msgs.msg import Twist

msg = Twist()

def callback(data):
    global msg
    if(abs(data.x) > 0.1):
      msg.linear.x = data.x
    else:
      msg.linear.x = 0
    if(abs(data.y) > 0.1):
      msg.angular.z = data.y
    else:
      msg.angular.z = 0
    print(msg)
    

    
def listener():

      rospy.init_node('listen_joystick_py', anonymous=True)
  
      rospy.Subscriber("joystick", joystick, callback)

      pub = rospy.Publisher("turtle1/cmd_vel",Twist, queue_size=10)
      r = rospy.Rate(10) #10hz

      

      while not rospy.is_shutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        r.sleep()

      rospy.spin()
  
if __name__ == '__main__':
    listener()