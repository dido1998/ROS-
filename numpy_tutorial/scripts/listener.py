#!/usr/bin/env python
import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Int32

def callback(data):
    print rospy.get_name(), "I heard %s"%str(data.data)

def listener():
    rospy.init_node('listener')
    rospy.Subscriber("floats", numpy_msg(Int32), callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

