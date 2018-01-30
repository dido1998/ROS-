#!/usr/bin/env python

import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Int32
import numpy
def talker():
    pub = rospy.Publisher('floats', numpy_msg(Int32),queue_size=10)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        a = numpy.array([1.0, 2.1, 3.2, 4.3, 5.4, 6.5], dtype=numpy.float32)
        pub.publish(24)
        r.sleep()

if __name__ == '__main__':
    talker()
