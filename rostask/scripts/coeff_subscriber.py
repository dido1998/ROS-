#!/usr/bin/env python
import rospy
import os
from rostask.msg import coeff 
def callback(msg):
	rospy.loginfo('the slope is %d and the constant is %d'%(msg.m,msg.c))
if __name__=='__main__':
	rospy.init_node('coeff')
	rospy.Subscriber('coeffecients',coeff,callback)
	rospy.spin()
