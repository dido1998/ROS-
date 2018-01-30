#!/usr/bin/env python
import rospy
import tf
import geometry_msgs.msg
if __name__=='__main__':
	rospy.init_node('message_broadcaster')
	br=tf.TransformBroadcaster()
	rate=rospy.Rate(10)
   	while not rospy.is_shutdown():
		br.sendTransform((0.1, 0.2, 0.0),(0.0, 0.0, 0.0, 1.0),rospy.Time.now(),"base_link","base_laser")
		rate.sleep()