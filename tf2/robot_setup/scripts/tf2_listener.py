#!/usr/bin/env python
import rospy
import tf
from geometry_msgs.msg import PointStamped
import PyKDL
import tf2_ros

if __name__=='__main__':
	rospy.init_node('message_listener')
	rate=rospy.Rate(1)
	tf_buffer=tf2_ros.Buffer()
	listener=tf.TransformListener()
	rate=rospy.Rate(0.1)
	trans=tf.TransformerROS(listener)
	while not rospy.is_shutdown():
		rate.sleep()
		laser_point=PointStamped()
  		laser_point.header.frame_id = "base_laser";

  
  		laser_point.header.stamp = rospy.Time(0);

  
  		laser_point.point.x = 1.0;
  		laser_point.point.y = 0.2;
  		laser_point.point.z = 0.0;
  		pt=listener.transformPoint("base_link",laser_point)
  		rospy.loginfo('point in base_link is (x:%f,y:%f,z:%f)'%(pt.point.x,pt.point.y,pt.point.y))
  		rate.sleep()
