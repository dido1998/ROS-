#!/usr/bin/env python  
import rospy
import tf2_ros
import tf2_msgs.msg
import geometry_msgs.msg


class FixedTFBroadcaster:

    def __init__(self):
    	    br = tf2_ros.TransformBroadcaster()
    	    t = geometry_msgs.msg.TransformStamped()


            t.header.frame_id = 'turtle1'
            t.child_frame_id ='carrot1'

            while not rospy.is_shutdown():

            	rospy.sleep(0.1)
            	t.header.stamp = rospy.Time.now()
            	t.transform.translation.x = 0.0
            	t.transform.translation.y = 2.0
            	t.transform.translation.z = 0.0

            	t.transform.rotation.x = 0.0
            	t.transform.rotation.y = 0.0
            	t.transform.rotation.z = 0.0
            	t.transform.rotation.w = 1.0

     	    	br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('fixed_tf2_broadcaster')
    tfb = FixedTFBroadcaster()

    rospy.spin()
