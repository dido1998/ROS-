#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
import os
from cv_bridge import CvBridge, CvBridgeError
def publisher():
	pub=rospy.Publisher('frame',Image)
	rospy.init_node('pub')
	bridge=CvBridge()
	cap=cv2.VideoCapture('/home/aniket/Desktop/manas-task-phase/opencvtask/project_video.avi')
	while not rospy.is_shutdown():
		ret,frame=cap.read()
		if ret==False:
			rospy.loginfo(ret)
			cv2.destroyAllWindows()
			os.system('rosnode kill sub')
			os.system('rosnode kill coeff')
			os.system('rosnode kill pub')
			break
		msg_frame = bridge.cv2_to_imgmsg(frame,"bgr8")
        	#rospy.loginfo(msg_frame)
        	pub.publish(msg_frame)


	

if __name__=='__main__':
    try:
        publisher()
	cv2.destroyAllWindows()
    except rospy.ROSInterruptException:
        pass


