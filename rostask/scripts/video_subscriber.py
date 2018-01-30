#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2
import os
import numpy as np
from rostask.msg import coeff 
from cv_bridge import CvBridge
past_cnt=None
m=None
c=None
pub_coeff=rospy.Publisher('coeffecients',coeff,queue_size=10)
def draw(past_cnt,frame,rough):
    hull=past_cnt
    x,y,w,h = cv2.boundingRect(hull)
    

    (xt,yt) = tuple(hull[hull[:, :, 1].argmin()][0])
    pts=np.array([[x-w/8,y+h],[xt-w/8,yt],[xt+w/10,yt],[x+9*w/8,y+h]])
    pts=pts.reshape(-1,1,2)
    cv2.circle(frame,(xt-w/8,yt),5,(0,0,255),2)
    cv2.circle(frame,(xt+w/10,yt),5,(0,0,255),2)
    cv2.circle(frame,(x+9*w/8,y+h),5,(0,0,255),2)
    cv2.circle(frame,(x-w/8,y+h),5,(0,0,255),2)                
    cv2.polylines(frame,[pts],True,(0,255,0),2)
    points=np.array([[0,719],[640,400],[1279,719]])
    cv2.fillPoly(rough,[points],(255,255,255))
    mid_x1=((xt-w/8)+(xt+w/8))/2
    mid_y1=yt
    mid_x2=((x-w/8)+(x+9*w/8))/2
    mid_y2=y+h
    m=(mid_y2-mid_y1)/(mid_x2-mid_x1)
    c=mid_y2-m*mid_x2
    return m,c    
def callback(data):
	global past_cnt,m,c,pub_coeff
	bridge=CvBridge()
	frame=bridge.imgmsg_to_cv2(data,"bgr8")
        frame_yuv=cv2.cvtColor(frame,cv2.COLOR_BGR2YUV)        
        (y,u,v)=cv2.split(frame_yuv)
        y_eq=cv2.equalizeHist(y)
               
        frm=cv2.merge((y_eq,u,v))
        frame_bgr=cv2.cvtColor(frm,cv2.COLOR_YUV2BGR)
        frame=frame_bgr
        frame_yuv=cv2.cvtColor(frame,cv2.COLOR_BGR2YUV)        
        (y,u,v)=cv2.split(frame_yuv)
        y_eq=cv2.equalizeHist(y)
               
        frm=cv2.merge((y_eq,u,v))
        frame_bgr=cv2.cvtColor(frm,cv2.COLOR_YUV2BGR)
        frame=frame_bgr
        
        (r,c,d)=frame.shape
        y=int(c/2)
        x=int(r-0.15*r)
        rough1=np.zeros((720,1280),dtype='uint8')
        rough=np.zeros((720,1280),dtype='uint8')
        
        cv2.circle(rough,(y,x),40,(255,255,255),-1)
        obj=cv2.bitwise_and(frame_yuv,frame_yuv,mask=rough)
        x=x-30
        y=y-25
        obj_roi=frame_yuv[x:x+50,y:y+50,:]
        obj_hsv=obj_roi
        h_min=np.min(obj_hsv[:,:,0])
        s_min=np.min(obj_hsv[:,:,1])
        v_min=np.min(obj_hsv[:,:,2])
        h_max=np.max(obj_hsv[:,:,0])
        s_max=np.max(obj_hsv[:,:,1])
        v_max=np.max(obj_hsv[:,:,2])        
        road_low=np.array([h_min,s_min,v_min])
        road_high=np.array([h_max,s_max,v_max])
        mask=cv2.inRange(frame_yuv,road_low,road_high)
        kernel=np.ones((5,5),dtype='uint8')
        erode=cv2.erode(mask,kernel,iterations=8)
        dilate=cv2.dilate(erode,kernel,iterations=10)
        _,contours,_=cv2.findContours(dilate,0,cv2.CHAIN_APPROX_SIMPLE)
        frame_and=cv2.bitwise_and(frame,frame,mask=dilate)
        min=40000
        min_cnt=None
        
        for cnt in contours:            
            M=cv2.moments(cnt)
            x1=M['m10']/M['m00']
            y1=M['m01']/M['m00']
            r=float(r)
            c=float(c) 
            dist=(x1-r)**2+(y1-c/2)**2
            if cv2.arcLength(cnt,True)>1000:           
                if (dist-min)<0 :                                        
                    min=dist
                    min_cnt=cnt
        x,y,w,h = cv2.boundingRect(min_cnt)
        
        if min_cnt is not None :
            t_x,t_y= tuple(min_cnt[min_cnt[:, :, 1].argmin()][0])
            if past_cnt is None:
                t_y1=t_y
		t_x1=t_x
            else:
                t_x1,t_y1=tuple(past_cnt[past_cnt[:, :, 1].argmin()][0])
            
            if (t_y-t_y1)**2>500 and past_cnt is not None :
                m,c=draw(past_cnt,frame,rough1)
            else:    
                if cv2.contourArea(min_cnt)<96000 :
                 
                    m,c=draw(min_cnt,frame,rough1)
                    past_cnt=min_cnt
                else:

                    m,c=draw(past_cnt,frame,rough1)
        else:
            m,c=draw(past_cnt,frame,rough1)
            
        
	co=coeff()
	co.m=m
	co.c=c
        pub_coeff.publish(co)
        

       

        rough2=cv2.bitwise_and(frame,frame,mask=rough1)
        mask2=cv2.bitwise_not(rough1)
        rough3=cv2.bitwise_and(frame,frame,mask=mask2)
        
        frame_hls=cv2.cvtColor(rough2,cv2.COLOR_BGR2HLS)        
        ret,thresh=cv2.threshold(frame_hls[:,:,2],100,255,0)
        kernel=np.ones((5,5),dtype='uint8')
        erode=cv2.erode(thresh,kernel,iterations=2)
        dilate=cv2.dilate(thresh,kernel,iterations=5)
        
        frame_hls1=cv2.cvtColor(rough2,cv2.COLOR_BGR2HLS)
        lower_yellow=np.uint8([30,150,150])
        higer_yellow=np.uint8([31,255,255])
        mask=cv2.inRange(frame_hls1,lower_yellow,higer_yellow)
        final_mask=cv2.bitwise_and(dilate,mask)

        dilate=cv2.dilate(final_mask,kernel,iterations=3)     
        factor=dilate==255
        rough2[factor]=(255,0,0)
        final_frame=cv2.bitwise_or(rough3,rough2)
	cv2.imshow('window',final_frame)
	if cv2.waitKey(2) & 0xFF==ord('q'):
		cv2.destroyAllWindows()
		os.system('rosnode kill pub')
		os.system('rosnode kill coeff')
		os.system('rosnode kill sub')
		
def subscriber():
	rospy.init_node('sub')
	rospy.Subscriber('frame',Image,callback)
	rospy.spin()

if __name__=='__main__':
	subscriber()
