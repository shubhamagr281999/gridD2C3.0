#!/usr/bin/env python

import rospy
import cv2
from math import sqrt, atan, pi
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose2D

class pose_publisher:
    def __init__(self):
        self.current_pose=np.array([[0.6858,0.2286,3.10],[0.6858,0.0762,3.10],[0.6858,-0.0762,3.10],[0.6858,-0.2286,3.10]])
        self.bot1_pose=Pose2D()
        self.bot2_pose=Pose2D()
        self.bot3_pose=Pose2D()
        self.bot4_pose=Pose2D()
        self.lin_threshold=0.3
        self.yaw_threshold=1.2
        self.factor=0.00269541
        self.pub_bot1_pose=rospy.Publisher('/bot1_pose',Pose2D,queue_size=1)
        self.pub_bot2_pose=rospy.Publisher('/bot2_pose',Pose2D,queue_size=1)
        self.pub_bot3_pose=rospy.Publisher('/bot3_pose',Pose2D,queue_size=1)
        self.pub_bot4_pose=rospy.Publisher('/bot4_pose',Pose2D,queue_size=1)
        self.vid = cv2.VideoCapture(2)
        # self.camera_sub=rospy.Subscriber("/arena/arena1/camera1/image_raw", Image, self.callback_opencv)
    def angle_bound(self,a):
        if(a<-pi):
            return 2*pi +a
        elif(a>pi):
            return a-2*pi
        return a

    def change_pose(self,bot,y,x,yaw):
        if(abs(y-self.current_pose[bot][1])<self.lin_threshold):
            self.current_pose[bot][1]=y
        if(abs(x-self.current_pose[bot][0])<self.lin_threshold):
            self.current_pose[bot][0]=x
        if(yaw!=0):
            self.current_pose[bot][2]=yaw
        print('yaw',yaw,'bot:',bot)

    def angle(self,y,x):
        x=x+0.000001
        if(x>0 and y>0):
            return atan(y/x)
        elif(x<0 and y>0):
            return (pi +atan(y/x))
        elif(x<0 and y<0):
            return (atan(y/x)-pi)
        else:
            return atan(y/x)

    def callback_opencv(self):
        while not rospy.is_shutdown():
            ret, img = self.vid.read()
            arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_50)
            arucoParams = cv2.aruco.DetectorParameters_create()
            (corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict,parameters=arucoParams)

            #now starting to localise bot wrt to the ids
            a=np.where(ids==10)
            if a[0].size==1:
                (topLeft, topRight, bottomRight, bottomLeft) = corners[a[0][0]][0]
                img=cv2.rectangle(img,(topLeft[0],topLeft[1]),(bottomRight[0],bottomRight[1]),(0,255,0),3)
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (float(bottomRight[0]), float(bottomRight[1]))
                bottomLeft = (float(bottomLeft[0]), float(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                self.change_pose(0,-(cX-img.shape[1]/2.0)*self.factor,-(cY-img.shape[0]/2.0)*self.factor,self.angle((bottomRight[0]- bottomLeft[0]),(bottomRight[1]- bottomLeft[1])))
                
            a=np.where(ids==20)
            if a[0].size==1:
                (topLeft, topRight, bottomRight, bottomLeft) = corners[a[0][0]][0]
                img=cv2.rectangle(img,(topLeft[0],topLeft[1]),(bottomRight[0],bottomRight[1]),(0,255,255),3)
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (float(bottomRight[0]), float(bottomRight[1]))
                bottomLeft = (float(bottomLeft[0]), float(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)        
                self.change_pose(1,-(cX-img.shape[1]/2.0)*self.factor,-(cY-img.shape[0]/2.0)*self.factor,self.angle((bottomRight[0]- bottomLeft[0]),(bottomRight[1]- bottomLeft[1])))
                
            a=np.where(ids==30)
            if a[0].size==1:
                (topLeft, topRight, bottomRight, bottomLeft) = corners[a[0][0]][0]
                img=cv2.rectangle(img,topLeft,bottomRight,(255,255,0),3)
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (float(bottomRight[0]), float(bottomRight[1]))
                bottomLeft = (float(bottomLeft[0]), float(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)     
                self.change_pose(2,-(cX-img.shape[1]/2.0)*self.factor,-(cY-img.shape[0]/2.0)*self.factor,self.angle((bottomRight[0]- bottomLeft[0]),(bottomRight[1]- bottomLeft[1])))
                
            a=np.where(ids==40)
            if a[0].size==1:
                (topLeft, topRight, bottomRight, bottomLeft) = corners[a[0][0]][0]
                img=cv2.rectangle(img,topLeft,bottomRight,(0,0,255),3)
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (float(bottomRight[0]), float(bottomRight[1]))
                bottomLeft = (float(bottomLeft[0]), float(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)     
                self.change_pose(3,-(cX-img.shape[1]/2.0)*self.factor,-(cY-img.shape[0]/2.0)*self.factor,self.angle((bottomRight[0]- bottomLeft[0]),(bottomRight[1]- bottomLeft[1])))
                

            cv2.imshow('frame', img)
            cv2.waitKey(1)
            self.bot1_pose.x=self.current_pose[0][0]
            self.bot1_pose.y=self.current_pose[0][1]
            self.bot1_pose.theta=self.current_pose[0][2] 
            self.pub_bot1_pose.publish(self.bot1_pose)

            self.bot2_pose.x=self.current_pose[1][0]
            self.bot2_pose.y=self.current_pose[1][1]
            self.bot2_pose.theta=self.current_pose[1][2]
            self.pub_bot2_pose.publish(self.bot2_pose)

            self.bot3_pose.x=self.current_pose[2][0]
            self.bot3_pose.y=self.current_pose[2][1]
            self.bot3_pose.theta=self.current_pose[2][2]
            self.pub_bot3_pose.publish(self.bot3_pose)

            self.bot4_pose.x=self.current_pose[3][0]
            self.bot4_pose.y=self.current_pose[3][1]
            self.bot4_pose.theta=self.current_pose[3][2]
            self.pub_bot4_pose.publish(self.bot4_pose)

if __name__ == '__main__':
    rospy.init_node('pose_estimator')
    rospy.loginfo('reading from camera')
    pose_pub_obj=pose_publisher()
    pose_pub_obj.callback_opencv()
    #to keep the function running in loop
    rospy.spin()