#!/usr/bin/env python

import rospy
import cv2
from math import sqrt, atan, pi
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from bot_control.msg import Poses

class pose_publisher:
    def __init__(self):
        self.n_agents=4
        self.current_pose=np.zeros([self.n_agents,3])+1
        self.poses=Poses()
        self.initialize_pose()
        self.pub_poses=rospy.Publisher('/poses',Poses,queue_size=1)
        self.vid = cv2.VideoCapture(0)
        self.camera_sub=rospy.Subscriber("/arena/arena1/camera1/image_raw", Image, self.callback_opencv)
        # self.vid = cv2.VideoCapture(0)
        # self.callback_opencv(1)

    def initialize_pose(self):
        temp_poses=[]
        for i in range(self.n_agents):
            temp_poses.append(Point())
        self.poses.posei=temp_poses
        
    def angle_bound(self,a):
        if(a<-pi):
            return 2*pi +a
        elif(a>pi):
            return a-2*pi
        return a

    def change_pose(self,bot,y,x,yaw):
        if(y!=0):
            self.current_pose[bot][1]=y
        if(x!=0):
            self.current_pose[bot][0]=x
        if(yaw!=0):
            self.current_pose[bot][2]=yaw

    def angle(self,y,x):
        x=x+0.000001
        if(x>0 and y>0):
            return atan(y/x)
        elif(x<0 and y>0):
            return (pi +atan(y/x))
        elif(x<0 and y<0):
            return (atan(y/x)-pi)
        else:
            return atan(y/(x+0.00001))

    def callback_opencv(self,data):
        # ret, img = self.vid.read()
        # ret, img1 = self.vid.read()
        # cv2.imshow('original_image',img1)
        # cv2.waitKey(1)

        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        img1 = bridge.imgmsg_to_cv2(data, "bgr8")

        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_50)
        arucoParams = cv2.aruco.DetectorParameters_create()
        #now starting to localise bot wrt to the ids
        # Tranform image properly here
        img=img[70:710,70:710]
        img1=img1[70:710,70:710]
        # resize tranformed image to 4 time orginal size
        img=cv2.resize(img,(2560,2560))
        (corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict,parameters=arucoParams)
        for i in range(self.n_agents):            
            a=np.where(ids==(i+1)*10)
            if a[0].size==1:
                # print(i)
                (topLeft, topRight, bottomRight, bottomLeft) = corners[a[0][0]][0]
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (float(bottomRight[0]), float(bottomRight[1]))
                bottomLeft = (float(bottomLeft[0]), float(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                # print(cX,cY)
                img=cv2.circle(img,(cX,cY),10,(250-i*10,10+i*15,30+i*20),-1)            
                self.change_pose(i,cY/4,cX/4,self.angle((-1*bottomRight[1]+bottomLeft[1]),(-1*bottomRight[0]+ bottomLeft[0])))
                img1=cv2.circle(img1,(int(self.current_pose[i][0]),int(self.current_pose[i][1])),10,(250-i*int(200/self.n_agents),10+i*int(200/self.n_agents),30+i*int(200/self.n_agents)),-1)
        
        # cv2.imshow('original_image', img)
        # cv2.waitKey(1)
        # cv2.imshow('processed_image',img1)
        # cv2.waitKey(1)
        # cv2.imwrite('arena.png',img1)
        for i in range(self.n_agents):
            self.poses.posei[i].x=self.current_pose[i][0]
            self.poses.posei[i].y=self.current_pose[i][1]
            self.poses.posei[i].z=self.current_pose[i][2]
        self.pub_poses.publish(self.poses)

if __name__ == '__main__':
    rospy.init_node('pose_estimator')
    rospy.loginfo('reading from camera')
    pose_pub_obj=pose_publisher()
    #to keep the function running in loop
    rospy.spin()