#!/usr/bin/env python

import rospy
import cv2
from math import sqrt, atan, pi
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose2D
# from bot_control.msg import StartGoal
from drdo_exploration.msg import teleopData
class pose_publisher:
    def __init__(self):
        self.current_pose=np.array([[234,311,pi/2],[264,312,0.0],[295,314,0.0],[325,315,0.0]])
        self.bot1_pose=Pose2D()
        self.bot2_pose=Pose2D()
        self.bot3_pose=Pose2D()
        self.bot4_pose=Pose2D()
        self.pub_bot1_pose=rospy.Publisher('/bot1_pose',Pose2D,queue_size=1)
        self.pub_bot2_pose=rospy.Publisher('/bot2_pose',Pose2D,queue_size=1)
        self.pub_bot3_pose=rospy.Publisher('/bot3_pose',Pose2D,queue_size=1)
        self.pub_bot4_pose=rospy.Publisher('/bot4_pose',Pose2D,queue_size=1)
        self.vid = cv2.VideoCapture(0)
        self.camera_sub=rospy.Subscriber("/arena/arena1/camera1/image_raw", Image, self.callback_opencv)
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
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        img1 = bridge.imgmsg_to_cv2(data, "bgr8")
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_50)
        arucoParams = cv2.aruco.DetectorParameters_create()
        #now starting to localise bot wrt to the ids
        # Tranform image properly here
        img=img[80:700,80:700]
        img1=img1[80:700,80:700]


        # resize tranformed image to 4 time orginal size
        img=cv2.resize(img,(2480,2480))

        (corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict,parameters=arucoParams)
        a=np.where(ids==10)
        if a[0].size==1:
            # print('1')
            (topLeft, topRight, bottomRight, bottomLeft) = corners[a[0][0]][0]
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (float(bottomRight[0]), float(bottomRight[1]))
            bottomLeft = (float(bottomLeft[0]), float(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            # print(cX,cY)
            img=cv2.circle(img,(cX,cY),10,(100,65,65),-1)            
            self.change_pose(0,cY/4,cX/4,self.angle((bottomRight[1]- bottomLeft[1]),(bottomRight[0]- bottomLeft[0])))
            img1=cv2.circle(img1,(int(self.current_pose[0][0]),int(self.current_pose[0][1])),10,(100,64,65),-1)

        a=np.where(ids==20)
        if a[0].size==1:
            # print('2')
            (topLeft, topRight, bottomRight, bottomLeft) = corners[a[0][0]][0]
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (float(bottomRight[0]), float(bottomRight[1]))
            bottomLeft = (float(bottomLeft[0]), float(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            img=cv2.circle(img,(cX,cY),10,(100,64,100),-1)
           
            self.change_pose(1,cY/4,cX/4,self.angle((bottomRight[1]- bottomLeft[1]),(bottomRight[0]- bottomLeft[0])))
            img1=cv2.circle(img1,(int(self.current_pose[1][0]),int(self.current_pose[1][1])),10,(100,64,100),-1)
            # img1=cv2.circle(img1,self.current_pose[1][0],self.current_pose[1][1],10,(100,64,100),-1)

        a=np.where(ids==30)
        if a[0].size==1:
            # print('3')
            (topLeft, topRight, bottomRight, bottomLeft) = corners[a[0][0]][0]
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (float(bottomRight[0]), float(bottomRight[1]))
            bottomLeft = (float(bottomLeft[0]), float(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            img=cv2.circle(img,(cX,cY),10,(50,60,100),-1)  
            
            self.change_pose(2,cY/4,cX/4,self.angle((bottomRight[1]- bottomLeft[1]),(bottomRight[0]- bottomLeft[0])))
            img1=cv2.circle(img1,(int(self.current_pose[2][0]),int(self.current_pose[2][1])),10,(100,64,120),-1)
            # img1=cv2.circle(img1,self.current_pose[2][0],self.current_pose[2][1],10,(50,60,100),-1)

        a=np.where(ids==40)
        if a[0].size==1:
            # print('4')
            (topLeft, topRight, bottomRight, bottomLeft) = corners[a[0][0]][0]
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (float(bottomRight[0]), float(bottomRight[1]))
            bottomLeft = (float(bottomLeft[0]), float(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            img=cv2.circle(img,(cX,cY),10,(50,100,80),-1)
            # img1=cv2.circle(img1,(int(cX/3),int(cY/3)),10,(50,100,80),-1)
            self.change_pose(3,cY/4,cX/4,self.angle((bottomRight[1]- bottomLeft[1]),(bottomRight[0]- bottomLeft[0])))
            img1=cv2.circle(img1,(int(self.current_pose[3][0]),int(self.current_pose[3][1])),10,(100,64,165),-1)
        
        # cv2.imshow('original_image', img)
        # cv2.waitKey(1)
        cv2.imshow('processed_image',img1)
        cv2.waitKey(1)
        # cv2.imwrite('arena.png',img1)

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
    #to keep the function running in loop
    rospy.spin()