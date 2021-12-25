#!/usr/bin/env python

import rospy
import cv2
from math import sqrt, atan, pi, floor, ceil
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose, PoseArray
# from bot_control.msg import Poses

class pose_publisher:
    def __init__(self):
        self.n_agents=4
        self.current_pose=np.zeros([self.n_agents,3])
        self.initialize_current_pose()
        self.poses=PoseArray()
        self.initialize_pose_msg()
        self.bridge = CvBridge()
        self.control_rate=rospy.Rate(10)
        self.pub_poses=rospy.Publisher('/poses',PoseArray,queue_size=1)
        # self.image_sub=rospy.Subscriber('/image',Image,self.callback_opencv,queue_size=1)
        self.cmd_vel_sub=rospy.Subscriber('/cmd_vel',PoseArray,self.callback_opencv,queue_size=1)
        # self.vid = cv2.VideoCapture(0)
        
        self.img_base=cv2.imread('/home/shubham/catkin_ws/src/gridD2C3.0/software/bot_control/scripts/maze.png')
        self.img_base=cv2.cvtColor(self.img_base, cv2.COLOR_BGR2GRAY)

    def initialize_current_pose(self):
        for i in range(self.n_agents):
            if(i<int(ceil(self.n_agents/2.0))):
                self.current_pose[i][0]=(4-i)*6+3
                
                if(i==0):
                    self.current_pose[i][1]=3
                    self.current_pose[i][2]=pi/2
                else :
                    self.current_pose[i][1]=9
                    self.current_pose[i][2]=0
            else :
                self.current_pose[i][0]=(9 + i - ceil(self.n_agents/2.0))*6 + 3
                
                if(i==ceil(self.n_agents/2.0)):
                    self.current_pose[i][1]=3
                    self.current_pose[i][2]=pi/2
                else :
                    self.current_pose[i][1]=9
                    self.current_pose[i][2]=pi
    def initialize_pose_msg(self):
        temp_poses=[]
        for i in range(self.n_agents):
            temp_poses.append(Pose())
        self.poses.poses=temp_poses
        
    def angle_bound(self,a):
        if(a<-pi):
            return 2*pi +a
        elif(a>pi):
            return a-2*pi
        return a

    def change_pose(self,bot,x,y,yaw):
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

    def callback_opencv(self,msg):
        for i in range(self.n_agents):
            self.current_pose[i][0]=msg.poses[i].position.x*0.1+self.current_pose[i][0]
            self.current_pose[i][1]=msg.poses[i].position.y*0.1+self.current_pose[i][1]
            self.current_pose[i][2]=msg.poses[i].position.z*0.1+self.current_pose[i][2]

        # img = self.bridge.imgmsg_to_cv2(msg, "mono8")
        # img1 = self.bridge.imgmsg_to_cv2(msg, "mono8")
        # arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_50)
        # arucoParams = cv2.aruco.DetectorParameters_create()
        # #now starting to localise bot wrt to the ids

        # # resize tranformed image to 4 time orginal size
        # resize_=4
        # img=cv2.resize(img,(resize_*img.shape[0],resize_*img.shape[1]))
        # (corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict,parameters=arucoParams)
        # for i in range(self.n_agents):            
        #     a=np.where(ids==(i+1))
        #     if a[0].size==1:
        #         print(i)
        #         (topLeft, topRight, bottomRight, bottomLeft) = corners[a[0][0]][0]
        #         topRight = (int(topRight[0]), int(topRight[1]))
        #         bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        #         bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        #         topLeft = (int(topLeft[0]), int(topLeft[1]))
        #         img1=cv2.circle(img1,(bottomRight),10,(250-i*int(200/self.n_agents),10+i*int(200/self.n_agents),30+i*int(200/self.n_agents)),-1)
        #         cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        #         cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        #         print(bottomRight,bottomLeft,topRight,topLeft)
        #         print(cX,cY)
        #         img=cv2.circle(img,(cX,cY),10,(250-i*10,10+i*15,30+i*20),-1)            
        #         self.change_pose(i,cX/resize_,cY/resize_,self.angle((bottomRight[1]-bottomLeft[1]),(bottomRight[0]- bottomLeft[0])))
        #         # img1=cv2.circle(img1,(int(self.current_pose[i][0]),int(self.current_pose[i][1])),10,(250-i*int(200/self.n_agents),10+i*int(200/self.n_agents),30+i*int(200/self.n_agents)),-1)
        
        # # cv2.imshow('original_image', img)
        # # cv2.waitKey(1)
    

        # cv2.imshow('processed_image',img1)
        # cv2.waitKey(1)
        # # # cv2.imwrite('arena.png',img1)

        for i in range(self.n_agents):
            self.poses.poses[i].position.x=self.current_pose[i][0]
            self.poses.poses[i].position.y=self.current_pose[i][1]
            self.poses.poses[i].position.z=self.current_pose[i][2]

        self.pub_poses.publish(self.poses)


if __name__ == '__main__':
    rospy.init_node('pose_estimator')
    rospy.loginfo('reading from camera')
    pose_pub_obj=pose_publisher()
    # pose_pub_obj.callback_opencv()
    #to keep the function running in loop
    rospy.spin()