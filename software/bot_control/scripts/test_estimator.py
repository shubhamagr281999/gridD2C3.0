#!/usr/bin/env python

import rospy
import cv2
from math import sqrt, atan, pi, ceil
import numpy as np
from geometry_msgs.msg import Pose, PoseArray

class pose_publisher:
    def __init__(self):
        self.n_agents=8
        self.current_pose=np.zeros([self.n_agents,3])
        self.initialize_current_pose()
        self.control_rate=rospy.Rate(10)

        #publisher
        self.pub_poses=rospy.Publisher('/poses',PoseArray,queue_size=1)
        self.poses=PoseArray()
        self.initialize_pose_msg()
        # while not rospy.is_shutdown():
        #     self.pose_pub()
        #     self.control_rate.sleep()
        # subscriber
        self.cmd_vel_sub=rospy.Subscriber('/cmd_vel',PoseArray,self.callback_opencv,queue_size=1)
        # self.vid = cv2.VideoCapture(0)


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
        self.pose_pub()
        print(self.current_pose)
    def pose_pub(self):
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