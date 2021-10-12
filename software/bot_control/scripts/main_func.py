#!/usr/bin/env python

import cv2
from math import sqrt, atan, pi
import numpy as np

class grid:
    def __init__(self):
        self.current_pose=np.array([[234,311,pi/2],[264,312,0.0],[295,314,0.0],[325,315,0.0]])
        self.goal_pose=[[234,311,0.0],[264,312,0.0],[295,314,0.0],[325,315,0.0]]
        self.lastError = np.zeros((4,1))
        self.lastError_small=np.zeros((4,1))
        self.sumError = np.zeros((4,1))

        self.count=0
        self.mission_bot=0
        self.vid = cv2.VideoCapture(0)
        # params to tune
        self.time_for_flipping=2
        self.lin_threshold_SE=200
        self.yaw_threshold_SE=2

        self.kp_lin = 1
        self.ki_lin = 0.0
        self.kd_lin = 0.4

        self.kp_angle = 200
        self.ki_angle = 0.0
        self.kd_angle = 0.0

        self.kp_angle_soft = 69
        self.ki_angle_soft = 0.0
        self.kd_angle_soft = 15.0

        self.max_vel_lin=1.8
        self.max_vel_ang=50
        self.lin_threshold=1
        self.yaw_threshold=0.3
        self.intergral_windup_yaw=20
        self.intergral_windup_lin=15

    def resetValues(self,bot):
        self.lastError[bot][0] = 0
        self.lastError_small[bot][0]=0
        self.sumError[bot][0]= 0

    def change_pose(self,bot,y,x,yaw):
        if(abs(y-self.current_pose[bot][1])<self.lin_threshold_SE):
            self.current_pose[bot][1]=y
        if(abs(x-self.current_pose[bot][0])<self.lin_threshold_SE):
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

    def estimation(self):
        ret, img = self.vid.read()
        ret, img1 = self.vid.read()
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_50)
        arucoParams = cv2.aruco.DetectorParameters_create()
        #now starting to localise bot wrt to the ids
        img=img[50:390,25:630]
        img1=img1[50:390,25:630]
        img=cv2.resize(img,(1815,1020))

        (corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict,parameters=arucoParams)
        a=np.where(ids==20)
        if a[0].size==1:
            (topLeft, topRight, bottomRight, bottomLeft) = corners[a[0][0]][0]
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (float(bottomRight[0]), float(bottomRight[1]))
            bottomLeft = (float(bottomLeft[0]), float(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            img=cv2.circle(img,(cX,cY),10,(100,65,65),-1)
            img1=cv2.circle(img1,(int(cX/3),int(cY/3)),10,(100,64,65),-1)
            print(cX/3,cY/3)
            self.change_pose(0,cY/3,cX/3,self.angle((bottomRight[0]- bottomLeft[0]),(bottomRight[1]- bottomLeft[1])))
            
        a=np.where(ids==10)
        if a[0].size==1:
            (topLeft, topRight, bottomRight, bottomLeft) = corners[a[0][0]][0]
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (float(bottomRight[0]), float(bottomRight[1]))
            bottomLeft = (float(bottomLeft[0]), float(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            img=cv2.circle(img,(cX,cY),10,(100,64,100),-1)
            img1=cv2.circle(img1,(int(cX/3),int(cY/3)),10,(100,64,100),-1)     
            self.change_pose(1,cX/3,cY/3,self.angle((bottomRight[0]- bottomLeft[0]),(bottomRight[1]- bottomLeft[1])))
       
        a=np.where(ids==30)
        if a[0].size==1:
            (topLeft, topRight, bottomRight, bottomLeft) = corners[a[0][0]][0]
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (float(bottomRight[0]), float(bottomRight[1]))
            bottomLeft = (float(bottomLeft[0]), float(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            img=cv2.circle(img,(cX,cY),10,(50,60,100),-1)  
            img1=cv2.circle(img1,(int(cX/3),int(cY/3)),10,(50,60,100),-1)
            self.change_pose(2,cX/3,cY/3,self.angle((bottomRight[0]- bottomLeft[0]),(bottomRight[1]- bottomLeft[1])))
            
        a=np.where(ids==40)
        if a[0].size==1:
            (topLeft, topRight, bottomRight, bottomLeft) = corners[a[0][0]][0]
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (float(bottomRight[0]), float(bottomRight[1]))
            bottomLeft = (float(bottomLeft[0]), float(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            img=cv2.circle(img,(cX,cY),10,(50,100,80),-1)
            img1=cv2.circle(img1,(int(cX/3),int(cY/3)),10,(50,100,80),-1)
            self.change_pose(3,cX,cY,self.angle((bottomRight[0]- bottomLeft[0]),(bottomRight[1]- bottomLeft[1])))
        
        cv2.imshow('original_image', img)
        cv2.waitKey(1)
        cv2.imshow('processed_image',img1)
        cv2.waitKey(1)

    def flipmotor(self,bot_num):
        print('fliiping pracel for bot', bot_num )
        time.sleep(self.time_for_flipping)

    def goal(self):
        print('goal called')
        print('new goal sent. Count:',self.count)
        if self.count==0:
            self.goal_pose[0][0]=244
            self.goal_pose[0][1]=72
        elif self.count==1:
            self.goal_pose[0][0]=53
            self.goal_pose[0][1]=65
        elif self.count==2:
            self.flipmotor(1)
            self.goal_pose[0][0]=244
            self.goal_pose[0][1]=72
        elif self.count==3:
            self.goal_pose[0][0]=234
            self.goal_pose[0][1]=311
        elif self.count==4:
            self.mission_bot=self.mission_bot+1
            self.goal_pose[0][0]=272
            self.goal_pose[0][1]=40
        elif self.count==5:
            self.goal_pose[0][0]=55
            self.goal_pose[0][1]=36
        elif self.count==6:
            self.flipmotor(2)
            self.goal_pose[0][0]=272
            self.goal_pose[0][1]=40
        elif self.count==7:
            self.goal_pose[0][0]=264
            self.goal_pose[0][1]=312
        elif self.count==8:
            self.mission_bot=self.mission_bot+1
            self.goal_pose[0][0]=305
            self.goal_pose[0][1]=42
        elif self.count==9:
            self.goal_pose[0][0]=556
            self.goal_pose[0][1]=46
        elif self.count==10:
            self.flipmotor(3)
            self.goal_pose[0][0]=305
            self.goal_pose[0][1]=42
        elif self.count==11:
            self.goal_pose[0][0]=295
            self.goal_pose[0][1]=314
        elif self.count==12:
            self.mission_bot=self.mission_bot+1
            self.goal_pose[0][0]=336
            self.goal_pose[0][1]=72
        elif self.count==13:
            self.goal_pose[0][0]=556
            self.goal_pose[0][1]=77
        elif self.count==14:
            self.flipmotor(4)
            self.goal_pose[0][0]=336
            self.goal_pose[0][1]=72
        elif self.count==15:
            self.goal_pose[0][0]=325
            self.goal_pose[0][1]=314
        if(self.count<16):
            self.PID(self.mission_bot)

    def PID(self,bot):
        self.estimation()
        path_angle=self.angle((self.goal_pose[bot][1]-self.current_pose[bot][1]),(self.goal_pose[bot][0]-self.current_pose[bot][0]))
        goal_distance=sqrt((self.current_pose[bot][0]-self.goal_pose[bot][0])**2+(self.current_pose[bot][1]-self.goal_pose[bot][1])**2)
        diff_yaw=path_angle-self.current_pose[bot][2]
        
        #aligining towards the path
        self.resetValues(bot)        
        while (abs(diff_yaw)>self.yaw_threshold):
            self.estimation()
            path_angle=self.angle((self.goal_pose[bot][1]-self.current_pose[bot][1]),(self.goal_pose[bot][0]-self.current_pose[bot][0]))
            diff_yaw=path_angle-self.current_pose[bot][2]
            control_signal_angle=self.kp_angle*(diff_yaw)+self.kd_angle*(diff_yaw-self.lastError[bot][0])+self.ki_angle*(self.sumError[bot][0])
            self.lastError[bot][0]=diff_yaw
            if(abs(self.sumError[bot][0]+diff_yaw)<self.intergral_windup_yaw):
                self.sumError[bot][0]=self.sumError[bot][0]+diff_yaw
            print(control_signal_angle)
            #introduce delay
            #pick from here

        #moving towards goal        
        self.resetValues(bot)
        while(goal_distance>self.lin_threshold):
            self.estimation()
            path_angle=self.angle((self.goal_pose[bot][1]-self.current_pose[bot][1]),(self.goal_pose[bot][0]-self.current_pose[bot][0]))
            diff_yaw=path_angle-self.current_pose[bot][2]
            diff_distance=sqrt((self.current_pose[bot][0]-self.goal_pose[bot][0])**2+(self.current_pose[bot][1]-self.goal_pose[bot][1])**2)
            control_signal_dist=self.kp_lin*(diff_distance)+self.kd_lin*(diff_distance-self.lastError[bot][0])+self.ki_lin*self.sumError[bot][0]
            control_signal_angle=self.kp_angle_soft*(diff_yaw)+self.kd_angle_soft*(diff_yaw-self.lastError_small[bot][0])
            self.lastError[bot][0]=diff_distance
            self.lastError_small[bot][0]=diff_yaw
            if(abs(self.sumError[bot][0]+diff_distance)<self.intergral_windup_lin):
                self.sumError[bot][0]=self.sumError[bot][0]+diff_distance
            print(control_signal_dist)
            #introduce delay
            #pick from here
        self.resetValues(bot)
        #send 0,0 from here
        self.count=self.count+1
        self.goal()
    

if __name__ == '__main__':
    grid_obj=grid()
    print('calling goal')
    grid_obj.goal()
    #to keep the function running in loop