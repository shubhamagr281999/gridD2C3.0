#!/usr/bin/env python

import rospy
import time
from time import sleep
from math import sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler , euler_matrix
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int16

from bot_control.msg import StartGoal, CompletePlan, PathArray, Poses
from geometry_msgs.msg import Point
import numpy as np

class goal_publisher:
    def __init__(self):
        self.rate=rospy.Rate(0.5)
        self.n_agents=1
        self.current_pose=np.zeros([self.n_agents,3])
        self.goal_pose=np.zeros([self.n_agents,3])
        self.time_for_flipping=2
        self.count=[0,0,0,0]
        self.pose_error=[]
        self.theta_error=0.1
        self.turning_points=self.empty_list(self.n_agents)
        self.lin_threshold=6
        self.upper_croner=[21,21]
        self.lower_bottom_corner=[621,621]

        self.pub_goal=rospy.Publisher('/goal_point',Point,queue_size=10)

        self.pub_parcel_flip1=rospy.Publisher('/Parcel_flip/1',Int16, queue_size=1)
        self.pub_parcel_flip2=rospy.Publisher('/Parcel_flip/2',Int16, queue_size=1)
        self.pub_parcel_flip3=rospy.Publisher('/Parcel_flip/3',Int16, queue_size=1)
        self.pub_parcel_flip4=rospy.Publisher('/Parcel_flip/4',Int16, queue_size=1)

        self.poses_sub=rospy.Subscriber('/poses',Poses,self.pose_callback,queue_size=10)
        self.plans_callback=rospy.Subscriber("/cbs/plan",CompletePlan,self.plan_callback,queue_size=1)
    def empty_list(self,i):
        k=[]
        for j in range(i):
            k.append([])
        return k

    def plan_callback(self,msg):
        for i in msg.agent:
            xi=[]
            yi=[]
            for j in i.statei:
                xi.append(j.x)
                yi.append(j.y)
            self.turning_point(xi,yi,i.bot_num)

    def turning_point(self,x,y,bot_num):
        turnpoints=[]
        for i in range(1,len(x)-1):
            #movement in y direction
            if x[i] == x[i+1] and y[i] != y[i+1]:
                if x[i] != x[i-1] and y[i] == y[i-1]:
                    turnpoints.append([x[i],y[i]]) 
            #movement in x direction
            elif x[i] != x[i+1] and y[i] == y[i+1]:
                if x[i] == x[i-1] and y[i] != y[i-1]:
                    turnpoints.append([x[i],y[i]]) 
            #Halt
            else:
                turnpoints.append([-100,-100])
        turnpoints.append([x[-1],y[-1]])
        self.turning_points[bot_num]=turnpoints
        # print(self.turning_points)
        # print('-----------------------------------------------------------')

    def pose_callback(self,msg):
        for i in range(self.n_agents):
            self.current_pose[i][0]=msg.posei[i].x
            self.current_pose[i][1]=msg.posei[i].y
            self.current_pose[i][2]=msg.posei[i].z

    def flipmotor(self,bot_num):
    	rospy.loginfo('fliiping pracel for bot', bot_num )
        msg=Int16()
        msg.data=1
        if(bot_num==1):
            self.pub_parcel_flip1.publish(msg)
        if(bot_num==2):
            self.pub_parcel_flip2.publish(msg)
        if(bot_num==3):
            self.pub_parcel_flip3.publish(msg)
        if(bot_num==4):
            self.pub_parcel_flip4.publish(msg)

    def goal_pub(self,bot_num):
        msg=Point()

        msg.x=(self.goal_pose[bot_num][0])
        msg.y=self.goal_pose[bot_num][1]
        msg.z=bot_num
        self.pub_goal.publish(msg)

    def goal(self):
        time.sleep(5)
        while not rospy.is_shutdown(): 
            for i in range(self.n_agents):
                if(len(self.turning_points[i])>0):
                    # print("turn point there")
                    goal_distance=sqrt((self.current_pose[i][0]-self.goal_pose[i][0])**2+(self.current_pose[i][1]-self.goal_pose[i][1])**2)
                    # print(goal_distance)
                    if(goal_distance<self.lin_threshold):
                        self.goal_pose[i][0]=self.turning_points[i][0][0]*(600.0/14.0)+(21+600.0/28)
                        self.goal_pose[i][1]=self.turning_points[i][0][1]*(600.0/14.0)+(21+600.0/28)
                        # print(self.turning_points[i].pop(0))
                        self.goal_pub(i)
                        # print("sending new goal:", self.goal_pose[i])
                else:
                    print('bot ',i," needs new plan")
            self.rate.sleep()  

if __name__ == '__main__':
    rospy.init_node('goal_publisher')
    rospy.loginfo("goal_publisher created | now goal for each bot will be published on goal topics")
    goal_pub_obj=goal_publisher()
    # rospy.spin()
    goal_pub_obj.goal()