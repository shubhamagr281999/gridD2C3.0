#!/usr/bin/env python

import rospy
import time
from time import sleep
from tf.transformations import euler_from_quaternion, quaternion_from_euler , euler_matrix
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int16

from bot_control.msg import StartGoal, CompletePlan, PathArray, Poses
from geometry_msgs.msg import Point

class goal_publisher:
    def __init__(self):
        self.rate=rospy.Rate(0.5)
        self.current_pose=[]
        self.goal_pose=[]
        self.time_for_flipping=2
        self.count=[0,0,0,0]
        self.pose_error=[]
        self.theta_error=0.1
        self.paths=[]
        self.turning_points=[]
        self.n_agents=4

        self.pub_goal=rospy.Publisher('/goal_point',Point,queue_size=10)

        self.pub_parcel_flip1=rospy.Publisher('/Parcel_flip/1',Int16, queue_size=1)
        self.pub_parcel_flip2=rospy.Publisher('/Parcel_flip/2',Int16, queue_size=1)
        self.pub_parcel_flip3=rospy.Publisher('/Parcel_flip/3',Int16, queue_size=1)
        self.pub_parcel_flip4=rospy.Publisher('/Parcel_flip/4',Int16, queue_size=1)

        self.poses_sub=rospy.Subscriber('/poses',Poses,self.pose_callback,queue_size=10)

        self.plans_callback=rospy.Subscriber("/cbs/plan",CompletePlan,self.plan_callback,queue_size=1)
    def plan_callback(self,msg):
        self.paths=[]
        n_agents=len(msg.agent)
        for i in range(n_agents):
            n_states=len(msg.agent[i].statei)
            xi=[]
            yi=[]
            for j in range(n_states):
                xi.append(msg.agent[i].statei[j].x)
                yi.append(msg.agent[i].statei[j].y)
            self.paths.append(xi)
            self.paths.append(yi)
        # print(self.paths)
        # print("-----------------------------------------------------------------")
        self.turning_point()

    def turning_point(self):
        # print("-----------------------------------------------------")
        self.turning_points=[]
        for jj in range(int(len(self.paths)/2)):
            x=self.paths[2*jj]
            y=self.paths[2*jj+1]
            turnpoints=[]
            for i in range(1,len(self.paths[2*jj])-1):
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
            self.turning_points.append(turnpoints)
        # print(self.turning_points)

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

    def goal(self):
    	time.sleep(5)
    	while not rospy.is_shutdown(): 
    		self.goal1.x=self.goal_pose(1,1)
    		self.goal1.y=self.goal_pose(1,2)
    		self.goal1.theta=self.goal_pose(1,3)

    		self.goal2.x=self.goal_pose(2,1)
    		self.goal2.y=self.goal_pose(2,2)
    		self.goal2.theta=self.goal_pose(2,3)

    		self.goal3.x=self.goal_pose(3,1)
    		self.goal3.y=self.goal_pose(3,2)
    		self.goal3.theta=self.goal_pose(3,3)

    		self.goal4.x=self.goal_pose(4,1)
    		self.goal4.y=self.goal_pose(4,2)
    		self.goal4.theta=self.goal_pose(4,3)

    		self.rate.sleep()  

if __name__ == '__main__':
    rospy.init_node('goal_publisher')
    rospy.loginfo("goal_publisher created | now goal for each bot will be published on goal topics")
    goal_pub_obj=goal_publisher()
    rospy.spin()
    # goal_pub_obj.goal()