#!/usr/bin/env python

import rospy
from time import sleep
from std_msgs.msg import UInt8
from bot_control.msg import CompletePlan, pose_bot
import numpy as np

class goal_publisher:
    def __init__(self):
        # self.rate=rospy.Rate(0.5)
        self.n_agents=1
        self.goal_pose=np.zeros([self.n_agents,3])
        self.turning_points=self.empty_list(self.n_agents)
        self.need_new_plan=np.zeros(self.n_agents)

        # publishers
        self.pub_goal=rospy.Publisher('/goal_point',pose_bot,queue_size=10)
        self.pub_new_plan = rospy.Publisher('/new_plan',UInt8,queue_size=10)

        # subscribers
        self.plans_callback=rospy.Subscriber("/cbs/plan",CompletePlan,self.plan_callback,queue_size=10)
        self.sub_flag_pid = rospy.Subscriber("/flag_pid",UInt8,self.flag_pid_callback,queue_size=10)
        self.one_step_goal = rospy.Subscriber("/one_step_goal",pose_bot,self.one_step_callback,queue_size=10)


    def empty_list(self,i):
        k=[]
        for j in range(i):
            k.append([])
        return k

    def plan_callback(self,msg):
        for i in msg.agent:
            xi=[]
            yi=[]
            di=[]
            for j in i.statei:
                xi.append(j.x)
                yi.append(j.y)
                di.append(j.z)
            self.turning_point(xi,yi,di,i.bot_num)

        for i in msg.agent :
            self.goal(i.bot_num,100)

    def turning_point(self,x,y,d,bot_num):
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
            elif d[i]==di[i+1]:
                turnpoints.append([-100,-100])
                
        self.turning_points[bot_num]=turnpoints
        # print(self.turning_points)
        # print('-----------------------------------------------------------')


    def flag_pid_callback(self,msg):
        self.need_new_plan[msg.data]=1
        self.goal(msg.data)        

    def one_step_callback(self,msg):
        self.turning_points[msg.header.seq]=[[msg.x,msg.y]]
        self.goal(msg.bot_num,msg.yaw)


    def goal_pub(self,bot_num,yaw):
        msg=Point()

        msg.x=self.goal_pose[bot_num][0]
        msg.y=self.goal_pose[bot_num][1]
        msg.z=yaw #100 is large impratical yaw just to indicate final yaw is not of significance
        msg.bot_num=bot_num
        self.pub_goal.publish(msg)

    def goal(self,bot_num,yaw):
        if(len(self.turning_points[bot_num])>0):
            self.goal_pose[bot_num][0]=self.turning_points[i][0][0]
            self.goal_pose[bot_num][1]=self.turning_points[i][0][1]
            self.turning_points[bot_num].pop(0)
            self.need_new_plan[bot_num]=0
            self.goal_pub(bot_num,yaw)
            
        else:
            pub_msgs=UInt8()
            pub_msgs.data=bot_num
            self.pub_new_plan.publish(pub_msgs)


if __name__ == '__main__':
    rospy.init_node('goal_publisher')
    rospy.loginfo("goal_publisher created | now goal for each bot will be published on goal topics")
    goal_pub_obj=goal_publisher()
    rospy.spin()