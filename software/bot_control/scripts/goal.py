#!/usr/bin/env python

import rospy
from time import sleep
from std_msgs.msg import UInt8
from bot_control.msg import CompletePlan, pose_bot
import numpy as np

class goal_publisher:
    def __init__(self):
        # self.rate=rospy.Rate(0.5)
        self.n_agents=4
        self.goal_pose=np.zeros([self.n_agents,3])
        self.turning_points=self.empty_list(self.n_agents)
        self.need_new_plan=np.zeros(self.n_agents)
        self.yaw=np.zeros(self.n_agents)+100

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

    def reverse_transform(self,pose):
        return [84-6*pose[1]-3,pose[0]*6+3]
    def plan_callback(self,msg):
        # print('heard from CBS')
        for i in msg.agent:
            xi=[]
            yi=[]
            di=[]
            for j in i.statei:
                xi.append(int(j.x))
                yi.append(int(j.y))
                di.append(int(j.z))
            self.turning_point(xi,yi,di,i.bot_num)
        for i in msg.agent :
            self.yaw[i.bot_num]=100
            self.goal(i.bot_num)

    def turning_point(self,x,y,d,bot_num):
        turnpoints=[]
        for i in range(1,len(x)-1):
            #print("hi")
            #movement in y direction
            if (d[i]!=d[i+1]):
                turnpoints.append(self.reverse_transform([x[i],y[i]]))
            elif d[i]==d[i+1] and x[i]==x[i+1] and y[i]==y[i+1]:
                turnpoints.append([-100,-100])
        if(len(x)>1):
            turnpoints.append(self.reverse_transform([x[-1],y[-1]]))
        self.turning_points[bot_num]=turnpoints
        print(self.turning_points)
        # print(turnpoints)
        # print('-----------------------------------------------------------')


    def flag_pid_callback(self,msg):
        self.need_new_plan[msg.data]=1
        self.goal(msg.data)        

    def one_step_callback(self,msg):
        self.turning_points[msg.bot_num]=[[msg.x,msg.y]]
        self.yaw[msg.bot_num]=msg.yaw
        self.goal(msg.bot_num)
        # print('heard')



    def goal_pub(self,bot_num,yaw):
        msg=pose_bot()

        msg.x=self.goal_pose[bot_num][0]
        msg.y=self.goal_pose[bot_num][1]
        msg.yaw=yaw #100 is large impratical yaw just to indicate final yaw is not of significance
        msg.bot_num=bot_num
        self.pub_goal.publish(msg)

    def goal(self,bot_num):
        if(len(self.turning_points[bot_num])>0):
            self.goal_pose[bot_num][0]=self.turning_points[bot_num][0][0]
            self.goal_pose[bot_num][1]=self.turning_points[bot_num][0][1]
            self.turning_points[bot_num].pop(0)
            self.need_new_plan[bot_num]=0
            self.goal_pub(bot_num,self.yaw[bot_num])
            print('new waypoint sent for bot ',bot_num)
            
        else:
            pub_msgs=UInt8()
            pub_msgs.data=bot_num
            self.pub_new_plan.publish(pub_msgs)
            print('bot :',bot_num, ' needs new plan')


if __name__ == '__main__':
    rospy.init_node('goal_publisher')
    rospy.loginfo("goal_publisher created | now goal for each bot will be published on goal topics")
    goal_pub_obj=goal_publisher()
    rospy.spin()