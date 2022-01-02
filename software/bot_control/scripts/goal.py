#!/usr/bin/env python

import rospy
from time import sleep
from std_msgs.msg import UInt8
from bot_control.msg import CompletePlan, pose_bot, Bot_status
from geometry_msgs.msg import PoseArray
import numpy as np
from math import ceil,pi

class goal_publisher:
    def __init__(self):
        # self.rate=rospy.Rate(0.5)
        self.n_agents=6
        self.goal_pose=np.zeros([self.n_agents,3])
        self.turning_points=self.empty_list(self.n_agents)
        self.need_new_plan=np.zeros(self.n_agents)
        self.yaw=np.zeros(self.n_agents)+100
        self.bot_status=np.zeros(self.n_agents)
        self.current_pose=np.zeros([self.n_agents,3])
        self.initialize_current_pose()

        # publishers
        self.pub_goal=rospy.Publisher('/goal_point',pose_bot,queue_size=10)
        self.pub_new_plan = rospy.Publisher('/new_plan',UInt8,queue_size=10)

        # subscribers
        self.plans_callback=rospy.Subscriber("/cbs/plan",CompletePlan,self.plan_callback,queue_size=10)
        self.sub_flag_pid = rospy.Subscriber("/flag_pid",UInt8,self.flag_pid_callback,queue_size=10)
        self.one_step_goal = rospy.Subscriber("/one_step_goal",pose_bot,self.one_step_callback,queue_size=10)
        self.botStatus_sub = rospy.Subscriber('/bot_status',Bot_status,self.bot_status_callback,queue_size=10)
        self.current_state_sub=rospy.Subscriber('/poses', PoseArray,self.current_state_callback,queue_size=10)

    def current_state_callback(self,msg):
        bot_found=np.zeros([2,6])
        for i in range(self.n_agents):
            self.current_pose[i][0]=msg.poses[i].position.x
            self.current_pose[i][1]=msg.poses[i].position.y
            self.current_pose[i][2]=msg.poses[i].position.z

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

    def bot_status_callback(self,msg):
        for i in range(self.n_agents):
            self.bot_status[i]=msg.status[i]

    def empty_list(self,i):
        k=[]
        for j in range(i):
            k.append([])
        return k

    def reverse_transform(self,pose):
        return [84-6*pose[1]-3,pose[0]*6+3]

    def transform(self,pose):
        x = pose[0]
        y = pose[1]
        # print("x:", x , "y : ", y)
        x_t = y//6
        y_t = 13 - (x//6)
        # print("x_t:", x_t , "y_t : ", y_t)
        return [x_t, y_t, 1]

    def location_match(self,x,y,bot_num):
        current_pose_transformed=self.transform(self.current_pose[bot_num])
        return (current_pose_transformed[0]==x and current_pose_transformed[1]==y)

    def plan_callback(self,msg):
        print('heard from CBS')
        for i in msg.agent:
            if(self.bot_status[int(i.bot_num)]==1 and self.bot_status[int(i.bot_num)]==2):
                continue
            xi=[]
            yi=[]
            di=[]
            for j in i.statei:
                xi.append(int(j.x))
                yi.append(int(j.y))
                di.append(int(j.z))
            if(self.location_match(xi[-1],yi[-1],i.bot_num)):
                continue
            self.turning_point(xi,yi,di,i.bot_num)

    def turning_point(self,x,y,d,bot_num):
        turnpoints=[]
        # turnpoints.append(self.reverse_transform([x[0],y[0]]))
        for i in range(1,len(x)-1):

        #     #print("hi")
        #     #movement in y direction
        #     if (d[i]!=d[i+1]):
        #         turnpoints.append(self.reverse_transform([x[i],y[i]]))
        #     elif d[i]==d[i+1] and x[i]==x[i+1] and y[i]==y[i+1]:
        #         turnpoints.append([-100,-100])

         #movement in y direction
            if x[i] == x[i+1] and y[i] != y[i+1]:
                if x[i] != x[i-1] and y[i] == y[i-1]:
                    turnpoints.append(self.reverse_transform([x[i],y[i]])) 
            #movement in x direction
            elif x[i] != x[i+1] and y[i] == y[i+1]:
                if x[i] == x[i-1] and y[i] != y[i-1]:
                    turnpoints.append(self.reverse_transform([x[i],y[i]])) 
            #Halt
            else:
                turnpoints.append([-100,2])
        if(len(x)>1):
            turnpoints.append(self.reverse_transform([x[-1],y[-1]]))
            self.turning_points[bot_num]=turnpoints
            self.goal(bot_num)
        # print(self.turning_points)
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
            if(bot_num==3):
                print(self.turning_points[3]) 
            self.goal_pose[bot_num][0]=self.turning_points[bot_num][0][0]
            self.goal_pose[bot_num][1]=self.turning_points[bot_num][0][1]
            self.turning_points[bot_num].pop(0)
            self.need_new_plan[bot_num]=0

            self.goal_pub(bot_num,self.yaw[bot_num])
            # print('new waypoint sent for bot ',bot_num)
            
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