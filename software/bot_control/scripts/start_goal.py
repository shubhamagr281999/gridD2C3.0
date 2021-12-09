#!/usr/bin/env python

import rospy
import time
from time import sleep
from std_msgs.msg import Int16
from bot_control.msg import StartGoal, CompletePlan, PathArray, Poses, dest_id
from geometry_msgs.msg import Point
import numpy as np


class start_goal_publisher:
    def __init__(self):
        self.rate=rospy.Rate(0.5)
        self.n_agents=1
        self.current_pose=np.zeros([self.n_agents,3])
        self.goal_pose=np.zeros([self.n_agents,3])
        self.poses_sub=rospy.Subscriber('/poses',Poses,self.pose_callback,queue_size=10)
        self.pub_goal=rospy.Publisher('/goal_point',Point,queue_size=10)
        self.dest_sub=rospy.Subscriber('/pkg_dest_id',dest_id,self.dest_callback,queue_size=1)
        self.dest_pub=rospy.Publisher('/start_goal_agents',StartGoal,queue_size=10)
        self.delivery_zone_occupancy = np.zeros([9,4]) -1
        self.startgoal= StartGoal()
        self.x0 = 0 #change according to camera for setting up the arena
        self.y0 = 0 #change according to camera for setting up the arena
        self.grid_locations = self.grid_location_assigner()  #3D array [dest_id][one of 4 block][0 for x | 1 for y]
        # print(self.grid_locations)

    #Mumbai's center is treated as origin and down the screen is +ve x-axis and left to right of screen is +ve y-axis
    def grid_location_assigner(self):
        k = []
        for i in range(9):
            a = []
            x=x0 + (i/3)*4
            y=y0 + (i%3)*4
            for j in range(4):
                if j < 2:
                    a.append([x + ((-1)**j)*0.5,y-1.5])
                else:
                    a.append([x-1.5,y + ((-1)**(j+1))*0.5])
            k.append(a)
        return k


    def dest_callback(self,msg,bot_num):
            for i in range(4):
                if self.delivery_zone_occupancy[self.msg.bot_num][i]==-1:
                    self.delivery_zone_occupancy[self.msg.bot_num][i] = bot_num
                    self.startgoal.bot_num.data = self.msg.bot_num
                    # self.startgoal.start_x.data = self.
                    # self.startgoal.start_y.data = self.
                    # self.startgoal.goal_x.data = self.
                    # self.startgoal.goal_x.data = self.
                    dest_pub.publish(self.startgoal)
                    break

        
    def pose_callback(self,msg):
        for i in range(self.n_agents):
            self.current_pose[i][0]=msg.posei[i].x
            self.current_pose[i][1]=msg.posei[i].y
            self.current_pose[i][2]=msg.posei[i].z
    def go_to_goal(self,bot_num,x,y,theta):
        msg=Point()
        msg.x=(self.x)
        msg.y=self.y
        msg.z=bot_num
        self.pub_goal.publish(msg)



if __name__ == '__main__':
    print("here ")
    rospy.init_node('Start_goal_publisher')
    rospy.loginfo("Start goal_publisher created | decides gaol based on the logic defined")
    start_goal_pub_obj=start_goal_publisher()
    # rospy.spin()
