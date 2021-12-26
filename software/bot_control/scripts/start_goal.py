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
        self.current_pose=np.zeros([self.n_agents,3]) # 2D array [bot_num][0 for x | 1 for y | 2 for yaw]
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
        self.status_msg = None
        self.test_msg = None

        #subscribers
        self.bot_status= rospy.Subscriber('/bot_status',Bot_status,self.update_status_callback, queue_size=10)
        self.task_flag=rospy.Subscriber('/task_flag',Bot_task,self.next_task_callback, queue_size=10)
        
        # publishers
        self.update_status = rospy.Publisher('/bot_status',Bot_status,queue_size=10)
        self.update_task = rospy.Publisher('/task_flag',Bot_task, queue_size=10)
        
        # print(self.grid_locations)
    def update_status_callback(self,msg):
        self.status_msg = msg
        
    #sequnce of operatrion
    def next_task_callback(self,msg):
        self.task_msg = msg
        for i in range(self.n_agents):
            if (self.task_msg.task[i] == 0): # 0 means task is assigned, 1 means task is done

                if (self.status_msg.status[i] == 0):
                    #the bot is at the top of queue in LS waiting for parcel and destination
                    self.task_msg.task[i] = 1    #task has been completed : we gave it the destination
                    self.update_task.publish(self.task_msg)

                    self.status_msg.status[i] = 1  #updated the status of the bot
                    self.update_status.publish(self.status_msg)


                elif (self.status_msg.status[i] == 1):
                    #the bot is on the way to drop the parcel
                    self.task_msg.task[i] = 1    #task has been completed: PID control to the destination
                    self.update_task.publish(self.task_msg)

                    self.status_msg.status[i] = 2  #updated the status of the bot
                    self.update_status.publish(self.status_msg)

                elif (self.status_msg.status[i] == 2):
                    #the bot is aligning itself to drop the parcel
                    self.task_msg.task[i] = 1    #task has been completed: Alignment & Flipper
                    self.update_task.publish(self.task_msg)

                    self.status_msg.status[i] = 3  #updated the status of the bot
                    self.update_status.publish(self.status_msg)

                elif (self.status_msg.status[i] == 3):
                    #the bot is waiting for decision on which LS it has to go to
                    self.task_msg.task[i] = 1    #task has been completed : Alloting LS
                    self.update_task.publish(self.task_msg)

                    self.status_msg.status[i] = 4  #updated the status of the bot
                    self.update_status.publish(self.status_msg)

                elif (self.status_msg.status[i] == 4):
                    #the bot is on the way to the loading station
                    self.task_msg.task[i] = 1    #task has been completed : PID Control back to LS
                    self.update_task.publish(self.task_msg)

                    self.status_msg.status[i] = 5  #updated the status of the bot
                    self.update_status.publish(self.status_msg)

                elif (self.status_msg.status[i] == 5):
                    #the bot is in the queue at loading station
                    self.task_msg.task[i] = 1    #task has been completed :Reached the top of queue
                    self.update_task.publish(self.task_msg)

                    self.status_msg.status[i] = 0  #updated the status of the bot
                    self.update_status.publish(self.status_msg)

    
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


    def dest_callback(self,msg):
            for i in range(4):
                if self.delivery_zone_occupancy[msg.bot_num.data][i]==-1:
                    self.delivery_zone_occupancy[msg.bot_num.data][i] = msg.bot_num.data
                    self.startgoal.bot_num.data = [msg.bot_num.data]
                    self.startgoal.start_x.data = [self.current_pose[msg.bot_num.data][0]]
                    self.startgoal.start_y.data = [self.current_pose[msg.bot_num.data][1]]
                    # add direction here
                    self.startgoal.goal_x.data = [self.grid_locations[msg.dest_id.data][i][0]]
                    self.startgoal.goal_y.data = [self.grid_locations[msg.dest_id.data][i][1]]
                    self.startgoal.goal_d.data = [0]
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
