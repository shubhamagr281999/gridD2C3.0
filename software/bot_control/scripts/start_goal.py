#!/usr/bin/env python

import rospy
import time
import pandas as pd
from time import sleep
from std_msgs.msg import Int16
from bot_control.msg import StartGoal, CompletePlan, PathArray, Poses, dest_id, dest_id, pkg_flag
from geometry_msgs.msg import Point
import numpy as np


class start_goal_publisher:
    def __init__(self):
        self.rate=rospy.Rate(10)
        self.n_agents=4
        self.current_pose=np.zeros([self.n_agents,3]) # 2D array [bot_num][0 for x | 1 for y | 2 for yaw]
       
        self.dicti = {0: 'Mumbai', 1: 'Delhi', 2: 'Kolkata', 3: 'Chennai', 4: 'Bengaluru', 5: 'Hyderabad', 6: 'Pune', 7: 'Ahmedabad', 8: 'Jaipur'}
        self.key_list = list(self.dicti.keys())
        self.val_list = list(self.dicti.values())
        self.excel = pd.read_excel(r'~/catkin_ws/src/gridD2C3.0/Sample Data.xls')
        self.destination= self.excel['Destination'].tolist()
        self.LS= self.excel['Induct Station'].tolist()
        self.list_split()
        self.pub_destination=rospy.Publisher('/pkg_dest_id',dest_id,queue_size=1)
        self.sub_station=rospy.Subscriber("/pkg_received", pkg_flag, self.assign)
        self.dest_id_msg = dest_id()

        self.delivery_zone_occupancy = np.zeros([9,4]) -1
        self.x0 = 0 #change according to camera for setting up the arena
        self.y0 = 0 #change according to camera for setting up the arena
        self.grid_locations = self.grid_location_assigner()  #3D array [dest_id][one of 4 block][0 for x | 1 for y]
        self.grid_locations_LS1= [self.x0+1.5, self.y0- 3.5]
        self.grid_locations_LS2= [self.x0 +6.5, self.y0 - 3.5]

        #subscribers
        self.current_state_sub=rospy.Subscriber('/poses', PoseArray,self.current_state_callback,queue_size=10)
        self.new_plan_sub=rospy.Subscriber('/new_plan',UInt8,self.new_plan_callback,queue_size=10)
        
        # publishers
        self.dest_pub=rospy.Publisher('/start_goal_agents',StartGoal,queue_size=10)
        self.startgoal= StartGoal()
        self.one_step_goal_pub=rospy.Publisher('/one_step_goal',PointStamped,queue_size=10)    



        self.lossfunction_para1= 1 % weighted loss function-LS selection 
        self.lossfunction_para2= 1

        self.queue_LS1_actual = np.zeros([6,1])
        for i in range(6):
            if (i < (self.n_agents%2)):
                self.queue_LS1_actual[i][0]= i
            else:
                self.queue_LS1_actual[i][0]= 100

        self.queue_LS2_actual = np.zeros([6,1])
        for i in range(6):
            if (i < (self.n_agents - (self.n_agents%2))):
                self.queue_LS2_actual[i][0]= i + self.n_agents
            else:
                self.queue_LS2_actual[i][0]= 100
        
        #queue_LS1_actual and queue_LS2_actual are updated using estimator NEED A FUNCTION FOR THIS IN ESTIMATOR BLOCK

        self.queue_LS1_pseudo_actual= self.queue_LS1_actual
        self.queue_LS2_pseudo_actual= self.queue_LS2_actual
        
        self.queue_LS1_assigned= np.array([(self.n_agents%2),1])

        for i in range(self.n_agents%2):
            self.queue_LS1_assigned[i][0]= i

        self.queue_LS2_assigned= np.array([(self.n_agents- (self.n_agents%2)),1])

        for i in range(self.n_agents-(self.n_agents%2)):
            self.queue_LS2_assigned[i][0]= i + (self.n_agents%2)

        self.LS_assigned = np.array([self.n_agents, 1])
        for i in range(self.n_agents):
            if (i< (self.nagents%2)):
                self.LS_assigned[i][0]= 1
            else:
                self.LS_assigned[i][0]= 2

        self.m_LS1= (self.n_agents % 2)
        self.m_LS2= (self.nagents - (self.n_agents%2))
        # print(self.grid_locations)

    def list_split(self):
        self.induct1 = []
        self.induct2 = []
        for i in range(len(self.LS)):
            if self.LS[i] == 1:
                self.induct1.append(self.destination[i])
            elif self.LS[i] ==2:
                self.induct2.append(self.destination[i])
        # print(self.induct1)

    def assign(self,msg):
        if msg.LS == 1:
            self.dest_id_msg.LS = 1
            self.dest_id_msg.bot_num = msg.bot_num
            self.dest_id_msg.dest_id = self.key_list[self.val_list.index(self.induct1[0])]
            self.induct1.pop(0)
            self.pub_destination.publish(self.dest_id_msg)
            # print(self.dest_id_msg.dest_id)
        elif msg.LS == 2:
            self.dest_id_msg.LS = 2
            self.dest_id_msg.bot_num = msg.bot_num
            self.dest_id_msg.dest_id = self.key_list[self.val_list.index(self.induct2[0])]
            self.induct2.pop(0)
            self.pub_destination.publish(self.dest_id_msg)

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

                    time.sleep(5) # TUNE LATER

                    if (LS_assigned[bot_num][0] == 1):
                        #PUT A CHECK HERE!! 
                        self.queue_LS1_assigned = np.delete(self.queue_LS1_assigned, [0])
                        self.queue_LS1_pseudo_actual= np.delete(self.queue_LS1_pseudo_actual, [0])
                        
                    elif (LS_assigned[bot_num][0] == 2):
                        #PUT A CHECK HERE!!
                        self.queue_LS2_assigned= np.delete(queue_LS2_assigned, [0])
                        self.queue_LS2_pseudo_actual= np.delete(self.queue_LS2_pseudo_actual, [0])
                        



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
                    #bot is assigned loading station and goes to the side flank of the queue
                    if (self.preffered_LS(bot_num) == 1):
                        return [self.x0 + 1.5 - self.queue_LS1_assigned.indexOf(bot_num, 0), self.y0 - 1.5]
                        #PUBLISH!!
                    elif (self.preffered_LS(bot_num) == 2):
                        return [self.x0 + 6.5 + self.queue_LS2_assigned.indexOf(bot_num,0), self.y0 -1.5] 
                        #PUBLISH!!
                    # MODIFY THIS PART LATER

                    self.task_msg.task[i] = 1    #task has been completed : reached side flank
                    self.update_task.publish(self.task_msg)

                    self.status_msg.status[i] = 4  #updated the status of the bot
                    self.update_status.publish(self.status_msg)

                elif (self.status_msg.status[i] == 4):
                    #the bot goes to the queue for assigned loading station
                    self.m_LS1=6
                    self.m_LS2=6
                    
                    while i<6:
                        if (queue_LS1_pseudo_actual[5-i][0] ==100]:
                            self.m_LS1 = 5-i
                        else:
                            break
                    
                    while j<6:
                        if (queue_LS2_pseudo_actual[5-j][0] ==100]:
                            self.m_LS2 = 5-j
                        else:
                            break

                    if (LS_assigned[bot_num][0] == 1):
                        self.queue_LS1_pseudo_actual[self.m_LS1][0] = bot_num
                        return [self.x0 - 6.5 + self.m_LS1,self.y0-2.5]
                        #PUBLISH

                    elif (LS_assigned[bot_num][0]==2):
                        self.queue_LS2_pseudo_actual[self.m_LS2][0] = bot_num
                        return [self.x0 +6.5  + self.m_LS2,self.y0-2.5]
                        #PUBLISH


                    self.task_msg.task[i] = 1    #task has been completed : reached the queue
                    self.update_task.publish(self.task_msg)

                    self.status_msg.status[i] = 5  #updated the status of the bot
                    self.update_status.publish(self.status_msg)


                elif (self.status_msg.status[i] == 5):
                    #the bot is in the queue at loading station

                    if (self.LS_assigned[bot_num][0]==1):

                        if (self.queue_LS1_actual.indexOf(bot_num, 0) == 0):
                            #Task done
                        elif (self.queue_LS1_actual[self.queue_LS1_actual.indexOf(bot_num, 0)-1][0]== 100):
                            
                            # move a step forward
                        else :
                            #stay and wait
                

                    elif(self.LS_assigned[bot_num][0]==2):


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

    def preffered_LS(self, bot_num)
        Loss_function_LS1 =lossfunction_para1 * sqrt((current_pose[bot_num][0] - grid_locations_LS1[0])^^2 + (current_pose[bot_num][1] - grid_locations_LS1[1])^^2) 
        Loss_function_LS2 =lossfunction_para1 * sqrt((current_pose[bot_num][0] - grid_locations_LS2[0])^^2 + (current_pose[bot_num][1] - grid_locations_LS2[1])^^2)
        if (Loss_function_LS1 > Loss_function_LS2)
            self.LS_assigned[bot_num][0]= 2
            numpy.append(self.queue_LS2_assigned, bot_num)
            return 2
        else
            self.LS_assigned[bot_num][0]= 1
            numpy.append(self.queue_LS1_assigned, bot_num)
            return 1


if __name__ == '__main__':
    print("here ")
    rospy.init_node('Start_goal_publisher')
    rospy.loginfo("Start goal_publisher created | decides gaol based on the logic defined")
    start_goal_pub_obj=start_goal_publisher()
    rospy.init_node('destination_assign')
    rospy.loginfo('Assigner node created')
    assigner = destination_assign()
    rospy.spin()