#!/usr/bin/env python

import rospy
import time
import pandas as pd
from time import sleep
from std_msgs.msg import UInt8
from bot_control.msg import StartGoal, CompletePlan, PathArray, Poses
from geometry_msgs.msg import Point
import numpy as np

class destination_assign:
    def __init__(self):
        self.dicti = {0: 'Mumbai', 1: 'Delhi', 2: 'Kolkata', 3: 'Chennai', 4: 'Bengaluru', 5: 'Hyderabad', 6: 'Pune', 7: 'Ahmedabad', 8: 'Jaipur'}
        self.key_list = list(self.dicti.keys())
        self.val_list = list(self.dicti.values())
        self.excel = pd.read_excel(r'~/catkin_ws/src/gridD2C3.0/Sample Data.xls')
        self.destination= self.excel['Destination'].tolist()

        self.LS= self.excel['Induct Station'].tolist()
        self.list_split()

    def list_split(self):
        self.induct1 = []
        self.induct2 = []
        for i in range(len(self.LS)):
            if self.LS[i] == 1:
                self.induct1.append(self.destination[i])
            elif self.LS[i] ==2:
                self.induct2.append(self.destination[i])
        # print(self.induct1)

    def assign(self,LS):
        if LS == 1:
            dest_id = self.key_list[self.val_list.index(self.induct1[0])]
            self.induct1.pop(0)
            return dest_id
        elif LS == 2:
            dest_id = self.key_list[self.val_list.index(self.induct2[0])]
            self.induct2.pop(0)
            return dest_id


class start_goal_publisher:
    def __init__(self):

        self.rate=rospy.Rate(10)
        self.n_agents=4
        
        self.lossfunction_para1= 1  #weighted loss function-LS selection 
        self.lossfunction_para2= 1
        self.dest_assigner = destination_assign()

        #subscribers
        self.current_state_sub=rospy.Subscriber('/poses', PoseArray,self.current_state_callback,queue_size=10)
        self.new_plan_sub=rospy.Subscriber('/new_plan',UInt8,self.new_plan_callback,queue_size=10)

        # publishers
        self.dest_pub=rospy.Publisher('/start_goal_agents',StartGoal,queue_size=10)
        self.startgoal= StartGoal()
        self.one_step_goal_pub=rospy.Publisher('/one_step_goal',PointStamped,queue_size=10)    

        # maze status/locations variables
        self.delivery_zone_occupancy = np.zeros([9,4]) -1
        self.x0 = 0 #it is the orgin that is top let corner of image
        self.y0 = 0 #it is the orgin that is top let corner of image
        self.grid_locations = self.grid_location_assigner()  #3D array [dest_id][one of 4 block][0 for x | 1 for y]
        self.LS_queue_locations = self.LS_location_assinger() #3D array [LS_num][one of 6 pose][0 for x | 1 for y]

        # bot staus variables
        self.current_pose=np.zeros([self.n_agents,3])
        self.initialize_current_pose()
        self.bot_status = np.zeros(self.n_agents)
        self.queue_LS_actual = np.zeros([2,6])
        self.queue_LS_pseudo_actual= np.zeros([2,6])
        self.queue_LS_assigned = np.zeros([2,6])
        self.LS_assigned = np.array([self.n_agents])
        self.initilize_LS_queue()
        
        


        self.m_LS1= (self.n_agents % 2)
        self.m_LS2= (self.nagents - (self.n_agents%2)) #check this
        # print(self.grid_locations)

    def current_state_callback(self,msg):
        for i in range(self.n_agents):
            self.current_pose[i][0]=msg.poses[i].position.x
            self.current_pose[i][1]=msg.poses[i].position.y
            self.current_pose[i][2]=msg.poses[i].position.z

            # updating queue_LS_actual based on estimator
            if(self.current_pose[i][1]<10.5):
                if(self.current_pose[i][1]<4.5):
                    if(self.current_pose[i][0]<28.5 and self.current_pose[i][0]>26.5):
                        self.queue_LS_actual[0][0]=i
                    elif (self.current_pose[i][0]<58.5 and self.current_pose[i][0]>56.5):
                        self.queue_LS_actual[1][0]=i
                if(self.current_pose[i][1]>7.5):
                    for j in range(5):
                        if(self.current_pose[j][0]<(6*i+4.5) and self.current_pose[j][0]>(6*i+1.5)):
                            self.queue_LS_actual[0][5-j]=i
                            break
                        if(self.current_pose[j][0]<(6*(i+9)+4.5) and self.current_pose[j][0]>(6*(i+9)+1.5)):
                            self.queue_LS_actual[1][j+1]=i
                            break

    def initilize_LS_queue(self):
        for i in range(self.n_agents):
            if(i<int(ceil(self.n_agents/2.0))):
                if(i==0):
                    self.queue_LS_actual[0][i]=i
                    self.queue_LS_pseudo_actual[0][i]=i
                    self.queue_LS_assigned[0][i]=i
                else :
                    self.queue_LS_actual[0][i+1]=i
                    self.queue_LS_pseudo_actual[0][i+1]=i
                    self.queue_LS_assigned[0][i+1]=i
                self.LS_assigned[i]=0
            else:
                k=i-int(ceil(self.n_agents/2.0))
                if(k==0):
                    self.queue_LS_actual[1][0]=i
                    self.queue_LS_pseudo_actual[1][0]=i
                    self.queue_LS_assigned[1][0]=i
                else:
                    self.queue_LS_actual[1][k+1]=i
                    self.queue_LS_pseudo_actual[1][k+1]=i
                    self.queue_LS_assigned[1][k+1]=i
                self.LS_assigned[i]=1

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

    def grid_location_assigner(self):
        k = []
        x0_=self.x0+15
        y0_=self.y0+21
        for i in range(9):
            a = []
            x=x0_ + (i/3)*24
            y=y0_ + (i%3)*24
            for j in range(4):
                if j < 2:
                    a.append([x + ((-1)**j)*3,y-9])
                else:
                    a.append([x-9,y + ((-1)**(j+1))*3])
            k.append(a)
        return k

    def LS_location_assinger(self):
        k=[]
        a=[]
        a.append([self.x0+27,self.y0+3])
        for i in range(5):
            a.append([self.x0 + (4-i)*6 + 3,self.y0 + 9])
        k.append(a)
        a=[]
        a.append([self.x0+57,self.y0+3])
        for i in range(5):
            a.append([self.x0 + (9+i)*6 + 3,self.y0 + 9])
        k.append(a)
        return k    

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
        Loss_function_LS1 =self.lossfunction_para1 * sqrt((self.current_pose[bot_num][0] - self.grid_locations_LS1[0])^^2 + (self.current_pose[bot_num][1] - self.grid_locations_LS1[1])^^2) 
        Loss_function_LS2 =self.lossfunction_para1 * sqrt((self.current_pose[bot_num][0] - self.grid_locations_LS2[0])^^2 + (self.current_pose[bot_num][1] - self.grid_locations_LS2[1])^^2)
        if (Loss_function_LS1 > Loss_function_LS2)
            self.LS_assigned[bot_num][0]= 2
            self.queue_LS_assigned[0]=np.append(self.queue_LS2_assigned, bot_num) #change logic here cant append rather have to check for kast empty element
            return 2
        else
            self.LS_assigned[bot_num][0]= 1
            self.queue_LS_assigned[1]=np.append(self.queue_LS1_assigned, bot_num)
            return 1


if __name__ == '__main__':
    print("here ")
    rospy.init_node('Start_goal_publisher')
    rospy.loginfo("Start goal_publisher created | decides gaol based on the logic defined")
    start_goal_pub_obj=start_goal_publisher()
    rospy.init_node('destination_assign')
    rospy.loginfo('Assigner node created')
    rospy.spin()