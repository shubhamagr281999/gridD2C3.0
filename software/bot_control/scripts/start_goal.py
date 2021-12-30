#!/usr/bin/env python

import rospy
from math import ceil,pi,sqrt
import pandas as pd
from time import sleep
from std_msgs.msg import UInt8
from bot_control.msg import StartGoal, CompletePlan, PathArray, pose_bot
from geometry_msgs.msg import PoseArray, PointStamped
import numpy as np

class destination_assign:
    def __init__(self):
        self.dicti = {0: 'Mumbai', 1: 'Delhi', 2: 'Kolkata', 3: 'Chennai', 4: 'Bengaluru', 5: 'Hyderabad', 6: 'Pune', 7: 'Ahmedabad', 8: 'Jaipur'}
        self.key_list = list(self.dicti.keys())
        self.val_list = list(self.dicti.values())
        self.excel = pd.read_excel(r'~/catkin_ws/src/gridD2C3.0/Sample Data.xls') #sheet of destinations for each parcel
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
        if LS == 0:
            dest_id = self.key_list[self.val_list.index(self.induct1[0])]
            self.induct1.pop(0)
            return dest_id
        elif LS == 1:
            dest_id = self.key_list[self.val_list.index(self.induct2[0])]
            self.induct2.pop(0)
            return dest_id


class start_goal_publisher:
    def __init__(self):

        self.rate=rospy.Rate(10)
        self.n_agents=4
        
        self.lossfunction_para1= 1  #weighted loss function-LS selection  para1 for distance
        self.lossfunction_para2= 1 # para2 for queue size
        self.dest_assigner = destination_assign() #assign function pof this class takes LS_num as param and return dest id

        # maze status/locations variables
        self.delivery_block_occupancy=np.zeros([9,4]) -1
        self.x0 = 0 #it is the orgin that is top let corner of image
        self.y0 = 0 #it is the orgin that is top let corner of image
        self.grid_locations = np.array(self.grid_location_assigner())  #3D array [dest_id][one of 4 block][0 for x | 1 for y]
        self.LS_queue_locations = np.array(self.LS_location_assinger()) #3D array [LS_num][one of 6 pose][0 for x | 1 for y]

        # bot staus variables
        self.current_pose=np.zeros([self.n_agents,3])
        self.initialize_current_pose()
        self.bot_status = np.zeros(self.n_agents)
        self.queue_LS_actual = np.zeros([2,6])-1
        self.queue_LS_pseudo_actual= []
        self.queue_LS_assigned = []
        self.LS_assigned = np.zeros(self.n_agents)
        self.initilize_LS_queue()
        self.assigned_dest_location=np.zeros([self.n_agents,3])            

        #subscribers
        self.current_state_sub=rospy.Subscriber('/poses', PoseArray,self.current_state_callback,queue_size=10)
        self.new_plan_sub=rospy.Subscriber('/new_plan',UInt8,self.new_plan_callback,queue_size=10)

        # publishers
        self.dest_pub=rospy.Publisher('/start_goal_agents',StartGoal,queue_size=10)
        self.one_step_goal_pub=rospy.Publisher('/one_step_goal',pose_bot,queue_size=10)
        self.one_step_msg=pose_bot()
        self.flipMotor_pub=rospy.Publisher('/flipmotor',UInt8,queue_size=10)  
        
        # print(self.grid_locations)

    def one_step_publish_(self,point,yaw,bot_num):
        self.one_step_msg.bot_num=bot_num
        self.one_step_msg.x=point[0]
        self.one_step_msg.y=point[1]
        self.one_step_msg.yaw=yaw
        # print('published')
        self.one_step_goal_pub.publish(self.one_step_msg)

    def current_state_callback(self,msg):
        bot_found=np.zeros([2,6])
        for i in range(self.n_agents):
            self.current_pose[i][0]=msg.poses[i].position.x
            self.current_pose[i][1]=msg.poses[i].position.y
            self.current_pose[i][2]=msg.poses[i].position.z

            # updating queue_LS_actual based on estimator
            if(self.current_pose[i][1]<12):
                if(self.current_pose[i][1]<=7.5):
                    if(self.current_pose[i][0]<28.5 and self.current_pose[i][0]>25.5):
                        self.queue_LS_actual[0][0]=i
                        bot_found[0][0]=1
                    elif (self.current_pose[i][0]<58.5 and self.current_pose[i][0]>55.5):
                        self.queue_LS_actual[1][0]=i
                        bot_found[1][0]=1
                if(self.current_pose[i][1]>7.5):
                    if self.current_pose[i][0]<30:
                        self.queue_LS_actual[0][((30-self.current_pose[i][0])//6)+1]=i
                        bot_found[0][((30-self.current_pose[i][0])//6)+1]=1
                    elif self.current_pose[i][0]>54:
                        self.queue_LS_actual[0][((self.current_pose[i][0]-54)//6)+1]=i
                        bot_found[0][((self.current_pose[i][0]-54)//6)+1]=1

                    # for j in range(5):
                    #     if(self.current_pose[i][0]<(6*j+4.5) and self.current_pose[i][0]>(6*j+1.5)):
                    #         self.queue_LS_actual[0][5-j]=i
                    #         bot_found[0][5-j]=1
                    #         break
                    #     if(self.current_pose[i][0]<(6*(j+9)+4.5) and self.current_pose[i][0]>(6*(j+9)+1.5)):
                    #         self.queue_LS_actual[1][j+1]=i
                    #         bot_found[1][j+1]=1
                    #         break
        not_bot=np.where(bot_found==0)
        for i in range(np.shape(not_bot)[1]):
            self.queue_LS_actual[not_bot[0][i]][not_bot[1][i]]=-1
        # print(self.queue_LS_actual)

    def initilize_LS_queue(self):
        for i in range(self.n_agents):
            if(i<int(ceil(self.n_agents/2.0))):
                if(i==0):
                    self.queue_LS_actual[0][i]=int(i)
                    self.bot_status[i]=2
                else :
                    self.queue_LS_actual[0][i+1]=int(i)
                self.LS_assigned[i]=0
            else:
                k=i-int(ceil(self.n_agents/2.0))
                if(k==0):
                    self.bot_status[i]=2
                    self.queue_LS_actual[1][0]=int(i)
                else:
                    self.queue_LS_actual[1][k+1]=int(i)
                self.LS_assigned[i]=1

        self.queue_LS_pseudo_actual.append((self.queue_LS_actual[0][2:int(ceil(self.n_agents/2.0))+1]).tolist())
        self.queue_LS_assigned.append((self.queue_LS_actual[0][2:int(ceil(self.n_agents/2.0))+1]).tolist())

        self.queue_LS_pseudo_actual.append((self.queue_LS_actual[1][2:self.n_agents - int(ceil(self.n_agents/2.0))+1]).tolist())
        self.queue_LS_assigned.append((self.queue_LS_actual[1][2:self.n_agents - int(ceil(self.n_agents/2.0))+1]).tolist())
        # print(self.queue_LS_actual)
        # print(self.queue_LS_pseudo_actual)
        # print(self.queue_LS_assigned)
        # print(self.LS_assigned)

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
        x0_=self.x0+18
        y0_=self.y0+24
        for i in range(9):
            a = []
            x=x0_ + (i/3)*24
            y=y0_ + (i%3)*24
            a.append([x - 9,y-3])
            a.append([x-3,y + 9])
            a.append([x-9,y + 3])
            a.append([x+3,y + 9])
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
   
    def transform(self,pose):
        x = pose[0]
        y = pose[1]
        # print("x:", x , "y : ", y)
        x_t = y//6
        y_t = 13 - (x//6)
        # print("x_t:", x_t , "y_t : ", y_t)
        return [x_t, y_t, 1]

    def preffered_LS(self, bot_num):
        Loss_function_LS1 =self.lossfunction_para1 * sqrt((self.current_pose[bot_num][0] - self.LS_queue_locations[0][4][0])**2 + (self.current_pose[bot_num][1] - self.LS_queue_locations[0][4][1])**2) 
        Loss_function_LS2 =self.lossfunction_para1 * sqrt((self.current_pose[bot_num][0] - self.LS_queue_locations[1][4][0])**2 + (self.current_pose[bot_num][1] - self.LS_queue_locations[1][4][1])**2)
        if(len(self.queue_LS_assigned)>3):
            print('possuble error check')
        if (Loss_function_LS1 > Loss_function_LS2):
            self.LS_assigned[bot_num]= 1
            self.queue_LS_assigned[1].append(bot_num) #change logic here cant append rather have to check for kast empty element
            return 1
        else :
            self.LS_assigned[bot_num]= 0
            self.queue_LS_assigned[0].append(bot_num)
            return 0

    def preffered_dest_block(self,dest_id):
        k=np.where(self.delivery_block_occupancy[dest_id]==-1)[0]
        if(len(k)>0):
            return k[0]
        else :
            print('All delivery zones assigned need to wait')
            return -1

    def distance(self,bot_num):
        x=self.current_pose[bot_num][0]-self.assigned_dest_location[bot_num][0]
        y=self.current_pose[bot_num][1]-self.assigned_dest_location[bot_num][1]
        return sqrt( x**2 + y**2 )

    def CBS_plan(self):
        msg=StartGoal()
        k=np.where(self.bot_status==3)[0]
        p=np.where(self.bot_status==6)[0]
        k=np.concatenate([k,p]).tolist()
        for i in k:
            if(self.distance(i)>9):                
                msg.bot_num.append(i)
                # print('here')
                start_pose=self.transform(self.current_pose[i])
                msg.start_x.append(start_pose[0])
                msg.start_y.append(start_pose[1])
                msg.start_d.append(start_pose[2])
                goal_pose=self.transform(self.assigned_dest_location[i])
                msg.goal_x.append(goal_pose[0])
                msg.goal_y.append(goal_pose[1])
                msg.goal_d.append(goal_pose[2])

        m=np.where(self.bot_status==4)[0]
        n=np.where(self.bot_status==5)[0]
        k=np.concatenate([m,n]).tolist()
        for i in k:                
            msg.bot_num.append(i)
            # print('here')
            start_pose=self.transform(self.current_pose[i])
            msg.start_x.append(start_pose[0])
            msg.start_y.append(start_pose[1])
            msg.start_d.append(start_pose[2])
            goal_pose=self.transform(self.current_pose[i])
            msg.goal_x.append(goal_pose[0])
            msg.goal_y.append(goal_pose[1])
            msg.goal_d.append(goal_pose[2])

        self.dest_pub.publish(msg)

    def new_plan_callback(self,msg):
        print('bot ',msg.data, ' needs new status')
        if(self.bot_status[msg.data]==6): #status was going to LS assigned it reached means now LS_assigned to LS_psuedo needs to be given
            LS_=int(self.LS_assigned[msg.data])
            self.one_step_publish_(self.LS_queue_locations[LS_][len(self.queue_LS_pseudo_actual[LS_])+2],100,msg.data)
            self.queue_LS_pseudo_actual[LS_].append(msg.data)
            k=np.where(np.array(self.queue_LS_assigned[LS_])==msg.data)[0][0]
            self.queue_LS_assigned[LS_].pop(k)
            self.queue_LS_assigned[LS_].insert(len(self.queue_LS_pseudo_actual[LS_])-1,msg.data)
            self.bot_status[msg.data]=0

        elif(self.bot_status[msg.data]==0 or self.bot_status[msg.data]==1): #status was 0 it means it has reached in the LS_queue
            self.bot_status[msg.data]=1
            # print('here')
            LS_=int(self.LS_assigned[msg.data])    
            k=np.where(self.queue_LS_actual[LS_]==msg.data)[0][0]
            # print(k)
            print(self.queue_LS_actual)
            if(k==2):
                if(self.queue_LS_actual[LS_][0]==-1 and self.queue_LS_actual[LS_][1]==-1):
                    self.one_step_publish_(self.LS_queue_locations[LS_][1],100,msg.data)
                    self.queue_LS_pseudo_actual[LS_].pop(0)
                    self.queue_LS_assigned[LS_].pop(0)
                else :
                    self.one_step_publish_([-100,-100],100,msg.data) #halt
            elif(k==1):
                self.one_step_publish_(self.LS_queue_locations[LS_][0],100,msg.data)
            elif(k==0):
                self.one_step_publish_([-100,-100],100,msg.data)
                self.bot_status[msg.data]=2
            else:
                if(self.queue_LS_actual[LS_][k-1]==-1):
                    self.one_step_publish_(self.LS_queue_locations[LS_][k-1],100,msg.data)
                else:
                    self.one_step_publish_([-100,-100],100,msg.data)

        elif(self.bot_status[msg.data]==2): #it was halting at LS for parcel and now it needs to go to destination
            LS_=int(self.LS_assigned[msg.data])
            self.LS_assigned[msg.data]=-1
            dest_id=self.dest_assigner.assign(LS_)
            dest_block=self.preffered_dest_block(dest_id) #if -1 if retuned | code for it -------------------------------------------------------------
            self.delivery_block_occupancy[dest_id][dest_block]=msg.data
            self.bot_status[msg.data]=3
            flip_direction=0
            if(dest_block==0 or dest_block==2):
                flip_direction=pi/2
            self.assigned_dest_location[msg.data]=[self.grid_locations[dest_id][dest_block][0],self.grid_locations[dest_id][dest_block][1],flip_direction]
            self.CBS_plan()

        elif(self.bot_status[msg.data]==3): # it was going from LS-dest, It would have reached there. Need to align for parcel drop
            self.bot_status[msg.data]=4
            self.one_step_publish_(self.assigned_dest_location[msg.data],self.assigned_dest_location[msg.data][2],msg.data)

        elif(self.bot_status[msg.data]==4): #bot has aligend itself now need to drop parcel
            self.bot_status[msg.data]=5
            self.one_step_publish_([-100,-100],100,msg.data)
            msg_flip=UInt8()
            msg_flip.data=msg.data
            self.flipMotor_pub.publish(msg_flip)
            
        elif(self.bot_status[msg.data]==5): #parcel has been dropped need to go back to one of the LS
            self.bot_status[msg.data]=6
            k=np.where(self.delivery_block_occupancy==msg.data)
            self.delivery_block_occupancy[k[0][0]][k[1][0]]=-1
            LS_=int(self.preffered_LS(msg.data)) 
            self.assigned_dest_location[msg.data]=[self.LS_queue_locations[LS_][len(self.queue_LS_assigned[LS_])-1][0],self.LS_queue_locations[LS_][len(self.queue_LS_assigned[LS_])-1][1],100]
            self.assigned_dest_location[msg.data][1]=self.assigned_dest_location[msg.data][1]+6
            self.queue_LS_assigned[LS_].append(msg.data)
            self.LS_assigned[msg.data]=LS_
            print('return path assigned for :' , msg.data)
            self.CBS_plan()

if __name__ == '__main__':
    rospy.init_node('Start_goal_publisher')
    rospy.loginfo("Start goal_publisher created | decides gaol based on the logic defined")
    start_goal_pub_obj=start_goal_publisher()
    rospy.spin()