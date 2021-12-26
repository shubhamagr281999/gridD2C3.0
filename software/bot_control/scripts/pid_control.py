#! /usr/bin/env python2.7

import os
import rospy
from time import sleep
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, PoseArray, Point,PointStamped
from std_msgs.msg import Bool, UInt8
from math import sqrt, pi, atan, ceil, floor,sin,cos
# from bot_control.msg import Poses
import numpy as np

class PID:
    def __init__(self):
        self.n_agents=4
        # defining tunable params
        self.kp_lin = 0.1/100
        self.ki_lin = 0.0
        self.kd_lin = 0.0

        self.kp_angle = -35.0/100.0
        self.ki_angle = 0.0
        self.kd_angle = -11.0/100.0

        self.kp_angle_soft = -20.0/80.0
        self.ki_angle_soft = 0.0/10.0
        self.kd_angle_soft = -10.0/80.0

        self.max_vel_lin=30.0/60.0
        self.max_vel_ang=1.1
        self.lin_threshold=5
        self.dist_large_error=20
        self.yaw_threshold=0.1
        self.yaw_large_error=0.7
        self.intergral_windup_yaw=20
        self.intergral_windup_lin=15
        self.control_rate=rospy.Rate(10)

       
        self.lastError_dist = np.zeros(self.n_agents)
        self.lastError_angle = np.zeros(self.n_agents)
        self.lastError_small=np.zeros(self.n_agents)
        self.sumError_dist = np.zeros(self.n_agents)
        self.sumError_angle = np.zeros(self.n_agents)

        #other valribales from here        
        
        self.current_pose=np.zeros([self.n_agents,3]) #[bot_num][0 for x | 1 for y | 2 for yaw]
        self.goal_pose=np.zeros([self.n_agents,3])
        self.v_x_output=np.zeros(self.n_agents)   #vx and vy are in global coordinate system
        self.v_y_output=np.zeros(self.n_agents)
        self.w_output=np.zeros(self.n_agents)
        self.initialize_current_pose()
        self.halt_count=np.zeros(self.n_agents)
        self.need_new_plan = np.zeros(self.n_agents) +1 #in PID sense of a new plan is a next waypoint
        # bot specs (see daig in documentation)
        self.l1=0.07
        self.l2=0.07
        self.l3=0.07

        # publishers
        self.control_input_pub = rospy.Publisher('/cmd_vel', PoseArray, queue_size=10)
        self.cmd_vel_msg=PoseArray() # here we use only poistion of Poses msg. x will have vx, y will be vy and z will be omega in position object 
        self.wheel_speed_pub=rospy.Publisher('/wheel_speed',PoseArray, queue_size=10)
        self.wheel_vel_msg=PoseArray() # here we use only poistion of Poses msg. x will have w1, y will be w1 and z will be w3 in position object
        self.initialize_cmd_vel_msg()        
        self.flag_pid_pub = rospy.Publisher('/flag_id',UInt8,queue_size=10)

        # subscribers
        self.current_state_sub=rospy.Subscriber('/poses', PoseArray,self.current_state_callback,queue_size=10)
        self.goal_pose_sub=rospy.Subscriber('/goal_point', PointStamped,self.goal_pose_callback,queue_size=10)

        sleep(2)

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

        for i in range(self.n_agents):
            if(i<int(ceil(self.n_agents/2.0))):
                self.goal_pose[i][0]=(4-i)*6+3
                
                if(i==0):
                    self.goal_pose[i][1]=3
                    self.goal_pose[i][2]=pi/2
                else :
                    self.goal_pose[i][1]=9
                    self.goal_pose[i][2]=0
            else :
                self.goal_pose[i][0]=(9 + i - ceil(self.n_agents/2.0))*6 + 3
                
                if(i==ceil(self.n_agents/2.0)):
                    self.goal_pose[i][1]=3
                    self.goal_pose[i][2]=pi/2
                else :
                    self.goal_pose[i][1]=9
                    self.goal_pose[i][2]=pi

    def initialize_cmd_vel_msg(self):
        temp_poses=[]
        temp_poses2=[]
        for i in range(self.n_agents):
            temp_poses.append(Pose())
            temp_poses2.append(Pose())
        self.cmd_vel_msg.poses=temp_poses
        self.wheel_vel_msg.poses=temp_poses2

    def current_state_callback(self,msg):
        for i in range(self.n_agents):
            self.current_pose[i][0]=msg.poses[i].position.x
            self.current_pose[i][1]=msg.poses[i].position.y
            self.current_pose[i][2]=msg.poses[i].position.z

    def goal_pose_callback(self,msg):
        self.goal_pose[msg.header.seq][0]=msg.point.x
        self.goal_pose[msg.header.seq][1]=msg.point.y
        self.goal_pose[msg.header.seq][2]=msg.point.z
        self.need_new_plan[msg.header.seq]=0
        # print(self.goal_pose)

    def twist_msg(self):
        for i in range(self.n_agents):            
            if abs(self.v_x_output[i])>self.max_vel_lin :
                self.cmd_vel_msg.poses[i].position.x = self.max_vel_lin*abs(self.v_x_output[i])/self.v_x_output[i]
                self.v_x_output[i]=self.max_vel_lin*abs(self.v_x_output[i])/self.v_x_output[i]    
            else :
               self.cmd_vel_msg.poses[i].position.x = self.v_x_output[i]

            if abs(self.v_y_output[i])>self.max_vel_lin :
                self.cmd_vel_msg.poses[i].position.y = self.max_vel_lin*abs(self.v_y_output[i])/self.v_y_output[i]    
                self.v_y_output[i] = self.max_vel_lin*abs(self.v_y_output[i])/self.v_y_output[i]
            else :
               self.cmd_vel_msg.poses[i].position.y = self.v_y_output[i]

            if(abs(self.w_output[i])>self.max_vel_ang):
                self.cmd_vel_msg.poses[i].position.z = self.max_vel_ang*abs(self.w_output[i])/self.w_output[i]
                self.w_output[i] = self.max_vel_ang*abs(self.w_output[i])/self.w_output[i]
            else :
                self.cmd_vel_msg.poses[i].position.z = self.w_output[i]  

        # computing the wheel speeds
        self.inverse_tranform()

        self.control_input_pub.publish(self.cmd_vel_msg)
        self.wheel_speed_pub.publish(self.wheel_vel_msg)

    def inverse_tranform(self):
        for i in range(self.n_agents):
            a=self.current_pose[i][2]
            inverse_kinematics_transform=np.array([[sin(a),-cos(a),-self.l1],[cos(pi/4+a),sin(pi/4+a),-self.l2],[-cos(pi/4-a),sin(pi/4-a),-self.l3]])
            req_vel=np.array([[self.v_x_output[i]],[self.v_y_output[i]],[self.w_output[i]]])
            wheel_speed=np.matmul(inverse_kinematics_transform,req_vel)
            # print(wheel_speed)
            self.wheel_vel_msg.poses[i].position.x=wheel_speed[0][0]
            self.wheel_vel_msg.poses[i].position.y=wheel_speed[1][0]
            self.wheel_vel_msg.poses[i].position.z=wheel_speed[2][0]

    def resetValues(self,i):
        self.lastError_dist[i] = 0
        self.lastError_angle[i] = 0
        self.lastError_small[i]=0
        self.sumError_dist[i] = 0
        self.sumError_angle[i] = 0        

    def angle(self,y,x):
        if(x>0 and y>0):
            return atan(y/(x+0.00001))
        elif(x<0 and y>0):
            return (pi +atan(y/(x+0.00001)))
        elif(x<0 and y<0):
            return (atan(y/(x+0.00001))-pi)
        else:
            return atan(y/(x+0.00001))

    def correct_diff_yaw(self, diff_yaw):
        if diff_yaw < -1*pi:
            return self.correct_diff_yaw(diff_yaw+2*pi)
        if diff_yaw > pi:
            return self.correct_diff_yaw(diff_yaw-2*pi)
        return diff_yaw

    def pid(self):
        # sleep(1)
        while not rospy.is_shutdown():
            for i in range(self.n_agents):          #1 should be replaced with self.n_agents
                if(self.need_new_plan[i] == 0):      
                    path_angle=self.angle((self.goal_pose[i][1]-self.current_pose[i][1]),(self.goal_pose[i][0]-self.current_pose[i][0]))
                    goal_distance=sqrt((self.current_pose[i][0]-self.goal_pose[i][0])**2+(self.current_pose[i][1]-self.goal_pose[i][1])**2)
                    diff_yaw=self.correct_diff_yaw(path_angle-self.current_pose[i][2])
                    # print("-----------------------------")
                    # print(self.goal_pose[i])

                    #aligining towards the path
                    condition1=((abs(diff_yaw)>self.yaw_threshold and self.v_x_output[i]==0) or abs(diff_yaw)>self.yaw_large_error)        
                    # print(condition1)
                    if (condition1):
                        self.w_output[i]=self.kp_angle*(diff_yaw)+self.kd_angle*(diff_yaw-self.lastError_angle[i])+self.ki_angle*(self.sumError_angle[i])
                        self.lastError_angle[i]=diff_yaw
                        if(abs(self.sumError_angle[i]+diff_yaw)<self.intergral_windup_yaw):
                            self.sumError_angle[i]=self.sumError_angle[i]+diff_yaw
                        # print(diff_yaw,path_angle,self.current_pose[i][2])

                    #moving towards goal        
                    condition2= goal_distance>self.lin_threshold and (not condition1)
                    if (condition2):
                        self.v_x_output[i]=self.kp_lin*(goal_distance)+self.kd_lin*(goal_distance-self.lastError_dist[i])+self.ki_lin*self.sumError_dist[i]
                        self.w_output[i]=self.kp_angle_soft*(diff_yaw)+self.kd_angle_soft*(diff_yaw-self.lastError_small[i])
                        self.lastError_dist[i]=goal_distance
                        self.lastError_small[i]=diff_yaw
                        if(abs(self.sumError_dist[i]+goal_distance)<self.intergral_windup_lin):
                            self.sumError_dist[i]=self.sumError_dist[i]+goal_distance
                
                    if( (not condition1) and (not condition2)): #this condition is called when given goal_pose is obtained
                        self.v_x_output[i]=0.0
                        self.w_output[i]=0.0
                        self.v_y_output[i]=0.0
                        self.need_new_plan[i]=1
                        self.resetValues(i)

                    # for halt we appned -100,-100 to goal_pose hence the below thing for the same
                    if (self.goal_pose[i][0]==-100 and self.goal_pose[i][0]==-100)
                        if(self.halt_count[i] < self.halt_time):
                            self.halt_count[i]=self.halt_count[i]+1
                        else :
                            self.need_new_plan[i]=1
                            self.halt_count[i]=0
                        self.v_x_output[i]=0.0
                        self.v_y_output[i]=0.0
                        self.w_output[i]=0.0
                if(self.need_new_plan[i] == 1)
                    pub_msgs=UInt8()
                    pub_msgs.data=i
                    self.flag_pid_pub.publish(pub_msgs)
                    self.need_new_plan[i]=2
                # print("dist:",goal_distance," | yaw:",diff_yaw, " | v:",self.v_x_output[i]," | w:",self.w_output[i])
            
            self.twist_msg()
            # print('hey there')
            self.control_rate.sleep() 
    
if __name__ == '__main__':

    rospy.init_node('controller_node')
    rospy.loginfo("controller_node for bot1 created")
    pid_controller=PID()
    pid_controller.pid()
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()
