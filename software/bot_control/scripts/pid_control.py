#! /usr/bin/env python2.7

import os
import rospy
from time import sleep
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from std_msgs.msg import Bool
from math import sqrt, pi, atan
from bot_control.msg import Poses
import numpy as np

class PID:
    def __init__(self):
        # defining tunable params
        self.kp_lin = 1
        self.ki_lin = 0.0
        self.kd_lin = 0.4

        self.kp_angle = 200
        self.ki_angle = 0.0
        self.kd_angle = 0.0

        self.kp_angle_soft = 69
        self.ki_angle_soft = 0.0
        self.kd_angle_soft = 15.0

        self.max_vel_lin=1.8
        self.max_vel_ang=50
        self.lin_threshold=1
        self.dist_large_error=10
        self.yaw_threshold=0.3
        self.yaw_large_error=0.9
        self.intergral_windup_yaw=20
        self.intergral_windup_lin=15
        self.control_rate=rospy.Rate(10)

        #other valribales from here
        self.n_agents=4
        self.control_input_pub0 = rospy.Publisher('/bot0/cmd_vel', Twist, queue_size=10)
        self.control_input_pub1 = rospy.Publisher('/bot1/cmd_vel', Twist, queue_size=10)
        self.control_input_pub2 = rospy.Publisher('/bot2/cmd_vel', Twist, queue_size=10)
        self.control_input_pub3 = rospy.Publisher('/bot3/cmd_vel', Twist, queue_size=10)
        # self.cmd_pub=[self.control_input_pub0,self.control_input_pub1,self.control_input_pub2,self.control_input_pub3]
        self.current_pose=np.zeros([self.n_agents,3])
        self.goal_pose=[]

        self.current_state_sub=rospy.Subscriber('/poses', Poses,self.current_state_callback,queue_size=10)
        self.goal_pose_sub=rospy.Subscriber('/goal_point', Point,self.goal_pose_callback,queue_size=10)
        self.goal_complete_flag=rospy.Publisher('/bot1_waypoint_flag',Bool, queue_size=10)        

        self.v_x_output=np.zeros(self.n_agents)
        self.w_output=np.zeros(self.n_agents)
        self.start = rospy.Time.now().to_nsec()
        self.lastError = np.zeros(self.n_agents)
        self.lastError_small=np.zeros(self.n_agents)
        self.sumError = np.zeros(self.n_agents)
        self.lastTime = np.zeros(self.n_agents)
        self.last = np.zeros(self.n_agents)
        sleep(3)
        self.goal_pose=self.current_pose

    def current_state_callback(self,msg):
        for i in range(self.n_agents):
            self.current_pose[i][0]=msg.posei[i].x
            self.current_pose[i][1]=msg.posei[i].y
            self.current_pose[i][2]=msg.posei[i].z

    def goal_pose_callback(self,msg):
        self.goal_pose[msg.z][0]=msg.x
        self.goal_pose[msg.z][1]=msg.y

    def twist_msg(self):
        msg_pub = [Twist(),Twist(),Twist(),Twist()]
        for i in range(self.n_agents):            
            if self.v_x_output[i]>self.max_vel_lin :
                msg_pub[i].linear.x = self.max_vel_lin    
            else :
               msg_pub[i].linear.x = self.v_x_output[i]
            if(self.w_output[i]>self.max_vel_ang):
                msg_pub[i].angular.z = self.max_vel_ang
            else :
                msg_pub[i].angular.z = self.w_output[i]    
            msg_pub[i].linear.y = 0
            msg_pub[i].linear.z = 0
            msg_pub[i].angular.x = 0
            msg_pub[i].angular.y = 0        
        self.control_input_pub0.publish(msg_pub[0])
        self.control_input_pub1.publish(msg_pub[1])
        self.control_input_pub2.publish(msg_pub[2])
        self.control_input_pub3.publish(msg_pub[3])
        # print(msg_pub.linear.x,msg_pub.angular.z)

    def resetValues(self,i):
        self.lastError[i] = 0
        self.lastError_small[i]=0
        self.sumError[i] = 0
        self.lastTime[i] = 0

    def angle(self,y,x):
        if(x>0 and y>0):
            return atan(y/x)
        elif(x<0 and y>0):
            return (pi +atan(y/x))
        elif(x<0 and y<0):
            return (atan(y/x)-pi)
        else:
            return atan(y/x)

    def pid(self):
        sleep(5)
        while not rospy.is_shutdown():
            for i in range(self.n_agents):                
                path_angle=self.angle((self.goal_pose[i][1]-self.current_pose[i][1]),(self.goal_pose[i][0]-self.current_pose[i][0]))
                goal_distance=sqrt((self.current_pose[i][0]-self.goal_pose[i][0])**2+(self.current_pose[i][1]-self.goal_pose[i][1])**2)
                diff_yaw=path_angle-self.current_pose[i][2]
                print("here")

                #aligining towards the path
                self.resetValues(i)
                condition1=((abs(diff_yaw)>self.yaw_threshold and self.v_x_output[i]==0) or abs(diff_yaw)>self.yaw_large_error) and goal_distance>self.dist_large_error        
                if (condition1):
                    self.w_output[i]=self.kp_angle*(diff_yaw)+self.kd_angle*(diff_yaw-self.lastError)+self.ki_angle*(self.sumError)
                    self.lastError[i]=diff_yaw
                    if(abs(self.sumError[i]+diff_yaw)<self.intergral_windup_yaw):
                        self.sumError[i]=self.sumError[i]+diff_yaw
                    # print(diff_yaw,path_angle,self.current_pose[i][2])

                #moving towards goal        
                self.resetValues(i)
                condition2= goal_distance>self.lin_threshold and (not condition1)
                if (condition2):
                    self.v_x_output[i]=self.kp_lin*(diff_distance)+self.kd_lin*(diff_distance-self.lastError[i])+self.ki_lin*self.sumError[i]
                    self.w_output[i]=self.kp_angle_soft*(diff_yaw)+self.kd_angle_soft*(diff_yaw-self.lastError_small[i])
                    self.lastError[i]=diff_distance
                    self.lastError_small[i]=diff_yaw
                    if(abs(self.sumError[i]+diff_distance)<self.intergral_windup_lin):
                        self.sumError[i]=self.sumError+diff_distance
                self.resetValues(i)
                if( (not condition1) and (not condition2)):
                    self.v_x_output[i]=0
                    self.w_output[i]=0
            
            self.twist_msg()
            self.control_rate.sleep() 
    
if __name__ == '__main__':

    rospy.init_node('controller_node_bot1')
    rospy.loginfo("controller_node for bot1 created")
    pid_controller=PID()
    pid_controller.pid()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
