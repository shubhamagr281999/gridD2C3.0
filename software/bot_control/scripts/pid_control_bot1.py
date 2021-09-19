#! /usr/bin/env python2.7

import os
import rospy
from time import sleep
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose2D, Twist
from std_msgs.msg import Bool
from math import sqrt, pi, atan



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
        self.yaw_threshold=0.3
        self.intergral_windup_yaw=20
        self.intergral_windup_lin=15
        self.control_rate=rospy.Rate(10)

        #other valribales from here
        self.control_input_pub = rospy.Publisher('/bot1/cmd_vel', Twist, queue_size=10)
        self.current_state_sub=rospy.Subscriber('/bot1_pose', Pose2D,self.current_state_callback,queue_size=10)
        self.goal_pose_sub=rospy.Subscriber('/bot1_goal', Pose2D,self.goal_pose_callback,queue_size=10)
        self.goal_complete_flag=rospy.Publisher('/bot1_waypoint_flag',Bool, queue_size=10)
        
        self.current_pose_x=0.0
        self.current_pose_y=0.0
        self.current_pose_yaw=0.0

        self.goal_pose_x=0.0
        self.goal_pose_y=0.0
        self.goal_pose_yaw=0.0

        self.start = rospy.Time.now().to_nsec()
        self.lastError = 0
        self.lastError_small=0
        self.sumError = 0
        self.lastTime = 0
        self.last = 0

    def current_state_callback(self,msg):
        self.current_pose_x=msg.x
        self.current_pose_y=msg.y
        self.current_pose_yaw=msg.theta
        # print("curent pose:", self.current_pose_x,self.current_pose_y,self.current_pose_yaw)

    def goal_pose_callback(self,msg):
        self.goal_pose_x=msg.x
        self.goal_pose_y=msg.y
        self.goal_pose_yaw=msg.theta
        self.PID()

    def twist_msg(self, v_x_output, w_output):
        msg_pub = Twist()
        if v_x_output>self.max_vel_lin :
            msg_pub.linear.x = self.max_vel_lin    
        else :
           msg_pub.linear.x = v_x_output
        if(w_output>self.max_vel_ang):
            msg_pub.angular.z = self.max_vel_ang
        else :
            msg_pub.angular.z = w_output    
        msg_pub.linear.y = 0
        msg_pub.linear.z = 0
        msg_pub.angular.x = 0
        msg_pub.angular.y = 0
        
        self.control_input_pub.publish(msg_pub)
        # print(msg_pub.linear.x,msg_pub.angular.z)

    def resetValues(self):
        self.lastError = 0
        self.lastError_small=0
        self.sumError = 0
        self.lastTime = 0

    def angle(self,y,x):
        if(x>0 and y>0):
            return atan(y/x)
        elif(x<0 and y>0):
            return (pi +atan(y/x))
        elif(x<0 and y<0):
            return (atan(y/x)-pi)
        else:
            return atan(y/x)

    def PID(self):
        path_angle=self.angle((self.goal_pose_y-self.current_pose_y),(self.goal_pose_x-self.current_pose_x))
        goal_distance=sqrt((self.current_pose_x-self.goal_pose_x)**2+(self.current_pose_y-self.goal_pose_y)**2)
        diff_yaw=path_angle-self.current_pose_yaw
        print("here")

        #aligining towards the path
        self.resetValues()        
        while (abs(diff_yaw)>self.yaw_threshold):
            path_angle=self.angle((self.goal_pose_y-self.current_pose_y),(self.goal_pose_x-self.current_pose_x))
            diff_yaw=path_angle-self.current_pose_yaw
            control_signal_angle=self.kp_angle*(diff_yaw)+self.kd_angle*(diff_yaw-self.lastError)+self.ki_angle*(self.sumError)
            self.twist_msg(0.0,control_signal_angle)
            self.lastError=diff_yaw
            if(abs(self.sumError+diff_yaw)<self.intergral_windup_yaw):
                self.sumError=self.sumError+diff_yaw
            print(diff_yaw,path_angle,self.current_pose_yaw)
            self.control_rate.sleep()

        #moving towards goal        
        self.resetValues()
        while(goal_distance>self.lin_threshold):
            path_angle=self.angle((self.goal_pose_y-self.current_pose_y),(self.goal_pose_x-self.current_pose_x))
            diff_yaw=path_angle-self.current_pose_yaw
            diff_distance=(sqrt((self.current_pose_x-self.goal_pose_x)**2+(self.current_pose_y-self.goal_pose_y)**2))/100
            control_signal_dist=self.kp_lin*(diff_distance)+self.kd_lin*(diff_distance-self.lastError)+self.ki_lin*self.sumError
            print(diff_distance)
            print(self.current_pose_x,self.current_pose_y)
            control_signal_angle=self.kp_angle_soft*(diff_yaw)+self.kd_angle_soft*(diff_yaw-self.lastError_small)
            self.twist_msg(control_signal_dist,control_signal_angle)
            self.lastError=diff_distance
            self.lastError_small=diff_yaw
            if(abs(self.sumError+diff_distance)<self.intergral_windup_lin):
                self.sumError=self.sumError+diff_distance
            self.control_rate.sleep()
        self.resetValues()
        self.twist_msg(0,0) 
    
if __name__ == '__main__':

    rospy.init_node('controller_node_bot1')
    rospy.loginfo("controller_node for bot1 created")
    pid_controller=PID()
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
