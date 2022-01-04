#! /usr/bin/env python2.7

import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, PoseArray, Point
from bot_control.msg import pose_bot
from std_msgs.msg import Bool, UInt8
from math import sqrt, pi, atan, ceil,sin,cos
import numpy as np
from time import sleep

class PID:
    def __init__(self):
        self.n_agents=1
        self.control_rate=rospy.Rate(10)

        self.scaling_factor=50
        # defining tunable params
        self.kp_lin_x= 0.2
        self.kd_lin_x= 0.4
        self.ki_lin_x= 0

        self.kp_soft_lin_x= 0.5
        self.kd_soft_lin_x= 0.8
        self.ki_soft_lin_x= 0

        self.kp_lin_y= 0.2
        self.kd_lin_y= 0.4
        self.ki_lin_y= 0

        self.kp_soft_lin_y= 0.5
        self.kd_soft_lin_y= 0.8
        self.ki_soft_lin_y= 0

        self.kp_angle= 10.6
        self.kd_angle= 1.55
        self.ki_angle= 1.19

        # self.kp_soft_angle= 120.0/20.0
        # self.kd_soft_angle= 0
        # self.ki_soft_angle= 0

        self.max_vel_lin= 3.0
        self.max_vel_ang= 35

        self.intergral_windup_yaw=3.0
        self.intergral_windup_lin_x=15.0
        self.intergral_windup_lin_y=15.0

        self.lin_x_threshold = 0.5
        self.lin_x_smalldiff = 2

        self.lin_y_threshold = 0.5
        self.lin_y_smalldiff = 2

        self.angle_threshold = 0.25
        # self.angle_smalldiff = 0.3

        self.halt_unit=10  #1 count is one time step which is 1/frequncy (control rate)
        # error varibales
        self.lastError_dist_x = np.zeros(self.n_agents)
        self.lastError_dist_y = np.zeros(self.n_agents)
        self.lastError_angle = np.zeros(self.n_agents)

        self.sumError_dist_x = np.zeros(self.n_agents)
        self.sumError_dist_y = np.zeros(self.n_agents)
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
        self.l1=0.048
        self.l2=0.054
        self.l3=0.054

        # publishers
        self.control_input_pub = rospy.Publisher('/cmd_vel', PoseArray, queue_size=10)
        self.cmd_vel_msg=PoseArray() # here we use only poistion of Poses msg. x will have vx, y will be vy and z will be omega in position object
        self.wheel_speed_pub=rospy.Publisher('/wheel_speed',PoseArray, queue_size=10)
        self.wheel_vel_msg=PoseArray() # here we use only poistion of Poses msg. x will have w1, y will be w1 and z will be w3 in position object
        self.initialize_cmd_vel_msg()
        self.flag_pid_pub = rospy.Publisher('/flag_pid',UInt8,queue_size=10)

        # subscribers
        self.current_state_sub=rospy.Subscriber('/poses', PoseArray,self.current_state_callback,queue_size=10)
        self.goal_pose_sub=rospy.Subscriber('/goal_point', pose_bot,self.goal_pose_callback,queue_size=10)

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
        self.cmd_vel_msg.poses=temp_poses        #why is this cmd_vel_msg."poses"???
        self.wheel_vel_msg.poses=temp_poses2

    def current_state_callback(self,msg):
        for i in range(self.n_agents):
            self.current_pose[i][0]=msg.poses[i].position.x
            self.current_pose[i][1]=msg.poses[i].position.y
            self.current_pose[i][2]=msg.poses[i].position.z

    def goal_pose_callback(self,msg):
        self.goal_pose[msg.bot_num][0]=msg.x
        self.goal_pose[msg.bot_num][1]=msg.y
        if (msg.yaw != 100):
            self.goal_pose[msg.bot_num][2]=msg.yaw
        self.need_new_plan[msg.bot_num]=0
        print(self.goal_pose)

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
            wheel_speed=wheel_speed*self.scaling_factor
            # print(wheel_speed)
            self.wheel_vel_msg.poses[i].position.x=wheel_speed[0][0]
            self.wheel_vel_msg.poses[i].position.y=wheel_speed[1][0]  
            self.wheel_vel_msg.poses[i].position.z=wheel_speed[2][0]

    def resetValues(self,i):
        self.lastError_dist_x[i] = 0
        self.lastError_dist_y[i] = 0
        self.lastError_angle[i] = 0

        self.sumError_dist_x[i] = 0
        self.sumError_dist_y[i] = 0
        self.sumError_angle[i] = 0

    def correct_diff_yaw(self, diff_yaw):
        if diff_yaw < -1*pi:
            return self.correct_diff_yaw(diff_yaw+2*pi)
        if diff_yaw > pi:
            return self.correct_diff_yaw(diff_yaw-2*pi)
        return diff_yaw

    def pid(self):
        while not rospy.is_shutdown():
            for i in range(self.n_agents):
                if(self.need_new_plan[i] == 0):  #means already there is a plan
                    diff_yaw=self.correct_diff_yaw(self.goal_pose[i][2]-self.current_pose[i][2])
                    distance_x= self.goal_pose[i][0] - self.current_pose[i][0]
                    distance_y= self.goal_pose[i][1] - self.current_pose[i][1]
                    #PID along x
                    if(abs(distance_x)>=abs(distance_y)):
                        if (abs(distance_y) > self.lin_y_smalldiff): #moving in x only after in line of motion
                            print("correcting only y")
                            self.v_y_output[i] = 0.5+ self.kp_lin_y*(distance_y) + self.kd_lin_y*(distance_y-self.lastError_dist_y[i])+self.ki_lin_y*self.sumError_dist_y[i]
                            self.lastError_dist_y[i]= distance_y
                            if(abs(self.sumError_dist_y[i] + distance_y)<self.intergral_windup_lin_y):
                                self.sumError_dist_y[i] = self.sumError_dist_y[i] + distance_y
                            self.v_x_output[i] =0

                        else : #moving in x
                            #minor corrections in y
                            if abs(distance_y)<self.lin_y_threshold :
                                self.v_y_output[i]=0
                            else :
                                self.v_y_output[i] = 0.5 + self.kp_soft_lin_y*(distance_y) + self.kd_soft_lin_y*(distance_y-self.lastError_dist_y[i])+self.ki_soft_lin_y*self.sumError_dist_y[i]
                                self.lastError_dist_y[i]= distance_y
                                if(abs(self.sumError_dist_y[i] + distance_y)<self.intergral_windup_lin_y):
                                    self.sumError_dist_y[i] = self.sumError_dist_y[i] + distance_y

                            #PID along x
                            if (abs(distance_x) > self.lin_x_threshold):
                                self.v_x_output[i] = 0.5 + self.kp_lin_x*(distance_x) + self.kd_lin_x*(distance_x-self.lastError_dist_x[i])+self.ki_lin_x*self.sumError_dist_x[i]
                                self.lastError_dist_x[i]= distance_x
                                if(abs(self.sumError_dist_x[i] + distance_x)<self.intergral_windup_lin_x):
                                    self.sumError_dist_x[i] = self.sumError_dist_x[i] + distance_x
                                print(self.v_x_output[i])
                            # elif (abs(distance_x) <= self.lin_x_smalldiff and abs(distance_x) > self.lin_x_threshold):
                            #     self.v_x_output[i] = self.kp_soft_lin_x*(distance_x) + self.kd_soft_lin_x*(distance_x-self.lastError_dist_x[i])+self.ki_soft_lin_x*self.sumError_dist_x[i]
                            #     self.lastError_dist_x[i]= distance_x
                            #     if(abs(self.sumError_dist_x[i] + distance_x)<self.intergral_windup_lin_x):
                            #         self.sumError_dist_x[i] = self.sumError_dist_x[i] + distance_x

                            else:
                                self.v_x_output[i]=0                                
                    else:  #PID along y

                        if (abs(distance_x) > self.lin_x_smalldiff):
                            if (abs(distance_x) > self.lin_x_smalldiff):
                                self.v_x_output[i] = self.kp_lin_x*(distance_x) + self.kd_lin_x*(distance_x-self.lastError_dist_x[i])+self.ki_lin_x*self.sumError_dist_x[i]
                                self.lastError_dist_x[i]= distance_x
                                if(abs(self.sumError_dist_x[i] + distance_x)<self.intergral_windup_lin_x):
                                    self.sumError_dist_x[i] = self.sumError_dist_x[i] + distance_x
                                self.v_y_output[i]=0

                        else:
                            if(abs(distance_x)<self.lin_x_threshold):
                                self.v_x_output[i]=0
                            else :
                                self.v_x_output[i] = self.kp_soft_lin_x*(distance_x) + self.kd_soft_lin_x*(distance_x-self.lastError_dist_x[i])+self.ki_soft_lin_x*self.sumError_dist_x[i]
                                self.lastError_dist_x[i]= distance_x
                                if(abs(self.sumError_dist_x[i] + distance_x)<self.intergral_windup_lin_x):
                                    self.sumError_dist_x[i] = self.sumError_dist_x[i] + distance_x
                            #PID along y
                            if (abs(distance_y) > self.lin_y_threshold): #moving in x only after in line of motion
                                self.v_y_output[i] = self.kp_lin_y*(distance_y) + self.kd_lin_y*(distance_y-self.lastError_dist_y[i])+self.ki_lin_y*self.sumError_dist_y[i]
                                self.lastError_dist_y[i]= distance_y
                                if(abs(self.sumError_dist_y[i] + distance_y)<self.intergral_windup_lin_y):
                                    self.sumError_dist_y[i] = self.sumError_dist_y[i] + distance_y
                            # elif(abs(distance_y)>self.lin_y_threshold):
                            #     self.v_y_output[i] = self.kp_soft_lin_y*(distance_y) + self.kd_soft_lin_y*(distance_y-self.lastError_dist_y[i])+self.ki_soft_lin_y*self.sumError_dist_y[i]
                            #     self.lastError_dist_y[i]= distance_y
                            #     if(abs(self.sumError_dist_y[i] + distance_y)<self.intergral_windup_lin_y):
                            #         self.sumError_dist_y[i] = self.sumError_dist_y[i] + distance_y
                            else :
                                self.v_y_output[i]=0



                    #PID angular
                    if (abs(diff_yaw) > self.angle_threshold):
                        self.w_output[i]= self.kp_angle*(diff_yaw) + self.kd_angle*(diff_yaw-self.lastError_angle[i]) + self.ki_angle*(self.sumError_angle[i])
                        self.lastError_angle[i] = diff_yaw
                        if(abs(self.sumError_angle[i]+diff_yaw)<self.intergral_windup_yaw):
                            self.sumError_angle[i]=self.sumError_angle[i]+diff_yaw
                    # elif (abs(diff_yaw) <= self.angle_smalldiff and abs(diff_yaw)>self.angle_threshold):
                    #     self.w_output[i]= self.kp_soft_angle*(diff_yaw) + self.kd_soft_angle*(diff_yaw-self.lastError_angle[i]) + self.ki_soft_angle*(self.sumError_angle[i])
                    #     self.lastError_angle[i] = diff_yaw
                    #     if(abs(self.sumError_angle[i]+diff_yaw)<self.intergral_windup_yaw):
                    #         self.sumError_angle[i]=self.sumError_angle[i]+diff_yaw
                    else:
                        self.w_output[i]=0

                    # print(diff_yaw,distance_y,distance_x)
                    # might have to tweak in case of overshoot or very high time
                    if(abs(diff_yaw) <= self.angle_threshold and abs(distance_y) <= self.lin_y_threshold and abs(distance_x) <= self.lin_x_threshold):
                        self.need_new_plan[i]=1
                        self.resetValues(i)
                        self.v_x_output[i]=0
                        self.v_y_output[i]=0
                        self.w_output[i]=0


                    # for halt we appned -100,-100 to goal_pose hence the below thing for the same
                    if (self.goal_pose[i][0]==-100 ):
                        if(self.halt_count[i] < self.halt_unit*self.goal_pose[i][1]):
                            self.halt_count[i]=self.halt_count[i]+1
                        else :
                            self.need_new_plan[i]=1
                            self.halt_count[i]=0
                        self.v_x_output[i]=0.0
                        self.v_y_output[i]=0.0
                        self.w_output[i]=0.0

                if(self.need_new_plan[i] == 1):
                    pub_msgs=UInt8()
                    pub_msgs.data=i
                    print('bot: ',i, ' needs new waypoint')
                    self.flag_pid_pub.publish(pub_msgs)
                    self.need_new_plan[i]=2

            self.twist_msg()
            print(self.goal_pose)
            # print(self.need_new_plan)
            # print('hey there')
            self.control_rate.sleep()


if __name__ == '__main__':

    rospy.init_node('controller_node')
    rospy.loginfo("controller_node for bot1 created")
    pid_controller=PID()
    sleep(7)
    pid_controller.pid()
    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()