#! /usr/bin/env python2.7

import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose, PoseArray, Point
from bot_control.msg import pose_bot
from std_msgs.msg import Bool, UInt8
from math import sqrt, pi, atan, ceil,sin,cos
import numpy as np
import itertools
import time
from time import sleep

class PID:
    def __init__(self):
        self.n_agents=6
        self.control_rate=rospy.Rate(10)


        # defining tunable params
        self.kp_lin_x= 1.0
        self.kd_lin_x= 1.0/10.0
        self.ki_lin_x= 1.0/2.0

        self.kp_soft_lin_x= 5.0
        self.kd_soft_lin_x= 5.0/10.0
        self.ki_soft_lin_x= 5.0/2.0

        self.kp_lin_y= 1.0
        self.kd_lin_y= 1.0/10.0
        self.ki_lin_y= 1.0/2.0

        self.kp_soft_lin_y= 5.0
        self.kd_soft_lin_y= 5.0/10.0
        self.ki_soft_lin_y= 5.0/2.0

        self.kp_angle= 1.0
        self.kd_angle= 1.0/10.0
        self.ki_angle= 1.0/2.0

        self.kp_soft_angle= 1.0/5
        self.kd_soft_angle= 1.0/50.0
        self.ki_soft_angle= 1.0/10.0

        self.max_vel_lin= 4.0
        self.max_vel_ang= 0.4

        self.intergral_windup_yaw=20.0
        self.intergral_windup_lin_x=15.0
        self.intergral_windup_lin_y=15.0

        self.lin_x_threshold = 0.5
        self.lin_x_smalldiff = 2

        self.lin_y_threshold = 0.5
        self.lin_y_smalldiff = 2

        self.angle_threshold = 0.1
        self.angle_smalldiff = 0.3

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
        self.l1=0.07
        self.l2=0.07
        self.l3=0.07

        # publishers
        self.control_input_pub = rospy.Publisher('/cmd_vel', PoseArray, queue_size=10)
        self.cmd_vel_msg=PoseArray() # here we use only poistion of Poses msg. x will have vx, y will be vy and z will be omega in position object
        self.wheel_speed_pub=rospy.Publisher('/wheel_speed',PoseArray, queue_size=10)
        self.wheel_vel_msg=PoseArray() # here we use only poistion of Poses msg. x will have w1, y will be w1 and z will be w3 in position object
        self.initialize_cmd_vel_msg()
        self.flag_pid_pub = rospy.Publisher('/flag_pid',UInt8,queue_size=10)
        self.collision_pub = rospy.Publisher('/direct_collision',Bool,queue_size=10)

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
        # print(self.goal_pose)
    
    def check_collision(self):
        bot_num = []             #an array of bot_num [0,1,2,3 ...]
        time_limit = 4           #how much far ahead do we want to predict collision
        threshold = 7           #threshold distance
        for i in range(self.n_agents):
            bot_num.append(i)
        pair_list = itertools.combinations(bot_num,2)  # all combination pairs [[0,1],[0,2],[0,3]....]
    
        for pair in pair_list:
            print("started outer loop")
            for time in range(time_limit):
                print("started inner loop")
                x1 = self.current_pose[pair[0]][0] + self.v_x_output[pair[0]]*time
                y1 = self.current_pose[pair[0]][1] + self.v_y_output[pair[0]]*time
                x2 = self.current_pose[pair[1]][0] + self.v_x_output[pair[1]]*time
                y2 = self.current_pose[pair[1]][1] + self.v_y_output[pair[1]]*time
    
                distance = sqrt((x1-x2)**2 + (y1-y2)**2)
    
                vx1 = self.v_x_output[pair[0]]
                vy1 = self.v_y_output[pair[0]]
                vx2 = self.v_x_output[pair[1]]
                vy2 = self.v_y_output[pair[1]]
    
                v1 = sqrt((vx1)**2 + (vy1)**2)
                v2 = sqrt((vx2)**2 + (vy2)**2)
    
                dot_prod = (vx2 - vx1)*(x1-x2) +  (vy2 - vy1)*(y1-y2)
                print(pair,distance,threshold,time)
    
                if ( distance <= threshold ) and ( (v2 - v1)*dot_prod >0 ) and (v1 !=0 and v2 !=0):
                    print("Predicted Collision between bot", pair[0], "and bot", pair[1], "distance:", distance, "in ", time ,"Seconds")
    
                    if( v1 - v2) > 0:    #implies v1> v2 and obtuse angle(dot_product <0) = collision = stop bot 1
    
                        self.v_x_output[pair[0]] = 0            #stopping bot_1
                        self.v_y_output[pair[0]] = 0
                        self.w_output[pair[0]] = 0
                        self.cmd_vel_msg.poses[pair[0]].position.x= 0
                        self.cmd_vel_msg.poses[pair[0]].position.y = 0
                        self.cmd_vel_msg.poses[pair[0]].position.z= 0
                        print("stopping bot",pair[0])
    
                    if( v2 - v1) > 0:   #implies v2>v1 and acute angle = stop bot 2
    
                        self.v_x_output[pair[1]] = 0            #stopping bot_2
                        self.v_y_output[pair[1]] = 0
                        self.w_output[pair[1]] = 0
                        self.cmd_vel_msg.poses[pair[1]].position.x= 0
                        self.cmd_vel_msg.poses[pair[1]].position.y = 0
                        self.cmd_vel_msg.poses[pair[1]].position.z= 0
                        print("stopping bot",pair[1])

    # def check_collision(self):
    #     bot_num = []             #an array of bot_num [0,1,2,3 ...]

    #     for i in range(self.n_agents):
    #         bot_num.append(i)
    #     pair_list = itertools.combinations(bot_num,2)  # all combination pairs [[0,1],[0,2],[0,3]....]

    #     for pair in pair_list:

    #         x1 = self.current_pose[pair[0]][0]
    #         y1 = self.current_pose[pair[0]][1]
    #         gx1 = self.goal_pose[pair[0]][0]
    #         gy1 = self.goal_pose[pair[0]][1]

    #         x2 = self.current_pose[pair[1]][0]
    #         y2 = self.current_pose[pair[1]][1]
    #         gx2 = self.goal_pose[pair[1]][0]
    #         gy2 = self.goal_pose[pair[1]][1]

    #         vx1 = self.v_x_output[pair[0]]
    #         vy1 = self.v_y_output[pair[0]]
    #         vx2 = self.v_x_output[pair[1]]
    #         vy2 = self.v_y_output[pair[1]]

    #         v1 = sqrt((vx1)**2 + (vy1)**2)
    #         v2 = sqrt((vx2)**2 + (vy2)**2)

    #         dx1 = gx1-x1
    #         dx2 = gx2-x2
    #         dy1 = gy1-y1
    #         dy2 = gy2-y2

    #         r1 = dx1/dy1
    #         r2 = dx2/dy2

    #         y_c = (x2-x1+r1*y1-r2*y2)/(r1-r2)
    #         x_c = x1 + ((y_c-y1)*r1)

    #         d1 = sqrt((x1-x_c)**2 + (y1-y_c)**2)
    #         d2 = sqrt((x2-x_c)**2 + (y2-y_c)**2)

    #         if vx1==0 and vy1==0:
    #             continue

    #         if vx2==0 and vy2==0:
    #             continue

    #         if d1<=6 and d2<=6:
    #             # if (vx1==0 and vx2==0) or (vy1==0 and vy2==0):
    #             #     print(vx1,vx2,vy1,vy2)
    #             #     print("maa chudao")
    #             #     for i in bot_num:
    #             #         self.v_x_output[i] = 0
    #             #         self.v_y_output[i] = 0
    #             #         self.w_output[i] = 0
    #             #     collision_msg=Bool()
    #             #     collision_msg.data=True
    #             #     self.collision_pub.publish(collision_msg)
    #             if d2>d1:
    #                 print("Predicted Collision between bot", pair[0], "and bot", pair[1], "in ", time ,"Seconds")
    #                 self.v_x_output[pair[1]] = 0            #stopping bot_2
    #                 self.v_y_output[pair[1]] = 0
    #                 self.w_output[pair[1]] = 0
    #             else:
    #                 print("Predicted Collision between bot", pair[0], "and bot", pair[1], "in ", time ,"Seconds")
    #                 self.v_x_output[pair[0]] = 0            #stopping bot_2
    #                 self.v_y_output[pair[0]] = 0
    #                 self.w_output[pair[0]] = 0

    #         # if((x_c-x1)*(gx1-x_c)>=0 and (y_c-y1)*(gy1-y_c)>=0 and (x_c-x2)*(gx2-x_c)>=0 and (y_c-y2)*(gy2-y_c)>=0):

    #         #     t1 = sqrt((x1-gx1)**2 + (y1-gy1)**2)/(v1+0.001)
    #         #     t2 = sqrt((x2-gx2)**2 + (y2-gy2)**2)/(v2+0.001)

    #         #     if ( t1 < t2):
    #         #         print("Predicted Collision between bot", pair[0], "and bot", pair[1], "in ", time ,"Seconds")
    #         #         self.collision_history += [[pair[0], pair[1], (x_c, y_c), (gx2, gy2)]]
    #         #         self.v_x_output[pair[1]] = 0            #stopping bot_2
    #         #         self.v_y_output[pair[1]] = 0
    #         #         self.w_output[pair[1]] = 0
    #         #         self.cmd_vel_msg.poses[pair[1]].position.x= 0
    #         #         self.cmd_vel_msg.poses[pair[1]].position.y = 0
    #         #         self.cmd_vel_msg.poses[pair[1]].position.z= 0

    #         #     if ( t1 > t2):
    #         #         print("Predicted Collision between bot", pair[0], "and bot", pair[1],  "in ", time ,"Seconds")
    #         #         self.collision_history += [[pair[1], pair[0], (x_c, y_c), (gx1, gy1)]]
    #         #         self.v_x_output[pair[0]] = 0            #stopping bot_1
    #         #         self.v_y_output[pair[0]] = 0
    #         #         self.w_output[pair[0]] = 0
    #         #         self.cmd_vel_msg.poses[pair[0]].position.x= 0
    #         #         self.cmd_vel_msg.poses[pair[0]].position.y = 0
    #         #         self.cmd_vel_msg.poses[pair[0]].position.z= 0


    def twist_msg(self):
        #check for Collision
        self.check_collision()

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
                            self.v_y_output[i] = self.kp_lin_y*(distance_y) + self.kd_lin_y*(distance_y-self.lastError_dist_y[i])+self.ki_lin_y*self.sumError_dist_y[i]
                            self.lastError_dist_y[i]= distance_y
                            if(abs(self.sumError_dist_y[i] + distance_y)<self.intergral_windup_lin_y):
                                self.sumError_dist_y[i] = self.sumError_dist_y[i] + distance_y
                            self.v_x_output[i] =0

                        else : #moving in x
                            #minor corrections in y
                            if abs(distance_y)<self.lin_y_threshold :
                                self.v_y_output[i]=0
                            else :
                                self.v_y_output[i] = self.kp_soft_lin_y*(distance_y) + self.kd_soft_lin_y*(distance_y-self.lastError_dist_y[i])+self.ki_soft_lin_y*self.sumError_dist_y[i]
                                self.lastError_dist_y[i]= distance_y
                                if(abs(self.sumError_dist_y[i] + distance_y)<self.intergral_windup_lin_y):
                                    self.sumError_dist_y[i] = self.sumError_dist_y[i] + distance_y

                            #PID along x
                            if (abs(distance_x) > self.lin_x_smalldiff):
                                self.v_x_output[i] = self.kp_lin_x*(distance_x) + self.kd_lin_x*(distance_x-self.lastError_dist_x[i])+self.ki_lin_x*self.sumError_dist_x[i]
                                self.lastError_dist_x[i]= distance_x
                                if(abs(self.sumError_dist_x[i] + distance_x)<self.intergral_windup_lin_x):
                                    self.sumError_dist_x[i] = self.sumError_dist_x[i] + distance_x

                            elif (abs(distance_x) <= self.lin_x_smalldiff and abs(distance_x) > self.lin_x_threshold):
                                self.v_x_output[i] = self.kp_soft_lin_x*(distance_x) + self.kd_soft_lin_x*(distance_x-self.lastError_dist_x[i])+self.ki_soft_lin_x*self.sumError_dist_x[i]
                                self.lastError_dist_x[i]= distance_x
                                if(abs(self.sumError_dist_x[i] + distance_x)<self.intergral_windup_lin_x):
                                    self.sumError_dist_x[i] = self.sumError_dist_x[i] + distance_x

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
                            if (abs(distance_y) > self.lin_y_smalldiff): #moving in x only after in line of motion
                                self.v_y_output[i] = self.kp_lin_y*(distance_y) + self.kd_lin_y*(distance_y-self.lastError_dist_y[i])+self.ki_lin_y*self.sumError_dist_y[i]
                                self.lastError_dist_y[i]= distance_y
                                if(abs(self.sumError_dist_y[i] + distance_y)<self.intergral_windup_lin_y):
                                    self.sumError_dist_y[i] = self.sumError_dist_y[i] + distance_y
                            elif(abs(distance_y)>self.lin_y_threshold):
                                self.v_y_output[i] = self.kp_soft_lin_y*(distance_y) + self.kd_soft_lin_y*(distance_y-self.lastError_dist_y[i])+self.ki_soft_lin_y*self.sumError_dist_y[i]
                                self.lastError_dist_y[i]= distance_y
                                if(abs(self.sumError_dist_y[i] + distance_y)<self.intergral_windup_lin_y):
                                    self.sumError_dist_y[i] = self.sumError_dist_y[i] + distance_y
                            else :
                                self.v_y_output[i]=0



                    #PID angular
                    if (abs(diff_yaw) > self.angle_smalldiff):
                        self.w_output[i]= self.kp_angle*(diff_yaw) + self.kd_angle*(diff_yaw-self.lastError_angle[i]) + self.ki_angle*(self.sumError_angle[i])
                        self.lastError_angle[i] = diff_yaw
                        if(abs(self.sumError_angle[i]+diff_yaw)<self.intergral_windup_yaw):
                            self.sumError_angle[i]=self.sumError_angle[i]+diff_yaw
                    elif (abs(diff_yaw) <= self.angle_smalldiff and abs(diff_yaw)>self.angle_threshold):
                        self.w_output[i]= self.kp_soft_angle*(diff_yaw) + self.kd_soft_angle*(diff_yaw-self.lastError_angle[i]) + self.ki_soft_angle*(self.sumError_angle[i])
                        self.lastError_angle[i] = diff_yaw
                        if(abs(self.sumError_angle[i]+diff_yaw)<self.intergral_windup_yaw):
                            self.sumError_angle[i]=self.sumError_angle[i]+diff_yaw
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
                    # print('bot: ',i, ' needs new waypoint')
                    self.flag_pid_pub.publish(pub_msgs)
                    self.need_new_plan[i]=2

            self.twist_msg()
            # print(self.goal_pose)
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
