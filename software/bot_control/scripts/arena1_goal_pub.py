#!/usr/bin/env python

import rospy
import time
from time import sleep
from tf.transformations import euler_from_quaternion, quaternion_from_euler , euler_matrix
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Bool

class goal_publisher:
    def __init__(self):
        self.rate_move=rospy.Rate(0.5)
        self.goal_pose=[[234,311,0.0],[264,312,0.0],[295,314,0.0],[325,315,0.0]]
        self.time_for_flipping=2
        self.count=0
        self.mission_bot=1
        self.pose_error=0.03
        self.theta_error=0.1
        self.goal1=Pose2D()
        self.goal2=Pose2D()
        self.goal3=Pose2D()
        self.goal4=Pose2D()
        self.pub_bot1_goal=rospy.Publisher('/bot1_goal',Pose2D,queue_size=10)
        self.pub_bot2_goal=rospy.Publisher('/bot2_goal',Pose2D,queue_size=10)
        self.pub_bot3_goal=rospy.Publisher('/bot3_goal',Pose2D,queue_size=10)
        self.pub_bot4_goal=rospy.Publisher('/bot4_goal',Pose2D,queue_size=10)
        self.pose_bot1_sub=rospy.Subscriber('/bot1_waypoint_flag',Bool,self.flag1_callback,queue_size=1)
        self.pose_bot2_sub=rospy.Subscriber('/bot2_waypoint_flag',Bool,self.flag2_callback,queue_size=1)
        self.pose_bot3_sub=rospy.Subscriber('/bot3_waypoint_flag',Bool,self.flag3_callback,queue_size=1)
        self.pose_bot4_sub=rospy.Subscriber('/bot4_waypoint_flag',Bool,self.flag4_callback,queue_size=1)
    def flag1_callback(self,msg):
        if(self.mission_bot==1):
            self.count=self.count+1
            self.goal()
            print('count addition from 1')

    def flag2_callback(self,msg):
        if(self.mission_bot==2):
            self.count=self.count+1
            self.goal()
            print('count addition from 2')

    def flag3_callback(self,msg):
        if(self.mission_bot==3):
            self.count=self.count+1
            self.goal()
            print('count addition from 3')

    def flag4_callback(self,msg):
        if(self.mission_bot==4):
            self.count=self.count+1
            self.goal()

    def flipmotor(self,bot_num):
    	print('fliiping pracel for bot', bot_num )
    	time.sleep(self.time_for_flipping)

    def pub_msg(self):
        msg1=Pose2D()
        msg2=Pose2D()
        msg3=Pose2D()
        msg0=Pose2D()

        msg0.x=self.goal_pose[0][0]
        msg0.y=self.goal_pose[0][1]
        msg0.theta=self.goal_pose[0][2]

        msg1.x=self.goal_pose[1][0]
        msg1.y=self.goal_pose[1][1]
        msg1.theta=self.goal_pose[1][2]

        msg2.x=self.goal_pose[2][0]
        msg2.y=self.goal_pose[2][1]
        msg2.theta=self.goal_pose[2][2]

        msg3.x=self.goal_pose[3][0]
        msg3.y=self.goal_pose[3][1]
        msg3.theta=self.goal_pose[3][2]

        self.pub_bot1_goal.publish(msg0)
        self.pub_bot2_goal.publish(msg1)
        self.pub_bot3_goal.publish(msg2)
        self.pub_bot4_goal.publish(msg3)

    def goal(self):
        if self.count==0:
            self.goal_pose[0][0]=-0.53
        elif self.count==1:
            self.goal_pose[0][1]=1.27
        elif self.count==2:
            self.flipmotor(1)
            self.goal_pose[0][1]=0.2286
        elif self.count==3:
            self.goal_pose[0][0]=0.6858

        elif self.count==4:
            self.mission_bot=self.mission_bot+1
            self.goal_pose[1][0]=-0.68
        elif self.count==5:
            self.goal_pose[1][1]=1.27
        elif self.count==6:
            self.flipmotor(2)
            self.goal_pose[1][1]=0.0762
        elif self.count==7:
            self.goal_pose[1][0]=0.6858

        elif self.count==8:
            self.mission_bot=self.mission_bot+1
            self.goal_pose[2][0]=-0.68
        elif self.count==9:
            self.goal_pose[2][1]=-1.27
        elif self.count==10:
            self.flipmotor(3)
            self.goal_pose[2][1]=-0.0762
        elif self.count==11:
            self.goal_pose[2][0]=0.6858

        elif self.count==12:
            self.mission_bot=self.mission_bot+1
            self.goal_pose[3][0]=-0.53
        elif self.count==13:
            self.goal_pose[3][1]=-1.27
        elif self.count==14:
            self.flipmotor(4)
            self.goal_pose[3][1]=-0.2286
        elif self.count==15:
            self.goal_pose[3][0]=0.6858
        self.pub_msg()
        print('goal sent| count:', self.count)




if __name__ == '__main__':
    rospy.init_node('goal_publisher')
    try:
        while (rospy.get_time()==0):
          pass
        rospy.loginfo("goal_publisher created | now goal for each bot will be published on goal topics")
        goal_pub_obj=goal_publisher()
        time.sleep(20)
        goal_pub_obj.goal()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass