#!/usr/bin/env python

import rospy
import time
from time import sleep
from tf.transformations import euler_from_quaternion, quaternion_from_euler , euler_matrix
from geometry_msgs.msg import Pose2D

class goal_publisher:
    def __init__(self):
        self.rate_move=rospy.Rate(0.5)
        self.current_pose=[]
        self.goal_pose=[]
        self.time_for_flipping=
        self.count=0
        self.pose_error=
        self.theta_error=
        self.goal1=Pose2D()
        self.goal2=Pose2D()
        self.goal3=Pose2D()
        self.goal4=Pose2D()
        self.pub_bot1_goal=rospy.Publisher('/bot1_goal',Pose2D,queue_size=10)
        self.pub_bot2_goal=rospy.Publisher('/bot2_goal',Pose2D,queue_size=10)
        self.pub_bot3_goal=rospy.Publisher('/bot3_goal',Pose2D,queue_size=10)
        self.pub_bot4_goal=rospy.Publisher('/bot1_goal',Pose2D,queue_size=10)
        self.pose_bot1_sub=rospy.Subscriber('/bot1_pose',Pose2D,self.pose1_callback,queue_size=10)
        self.pose_bot2_sub=rospy.Subscriber('/bot2_pose',Pose2D,self.pose2_callback,queue_size=10)
        self.pose_bot3_sub=rospy.Subscriber('/bot3_pose',Pose2D,self.pose3_callback,queue_size=10)
        self.pose_bot4_sub=rospy.Subscriber('/bot4_pose',Pose2D,self.pose4_callback,queue_size=10)
    def pose1_callback(self,msg):
    	self.current_pose(1,1)=msg.x
    	self.current_pose(1,2)=msg.y
    	self.current_pose(1,3)=msg.theta
	def pose2_callback(self,msg):
    	self.current_pose(2,1)=msg.x
    	self.current_pose(2,2)=msg.y
    	self.current_pose(2,3)=msg.theta
	def pose3_callback(self,msg):
    	self.current_pose(3,1)=msg.x
    	self.current_pose(3,2)=msg.y
    	self.current_pose(3,3)=msg.theta
	def pose4_callback(self,msg):
    	self.current_pose(4,1)=msg.x
    	self.current_pose(4,2)=msg.y
    	self.current_pose(4,3)=msg.theta
    def flipmotor(self,bot_num):
    	rospy.loginfo('fliiping pracel for bot', bot_num )
    	time.sleep(self.time_for_flipping)
    def goal(self):
    	time.sleep(5)
    	while not rospy.is_shutdown(): 
    		if self.count==0:
    			self.goal_pose(1,1)=
    			self.goal_pose(1,2)=
    			self.goal_pose(1,3)=

    			self.goal_pose(2,1)=self.current_pose(2,1)
    			self.goal_pose(2,2)=self.current_pose(2,2)
    			self.goal_pose(2,3)=self.current_pose(2,3)

    			self.goal_pose(3,1)=self.current_pose(3,1)
    			self.goal_pose(3,2)=self.current_pose(3,2)
    			self.goal_pose(3,3)=self.current_pose(3,3)

    			self.goal_pose(4,1)=self.current_pose(4,1)
    			self.goal_pose(4,2)=self.current_pose(4,2)
    			self.goal_pose(4,3)=self.current_pose(4,3)
    			if (abs(self.goal_pose(1,1)-self.current_pose(1,1)))<self.pose_error and (abs(self.goal_pose(1,3)-self.current_pose(1,3)))<self.theta_error and (abs(self.goal_pose(1,2)-self.current_pose(1,2)))<self.pose_error :
    				self.flipmotor(self,1)
    				self.count=1

    		elif self.count==1:
    			self.goal_pose(1,1)=
    			self.goal_pose(1,2)=
    			self.goal_pose(1,3)=

    			self.goal_pose(2,1)=self.current_pose(2,1)
    			self.goal_pose(2,2)=self.current_pose(2,2)
    			self.goal_pose(2,3)=self.current_pose(2,3)

    			self.goal_pose(3,1)=self.current_pose(3,1)
    			self.goal_pose(3,2)=self.current_pose(3,2)
    			self.goal_pose(3,3)=self.current_pose(3,3)

    			self.goal_pose(4,1)=self.current_pose(4,1)
    			self.goal_pose(4,2)=self.current_pose(4,2)
    			self.goal_pose(4,3)=self.current_pose(4,3)
    			if (abs(self.goal_pose(1,1)-self.current_pose(1,1)))<self.pose_error and (abs(self.goal_pose(1,3)-self.current_pose(1,3)))<self.theta_error and (abs(self.goal_pose(1,2)-self.current_pose(1,2)))<self.pose_error :
    				self.count=2

			if self.count==2:
    			self.goal_pose(2,1)=
    			self.goal_pose(2,2)=
    			self.goal_pose(2,3)=

    			self.goal_pose(1,1)=self.current_pose(1,1)
    			self.goal_pose(1,2)=self.current_pose(1,2)
    			self.goal_pose(1,3)=self.current_pose(1,3)

    			self.goal_pose(3,1)=self.current_pose(3,1)
    			self.goal_pose(3,2)=self.current_pose(3,2)
    			self.goal_pose(3,3)=self.current_pose(3,3)

    			self.goal_pose(4,1)=self.current_pose(4,1)
    			self.goal_pose(4,2)=self.current_pose(4,2)
    			self.goal_pose(4,3)=self.current_pose(4,3)
    			if (abs(self.goal_pose(2,1)-self.current_pose(2,1)))<self.pose_error and (abs(self.goal_pose(2,3)-self.current_pose(2,3)))<self.theta_error and (abs(self.goal_pose(2,2)-self.current_pose(2,2)))<self.pose_error :
    				self.flipmotor(self,2)
    				self.count=3

    		elif self.count==3:
    			self.goal_pose(2,1)=
    			self.goal_pose(2,2)=
    			self.goal_pose(2,3)=

    			self.goal_pose(1,1)=self.current_pose(1,1)
    			self.goal_pose(1,2)=self.current_pose(1,2)
    			self.goal_pose(1,3)=self.current_pose(1,3)

    			self.goal_pose(3,1)=self.current_pose(3,1)
    			self.goal_pose(3,2)=self.current_pose(3,2)
    			self.goal_pose(3,3)=self.current_pose(3,3)

    			self.goal_pose(4,1)=self.current_pose(4,1)
    			self.goal_pose(4,2)=self.current_pose(4,2)
    			self.goal_pose(4,3)=self.current_pose(4,3)
    			if (abs(self.goal_pose(2,1)-self.current_pose(2,1)))<self.pose_error and (abs(self.goal_pose(2,3)-self.current_pose(2,3)))<self.theta_error and (abs(self.goal_pose(2,2)-self.current_pose(2,2)))<self.pose_error :
    				self.count=4

			if self.count==4:
    			self.goal_pose(3,1)=
    			self.goal_pose(3,2)=
    			self.goal_pose(3,3)=

    			self.goal_pose(1,1)=self.current_pose(1,1)
    			self.goal_pose(1,2)=self.current_pose(1,2)
    			self.goal_pose(1,3)=self.current_pose(1,3)

    			self.goal_pose(2,1)=self.current_pose(2,1)
    			self.goal_pose(2,2)=self.current_pose(2,2)
    			self.goal_pose(2,3)=self.current_pose(2,3)

    			self.goal_pose(4,1)=self.current_pose(4,1)
    			self.goal_pose(4,2)=self.current_pose(4,2)
    			self.goal_pose(4,3)=self.current_pose(4,3)
    			if (abs(self.goal_pose(3,1)-self.current_pose(3,1)))<self.pose_error and (abs(self.goal_pose(3,3)-self.current_pose(3,3)))<self.theta_error and (abs(self.goal_pose(3,2)-self.current_pose(3,2)))<self.pose_error :
    				self.flipmotor(self,3)
    				self.count=5

    		elif self.count==5:
    			self.goal_pose(3,1)=
    			self.goal_pose(3,2)=
    			self.goal_pose(3,3)=

    			self.goal_pose(1,1)=self.current_pose(1,1)
    			self.goal_pose(1,2)=self.current_pose(1,2)
    			self.goal_pose(1,3)=self.current_pose(1,3)

    			self.goal_pose(2,1)=self.current_pose(2,1)
    			self.goal_pose(2,2)=self.current_pose(2,2)
    			self.goal_pose(2,3)=self.current_pose(2,3)

    			self.goal_pose(4,1)=self.current_pose(4,1)
    			self.goal_pose(4,2)=self.current_pose(4,2)
    			self.goal_pose(4,3)=self.current_pose(4,3)
    			if (abs(self.goal_pose(3,1)-self.current_pose(3,1)))<self.pose_error and (abs(self.goal_pose(3,3)-self.current_pose(3,3)))<self.theta_error and (abs(self.goal_pose(3,2)-self.current_pose(3,2)))<self.pose_error :
    				self.count=6
			if self.count==6:
    			self.goal_pose(4,1)=
    			self.goal_pose(4,2)=
    			self.goal_pose(4,3)=

    			self.goal_pose(1,1)=self.current_pose(1,1)
    			self.goal_pose(1,2)=self.current_pose(1,2)
    			self.goal_pose(1,3)=self.current_pose(1,3)

    			self.goal_pose(2,1)=self.current_pose(2,1)
    			self.goal_pose(2,2)=self.current_pose(2,2)
    			self.goal_pose(2,3)=self.current_pose(2,3)

    			self.goal_pose(3,1)=self.current_pose(3,1)
    			self.goal_pose(3,2)=self.current_pose(3,2)
    			self.goal_pose(3,3)=self.current_pose(3,3)
    			if (abs(self.goal_pose(4,1)-self.current_pose(4,1)))<self.pose_error and (abs(self.goal_pose(4,3)-self.current_pose(4,3)))<self.theta_error and (abs(self.goal_pose(4,2)-self.current_pose(4,2)))<self.pose_error :
    				self.flipmotor(self,4)
    				self.count=7

    		elif self.count==7:
    			self.goal_pose(4,1)=
    			self.goal_pose(4,2)=
    			self.goal_pose(4,3)=

    			self.goal_pose(1,1)=self.current_pose(1,1)
    			self.goal_pose(1,2)=self.current_pose(1,2)
    			self.goal_pose(1,3)=self.current_pose(1,3)

    			self.goal_pose(2,1)=self.current_pose(2,1)
    			self.goal_pose(2,2)=self.current_pose(2,2)
    			self.goal_pose(2,3)=self.current_pose(2,3)

    			self.goal_pose(3,1)=self.current_pose(3,1)
    			self.goal_pose(3,2)=self.current_pose(3,2)
    			self.goal_pose(3,3)=self.current_pose(3,3)
    			if (abs(self.goal_pose(4,1)-self.current_pose(4,1)))<self.pose_error and (abs(self.goal_pose(4,3)-self.current_pose(4,3)))<self.theta_error and (abs(self.goal_pose(4,2)-self.current_pose(4,2)))<self.pose_error :
    				self.count=8
    		else:
    			rospy.loginfo('task accomplished')
    		self.goal1.x=self.goal_pose(1,1)
    		self.goal1.y=self.goal_pose(1,2)
    		self.goal1.theta=self.goal_pose(1,3)

    		self.goal2.x=self.goal_pose(2,1)
    		self.goal2.y=self.goal_pose(2,2)
    		self.goal2.theta=self.goal_pose(2,3)

    		self.goal3.x=self.goal_pose(3,1)
    		self.goal3.y=self.goal_pose(3,2)
    		self.goal3.theta=self.goal_pose(3,3)

    		self.goal4.x=self.goal_pose(4,1)
    		self.goal4.y=self.goal_pose(4,2)
    		self.goal4.theta=self.goal_pose(4,3)

    		self.rate.sleep()
        


if __name__ == '__main__':
    rospy.init_node('goal_publisher')
    try:
        while (rospy.get_time()==0):
          pass
        rospy.loginfo("goal_publisher created | now goal for each bot will be published on goal topics")
        goal_pub_obj=goal_publisher()
        goal_pub_obj.goal()

    except rospy.ROSInterruptException:
        pass