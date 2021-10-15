#!/usr/bin/env python

import rospy
import time
from bot_control.msg import StartGoal
# from drdo_exploration.msg import teleopData
class pose_publisher:
    def __init__(self):
        self.pub_test=rospy.Publisher('/start_goal_agents',StartGoal,queue_size=1)
        self.data=StartGoal()
        self.data.start_x=[0,0,13,8,8]
        self.data.start_y=[6,7,5,0,3]
        self.data.goal_x=[2,0,5,0,0]
        self.data.goal_y=[4,2,2,7,11]
        self.data.bot_num=[0,1,2,3,4]
        print(self.data)
        while not rospy.is_shutdown(): 
            time.sleep(1) 
            self.pub_test.publish(self.data)


if __name__ == '__main__':
    rospy.init_node('pose_estimator')
    rospy.loginfo('reading from camera')
    pose_pub_obj=pose_publisher()
    #to keep the function running in loop
    rospy.spin()