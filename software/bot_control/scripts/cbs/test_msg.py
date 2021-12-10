#!/usr/bin/env python

import rospy
import time
from bot_control.msg import StartGoal

# from drdo_exploration.msg import teleopData
class pose_publisher:
    def __init__(self):
        self.pub_test=rospy.Publisher('/start_goal_agents',StartGoal,queue_size=1)
        self.data=StartGoal()
        self.data.start_x=[0,0,13,8]
        self.data.start_y=[6,7,5,0]
        self.data.start_d=[0,0,0,0]
        self.data.goal_x=[2,0,5,0]
        self.data.goal_y=[4,2,2,7]
        self.data.goal_d=[0,0,0,0]
        self.data.bot_num=[0,1,2,3]

        self.data.start_x=[0]
        self.data.start_y=[6]
        self.data.goal_x=[2]
        self.data.goal_y=[4]
        self.data.bot_num=[0]

        # print(self.data)
        # while not rospy.is_shutdown(): 
        self.pub_test.publish(self.data)
        print("publihsed to cbs")
        time.sleep(2)
        self.pub_test.publish(self.data)
        print("publihsed to cbs")
            # time.sleep(2)


if __name__ == '__main__':
    rospy.init_node('test_node')
    # rospy.loginfo('reading from camera')
    pose_pub_obj=pose_publisher()
    #to keep the function running in loop
    # rospy.spin()