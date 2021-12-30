#!/usr/bin/env python

import rospy
import time
from bot_control.msg import StartGoal, pose_bot

# from drdo_exploration.msg import teleopData
class pose_publisher:
    def __init__(self):
        self.pub_test=rospy.Publisher('/start_goal_agents',StartGoal,queue_size=1)
        # self.pub_check=rospy.Publisher('/check',pose_bot,queue_size=1)
        self.data=StartGoal()
        # self.msg=pose_bot()
        # self.msg.x=1.0
        # self.msg.y=2.0
        # self.msg.yaw=2.0
        # self.msg.bot_num=1
        self.data.start_x=[0,0]
        self.data.start_y=[9,4]
        self.data.start_d=[0,0]
        self.data.goal_x=[2,9]
        self.data.goal_y=[9,3]
        self.data.goal_d=[0,0]
        self.data.bot_num=[0,1]


        # print(self.data)
        while not rospy.is_shutdown(): 
            # self.pub_check.publish(self.msg)
            self.pub_test.publish(self.data)
            print("publihsed to cbs")
            time.sleep(2)
            # time.sleep(2)


if __name__ == '__main__':
    rospy.init_node('test_node')
    # rospy.loginfo('reading from camera')
    pose_pub_obj=pose_publisher()
    #to keep the function running in loop
    # rospy.spin()