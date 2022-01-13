#!/usr/bin/env python

import rospy
import cv2
from time import sleep
from math import sqrt, atan, pi
import numpy as np
from bot_control.msg import pose_bot

class goal_publisher:
    def __init__(self):
        self.n_agents=1
        self.control_rate=rospy.Rate(10)
        self.goal_pose=np.zeros([self.n_agents,3])
        self.goal_pose[0] = [5,3,pi/2]
        self.pub_goal=rospy.Publisher('/goal_point',pose_bot,queue_size=10)
        msg=pose_bot()
        bot_num = 0 #considering only 1 bot for testing
        msg.x=self.goal_pose[bot_num][0]
        msg.y=self.goal_pose[bot_num][1]
        msg.yaw= self.goal_pose[bot_num][2]
        msg.bot_num=bot_num
        sleep(2)
        self.pub_goal.publish(msg)



if __name__ == '__main__':
    rospy.init_node('goal_publisher')
    rospy.loginfo('publishing to the topic /goal_point')
    goal_pub_obj=goal_publisher()
    #to keep the function running in loop
    rospy.spin()
