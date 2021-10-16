#!/usr/bin/env python

import rospy
import time
from bot_control.msg import StartGoal
from geometry_msgs.msg import Point
# from drdo_exploration.msg import teleopData
class pose_publisher:
    def __init__(self):
        self.pub_test=rospy.Publisher('/goal_point',Point,queue_size=1)
        self.data=Point()
        self.data.x=203.0
        self.data.y=450.0
        self.data.z=0
        # self.data.start_y=[6,7,5,0]
        # self.data.goal_x=[2,0,5,0]
        # self.data.goal_y=[4,2,2,7]
        # self.data.bot_num=[0,1,2,3]
        print(self.data)
        while not rospy.is_shutdown(): 
            # time.sleep(10) 
            self.pub_test.publish(self.data)


if __name__ == '__main__':
    rospy.init_node('test_node')
    # rospy.loginfo('reading from camera')
    pose_pub_obj=pose_publisher()
    #to keep the function running in loop
    rospy.spin()