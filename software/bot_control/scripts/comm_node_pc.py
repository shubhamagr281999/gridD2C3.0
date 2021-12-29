#!/usr/bin/env python

#start server assignment from 240-249 it is being done because the last digit of 24x will represent bot number

import rospy
from std_msgs.msg import UInt8
from geometry_msgs.msg import PoseArray
import requests

class communication:
    def __init__(self):
        self.wheel_status= rospy.Subscriber('/wheel_speed',PoseArray,self.wheel_speed_callback, queue_size=10)
        self.flipper_status= rospy.Subscriber('/flipper',UInt8,self.flipper_callback, queue_size=10)
        self.rate=rospy.Rate(10)
        self.url='http://192.168.0.24'

       
    def wheel_speed_callback(self,msg):
        for i in range(1): #change 1 to self.n_agents ****
            urlt = self.url+str(i)+"/" 
            self.pub_string = str(msg.poses[i].position.x) + "," + str(msg.poses[i].position.y) + "," + str(msg.poses[i].position.z) + "0"
            requests.post(urlt, data= self.pub_string)
            print(self.pub_string)

    def flipper_callback(self,msg):
        urlt = self.url+str(msg.data)+"/" 
        self.pub_string = "0,0,0,1"
        requests.post(urlt, data= self.pub_string)
        print(self.pub_string)

if __name__ == '__main__':
    rospy.init_node('comm_node')
    rospy.loginfo('communication script started')
    communication_obj=communication()
    rospy.spin()