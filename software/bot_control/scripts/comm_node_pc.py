#!/usr/bin/env python3

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
        self.url='http://192.168.68.24'
        self.scaling_factor = 1


    def wheel_speed_callback(self,msg):
        for i in range(1): #change 1 to self.n_agents ****
            urlt = self.url+str(i)+"/"
            if(msg.poses[i].position.x>=0):
                msg.poses[i].position.x += 35
            else:
                msg.poses[i].position.x -= 35

            if(msg.poses[i].position.y>=0):
                msg.poses[i].position.y += 35
            else:
                msg.poses[i].position.y -= 35

            if(msg.poses[i].position.z>=0):
                msg.poses[i].position.z += 35
            else:
                msg.poses[i].position.z -= 35

            if (abs(msg.poses[i].position.x) <= 45) :
                msg.poses[i].position.x = 0
            if (abs(msg.poses[i].position.y) <= 45) :
                msg.poses[i].position.y = 0
            if (abs(msg.poses[i].position.z) <= 45) :
                msg.poses[i].position.z = 0
            self.pub_string = str(msg.poses[i].position.x * self.scaling_factor) + "," + str(msg.poses[i].position.y * self.scaling_factor) + "," + str(msg.poses[i].position.z * self.scaling_factor) + "0"
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
