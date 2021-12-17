#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Int16
import requests

# from drdo_exploration.msg import teleopData
class pose_publisher:
    def __init__(self):
        self.pub_test=rospy.Publisher('/test_com',Int16,queue_size=1)
        self.msg=Int16()
        self.msg.data=1;
        self.rate=rospy.Rate(10)
        self.url='http://192.168.0.250/'
        rospy.Subscriber('/test_com',Int16,self.callback)
        time.sleep(2)
        print("publihsed to cbs")
            # time.sleep(2)
    def callback(self,msg):
        print(msg.data)
        x = requests.post(self.url, data=str(msg.data))
        print(x.text)
    def pubjj(self):
        while not rospy.is_shutdown():
            self.msg.data+=1
            self.msg.data=self.msg.data%10
            self.pub_test.publish(self.msg)
            self.rate.sleep()
if __name__ == '__main__':
    rospy.init_node('test_node')
    # rospy.loginfo('reading from camera')
    pose_pub_obj=pose_publisher()
    pose_pub_obj.pubjj()
    #to keep the function running in loop
    # rospy.spin()