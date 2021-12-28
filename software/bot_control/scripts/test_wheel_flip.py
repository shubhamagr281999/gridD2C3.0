import rospy
import numpy as np
import time
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose, PoseArray,String
import requests
import urllib.request
import http
base = "http://192.168.0.250/"

class wheel_flip_publisher:
    def __init__(self):
        #self.wheel_status=None
        self.flip_wheel_status=np.zeros((self.n_agents,3))#replace 3 with the total variable fpr storing speed of wheel
        self.wheel_status= rospy.Subscriber('/wheel_speed',PoseArray,self.wheel_state_callback, queue_size=10)
        self.flipper_status= rospy.Subscriber('/flipper_cmd',String,self.callback, queue_size=10)
        #self.com_to_node=rospy.Subscribe()
    def wheel_state_callback(self,msg):
        for i in range(self.n_agents):
            self.flip_wheel_status[i][0]=msg.PoseArray[i][0]
            self.flip_wheel_status[i][1]=msg.PoseArray[i][1]
    def callback(self,msg):
        for i in range(self.n_agents):
            self.wheel_speed[i][2]=msg.String[i]
    def msg_nodemcu(self,arr):
        arr=np.array_str(arr)
        try:
            n = urllib.request.urlopen(base + arr).read()
            n = n.decode("utf-8")
            return n
        except http.client.HTTPException as e:
        return e


if __name__ == '__main__':
    rospy.init_node('node_wheel_fip')
    wheel_flip_cmd = wheel_flip_publisher() 
    wheel_flip_cmd.msg_nodemcu()

