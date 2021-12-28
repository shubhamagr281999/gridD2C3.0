import rospy
import time
from std_msgs.msg import Int16
import requests

class wheel_flip_publisher:
    def __init__(self):
        self.wheel_status=None
        self.wheel_status= rospy.Subscriber('/wheel_speed',wheel_speed,self.callback, queue_size=10)
        self.flipper_status= rospy.Subscriber('/flipper_cmd',flipper_cmd,self.callback, queue_size=10)
        rospy.Subscriber("chatter", String, callback)
    def callback(msg):
       rospy.loginfo(data.data)


if __name__='__main__':
    rospy.init_node('node_wheel_fip')
    wheel_flip_cmd = wheel_flip_publisher() 