import rospy
import cv2
from math import sqrt, atan, pi
import numpy as np
from geometry_msgs.msg import Point
from bot_control.msg import Poses

class pose_publisher:
    def __init__(self):
        self.n_agents=1
        self.current_pose=np.zeros([self.n_agents,3])+1
        self.poses=Poses()
        self.initialize_pose()
        self.pub_poses=rospy.Publisher('/poses',Poses,queue_size=1)
        self.current_pose=[[85.28,299.57,0]]
        self.poses.posei[0].x=self.current_pose[0][0]
        self.poses.posei[0].y=self.current_pose[0][1]
        self.poses.posei[0].z=self.current_pose[0][2]
        # print(self.data)
        while not rospy.is_shutdown(): 
            self.pub_poses.publish(self.poses)
            print("publihsed to estimate")
            time.sleep(2)

    def initialize_pose(self):
        temp_poses=[]
        for i in range(self.n_agents):
            temp_poses.append(Point())
        self.poses.posei=temp_poses

if __name__ == '__main__':
    rospy.init_node('pose_estimator')
    rospy.loginfo('reading from camera')
    pose_pub_obj=pose_publisher()
    #to keep the function running in loop
    rospy.spin()