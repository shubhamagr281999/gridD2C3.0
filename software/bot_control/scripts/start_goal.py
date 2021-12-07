import rospy
import time
from time import sleep
from math import sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler , euler_matrix
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int16

from bot_control.msg import StartGoal, CompletePlan, PathArray, Poses
from geometry_msgs.msg import Point
import numpy as np

class starat_goal_publisher:
    def __init__(self):
        self.rate=rospy.Rate(0.5)
        self.n_agents=1
        self.current_pose=np.zeros([self.n_agents,3])
        self.goal_pose=np.zeros([self.n_agents,3])
        self.poses_sub=rospy.Subscriber('/poses',Poses,self.pose_callback,queue_size=10)
        self.pub_goal=rospy.Publisher('/goal_point',Point,queue_size=10)

    def pose_callback(self,msg):
        for i in range(self.n_agents):
            self.current_pose[i][0]=msg.posei[i].x
            self.current_pose[i][1]=msg.posei[i].y
            self.current_pose[i][2]=msg.posei[i].z
    def go_to_goal(self,bot_num,x,y):
        msg=Point()
        msg.x=(self.x)
        msg.y=self.y
        msg.z=bot_num
        self.pub_goal.publish(msg)



if __name__ == '__main__':
    rospy.init_node('Start_goal_publisher')
    rospy.loginfo("Start goal_publisher created | decides gaol based on the logic defined")
    start_goal_pub_obj=start_goal_publisher()
    # rospy.spin()
