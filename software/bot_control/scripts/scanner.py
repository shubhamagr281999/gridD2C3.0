import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2
import matplotlib.pyplot as plt
from math import sqrt
from std_msgs.msg import Int16
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped Pose2D

def callback_opencv(data):
	bridge = CvBridge()
	img = bridge.imgmsg_to_cv2(data, "bgr8")

	arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
	arucoParams = cv2.aruco.DetectorParameters_create()
		 
	img = cv2.medianBlur(img,3)
	(corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict,parameters=arucoParams)
	print(ids)
	cX = 0
	cY = 0

	#cv2.circle(img,(img.shape[1]//2,img.shape[0]//2),4,(255,0,0),-1)

	# aruco = Pose2D()
	# aruco.x = 0.0
	# aruco.y = 0.0
	# aruco.theta = 0.0

if __name__ == '__main__':

	 rospy.init_node('pose_estimator', anonymous=True)
	 rospy.Subscriber("/arena/arena1/camera1/image_raw/", Image, callback_opencv)
	 
	 rospy.spin()