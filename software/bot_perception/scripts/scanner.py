#!/usr/bin/env python

import rospy
import imutils
from sensor_msgs.msg import Image
import numpy as np
import cv2
import matplotlib.pyplot as plt
from math import sqrt
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose2D

def callback_opencv(data):
	bridge = CvBridge()
	img = bridge.imgmsg_to_cv2(data, "bgr8")
	arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_50)
	arucoParams = cv2.aruco.DetectorParameters_create()
	(corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict,parameters=arucoParams)

	#now starting to localise bot wrt to the ids
	print(ids)
	while ()
	cX = 0
	cY = 0

if __name__ == '__main__':

	 rospy.init_node('pose_estimator', anonymous=True)
	 rospy.Subscriber("/arena/arena1/camera1/image_raw", Image, callback_opencv)
	 
	 rospy.spin()