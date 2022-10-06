#!/usr/bin/env python2

# Python code for Multiple Color Detection
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
from sofar_assignment.srv import DetectColor
import time

# Capturing video through webcam
#webcam = cv2.VideoCapture(0)

img = None
bridge = CvBridge()
sub = rospy.Subscriber("/xtion/rgb/image_raw", Image )

# Start a while loop
def callbackFunc(req):
	
	# Reading the video from the
	# webcam in image frames
	global img
	

	try:
		imageFrame = bridge.imgmsg_to_cv2(img, "passthrough")
	except CvBridgeError as e:
		rospy.logerr("Cv bridge error: {0}".format(e))
	
	"""
	cv2.namedWindow("Image window",1)
	cv2.imshow("Image window", imageFrame)
	cv2.waitKey(10)
	"""

	#_, imageFrame = img_msg.header
	# rospy.loginfo(img_msg.header)
	# Convert the imageFrame in
	# BGR(RGB color space) to
	# HSV(hue-saturation-value)
	# color space
	
	hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

	
	# Set range for blue color and
	# define mask
	blue_lower = np.array([94, 80, 2], np.uint8)
	blue_upper = np.array([120, 255, 255], np.uint8)
	blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
	#cv2.imshow('image window name', imageFrame)
	#cv2.imshow('mask window name', blue_mask)
	#cv2.waitKey(10)


	# Set range for green color and
	# define mask
	green_lower = np.array([25, 52, 72], np.uint8)
	green_upper = np.array([102, 255, 255], np.uint8)
	green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)
	

	# Morphological Transform, Dilation
	# for each color and bitwise_and operator
	# between imageFrame and mask determines
	# to detect only that particular color
	kernal = np.ones((5, 5), "uint8")

	# For blue color
	blue_mask = cv2.dilate(blue_mask, kernal)
	res_blue = cv2.bitwise_and(imageFrame, imageFrame,
							mask = blue_mask)
	
	# For green color
	green_mask = cv2.dilate(green_mask, kernal)
	res_green = cv2.bitwise_and(imageFrame, imageFrame,
								mask = green_mask)
	
	

	
	# Creating contour to track green color
	contours, hierarchy = cv2.findContours(green_mask,
										cv2.RETR_TREE,
										cv2.CHAIN_APPROX_SIMPLE)[-2:]
	
	for pic, contour in enumerate(contours):
		area = cv2.contourArea(contour)
		if(area > 300):
			x, y, w, h = cv2.boundingRect(contour)
			imageFrame = cv2.rectangle(imageFrame, (x, y),
									(x + w, y + h),
									(0, 255, 0), 2)
			
			cv2.putText(imageFrame, "Green Colour", (x, y),
						cv2.FONT_HERSHEY_SIMPLEX,
						1.0, (0, 255, 0))
			# Program Termination
			cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
			#cv2.waitKey(30)
			#time.sleep(5)
			if cv2.waitKey(10) & 0xFF == ord('q'): ################## I BELIEVE THAT being called once it doesnt manage to show the image, if we don t solve, cancel this part of the code showing rectangles
				cap.release()
				cv2.destroyAllWindows()

			return True

	# Creating contour to track blue color
	contours, hierarchy = cv2.findContours(blue_mask,
										cv2.RETR_TREE,
										cv2.CHAIN_APPROX_SIMPLE)[-2:]
	for pic, contour in enumerate(contours):
		area = cv2.contourArea(contour)
		if(area > 300):
			x, y, w, h = cv2.boundingRect(contour)
			imageFrame = cv2.rectangle(imageFrame, (x, y),
									(x + w, y + h),
									(255, 0, 0), 2)
			
			cv2.putText(imageFrame, "Blue Colour", (x, y),
						cv2.FONT_HERSHEY_SIMPLEX,
						1.0, (255, 0, 0))
			# Program Termination
			cv2.imshow("Multiple Color Detection in Real-TIme", imageFrame)
			if cv2.waitKey(10) & 0xFF == ord('q'): ##################
				cap.release()
				cv2.destroyAllWindows()

			return False
	
	

def imageCallback(img_msg):
	global img
	img = img_msg
		

def setup_colorDetect_server():
    
	rospy.init_node("color_detect_server_node")
	rospy.Subscriber("/xtion/rgb/image_raw", Image,  imageCallback)
	rospy.Service("color_detect_service", DetectColor, callbackFunc)

	print("COLOR DETECTOR SERVER READY TO PROCESS CLIENT REQUEST")
	rospy.spin()

if __name__=='__main__':

	setup_colorDetect_server()
    #rospy.init_node("color_detection_node")

    





