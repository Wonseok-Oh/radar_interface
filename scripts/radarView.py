#!/usr/bin/env python

# -------------------- Imports -------------------- #
import rospy
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import sys

# -------------------- Definitions -------------------- #
NODE_NAME		= "radarView"
TOPIC			= None

NODE_PREFIX		= "~"
COMMAND_PREFIX	= "_"
VIEW_PARAM		= "view"
ERROR_NO_VIEW	= "A view parameter must be specified: camView " + COMMAND_PREFIX + VIEW_PARAM + ":=<view_topic>"

COLOR_ERR		= "\033[91m"
COLOR_END		= "\033[0m"

# Format bridge for images
BRIDGE = CvBridge()

# Subscriber parameters
QUEUE_SIZE		= 1
#BUFF_SIZE		= 2**26


# -------------------- Functions -------------------- #
def display(image):
	#print "Delay: ", (rospy.Time.now() - image.header.stamp).to_sec()
	frame = BRIDGE.imgmsg_to_cv2(image)
	cv2.imshow("Radar scan image", frame)
	cv2.waitKey(1)

def error(msg):
	print COLOR_ERR + msg + COLOR_END
	exit()

def init():
	rospy.init_node(NODE_NAME, anonymous=True)


# -------------------- MAIN -------------------- #
if __name__ == '__main__':
	init()
	rospy.Subscriber("radar_image", Image, display, queue_size=QUEUE_SIZE)

	try:	
		rospy.spin()
	except KeyboardInterrupt:
		print "Keyboard interrupt"

