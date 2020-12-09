#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from multiprocessing import Process, Queue, Lock	#parallel function usage by multiprocessing, sharing data by Queue
import socket
import time

# Mutex init
#lock = Lock()
# Format bridge for images
BRIDGE = CvBridge()
# Queue
#p = Queue(5)

# auxiliary functions
def recvall(sock, count):
	buf = b''
	while count:
		newbuf = sock.recv(count)
		if not newbuf: return None
		buf += newbuf
		count -= len(newbuf)
	return buf

def frameToMsg(frame, stamp):
	msg = BRIDGE.cv2_to_imgmsg(frame, "bgra8")
	msg.header.stamp = stamp
	return msg

# python client
def radar_interface():

	# server
	global sock, sockClient

	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
	sock.bind(('192.168.30.150', 4388)) # my adr
	sock.listen(5)	
	sockClient, addr = sock.accept()

def receive(q, lock):
	global img
	rospy.init_node('scanimg_receive', anonymous=True)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		# note that the size of socket buffer is far less than the size of images
		
		lock.acquire()
		stringData = recvall(sockClient, 512*512*4)
		np_data = np.fromstring(stringData, dtype='uint8')
		img = np_data.reshape(512,512,4)
		lock.release()
		q.put(img)

		rate.sleep()

		
		#cv2.imshow('img',img)
		#cv2.waitKey(100)

	rospy.spin()

def main(q, lock):

	pub = rospy.Publisher('radar_image', Image, queue_size = 1)
	rospy.init_node('radar_interface_node', anonymous = True)


	while not rospy.is_shutdown():

		try:
			img = q.get()
		except Queue.Empty:
			pass
		cv2.imshow('img',img)
		msgRadar = frameToMsg(img, rospy.Time.now())
		pub.publish(msgRadar)
		rospy.loginfo('publishing')


		#rate.sleep() #the while loop will repeated with the set Hz

	rospy.spin()


if __name__ == '__main__':

	lock = Lock()
	q = Queue(5)

	preceive = Process(target = receive, args = (q, lock)) # receiving data from rugged-pc	
	pmain = Process(target = main, args = (q, lock)) # main loop to publish

	try:
		radar_interface()
		preceive.start() # to receive radar scan img via TCP comm.
		pmain.start() # to publish img message in a ROS interface

		preceive.join(1)
		pmain.join(1)			
	except KeyboardInterrupt:

		pass
