#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import sys
import time
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

bridge = None
initial_img = None
prev_bbox = None

def ImageCallback(img_data):
	global bridge, initial_img, prev_bbox

	try:
		cv_image = bridge.imgmsg_to_cv2(img_data,'bgr8')
	except CvBridgeError as e:
		print(e)

	qrDetector = cv2.QRCodeDetector()
	data, bbox = qrDetector.detect(cv_image)

	#data, bbox, rectifiedImage = qrDetector.detectAndDecode(cv_image)

	if data:
		if initial_img is None:
			initial_img = cv_image
			prev_bbox = bbox
			#cv2.imshow("Initial Image",initial_img)
		else:
			initial_img = draw_traj(initial_img, bbox, prev_bbox)
			prev_bbox = bbox
			cv2.imshow("Initial Img with traj",initial_img)

		print("QR Code Detected")
		display(cv_image, bbox)

		#code below would display a QR tag image and the content of the QR tag
		#print("Decoded Data: {}".format(data))
		#rectifiedImage = np.uint8(rectifiedImage)
		#cv2.imshow("Rectified QRCode", rectifiedImage)
	else:
		print("QR Code Not detected")
		cv2.imshow('Camera Image', cv_image)

	cv2.waitKey(3)
	#rospy.sleep(0.5)

def display(image, bbox):
	n = len(bbox)
	for j in range(n):
		cv2.line(image, tuple(bbox[j][0]), tuple(bbox[(j+1) % n][0]),(255,0,0),3)

	cv2.imshow("QRCode Results",image)

def draw_traj(img,bbox,prev_bbox):
	
	#prev_x = int((prev_bbox[0][0][0] + prev_bbox[2][0][0])/2)
	#prev_y = int((prev_bbox[0][0][0] + prev_bbox[2][0][1])/2)
	#curr_x = int((bbox[0][0][0] + bbox[2][0][0])/2)
	#curr_y = int((bbox[0][0][1] + bbox[2][0][1])/2)

	prev_x = prev_bbox[0][0][0]
	prev_y = prev_bbox[0][0][1]
	curr_x = bbox[0][0][0]
	curr_y = bbox[0][0][1]


	cv2.arrowedLine(img,(prev_x,prev_y),(curr_x,curr_y),(0,0,255),1)

	return img


def main():

	rospy.init_node('sensor_img_listner', anonymous=True)
	topic = '/camera/color/image_raw'

	camera_sub = rospy.Subscriber(topic,Image,ImageCallback)

	rospy.spin()

	cv2.destroyAllWindows()

if __name__ == '__main__':
	bridge = CvBridge()
	main()
