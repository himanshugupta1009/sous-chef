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

def ImageCallback(img_data):
	global bridge

	try:
		cv_image = bridge.imgmsg_to_cv2(img_data,'bgr8')
	except CvBridgeError as e:
		print(e)

	cv2.imshow('Camera Image', cv_image)

	qrDecoder = cv2.QRCodeDetector()

	data, bbox, rectifiedImage = qrDecoder.detectAndDecode(cv_image)

	if len(data) > 0:
		print("Decoded Data: {}".format(data))
		display(cv_image, bbox)
		rectifiedImage = np.uint8(rectifiedImage)
		cv2.imshow("Rectified QRCode", rectifiedImage)
	else:
		print("QR Code not detected")
		cv2.imshow("Results", cv_image)

	cv2.waitKey(3)

	rospy.sleep(0.5)

def display(image, bbox):
	n = len(bbox)
	for j in range(n):
		cv2.line(image, tuple(bbox[j][0]), tuple(bbox[(j+1) % n][0]),(255,0,0),3)

	cv2.imshow("Results",image)


def main():

	#if len(sys.argv) > 1:
		#input_image = cv2.imread(sys.argv[1])

	rospy.init_node('sensor_img_listner', anonymous=True)
	topic = '/camera/color/image_raw'
	camera_sub = rospy.Subscriber(topic,Image,ImageCallback)

	rospy.spin()

	cv2.destroyAllWindows()

if __name__ == '__main__':
	bridge = CvBridge()
	main()
