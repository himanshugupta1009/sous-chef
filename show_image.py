#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sys
import os

bridge = None
img_count = 0

def imageCallback(img_data):
	global bridge, img_count

	try:
		cv_image = bridge.imgmsg_to_cv2(img_data,'bgr8')
	except CvBridgeError as e:
		print(e)
		
	cv2.imshow('Display Image',cv_image)
	cv2.waitKey(3)
		
	os.chdir('/home/cassiecao/catkin_ws/src/realsense-ros/realsense2_camera/scripts/sensor_images')	
	
	filename = "my_image" + str(img_count) + ".png"
	cv2.imwrite(filename,cv_image)
	img_count += 1

	print("Image Saved Successfully!")
	
	rospy.sleep(1)

def main():
	rospy.init_node('sensor_img_listner',anonymous=True)
	topic = '/camera/color/image_raw'
	camera_sub = rospy.Subscriber(topic,Image,imageCallback)
	
	rospy.spin()
	
	cv2.destroyAllWindows()

if __name__ == '__main__':
	bridge = CvBridge()
	main()
