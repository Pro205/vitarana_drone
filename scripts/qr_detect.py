#!/usr/bin/env python


'''
This is a boiler plate script that contains an example on how to subscribe a rostopic containing camera frames
and store it into an OpenCV image to use it further for image processing tasks.
Use this code snippet in your code or you can also continue adding your code in the same file
'''

from pyzbar.pyzbar import decode
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float32
from vitarana_drone.msg import *
import cv2
import numpy as np
import rospy

class image_proc():

	# Initialise everything
	def __init__(self):
		rospy.init_node('barcode_test') #Initialise rosnode


		#Declaring stuff
		self.qr_cord = qr_data()

		# Publishing services
		self.qr_pub = rospy.Publisher('/qr_cmd', qr_data, queue_size=1)

		#subscriber services
		self.image_sub = rospy.Subscriber("/edrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic

		self.img = np.empty([]) # This will contain your image frame from camera
		self.bridge = CvBridge()

		# Declaring stuff
		self.gps_package_cord = [0.0, 0.0, 0.0]



	# Callback function of camera topic
	def image_callback(self, data):
		try:
			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
		except CvBridgeError as e:
			print(e)
			return self.img

	def QR_detector(self):

		# 	# self.detector_qr = decode(self.img)
		# 	# self.QR_code =  cv2.imshow("QR", self.img)
		# 	# cv2.waitkey(1)
		#     # print(self.detector_qr.data.decode())
		# 	# print(self.QR_code)
		self.data = cv2.imshow("result", self.img)
		#self.data = decode(self.img)
		for decoded in decode(self.img):
			# self.gps_package_cord  [x.strip() for x in self.gps_package_cord.split(',')]
			self.gps_package_string = decoded.data.split(',')
			self.gps_package_numbers = map(float, self.gps_package_string)
			self.gps_package_cord = list(self.gps_package_numbers)

		print(self.gps_package_cord)
		self.qr_cord.qr_long = self.gps_package_cord[0]
		self.qr_cord.qr_lat = self.gps_package_cord[1]
		self.qr_cord.qr_alt = self.gps_package_cord[2]

		self.qr_pub.publish(self.qr_cord)


if __name__ == '__main__':
	image_proc_obj = image_proc()
	r = rospy.Rate(25)
	while not rospy.is_shutdown():
		image_proc_obj.QR_detector()
		r.sleep()
