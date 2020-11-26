#!/usr/bin/env python
#*********************************** This is the 2.0 version of position_controller**************************

from vitarana_drone.srv import Gripper, GripperRequest
from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix, LaserScan
import rospy
import time
import math

class Edrone():
	def __init__(self):
 		rospy.init_node('position_controller')


		#variables Declaring
		self.sample_time = 0.060  # in seconds

		# drone command
		self.drone_cmd = edrone_cmd()

		# pid values  kp[0] : longitude, kp[1] : latitude, kp[2] : altitude and likewise.
		self.kp = [0.0208, 0.0290, 508]
		self.ki = [0.0066, 0.0082, 0.070]
		self.kd = [1.692, 1.79, 5334]

		# previous errors
		self.prev_values = [0.0, 0.0, 0.0]

		# euler setpoint
		self.yaw_setpoint = 1500

		# constants to keep the drone stable and fast aswell (#Experiment)
		self.const_longitude = 0.0000300
		self.const_latitude = 0.000032
		self.const_altitude = 4.0
		#To check the setpoints
		self.land_latitude_required = 0.0
	 	self.land_latitude_current = 0.0
		self.land_longitude_required = 0.0
		self.land_longitude_current = 0.0
		#inputs 19.0007046575  71.9998955286
		self.altitude_base = 0.35
		self.longitude = 71.00
		self.latitude  = 19.000
		self.altitude = self.altitude_base  + self.const_altitude
		# To know if package has been picked up or not
		self.package_index = 0
		#just an Experiment thing
		self.altitude_correction = 1
		# obstacle avoiding thing
		self.isforward = 0
		self.obstacle_roll = 0
		self.obstacle_pitch = 0

		# [1] : longitude, [2] : latitude, [3] : altitude
		self.sensor_data = [0.0, 0.0, 0.0, 0.0]
		self.gps_required_cord_in_meter = [0.0, 0.0]
		self.gps_package_cord = [0.0, 0.0, 0.0]
		self.gps_required_cord = [self.longitude, self.latitude, self.altitude]
		self.gps_current_cord = [0.0, 0.0, 0.0]
		#*******************publisher topics***********************
		self.cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
		#self.zero_error_pub = rospy.Publisher('/drone_command', zero_error_cmd, queue_size=1)
		#self.z_error_pub = rospy.Publisher('/drone_command', z_error_cmd, queue_size=1)
		# *******************Subscriber topics***********************
		rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
		rospy.Subscriber('/gripper_status', gripper, self.gripper_res)
		rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
		rospy.Subscriber('/qr_cmd', qr_data, self.qr_data_callback)
		rospy.Subscriber('/edrone/range_finder_top', LaserScan, self.sensor_callback)
    	# rospy.Subscriber('/pid_tuning_roll', PidTune, self.longitude_set_pid)
		# rospy.Subscriber('/pid_tuning_pitch', PidTune, self.latitude_set_pid)

	def gripper_res(self, msg):
		self.package_index = msg.result

	def sensor_callback(self, msg):
		self.sensor_data[0] = msg.ranges[0]# Right sensor
		self.sensor_data[1] = msg.ranges[1]# back sensor
		self.sensor_data[2] = msg.ranges[2]# left sensor
		self.sensor_data[3] = msg.ranges[3]# front sensor

	def gps_callback(self, msg):
		self.gps_current_cord[0] = msg.longitude
		self.gps_current_cord[1] = msg.latitude
		self.gps_current_cord[2] = msg.altitude

	def qr_data_callback(self, msg):
		self.gps_package_cord[0] = msg.qr_long
		self.gps_package_cord[1] = msg.qr_lat
		self.gps_package_cord[2] = msg.qr_alt

	def package_attacment(self):
		self.package_attacment_service = rospy.ServiceProxy('/edrone/activate_gripper', Gripper)
		self.package_attacment_service.wait_for_service()
		self.result_package = self.package_attacment_service(self.gripper_state)


	# def longitude_set_pid(self, roll):
	# 	self.Kp[0] = roll.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
	# 	self.Ki[0] = roll.Ki * 0.008
	# 	self.Kd[0] = roll.Kd * 0.3
	#
	# def latitude_set_pid(self, pitch):
	#         self.Kp[0] = pitch.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
	#         self.Ki[0] = pitch.Ki * 0.008
	#         self.Kd[0] = pitch.Kd * 0.3

	def altitude_set_pid(self, throttle):
		   self.kp[1] = throttle.Kp * 0.0006
		   self.ki[1] = throttle.Ki * 0.008
		   self.kd[1] = throttle.Kd * 0.003


	def controller(self):



		time.sleep(self.sample_time)
		error_cord = [0.0, 0.0, 0.0]
		iterm_cord = [0.0, 0.0, 0.0]
		error_cord[0] = (self.gps_required_cord[0] - self.gps_current_cord[0])*200000
		error_cord[1] = (self.gps_required_cord[1] - self.gps_current_cord[1])*200000
		error_cord[2] = (self.gps_required_cord[2] - self.gps_current_cord[2])


			# This is an Experiment thing to keep the drone stable with keeping the throttle more
		self.latitude_check = math.floor( (self.gps_required_cord[1] - self.gps_current_cord[1]) * math.pow(10, 4))
		self.longitude_check =  math.floor( (self.gps_required_cord[0] - self.gps_current_cord[0]) * math.pow(10, 4))

		if self.longitude_check > 0:
			self.gps_required_cord[0] = self.longitude + self.const_longitude
			self.land_longitude_required =  math.floor( (self.gps_required_cord[0] - self.const_longitude) * math.pow(10, 6))
			self.land_longitude_current = math.floor( (self.gps_current_cord[0]) * math.pow(10, 6))

		if self.latitude_check > 0 :
			self.gps_required_cord[1] = self.latitude + self.const_latitude
			self.land_latitude_required =  math.floor( (self.gps_required_cord[1] - self.const_latitude ) * math.pow(10, 6))
			self.land_latitude_current =  math.floor( (self.gps_current_cord[1]) * math.pow(10, 6))

		if self.longitude_check < 0:
			self.gps_required_cord[0] = self.longitude - self.const_longitude
			self.land_longitude_required =  math.floor( (self.gps_required_cord[0] + self.const_longitude) * math.pow(10, 6))
			self.land_longitude_current = math.floor( (self.gps_current_cord[0]) * math.pow(10, 6))

		if self.latitude_check < 0 :
			self.gps_required_cord[1] = self.latitude - self.const_latitude
			self.land_latitude_required =  math.floor( (self.gps_required_cord[1] + self.const_latitude ) * math.pow(10, 6))
			self.land_latitude_current =  math.floor( (self.gps_current_cord[1]) * math.pow(10, 6))

		# if (self.land_latitude_required - 0.0000014444 ) == self.land_latitude_current:
		# 	self.gps_required_cord[2] = 5.8
		#
		# if self.gps_current_cord[2] <= 4.0 and self.land_latitude_required >= self.land_latitude_current:
		# 	self.gps_required_cord[2] = 4.5

		# package picking landing sequence
		if  (self.land_latitude_required + 2.544) >= self.land_latitude_current and self.altitude_correction == 1 and self.package_index == 0:
		 	 # self.gps_required_cord[1] = self.latitude
			 self.gps_required_cord[2] = self.altitude_base + 1.14
			# self.gps_required_cord[2] = 1.3
		# #
		if self.gps_current_cord[2] <= self.altitude_base and  (self.land_latitude_required + 2.544) >= self.land_latitude_current and self.altitude_correction == 1 :
			self.gps_required_cord[2] = 0.0
 			self.altitude_correction = 0

 		# picking up the package
		if self.gps_current_cord[2] <= self.altitude_base and self.package_index == 0 :
			self.gripper_state = True
			self.package_attacment()
		# getting the setpoint/gps cord from barcode
		if  self.gps_current_cord[2] <= self.altitude_base and self.package_index == 1 :
			self.gps_required_cord[2] = self.altitude
			self.latitude = self.gps_package_cord[0]
			self.longitude = self.gps_package_cord[1]


		# if self.gps_current_cord[2] >= 25.0 and self.package_index == 1:
		# 	self.gps_required_cord[1] = self.gps_package_cord[0]
		# 	self.gps_required_cord[0] = self.gps_package_cord[1]
		#***************** obstacle avoiding******************
		if self.latitude_check < 0 and self.package_index == 1:
			self.isforward = 1

		if self.isforward == 1 :
			if self.sensor_data[3] <= 18 and self.sensor_data[3] >= 0.5:
				self.obstacle_roll = 1

			if self.sensor_data[3] <= 5 and self.sensor_data[3] >= 0.5:
				self.obstacle_pitch = 1
		# making the drone achieved its package cord
		if self.gps_current_cord[1] <= 19.00034 and self.package_index == 1:
			self.obstacle_roll = 0
			self.obstacle_pitch = 0
			self.longitude = self.gps_package_cord[1] + self.const_longitude

		# package dropping landing sequence
		if  (self.land_latitude_required + 2.544) >= self.land_latitude_current and self.altitude_correction == 0 and self.package_index == 1:
			 self.gps_required_cord[2] = self.gps_package_cord[2] + 1.24


		if self.gps_current_cord[2] <= (self.gps_package_cord[2] + 0.4) and  (self.land_latitude_required + 2.544) >= self.land_latitude_current and self.altitude_correction == 0 :
			self.gps_required_cord[2] = 0.0
			self.altitude_correction = 1
		#dropping the package
		if self.gps_current_cord[2] <= (self.gps_package_cord[2] + 0.4) and self.package_index == 1 :
			self.gripper_state = False
			self.package_attacment()



		#pid tuning for longitude
		iterm_cord[0] = (iterm_cord[0] + error_cord[0])*self.ki[0]
		self.out_longitude = self.kp[0]*error_cord[0] + iterm_cord[0] + self.kd[0]*(error_cord[0] - self.prev_values[0])
		self.prev_values[0] = error_cord[0]

		#pid tuning for latitude
		iterm_cord[1] = (iterm_cord[1] + error_cord[1])*self.ki[1]
		self.out_latitude = self.kp[1]*error_cord[1] + iterm_cord[1] + self.kd[1]*(error_cord[1] - self.prev_values[1])
		self.prev_values[1] = error_cord[1]

		#pid tuning for throttle
		iterm_cord[2] = (iterm_cord[2] + error_cord[2])*self.ki[2]
		self.out_throttle = self.kp[2]*error_cord[2] + iterm_cord[2] + self.kd[2]*(error_cord[2] - self.prev_values[2])
		self.prev_values[2] = error_cord[2]

		print(self.gps_current_cord)
		print(self.gps_package_cord)
		print(self.gps_required_cord)

		#scaling the setpoints
		self.pitch_setpoint = self.out_latitude + self.obstacle_pitch
		self.roll_setpoint = self.out_longitude + self.obstacle_roll

		#publishing the setpoints
	 	self.drone_cmd.rcThrottle = self.out_throttle
		self.drone_cmd.rcRoll = self.roll_setpoint
		self.drone_cmd.rcPitch = self.pitch_setpoint
		self.drone_cmd.rcYaw = self.yaw_setpoint


		self.cmd_pub.publish(self.drone_cmd)




if __name__ == '__main__':
	e_drone = Edrone()
	r = rospy.Rate(60)
	while not rospy.is_shutdown():
		e_drone.controller()
		r.sleep()
