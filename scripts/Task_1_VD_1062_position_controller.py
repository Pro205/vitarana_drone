#!/usr/bin/env python


from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix
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
		self.kp = [0.1584, 0.0800, 508]
		self.ki = [0.0018, 0.0012, 0.072]
		self.kd = [1.98, 1.11, 5034]

		# previous errors
		self.prev_values = [0.0, 0.0, 0.0]

		# euler setpoint
		self.yaw_setpoint = 1500
		self.const_longitude = 0.00001000
		self.const_latitude = 0.00003
		self.const_altitude = 1.0

		self.land_latitude_required = 0.0
	 	self.land_latitude_current = 0.0
		#inputs
		self.longitude =  72.000
		self.latitude  = 19.00000
		self.altitude =  3.0 + self.const_altitude

		# [1] : longitude, [2] : latitude, [3] : altitude
		self.gps_required_cord = [self.longitude, self.latitude, self.altitude]
		self.gps_current_cord = [0.0, 0.0, 0.0]
		#publisher topics
		self.cmd_pub = rospy.Publisher('/drone_command', edrone_cmd, queue_size=1)
		#self.zero_error_pub = rospy.Publisher('/drone_command', zero_error_cmd, queue_size=1)
		#self.z_error_pub = rospy.Publisher('/drone_command', z_error_cmd, queue_size=1)
		# Subscribing to gps
		# rospy.Subscriber('/pid_tuning_altitude', PidTune, self.altitude_set_pid)
		rospy.Subscriber('/edrone/gps', NavSatFix, self.gps_callback)
    	#rospy.Subscriber('/pid_tuning_roll', PidTune, self.longitude_set_pid)
			# rospy.Subscriber('/pid_tuning_pitch', PidTune, self.latitude_set_pid)



	def gps_callback(self, msg):
		self.gps_current_cord[0] = msg.longitude
		self.gps_current_cord[1] = msg.latitude
		self.gps_current_cord[2] = msg.altitude

	# def longitude_set_pid(self, roll):
	# 	self.Kp[0] = roll.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
	# 	self.Ki[0] = roll.Ki * 0.008
	# 	self.Kd[0] = roll.Kd * 0.3
	#
	# def latitude_set_pid(self, roll):
    #     self.Kp[1] = pitch.Kp * 0.06  # This is just for an example. You can change the ratio/fraction value accordingly
    #     self.Ki[1] = pitch.Ki * 0.008
    #     self.Kd[1] = pitch.Kd * 0.3

	# def altitude_set_pid(self, throttle):
	# 	   self.kp[1] = throttle.Kp * 1
	# 	   self.ki[1] = throttle.Ki * 0.008
	# 	   self.kd[1] = throttle.Kd * 5


	def controller(self):

		time.sleep(self.sample_time)
		error_cord = [0.0, 0.0, 0.0]
		iterm_cord = [0.0, 0.0, 0.0]
		error_cord[0] = (self.gps_required_cord[0] - self.gps_current_cord[0])*200000
		error_cord[1] = (self.gps_required_cord[1] - self.gps_current_cord[1])*200000
		error_cord[2] = (self.gps_required_cord[2] - self.gps_current_cord[2])



		self.latitude_check = math.floor( (self.gps_required_cord[1] - self.gps_current_cord[1]) * math.pow(10, 4))
		self.longitude_check =  math.floor( (self.gps_required_cord[0] - self.gps_current_cord[0]) * math.pow(10, 4))

		if self.latitude_check > 0 :
			self.gps_required_cord[1] = self.latitude + self.const_latitude
			self.land_latitude_required =  math.floor( (self.gps_required_cord[1] - self.const_latitude ) * math.pow(10, 6))
			self.land_latitude_current =  math.floor( (self.gps_current_cord[1]) * math.pow(10, 6))



		if self.latitude_check < 0 :
			self.gps_required_cord[1] = self.latitude - self.const_latitude
			self.land_latitude_required =  math.floor( (self.gps_required_cord[1] + self.const_latitude ) * math.pow(10, 6))
			self.land_latitude_current =  math.floor( (self.gps_current_cord[1]) * math.pow(10, 6))

		# #Landing procedure
		land_longitude_required = math.floor( (self.gps_required_cord[0]) * math.pow(10, 6))
		land_longitude_current = math.floor( (self.gps_current_cord[0]) * math.pow(10, 6))

		# if (self.land_latitude_required - 0.0000014444 ) == self.land_latitude_current:
		# 	self.gps_required_cord[2] = 5.8
		#
		# if self.gps_current_cord[2] <= 4.0 and self.land_latitude_required >= self.land_latitude_current:
		# 	self.gps_required_cord[2] = 4.5

		if  (self.land_latitude_required + 0.000002544) >= self.land_latitude_current:
		 	 # self.gps_required_cord[1] = self.latitude
			 self.gps_required_cord[2] = 1.44
			# self.gps_required_cord[2] = 1.3
		#
		if self.gps_current_cord[2] <= 0.5 and  self.land_latitude_required >= self.land_latitude_current:
			self.gps_required_cord[2] = 0.0


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

		print(self.gps_current_cord[2])
		print(self.gps_current_cord[1])
		print(self.gps_current_cord[0])

		#scaling the setpoints
		self.pitch_setpoint = self.out_latitude
		self.roll_setpoint = self.out_longitude

		#publishing the setpoints
	 	self.drone_cmd.rcThrottle = self.out_throttle
		self.drone_cmd.rcRoll = self.roll_setpoint
		self.drone_cmd.rcPitch = self.pitch_setpoint
		self.drone_cmd.rcYaw = self.yaw_setpoint


		self.cmd_pub.publish(self.drone_cmd)



if __name__ == '__main__':
	e_drone = Edrone()
	r = rospy.Rate(30)
	while not rospy.is_shutdown():
		e_drone.controller()
		r.sleep()
