#!/usr/bin/env python

'''
This python file runs a ROS-node of name attitude_control which controls the roll pitch and yaw angles of the eDrone.
This node publishes and subsribes the following topics:
        PUBLICATIONS            SUBSCRIPTIONS
        /roll_error             /pid_tuning_altitude
        /pitch_error            /pid_tuning_pitch
        /yaw_error              /pid_tuning_roll
        /edrone/pwm             /edrone/imu/data
                                /edrone/drone_command

Rather than using different variables, use list. eg : self.setpoint = [1,2,3], where index corresponds to x,y,z ...rather than defining self.x_setpoint = 1, self.y_setpoint = 2
CODE MODULARITY AND TECHNIQUES MENTIONED LIKE THIS WILL HELP YOU GAINING MORE MARKS WHILE CODE EVALUATION.
'''

# Importing the required libraries

from vitarana_drone.msg import *
from pid_tune.msg import PidTune
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import rospy
import time
import tf


class Edrone():
    """docstring for Edrone"""
    def __init__(self):
        rospy.init_node('attitude_controller')  # initializing ros node with name drone_control

        # This corresponds to your current orientation of eDrone in quaternion format. This value must be updated each time in your imu callback
        # [x,y,z,w]
        self.drone_orientation_quaternion = [0.0, 0.0, 0.0, 0.0]

        # This corresponds to your current orientation of eDrone converted in euler angles form.
        # [r,p,y]
        self.drone_orientation_euler = [0.0, 0.0, 0.0]

        # This is the setpoint that will be received from the drone_command in the range from 1000 to 2000
        # [r_setpoint, p_setpoint, y_setpoint]
        self.setpoint_cmd = [0.0, 0.0, 0.0]

        # The setpoint of orientation in euler angles at which you want to stabilize the drone
        # [r_setpoint, p_psetpoint, y_setpoint]
        self.setpoint_euler = [0.0, 0.0, 0.0]

        # Declaring pwm_cmd of message type prop_speed and initializing values
        # Hint: To see the message structure of prop_speed type the following command in the terminal
        # rosmsg show vitarana_drone/prop_speed

        self.pwm_cmd = prop_speed()


        # initial setting of Kp, Kd and ki for [roll, pitch, yaw]. eg: self.Kp[2] corresponds to Kp value in yaw axis
        # after tuning and computing corresponding PID parameters, change the parameters
        self.Kp = [3.6, 1.44, 637.6]
        self.Ki = [0.016, 0.024, 0.32]
        self.Kd = [800.9, 766.5, 1390.3]
        # -----------------------Add other required variables for pid here ----------------------------------------------
	self.propspeed = 0
        self.out_longitude = 0
	self.prev_values = [0,0,0]
	self.max_values = [1024, 1024, 1024, 1024]
	self.min_values = [0, 0, 0, 0]
        # Hint : Add variables for storing previous errors in each axis, like self.prev_values = [0,0,0] where corresponds to [roll, pitch, yaw]
        #        Add variables for limiting the values like self.max_values = [1024, 1024, 1024, 1024] corresponding to [prop1, prop2, prop3, prop4]
        #                                                   self.min_values = [0, 0, 0, 0] corresponding to [prop1, prop2, prop3, prop4]
        #
        # ----------------------------------------------------------------------------------------------------------

        # # This is the sample time in which you need to run pid. Choose any time which you seem fit. Remember the stimulation step time is 50 ms
        self.sample_time = 0.060  # in seconds

        # Publishing /edrone/pwm, /roll_error, /pitch_error, /yaw_error
        self.pwm_pub = rospy.Publisher('/edrone/pwm', prop_speed, queue_size=1)
        # ------------------------Add other ROS Publishers here-----------------------------------------------------
        self.roll_pub = rospy.Publisher('/roll_error', prop_speed, queue_size=1)
        self.pitch_pub = rospy.Publisher('/pitch_error', prop_speed, queue_size=1)
        self.yaw_pub = rospy.Publisher('/yaw_error', prop_speed, queue_size=1)
        # -----------------------------------------------------------------------------------------------------------

        # Subscribing to /drone_command, imu/data, /pid_tuning_roll, /pid_tuning_pitch, /pid_tuning_yaw
        rospy.Subscriber('/drone_command', edrone_cmd, self.drone_command_callback)
        rospy.Subscriber('/edrone/imu/data', Imu, self.imu_callback)
        # rospy.Subscriber('/pid_tuning_roll', PidTune, self.roll_set_pid)
        # -------------------------Add other ROS Subscribers here----------------------------------------------------
        # rospy.Subscriber('/pid_tuning_pitch', PidTune, self.pitch_set_pid)
	    #rospy.Subscriber('/pid_tuning_yaw', PidTune, self.yaw_set_pid)
        # ------------------------------------------------------------------------------------------------------------


    def imu_callback(self, msg):

        self.drone_orientation_quaternion[0] = msg.orientation.x
        self.drone_orientation_quaternion[1] = msg.orientation.y
        self.drone_orientation_quaternion[2] = msg.orientation.z
        self.drone_orientation_quaternion[3] = msg.orientation.w


    def drone_command_callback(self, msg):
        self.setpoint_cmd[0] = msg.rcRoll
	self.setpoint_cmd[1] = msg.rcPitch
	self.setpoint_cmd[2] = msg.rcYaw
	self.propspeed = msg.rcThrottle
        self.out_longitude = msg.aux1



        # ---------------------------------------------------------------------------------------------------------------

    # Callback function for /pid_tuning_roll
    # This function gets executed each time when /tune_pid publishes /pid_tuning_roll/pid_tuning_yaw
    # def roll_set_pid(self, roll):
    #     self.Kp[0] = roll.Kp * 0.06
    #     self.Ki[0] = roll.Ki * 0.008
    #     self.Kd[0] = roll.Kd * 0.3

	#------------------------Pitch ----------------------------------
    # def pitch_set_pid(self, pitch):
    #     self.Kp[1] = pitch.Kp * 0.06
    #     self.Ki[1] = pitch.Ki * 0.008
    #     self.Kd[1] = pitch.Kd * 0.3

	#------------------------yaw ----------------------------------
    # def yaw_set_pid(self, yaw):
    #     self.Kp[2] = yaw.Kp * 0.2
    #     self.Ki[2] = yaw.Ki * 0.008
    #     self.Kd[2] = yaw.Kd * 0.3


    # ----------------------------------------------------------------------------------------------------------------------

    def pid(self):
        # -----------------------------Write the PID algorithm here--------------------------------------------------------------

        # Converting quaternion to euler angles
        (self.drone_orientation_euler[0], self.drone_orientation_euler[1], self.drone_orientation_euler[2]) = 				tf.transformations.euler_from_quaternion([self.drone_orientation_quaternion[0], self.drone_orientation_quaternion[1],  	self.drone_orientation_quaternion[2], self.drone_orientation_quaternion[3]])

        # Convertng the range from 1000 to 2000 in the range of -10 degree to 10 degree for yaw axis
        self.setpoint_euler[0] = self.setpoint_cmd[0]
        self.setpoint_euler[1] = self.setpoint_cmd[1]
        self.setpoint_euler[2] = self.setpoint_cmd[2] * 0.02 - 30


        # Complete the equations for pitch and yaw axis
	error = [0.0, 0.0, 0.0]
	Iterm = [0.0, 0.0, 0.0]
	error[0] = self.setpoint_euler[0] - self.drone_orientation_euler[0]
	error[1] = self.setpoint_euler[1] - self.drone_orientation_euler[1]
	error[2] = self.setpoint_euler[2] - self.drone_orientation_euler[2]

        # pid Output for Roll
	Iterm[0] = (Iterm[0] + error[0]) * self.Ki[0]
	self.out_roll = self.Kp[0]*error[0] + Iterm[0] + self.Kd[0]*(error[0] - self.prev_values[0])

	# pid Output for Pitch
	Iterm[1] = (Iterm[1] + error[1]) * self.Ki[1]
	self.out_pitch = self.Kp[1]*error[1] + Iterm[1] + self.Kd[1]*(error[1] - self.prev_values[1])

	# pid Output for Yaw
	Iterm[2] = (Iterm[2] + error[2]) * self.Ki[2]
	self.out_yaw = self.Kp[2]*error[2] + Iterm[2] + self.Kd[2]*(error[2] - self.prev_values[2])


	# Computing pwm to each propeller
	self.pwm_cmd.prop1 = self.propspeed - self.out_pitch - self.out_yaw  + self.out_roll
	self.pwm_cmd.prop2 = self.propspeed - self.out_pitch + self.out_yaw  - self.out_roll
	self.pwm_cmd.prop3 = self.propspeed + self.out_pitch - self.out_yaw  - self.out_roll
	self.pwm_cmd.prop4 = self.propspeed + self.out_pitch + self.out_yaw  + self.out_roll


	# Sleep for 60ms
        time.sleep(self.sample_time)

	# Setting the limits from 0 to 1023
	if self.pwm_cmd.prop1 > self.max_values[1]:
		self.pwm_cmd.prop1 = self.max_values[1]
	if self.pwm_cmd.prop2 > self.max_values[1]:
		self.pwm_cmd.prop2 = self.max_values[1]
	if self.pwm_cmd.prop3 > self.max_values[1]:
		self.pwm_cmd.prop3 = self.max_values[1]
	if self.pwm_cmd.prop4 > self.max_values[1]:
		self.pwm_cmd.prop4 = self.max_values[1]
	if self.pwm_cmd.prop1 < self.min_values[1]:
		self.pwm_cmd.prop1 = self.min_values[1]
	if self.pwm_cmd.prop2 < self.min_values[1]:
		self.pwm_cmd.prop2 = self.min_values[1]
	if self.pwm_cmd.prop3 < self.min_values[1]:
		self.pwm_cmd.prop3 = self.min_values[1]
	if self.pwm_cmd.prop4 < self.min_values[1]:
		self.pwm_cmd.prop4 = self.min_values[1]

	# Setting previous errror with the current one
	self.prev_values[0] = error[0]
	self.prev_values[1] = error[1]
	self.prev_values[2] = error[2]
        # ------------------------------------------------------------------------------------------------------------------------

        self.pwm_pub.publish(self.pwm_cmd)


if __name__ == '__main__':

    e_drone = Edrone()
    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        e_drone.pid()
        r.sleep()
