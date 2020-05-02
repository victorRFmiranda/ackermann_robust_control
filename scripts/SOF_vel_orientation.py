#!/usr/bin/env python

import time
import threading, os
import numpy as np
import rospy
from lib.pid_class import PID

# messages
from geometry_msgs.msg import Twist, Wrench
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


## Class running on robot
class robot:
	def __init__(self):
		rospy.init_node('velocity_control_robot', anonymous=True)
		rospy.Subscriber("cmd_vel", Twist, self.callback_velocity)
		


	def callback_velocity(self, data):
		print("AQUI")

	def run(self):
		while not rospy.is_shutdown():
			teste = 2
			print(teste)


## Class running on simulator
class simulator:
	def __init__(self):
		self.orientation = 0.0
		self.orientation_ref = 0.0
		self.steer_angle = Float32()
		self.K = 1.6079

		rospy.init_node('Vel_orientation_PID_control_sim', anonymous=True)
		rospy.Subscriber("/odom", Odometry, self.callback_odom)
		rospy.Subscriber("/cmd_vel", Twist, self.callback_reference)
		self.pub_angle = rospy.Publisher('/cmd_steer', Float32, queue_size=1)


	def callback_odom(self, data):
		self.orientation = data.twist.twist.angular.z

	def callback_reference(self, data):
		self.orientation_ref = data.angular.z


	def run(self):
		rate = rospy.Rate(rospy.get_param('ackermann_control/steer_angle_frequency'))
		while not rospy.is_shutdown():
			error = self.orientation_ref - self.orientation
			self.steer_angle.data = self.K*error
			self.pub_angle.publish(self.steer_angle)
			rate.sleep()



########### MAIN #####################
if __name__ == '__main__':
	try:
		platform = rospy.get_param('ackermann_control/platform')
		if (platform == 1):
			node = simulator()
			node.run()
		else:
			node = robot()
			node.run()

	except rospy.ROSInterruptException:
		pass
