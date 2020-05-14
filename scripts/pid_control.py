#!/usr/bin/env python
# Control of longitudinal velocity for the ackermann robot
# using a PID


import time
import threading, os
import numpy as np
import rospy
from lib.pid_class import PID

# messages
from geometry_msgs.msg import Twist, Wrench, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


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
		self.velocity = 0.0
		self.vel_ref = 0.0
		self.torque = Wrench()
		KP = rospy.get_param('ackermann_control/pid_controller/kp')
		KI = rospy.get_param('ackermann_control/pid_controller/ki')
		KD = rospy.get_param('ackermann_control/pid_controller/kd')
		TS = rospy.get_param('ackermann_control/pid_controller/ts')
		ETOL = rospy.get_param('ackermann_control/pid_controller/etol')
		self.controlador = PID(KP,KI,KD,TS,ETOL)


		rospy.init_node('velocity_PID_control_sim', anonymous=True)
		rospy.Subscriber("/odom", Odometry, self.callback_velocity)
		# rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.EKF)
		rospy.Subscriber("cmd_vel", Twist, self.callback_reference)
		self.pub_torque = rospy.Publisher('/cmd_torque', Wrench, queue_size=1)



	def EKF(self, data):
		self.velocity = data.twist.twist.linear.x

	def callback_velocity(self, data):
		self.velocity = data.twist.twist.linear.x

	def callback_reference(self, data):
		self.vel_ref = data.linear.x


	def run(self):
		rate = rospy.Rate(rospy.get_param('ackermann_control/torque_frequency'))
		while not rospy.is_shutdown():
			self.torque.torque.x = self.controlador.update(self.vel_ref,self.velocity)
			# print(self.torque.torque.x)
			self.pub_torque.publish(self.torque)
			#print("Torque CMD: ",self.torque.torque.x)
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
