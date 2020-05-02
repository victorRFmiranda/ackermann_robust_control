#!/usr/bin/env python

import time
import threading, os
import numpy as np
import rospy
from lib.pid_class import PID
import tf

# messages
from geometry_msgs.msg import Twist, Wrench, Point, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu


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
		self.position = Point()
		self.velocity = 0.0
		KP = rospy.get_param('ackermann_control/pid_controller/ori_kp')
		KI = rospy.get_param('ackermann_control/pid_controller/ori_ki')
		KD = rospy.get_param('ackermann_control/pid_controller/ori_kd')
		TS = rospy.get_param('ackermann_control/pid_controller/ori_ts')
		ETOL = rospy.get_param('ackermann_control/pid_controller/ori_etol')
		self.controlador = PID(KP,KI,KD,TS,ETOL)


		rospy.init_node('orientation_PID_control_sim', anonymous=True)
		rospy.Subscriber("/robot_pose_ekf/odom_combined", PoseWithCovarianceStamped, self.EKF)
		# rospy.Subscriber("/imu_data", Imu, self.callback_odom)
		rospy.Subscriber("yaw_angle", Float32, self.callback_reference)
		self.pub_angle = rospy.Publisher('/cmd_steer', Float32, queue_size=1)


	def fix(self, number):
		if number >= 0:
			return np.floor(number)
		else:
			return np.ceil(number)


	def EKF(self, data):
		(r, p, y) = tf.transformations.euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w])
		self.orientation = y

		#print(y)


	def callback_odom(self, data):
		(r, p, y) = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])
		
		#angle = y * 180.0/np.pi

		#angle = angle % 360.0

		#radians = angle * np.pi/180.0
			
		#self.orientation = radians 

		self.orientation = y

		#print(self.orientation)
		#self.orientation = data.pose.pose.orientation.z


	def callback_reference(self, data):
		self.orientation_ref = data.data


	def run(self):
		rate = rospy.Rate(rospy.get_param('ackermann_control/steer_angle_frequency'))
		while not rospy.is_shutdown():
			### Controlador SOF
			#error = self.orientation_ref - self.orientation
			#self.steer_angle.data = 2.5370 * error * np.sign(self.velocity)

			### Controlador PID
			#self.steer_angle.data = self.controlador.update(self.orientation_ref,self.orientation)

			### PID sin error
			error = np.sin(self.orientation_ref - self.orientation)
			self.steer_angle.data = self.controlador.update_with_error(error)

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
