#!/usr/bin/env python
# Control of orientation for the ackermann robot
# using a PID

import sys
import time
import threading, os
import numpy as np
import rospy
from lib.pid_class import PID
import tf

# messages
from geometry_msgs.msg import Twist, WrenchStamped, PoseWithCovarianceStamped, TwistStamped, QuaternionStamped
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
	def __init__(self, number):
		# self.vehicle_number = rospy.get_param('vehicle_number')
		self.vehicle_number = number
		self.orientation = 0.0
		self.orientation_ref = 0.0
		self.steer_angle = QuaternionStamped()
		self.velocity = 0.0
		self.orientation_vel = 0.0
		self.linear_vel = 0.0
		KP = rospy.get_param('ackermann_control/pid_controller/ori_kp')
		KI = rospy.get_param('ackermann_control/pid_controller/ori_ki')
		KD = rospy.get_param('ackermann_control/pid_controller/ori_kd')
		TS = rospy.get_param('ackermann_control/pid_controller/ori_ts')
		ETOL = rospy.get_param('ackermann_control/pid_controller/ori_etol')
		self.controlador = PID(KP,KI,KD,TS,ETOL)

 
		rospy.init_node('orientation_PID_control_sim_'+str(self.vehicle_number), anonymous=True)
		rospy.Subscriber("/odom", Odometry, self.callback_velocity)
		rospy.Subscriber("/imu_data", Imu, self.callback_imu)
		rospy.Subscriber("/yaw_angle", QuaternionStamped, self.callback_reference)
		#rospy.Subscriber("/cmd_vel", TwistStamped, self.callback_reference)
		#rospy.Subscriber("/cmd_vel", Twist, self.callback_reference)
		self.pub_angle = rospy.Publisher('/cmd_steer', QuaternionStamped, queue_size=1)


	def callback_velocity(self, data):
		if (data.header.frame_id == (("vehicle_")+str(self.vehicle_number))):
			self.linear_vel = data.twist.twist.linear.x
			self.orientation_vel = data.twist.twist.angular.z


	def callback_imu(self, data):
		if (data.header.frame_id == (("imu_link_")+str(self.vehicle_number))):
			(r, p, y) = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])	
			self.orientation = y


	def callback_reference(self, data):
		if (data.header.frame_id == (("vehicle_")+str(self.vehicle_number))):
			#self.orientation_ref = data.twist.angular.z
			self.orientation_ref = data.quaternion.z
		#self.orientation_ref = data.angular.z


	def run(self):
		rate = rospy.Rate(rospy.get_param('ackermann_control/steer_angle_frequency'))
		while not rospy.is_shutdown():
			### PID sin error
			error = np.sin(self.orientation_ref - self.orientation)
			#error = np.sin(self.orientation_ref - self.orientation_vel)
			#error = np.sin(self.orientation_vel - self.orientation_ref)
			self.steer_angle.header.stamp = rospy.get_rostime()
			self.steer_angle.header.frame_id = ("vehicle_")+str(self.vehicle_number)
			self.steer_angle.quaternion.z = self.controlador.update_with_error(error)
			print("Error: %.4f\n" % error)
			print("Orientation: %.4f" % self.steer_angle.quaternion.z)
			self.pub_angle.publish(self.steer_angle)
			rate.sleep()



########### MAIN #####################
if __name__ == '__main__':
	try:
		platform = rospy.get_param('ackermann_control/platform')
		if (platform == 1):
			node = simulator(sys.argv[1])
			node.run()
		else:
			node = robot()
			node.run()

	except rospy.ROSInterruptException:
		pass
