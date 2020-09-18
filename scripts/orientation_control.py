#!/usr/bin/env python
# Control of orientation for the ackermann robot
# using Integral Backstepping

import sys
import rospy
# library
from lib.backstepping_class import Orientation_control
import tf

# messages
from geometry_msgs.msg import Twist, WrenchStamped, PoseWithCovarianceStamped, TwistStamped, QuaternionStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu

 
## Class running on robot
class robot:
	def __init__(self):
		rospy.init_node('orientation_control_robot', anonymous=True)
		rospy.Subscriber("yaw_angle", Float32, self.callback_velocity)
		


	def callback_velocity(self, data):
		print("AQUI")

	def run(self):
		while not rospy.is_shutdown():
			teste = 2
			print(teste)



## Class running on simulator
class simulator:
	def __init__(self, number):
		self.vehicle_number = number
		self.orientation = 0.0
		self.orientation_ref = 0.0
		self.orientation_vel = 0.0
		self.linear_vel = 0.0
		self.steer_angle = QuaternionStamped()
		k = rospy.get_param('ackermann_control/controller/orie_k')
		c1 = rospy.get_param('ackermann_control/controller/orie_c1')
		c2 = rospy.get_param('ackermann_control/controller/orie_c2')
		Tz = rospy.get_param('ackermann_control/controller/orie_Ts')
		etol = rospy.get_param('ackermann_control/controller/orie_etol')
		self.controlador = Orientation_control(k, c1, c2, Tz, etol)


		rospy.init_node('orientation_control_sim', anonymous=True)
		rospy.Subscriber("/odom", Odometry, self.callback_odom)
		rospy.Subscriber("/imu_data", Imu, self.callback_imu)
		rospy.Subscriber("/yaw_angle", QuaternionStamped, self.callback_reference)
		self.pub_angle = rospy.Publisher('/cmd_steer_'+str(self.vehicle_number), QuaternionStamped, queue_size=1)


	def callback_imu(self, data):
		if (data.header.frame_id == (("imu_link_")+str(self.vehicle_number))):
			(r, p, y) = tf.transformations.euler_from_quaternion([data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w])	
			self.orientation = y

	def callback_odom(self, data):
		if (data.header.frame_id == (("vehicle_")+str(self.vehicle_number))):
			self.linear_vel = data.twist.twist.linear.x
			self.orientation_vel = data.twist.twist.angular.z

	def callback_reference(self, data):
		if (data.header.frame_id == (("vehicle_")+str(self.vehicle_number))):
			self.orientation_ref = data.quaternion.z

	def run(self):
		rate = rospy.Rate(rospy.get_param('ackermann_control/steer_angle_frequency'))
		while not rospy.is_shutdown():
			self.steer_angle.header.stamp = rospy.get_rostime()
			self.steer_angle.header.frame_id = ("vehicle_")+str(self.vehicle_number)
			self.steer_angle.quaternion.z = self.controlador.update(self.orientation_ref,self.linear_vel, self.orientation, self.orientation_vel)

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
