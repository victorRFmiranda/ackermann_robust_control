#!/usr/bin/env python

import rospy
# library
from lib.backstepping_class import Orientation_control

# messages
from geometry_msgs.msg import Twist, Wrench
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


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
	def __init__(self):
		self.orientation = 0.0
		self.orientation_ref = 0.0
		self.orientation_vel = 0.0
		self.linear_vel = 0.0
		self.steer_angle = Float32()
		k = rospy.get_param('ackermann_control/controller/orie_k')
		c1 = rospy.get_param('ackermann_control/controller/orie_c1')
		c2 = rospy.get_param('ackermann_control/controller/orie_c2')
		Tz = rospy.get_param('ackermann_control/controller/orie_Ts')
		etol = rospy.get_param('ackermann_control/controller/orie_etol')
		self.controlador = Orientation_control(k, c1, c2, Tz, etol)


		rospy.init_node('orientation_control_sim', anonymous=True)
		rospy.Subscriber("/odom", Odometry, self.callback_odom)
		rospy.Subscriber("yaw_angle", Float32, self.callback_reference)
		self.pub_angle = rospy.Publisher('/cmd_steer', Float32, queue_size=1)


	def callback_odom(self, data):
		self.linear_vel = data.twist.twist.linear.x
		self.orientation_vel = data.twist.twist.angular.z
		self.orientation = data.pose.pose.orientation.z

	def callback_reference(self, data):
		self.orientation_ref = data.data

	def run(self):
		rate = rospy.Rate(rospy.get_param('ackermann_control/steer_angle_frequency'))
		while not rospy.is_shutdown():
			self.steer_angle.data = self.controlador.update(self.orientation_ref,self.linear_vel, self.orientation, self.orientation_vel)
			#self.steer_angle.data = self.controlador.update_ori_vel(self.orientation_ref,self.linear_vel, self.orientation_vel)
			print(self.steer_angle)
			self.pub_angle.publish(self.steer_angle)
			#self.pub_angle.publish(0.0)
			#print("Orientation CMD: ", self.steer_angle.data, self.orientation_ref - self.orientation)
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
