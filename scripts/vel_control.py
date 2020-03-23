#!/usr/bin/env python

import rospy
# library
from lib.backstepping_class import Vel_control

# messages
from geometry_msgs.msg import Twist, Wrench
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
		k = rospy.get_param('ackermann_control/controller/vel_k')
		c1 = rospy.get_param('ackermann_control/controller/vel_c1')
		c2 = rospy.get_param('ackermann_control/controller/vel_c2')
		Tz = rospy.get_param('ackermann_control/controller/vel_Ts')
		etol = rospy.get_param('ackermann_control/controller/vel_etol')
		self.controlador = Vel_control(k, c1, c2, Tz, etol)


		rospy.init_node('velocity_control_sim', anonymous=True)
		rospy.Subscriber("sim_ros_interface/odom", Odometry, self.callback_velocity)
		rospy.Subscriber("cmd_vel", Twist, self.callback_reference)
		self.pub_torque = rospy.Publisher('sim_ros_interface/cmd_torque', Wrench, queue_size=1)


	def callback_velocity(self, data):
		self.velocity = data.twist.twist.linear.x

	def callback_reference(self, data):
		self.vel_ref = data.linear.x


	def run(self):
		rate = rospy.Rate(rospy.get_param('ackermann_control/torque_frequency'))
		while not rospy.is_shutdown():
			self.torque.torque.x = self.controlador.update(self.vel_ref,self.velocity)
			self.pub_torque.publish(self.torque)
			print("Velocity CMD: ",self.torque.torque.x)
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
