#!/usr/bin/env python
# Control of velocity for the ackermann robot
# using Integral Backstepping

import sys
import rospy
# library
from lib.backstepping_class import Vel_control

# messages
from geometry_msgs.msg import Twist, WrenchStamped, PoseWithCovarianceStamped, TwistStamped
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
	def __init__(self, number):
		self.vehicle_number = number
		self.velocity = 0.0
		self.vel_ref = 0.0
		self.torque = WrenchStamped()
		k = rospy.get_param('ackermann_control/controller/vel_k')
		c1 = rospy.get_param('ackermann_control/controller/vel_c1')
		c2 = rospy.get_param('ackermann_control/controller/vel_c2')
		Tz = rospy.get_param('ackermann_control/controller/vel_Ts')
		etol = rospy.get_param('ackermann_control/controller/vel_etol')
		self.controlador = Vel_control(k, c1, c2, Tz, etol)


		rospy.init_node('velocity_control_sim', anonymous=True)
		rospy.Subscriber("/odom", Odometry, self.callback_velocity)
		rospy.Subscriber("/cmd_vel_"+str(self.vehicle_number), TwistStamped, self.callback_reference)
		self.pub_torque = rospy.Publisher('/cmd_torque_'+str(self.vehicle_number), WrenchStamped, queue_size=1)


	def callback_velocity(self, data):
		if (data.header.frame_id == (("vehicle_")+str(self.vehicle_number))):
			self.velocity = data.twist.twist.linear.x

	def callback_reference(self, data):
		if (data.header.frame_id == (("vehicle_")+str(self.vehicle_number))):
			self.vel_ref = data.twist.linear.x


	def run(self):
		rate = rospy.Rate(rospy.get_param('ackermann_control/torque_frequency'))
		while not rospy.is_shutdown():
			self.torque.header.stamp = rospy.get_rostime()
			self.torque.header.frame_id = ("vehicle_")+str(self.vehicle_number)
			self.torque.wrench.torque.x = self.controlador.update(self.vel_ref,self.velocity)
			self.pub_torque.publish(self.torque)
			#print("Torque CMD: ",self.torque.torque.x)
			rate.sleep()



########### MAIN #####################
if __name__ == '__main__':
	try:
		platform = rospy.get_param('ackermann_control/platform')
		if (platform == 1):
			print("\33[92mVehicle Number:\t %s \33[0m" % sys.argv[1])
			node = simulator(sys.argv[1])
			node.run()
		else:
			node = robot()
			node.run()

	except rospy.ROSInterruptException:
		pass
